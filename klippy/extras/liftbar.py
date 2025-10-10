# Liftbar module for toolchanger applications
#
# Copyright (C) 2025 Eytecz
#
# This file may be distributed under the terms of the GNU GPLv3 license

import math
import time
import logging
import threading

class Liftbar:
    def __init__(self, config):
        self.config = config
        self.printer = self.config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        
        # Read config from [liftbar] section
        self.rail_name_stepper_a = config.get('stepper_a', 'manual_stepper stepper_lb0')
        self.rail_name_stepper_b = config.get('stepper_b', 'manual_stepper stepper_lb1')
        self.homing_speed = config.getfloat('homing_speed', 10., above=0.)
        self.release_speed = config.getfloat('release_speed', 20., above=0.)
        self.position_min = config.getfloat('position_min', 185.)
        self.position_max = config.getfloat('position_max', 485.)
        self.position_endstop = config.getfloat('position_endstop', 490.)
        self.safe_tracking_distance = config.getfloat('safe_tracking_distance', 150., minval=0.)
        self.safe_y_position = config.getfloat('safe_y_position', 50.)
        self.safe_z_offset = config.getfloat('safe_z_offset', 20.)
        self.clear_z_offset = config.getfloat('clear_z_offset', 10.)
        self.clear_y_offset = config.getfloat('clear_y_offset', 10.)
        self.velocity = config.getfloat('velocity', 10., above=0.)
        self.accel = config.getfloat('accel', 1000., above=0.)

        # Create a dock dictionary to hold dock positions
        self.docks = {}
        dock_number = 0  # Start from dock_0_pos
        while True:
            dock_key = f'dock_{dock_number}_pos'
            dock_value = config.get(dock_key, None)  # Get the value, or None if not found
            if dock_value is None:
                break  # Stop if no more docks are found
            self.docks[dock_number] = self._parse_pos(dock_value)
            dock_number += 1

        # Initial state
        self.synced = False
        self.sync_status_mem = None
        self.homed = False
        self.mounted_tool = None
        self.init_toolhead_pos = None
        self.init_liftbar_pos = None
        self.liftbar_pos = None

        # Register handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.register_event_handler("stepper_enable:motor_off", self._stepper_status)

        # Register g-code commands
        self.gcode.register_command('LIFTBAR', self.cmd_LIFTBAR,
                                    desc = "Command a configured liftbar")
        self.gcode.register_command('LIFTBAR_HOME', self.cmd_LIFTBAR_HOME,
                                    desc = "Command to home the liftbar")
        self.gcode.register_command('LIFTBAR_SYNC', self.cmd_LIFTBAR_SYNC,
                                    desc = "Synchronizes the liftbar to the z-axis motion")
        self.gcode.register_command('LIFTBAR_STATUS', self.cmd_LIFTBAR_STATUS,
                                    desc = "Returns the status of the liftbar")
        self.gcode.register_command('LIFTBAR_DROPOFF', self.cmd_LIFTBAR_DROPOFF,
                                    desc = "Docks a toolhead")
        self.gcode.register_command('LIFTBAR_PICKUP', self.cmd_LIFTBAR_PICKUP,
                                    desc = "Undocks a toolhead")
        self.gcode.register_command('TOOLCHANGER_STATS', self.cmd_TOOLCHANGER_STATS,
                                    desc = "Returns the stats of the toolchanger")                           
 
    
    def handle_ready(self):
        self.min_event_systime = self.reactor.monotonic()
    
    def handle_connect(self):   
        for manual_stepper in self.printer.lookup_objects('manual_stepper'):
            rail_name = manual_stepper[1].get_steppers()[0].get_name()
            if rail_name == self.rail_name_stepper_a:
                self.stepper_a = manual_stepper[1]
            elif rail_name == self.rail_name_stepper_b:
                self.stepper_b = manual_stepper[1]
        if not self.stepper_a:
            raise self.printer.config_error(f"manual_extruder_stepper '{self.rail_name_stepper_a}' must be specified")
        if not self.stepper_b:
            raise self.printer.config_error(f"manual_extruder_stepper '{self.rail_name_stepper_b}' must be specified")
        self.steppers = [self.stepper_a, self.stepper_b]

        # Intercept z-axis motions on the toolhead and replace with custom one
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gcode_move = self.printer.lookup_object('gcode_move')
        self.gcode_axis_sync = self.printer.lookup_object('gcode_axis_sync')
        
    def cmd_LIFTBAR(self, gcmd):
        enable = gcmd.get_int('ENABLE', None)
        if enable is not None:
            self.do_enable(enable)
        setpos = gcmd.get_float('SET_POSITION', None)
        if setpos is not None:
            self.do_set_position(setpos)
        speed = gcmd.get_float('SPEED', self.velocity, above=0.)
        accel = gcmd.get_float('ACCEL', self.accel, minval=0.)
        homing_move = gcmd.get_int('STOP_ON_ENDSTOP', 0)
        if homing_move:
            movepos = gcmd.get_float('MOVE')
            self.do_homing_move(movepos, speed, accel, homing_move > 0, abs(homing_move) == 1)
        elif gcmd.get_float('MOVE', None) is not None:
            self.do_sync(sync=False)
            movepos = gcmd.get_float('MOVE')
            sync = gcmd.get_int('SYNC', 1)
            self.do_move(movepos, speed, accel, sync)
        elif gcmd.get_int('SYNC', 0):
            self.stepper_a.sync_print_time()
            self.stepper_b.sync_print_time()        
    
    def cmd_LIFTBAR_HOME(self, gcmd):
        self.do_home()

    def cmd_LIFTBAR_SYNC(self, gcmd):
        sync = gcmd.get('SYNC', True)
        self.do_sync(sync)
    
    def cmd_LIFTBAR_STATUS(self, gcmd):
        self.gcode.respond_info(f"Liftbar homed: {self.homed}")
        self.gcode.respond_info(f"Liftbar synced: {self.synced}")
        self.gcode.respond_info(f"liftbar_pos = {self.get_position()[0]}")
        self.gcode.respond_info(f"toolhead_pos = {self.toolhead.get_position()[2]}")
        if self.docks:
            for dock_number, (x, y) in self.docks.items():
                self.gcode.respond_info(f"Dock {dock_number}: ({x}, {y})")
        else:
            self.gcode.respond_info("No docks configured")
    
    def cmd_LIFTBAR_DROPOFF(self, gcmd):
        dock = gcmd.get_int('DOCK', 0)
        self.do_tool_dropoff(dock)
    
    def cmd_LIFTBAR_PICKUP(self, gcmd):
        dock = gcmd.get_int('DOCK', 0)
        self.do_tool_pickup(dock)
    
    def cmd_TOOLCHANGER_STATS(self, gcmd):
        toolchanger_stats = self.printer.lookup_object('toolchanger').get_status(self.reactor.monotonic())
        self.gcode.respond_info(f"name: {toolchanger_stats['name']}")
        self.gcode.respond_info(f"status: {toolchanger_stats['status']}")
        self.gcode.respond_info(f"tool: {toolchanger_stats['tool']}")
        self.gcode.respond_info(f"tool_number: {toolchanger_stats['tool_number']}")
        self.gcode.respond_info(f"detected_tool: {toolchanger_stats['detected_tool']}")
        self.gcode.respond_info(f"detected_tool_number: {toolchanger_stats['detected_tool_number']}")
        self.gcode.respond_info(f"tool_numbers: {toolchanger_stats['tool_numbers']}")
        self.gcode.respond_info(f"tool_names: {toolchanger_stats['tool_names']}")
        self.gcode.respond_info(f"has_detection: {toolchanger_stats['has_detection']}")

    def do_enable(self, enable):
        self.stepper_a.do_enable(enable)
        self.toolhead.wait_moves()
        self.stepper_b.do_enable(enable)
        self.toolhead.wait_moves() 
    
    def do_set_position(self, setpos):
        self.stepper_a.do_set_position(setpos)
        self.toolhead.wait_moves()
        self.stepper_b.do_set_position(setpos)
        self.toolhead.wait_moves()

    def do_homing_move(self, movepos, speed, accel, triggered, check_triggered):
        if not self.stepper_a.can_home:
            raise self.printer.command_error(f"No endstop configured for '{self.rail_name_stepper_a}'")
        if not self.stepper_b.can_home:
            raise self.printer.command_error(f"No endstop configured for '{self.rail_name_stepper_b}'")
        
        self.homing_accel = accel
        endstop_a = self.stepper_a.rail.get_endstops()
        endstop_b = self.stepper_b.rail.get_endstops()
        phoming = self.printer.lookup_object('homing')
        self.do_enable(enable=0)
        
        pos = [self.position_min, 0., 0., 0.]
        try:
            phoming.manual_home(self.stepper_a, endstop_a, pos, speed, triggered, check_triggered=True)
            self.toolhead.wait_moves()
        except Exception as e:
            logging.info(f"Exception during homing of stepper a: {e}")

        pos2 = [self.position_max - 10, 0., 0., 0.]
        try:
            phoming.manual_home(self.stepper_b, endstop_b, pos2, speed, triggered, check_triggered)
            self.toolhead.wait_moves()
        except Exception as e:
            logging.info(f"Exception during homing of stepper b: {e}")

    def do_home(self):
        self.do_set_position(setpos=self.position_max)
        self.do_homing_move(self.position_endstop, self.homing_speed, self.accel,
                            triggered=True, check_triggered=True)
        self.do_set_position(setpos=self.position_min)
        retract_pos = self.position_min + (self.position_min - self.position_endstop)
        self.do_move(retract_pos, self.homing_speed, self.accel, sync=0)
        self.do_set_position(setpos=self.position_min)
        self.homed = True
        self.liftbar_pos = self.position_min

    def do_move(self, movepos, speed, accel, sync):
        # Check if movepos is within specified limits
        if movepos < self.position_endstop or movepos > self.position_max:
            raise self.printer.command_error(f"Move position {movepos} is out of bounds. "
                            f"Allowed range: {self.position_min} to {self.position_max}.")
        # Proceed with the move
        self.stepper_a.do_move(movepos, speed, accel, sync=0)
        self.stepper_b.do_move(movepos, speed, accel, sync)
        self.liftbar_pos = movepos
    
    def do_sync(self, sync):
        if sync == None:
            self.synced = not self.synced
        else:
            self.synced = sync
        if self.synced == True:
            self.gcode_axis_sync.sync_manual_stepper(self.rail_name_stepper_a.split()[1], master_axis_id="Z", absolute=True,
                                        limited=True, invert=False, offset=self.safe_tracking_distance)
            self.gcode_axis_sync.sync_manual_stepper(self.rail_name_stepper_b.split()[1], master_axis_id="Z", absolute=True,
                                        limited=True, invert=False, offset=self.safe_tracking_distance)
            self.gcode_axis_sync.run_presync_queue()
        else:
            self.gcode_axis_sync.unsync_manual_stepper(self.rail_name_stepper_a.split()[1])
            self.gcode_axis_sync.unsync_manual_stepper(self.rail_name_stepper_b.split()[1])

    def do_tool_dropoff(self, dock):
        # Unsync liftbar to allow independent motion
        self.sync_status_mem = self.synced
        self.synced = False

        try:
            x, y = self.docks[dock]

            # Get positions of toolhead and liftbar
            self.init_toolhead_pos = self.toolhead.get_position()
            self.init_liftbar_pos = self.get_position()[0]
            logging.info(f"self.init_liftbar_pos = {self.init_liftbar_pos} set")

            # Calculate effective toolchange height
            toolchange_pos_z = (self.init_toolhead_pos[2] + self.init_liftbar_pos) / 2

            if toolchange_pos_z < self.position_min:
                liftbar_pos = self.position_min
                toolhead_pos = liftbar_pos + self.safe_z_offset
            else:
                liftbar_pos = toolchange_pos_z
                toolhead_pos = toolchange_pos_z + self.safe_z_offset

            # If unsafe travel condition, move y-axis to safe position
            pos = self.init_toolhead_pos
            if pos[1] < self.safe_y_position:
                pos[1] = self.safe_y_position
                self.toolhead.move(pos, self.toolhead.max_velocity)
                self.toolhead.wait_moves()
            
            # Move toolhead and liftbar to position in front of dock at right toolchange z-height
            pos[0] = x
            pos[1] = self.safe_y_position
            pos[2] = toolhead_pos
            self.do_move(liftbar_pos, self.velocity, self.accel, sync=0)          
            self.toolhead.move(pos, self.toolhead.max_velocity)
            # self.toolhead.wait_moves()

            # Move toolhead above dock
            pos[1] = y
            self.toolhead.move(pos, self.toolhead.max_velocity)
            # self.toolhead.wait_moves()

            # Lower toolhead onto dock and proceed to release position
            pos[2] = liftbar_pos - self.clear_z_offset
            self.toolhead.move(pos, self.release_speed)
            # self.toolhead.wait_moves()
            pos[1] = y + self.clear_y_offset
            self.toolhead.move(pos, self.toolhead.max_velocity)
            self.toolhead.wait_moves()
        
        except Exception as e:
            self.printer.command_error(f"Error during tool dropoff: {e}")
        
        finally:
            self.synced = self.sync_status_mem
            self.toolhead.wait_moves()

    def do_tool_pickup(self, dock):
        # Unsync liftbar to allow independent motion
        self.sync_status_mem = self.synced
        self.synced = False

        try:
            x, y = self.docks[dock]

            # Get positions of toolhead and liftbar
            init_toolhead_pos = self.toolhead.get_position()
            init_liftbar_pos = self.get_position()[0]

            # Calculate effective toolchange height
            toolchange_pos_z = ((init_toolhead_pos[2] + init_liftbar_pos) + self.clear_z_offset) / 2

            if toolchange_pos_z < self.position_min:
                liftbar_pos = self.position_min
                toolhead_pos = liftbar_pos - self.clear_z_offset
            else:
                liftbar_pos = toolchange_pos_z
                toolhead_pos = toolchange_pos_z - self.clear_z_offset

            # If unsafe travel condition, move y-axis to safe position
            pos = init_toolhead_pos
            if pos[1] < y + self.clear_y_offset:
                pos[1] = y + self.clear_y_offset
                self.toolhead.move(pos, self.toolhead.max_velocity)
                # self.toolhead.wait_moves()
            
            # Move toolhead and liftbar to position in front of dock at right toolchange z-height
            pos[0] = x
            pos[1] = y + self.clear_y_offset
            pos[2] = toolhead_pos
            self.do_move(liftbar_pos, self.velocity, self.accel, sync=0)          
            self.toolhead.move(pos, self.toolhead.max_velocity)
            # self.toolhead.wait_moves()

            # Move carriage against guides and raise to pickup toolhead
            pos[1] = y
            self.toolhead.move(pos, self.release_speed)
            # self.toolhead.wait_moves()
            pos[2] = liftbar_pos + self.clear_z_offset
            self.toolhead.move(pos, self.release_speed)
            # self.toolhead.wait_moves()

            # Move back to safe position with toolhead attached
            pos[1] = self.safe_y_position
            self.toolhead.move(pos, self.toolhead.max_velocity)
            self.toolhead.wait_moves()

        except Exception as e:
            raise self.printer.command_error(f"Error during tool pickup: {e}")
        
        finally:
            self.synced = self.sync_status_mem
            if not self.synced:
                self.do_move(self.init_liftbar_pos, self.velocity, self.accel, sync=0)
            self.toolhead.wait_moves()

    def get_position(self):
        return self.stepper_a.get_position()

    def _parse_pos(self, value):
        try:
            x, y = map(float, value.replace(" ", "").split(','))
            return (x, y)
        except ValueError:
            raise ValueError(f"Invalid position format for {value}. Expected format 'x, y'.")

    def _stepper_status(self):
        stepper_enable = self.printer.lookup_object('stepper_enable')
        steppers  = [self.rail_name_stepper_a, self.rail_name_stepper_b]
        for s in steppers:
            se = stepper_enable.lookup_enable(s)
            is_enabled = se.is_enabled
            if is_enabled == False:
                self.homed = False
  
    def get_status(self, eventtime):
            return {'mounted_tool': self.mounted_tool,
                    'liftbar_pos': self.liftbar_pos}

def load_config(config):
    return Liftbar(config)