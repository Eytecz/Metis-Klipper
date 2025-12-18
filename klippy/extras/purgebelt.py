# PurgeBelt module for klipper
#
# Copyright (C) 2025 Eytecz Engineering
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math

class PurgeBelt:
    def __init__(self, config):
        self.config = config
        self.printer = self.config.get_printer()
        self.reactor = self.printer.get_reactor()

        # Read config from [purgebelt] section
        self.park_pos_x = config.getfloat('park_pos_x', 10.)                                   
        self.park_pos_y = config.getfloat('park_pos_y', 10.)                                   
        self.park_pos_z = config.getfloat('park_pos_z', 2.)                           
        self.purge_belt_z = config.getfloat('purge_belt_z', 0.)                        
        self.filament_diameter = config.getfloat('filament_diameter', 1.75, above=0)   
        self.purge_length = config.getfloat('purge_length', 50., above=0)
        self.purged_length_section_max = config.getfloat('purged_length_section_max', 200., above=0)         
        self.purge_layer_height = config.getfloat('purge_layer_height', 0.4, above=0.)          
        self.purge_extrusion_width = config.getfloat('purge_extrusion_width', 0.4, above=0.)    
        self.purge_belt_outfeed_length = config.getfloat('purge_belt_outfeed_length', 100., above=0)
        self.purge_belt_retract_travel_dist = config.getfloat('purge_belt_retract_travel_dist', 10., above=0)
        self.extrude_speed = config.getfloat('extrude_speed', 5., above=0)
        self.travel_speed = config.getfloat('travel_speed', 100., above=0)
        self.approach_speed = config.getfloat('approach_speed', 5., above=0)
        self.retract_speed = config.getfloat('retract_speed', 20., above=0)
        self.retract_dist = config.getfloat('retract_dist', 5., above=0)
        self.pause_time = config.getfloat('pause_time', 1., above=0)
        self.pause_qty = config.getint('pause_qty', 1)
        self.return_to_start_pos = config.getboolean('return_to_start_pos', True)
        self.max_flow_rate = config.getfloat('max_flow_rate', 40., above=0)
        self.flow_rate = config.getfloat('flow_rate', 30., above=0)
        self.stepper_name = config.get('stepper_name', 'purge_belt_stepper')

        # Register handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        # Register g-code commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('SYNC_PURGE_BELT', self.cmd_SYNC_PURGE_BELT,
                                    desc = "Synchronizes the purge belt to the active extruder")
        self.gcode.register_command('UNSYNC_PURGE_BELT', self.cmd_UNSYNC_PURGE_BELT,
                                    desc = "Unsynchronizes the purge belt")
        self.gcode.register_command('PURGE_WITH_BELT', self.cmd_PURGE_WITH_BELT, desc="Run purge belt cycle")

        # Internal state
        self.sync_status = False
        self.purge_belt_stepper = None
        self.last_purge_belt_rotation_distance = None

    def handle_ready(self):
        self.min_event_systime = self.reactor.monotonic()

    def handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')

        for manual_stepper in self.printer.lookup_objects('manual_stepper'):
            rail_name = manual_stepper[1].get_steppers()[0].get_name()
            if rail_name == f'manual_stepper {self.stepper_name}':
                self.purge_belt_stepper = manual_stepper[1]
        if self.purge_belt_stepper is None:
            raise self.printer.config_error(f"manual_stepper {self.stepper_name} must be specified")
    
    # Calculate new purge belt rotation distance when synchronized to the extruder
    def calc_purge_belt_rotation_distance_synced(self, layer_height, extrusion_width):
        extruder_flow_area = (math.pi / 4) * self.filament_diameter ** 2
        purge_belt_flow_correction_factor = extruder_flow_area / (layer_height * extrusion_width)
        purge_belt_rotation_distance_initial = self.purge_belt_stepper.get_steppers()[0].get_rotation_distance()[0]

        # When synched, klipper automatically wants to extrude an equal amount of material, however when synched, the goal
        # is to actually run the belt faster/slower based on the desired layer height and width. However, we let klipper
        # think it is still 'matched' to before/after extruding
        purge_belt_rotation_distance_synced = purge_belt_rotation_distance_initial / purge_belt_flow_correction_factor
        return purge_belt_rotation_distance_synced

    def cmd_SYNC_PURGE_BELT(self, gcmd):
        layer_height = gcmd.get_float('LAYER_HEIGHT', self.purge_layer_height)
        extrusion_width = gcmd.get_float('EXTRUSION_WIDTH', self.purge_extrusion_width)
        self.sync_purge_belt(layer_height, extrusion_width)

    def cmd_UNSYNC_PURGE_BELT(self, gcmd):
        self.unsync_purge_belt()
    
    def sync_purge_belt(self, layer_height, extrusion_width):
        if self.sync_status:
            pass
        else:
            self.last_purge_belt_rotation_distance = self.purge_belt_stepper.get_steppers()[0].get_rotation_distance()[0]
            rotation_dist = self.calc_purge_belt_rotation_distance_synced(layer_height, extrusion_width)
            self.purge_belt_stepper.get_steppers()[0].set_rotation_distance(rotation_dist)
            axis_sync = self.printer.lookup_object('axis_sync')
            extruder_name = self.toolhead.get_extruder().get_name()
            stepper_name = f'manual_stepper {self.stepper_name}'
            axis_sync.sync_stepper_to_extruder(stepper_name, extruder_name)
            self.sync_status = True

    def unsync_purge_belt(self):
        if not self.sync_status:
            pass
        else:
            axis_sync = self.printer.lookup_object('axis_sync')
            stepper_name = f'manual_stepper {self.stepper_name}'
            axis_sync.sync_stepper_to_extruder(stepper_name, extruder_name=None)
            if self.last_purge_belt_rotation_distance:
                self.purge_belt_stepper.get_steppers()[0].set_rotation_distance(self.last_purge_belt_rotation_distance)
            self.sync_status = False

    def get_init_toolhead_pos(self):
        init_toolhead_pos = self.toolhead.get_position()
        return list(init_toolhead_pos)

    def cmd_PURGE_WITH_BELT(self, gcmd):
        layer_height = gcmd.get_float('LAYER_HEIGHT', self.purge_layer_height)
        extrusion_width = gcmd.get_float('EXTRUSION_WIDTH', self.purge_extrusion_width)
        purge_volume = gcmd.get_float('PURGE_VOLUME', None)
        if purge_volume:
            purge_length = purge_volume / ((math.pi / 4) * (self.filament_diameter ** 2))
        else:
            purge_length = gcmd.get_float('PURGE_LENGTH', self.purge_length)
            purge_volume = ((math.pi / 4) * (self.filament_diameter ** 2)) * purge_length
        purged_length = purge_volume / (extrusion_width * layer_height)
        pause_qty = gcmd.get_int('PAUSE_QTY', None)
        if pause_qty is None and purged_length > self.purged_length_section_max:
            pause_qty = int(math.ceil(purged_length / self.purged_length_section_max))
        else:
            pause_qty = gcmd.get_int('PAUSE_QTY', self.pause_qty)
        flow_rate = gcmd.get_float('FLOW_RATE', None)
        if flow_rate is not None:
            if flow_rate > self.max_flow_rate:
                flow_rate = self.max_flow_rate
            extrusion_speed = flow_rate / ((math.pi / 4) * (self.filament_diameter ** 2))
        else:
            extrusion_speed = gcmd.get_float('EXTRUSION_SPEED', None)
            if extrusion_speed is not None:
                extrusion_speed = self.extrude_speed
                flow_rate = extrusion_speed * ((math.pi / 4) * (self.filament_diameter ** 2))
            else:
                flow_rate = self.flow_rate
                extrusion_speed = flow_rate / ((math.pi / 4) * (self.filament_diameter ** 2))
        pause_time = gcmd.get_float('PAUSE_TIME', self.pause_time)
        self.gcode.respond_info(f"Purging {purge_volume:.2f} mm³ at {flow_rate:.2f} mm³/s in {pause_qty + 1} step(s)")
        self.purge_cycle(purge_length, layer_height, extrusion_width, extrusion_speed, pause_qty, pause_time)

    def move_to_park_pos(self):
        # Get initial position to return to
        self.init_pos = self.get_init_toolhead_pos()

        # Travel to park location - create new position array
        new_pos = list(self.init_pos)
        new_pos[0] = self.park_pos_x
        new_pos[1] = self.park_pos_y
        new_pos[2] = self.park_pos_z
        # Keep current extruder position
        self.toolhead.move(new_pos, self.travel_speed)
        #self.toolhead.wait_moves()
    
    def restore_pos(self):
        # Ensure all moves are done before restoring position
        #self.toolhead.wait_moves()
        self.toolhead.set_position(self.toolhead.get_position())

        # Return to initial position
        if self.return_to_start_pos:
            # Restore XYZ but keep current extruder position
            current_pos = self.toolhead.get_position()
            restore_pos = list(self.init_pos)
            restore_pos[3] = current_pos[3]  # Keep current E position
            self.toolhead.move(restore_pos, self.travel_speed)
            #self.toolhead.wait_moves()
    
    def purge_cycle(self, purge_length, layer_height, extrusion_width, extrusion_speed, pause_qty, pause_time):
        # Travel to park position
        self.move_to_park_pos()

        # Pre-calculate purge height
        purge_height = self.purge_belt_z + layer_height

        # Purge with or without pauses
        if pause_qty == 0:
            # Travel to purge height above belt depending on layer height
            current_pos = self.toolhead.get_position()
            purge_pos = list(current_pos)
            purge_pos[2] = purge_height
            self.toolhead.move(purge_pos, self.approach_speed) 
            #self.toolhead.wait_moves()      

            # Do the actual purging
            self.sync_purge_belt(layer_height, extrusion_width)
            current_pos = self.toolhead.get_position()
            extrude_pos = list(current_pos)
            extrude_pos[3] += purge_length
            self.toolhead.move(extrude_pos, extrusion_speed)
            #self.toolhead.wait_moves()
            self.unsync_purge_belt()

            # Retract filament and keep belt running for an additional distance
            current_pos = self.toolhead.get_position()
            retract_pos = list(current_pos)
            retract_pos[3] -= self.retract_dist
            movepos = self.purge_belt_stepper.get_position()[0] + self.purge_belt_retract_travel_dist
            self.purge_belt_stepper.do_move(movepos, self.purge_belt_stepper.velocity, self.purge_belt_stepper.accel, sync=True)
            self.toolhead.move(retract_pos, self.retract_speed)
            #self.toolhead.wait_moves()
            
            # Return to parking position
            current_pos = self.toolhead.get_position()
            park_pos = list(current_pos)
            park_pos[2] = self.park_pos_z
            self.toolhead.move(park_pos, self.travel_speed)
            #self.toolhead.wait_moves()
        else:
            purge_length_per_section = purge_length / (pause_qty + 1)
            for i in range(pause_qty + 1):
                # Travel to purge height above belt depending on layer height
                current_pos = self.toolhead.get_position()
                purge_pos = list(current_pos)
                purge_pos[2] = purge_height
                self.toolhead.move(purge_pos, self.approach_speed) 
                #self.toolhead.wait_moves()

                # Do the actual purging
                self.sync_purge_belt(layer_height, extrusion_width)
                current_pos = self.toolhead.get_position()
                extrude_pos = list(current_pos)
                extrude_pos[3] += purge_length_per_section
                self.toolhead.move(extrude_pos, extrusion_speed)
                #self.toolhead.wait_moves()
                self.unsync_purge_belt()

                # Retract filament and keep belt running for an additional distance
                current_pos = self.toolhead.get_position()
                retract_pos = list(current_pos)
                retract_pos[3] -= self.retract_dist
                movepos = self.purge_belt_stepper.get_position()[0] + self.purge_belt_retract_travel_dist
                self.purge_belt_stepper.do_move(movepos, self.purge_belt_stepper.velocity, self.purge_belt_stepper.accel, sync=True)
                self.toolhead.move(retract_pos, self.retract_speed)
                #self.toolhead.wait_moves()

                # Return to parking position
                current_pos = self.toolhead.get_position()
                park_pos = list(current_pos)
                park_pos[2] = self.park_pos_z
                self.toolhead.move(park_pos, self.travel_speed)
                #self.toolhead.wait_moves()

                # Dwell for the defined pause time if not last iteration
                if i < pause_qty:
                    delay = pause_time
                    self.toolhead.dwell(delay)
        
        # Clear belt from filament
        pos_belt = self.purge_belt_stepper.get_position()[0]
        movepos = pos_belt + self.purge_belt_outfeed_length
        self.purge_belt_stepper.do_move(movepos, self.purge_belt_stepper.velocity, self.purge_belt_stepper.accel, sync=True)
        #self.toolhead.wait_moves()

        # Return to start position
        self.restore_pos()
        
def load_config(config):
  return PurgeBelt(config)