# Toolhead docking module - manages complete pickup/dropoff sequences
# Can optionally use docking_axis for dynamic dock positioning
#
# Copyright (C) 2025 Eytecz
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import math

# Docking axis behaviour options
axis_modes = {
    'static': 'static',         # No docking axis movement
    'balanced': 'balanced',     # Split motion, docking axis and z-axis meet halfway
    'minimize_z': 'minimize_z', # Minimize z-axis movement, docking axis does most of the work
}

MAX_CURRENT = 2.000

class TMCCurrentHelper:
    def __init__(self, config):
        self.config = config
        self.printer= config.get_printer()
        
        # Initial state
        self.tmc_drivers = {}

        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
    
    def handle_connect(self):
        for name, obj in self.printer.lookup_objects():
            if name.startswith('tmc'):
                logging.info(f"Found TMC driver object: {name}")
                stepper_name = name.split()[1] if len(name.split()) > 1 else None
                if stepper_name and (stepper_name.startswith('stepper_x') or 
                                    stepper_name.startswith('stepper_y')):
                    self.tmc_drivers[stepper_name] = {
                        'object': obj,
                        'fields': obj.fields,
                        'mcu_tmc': obj.mcu_tmc,
                        'sense_resistor': self.config.getsection(name).getfloat('sense_resistor', 0.110, above=0.),
                        'req_hold_current': self.config.getsection(name).getfloat('hold_current', MAX_CURRENT, above=0., maxval=MAX_CURRENT),
                    }
        if 'stepper_x' not in self.tmc_drivers or 'stepper_y' not in self.tmc_drivers:
            raise self.config.error(
                "Missing required tmc drivers for stepper_x and stepper_y to set cutting current")
    
    def _calc_current_bits(self, current, vsense, name):
        sense_resistor = self.tmc_drivers[name]['sense_resistor'] + 0.020
        vref = 0.32
        if vsense:
            vref = 0.18
        cs = int(32. * sense_resistor * current * math.sqrt(2.) / vref + .5) - 1
        return max(0, min(31, cs))

    def _calc_current_from_bits(self, cs, vsense, name):
        sense_resistor = self.tmc_drivers[name]['sense_resistor'] + 0.020
        vref = 0.32
        if vsense:
            vref = 0.18
        return (cs + 1) * vref / (32. * sense_resistor * math.sqrt(2.))
        
    def _calc_current(self, run_current, hold_current, name):
        vsense = True
        irun = self._calc_current_bits(run_current, True, name)
        if irun == 31:
            cur = self._calc_current_from_bits(irun, True, name)
            if cur < run_current:
                irun2 = self._calc_current_bits(run_current, False, name)
                cur2 = self._calc_current_from_bits(irun2, False, name)
                if abs(run_current - cur2) < abs(run_current - cur):
                    vsense = False
                    irun = irun2
        ihold = self._calc_current_bits(min(hold_current, run_current), vsense, name)
        return vsense, irun, ihold

    def get_current(self, name):
        driver = self.tmc_drivers.get(name, None)
        if driver is None:
            raise self.printer.command_error(f"Requested TMC driver '{name}' not found")
        irun = driver['fields'].get_field('irun')
        ihold = driver['fields'].get_field('ihold')
        vsense = driver['fields'].get_field("vsense")
        run_current = self._calc_current_from_bits(irun, vsense, name)
        hold_current = self._calc_current_from_bits(ihold, vsense, name)
        req_hold_current = driver['req_hold_current']
        return run_current, hold_current, req_hold_current, MAX_CURRENT

    def set_current(self, run_current, hold_current, print_time, name):
        driver = self.tmc_drivers.get(name, None)
        if driver is None:
            raise self.printer.command_error(f"Requested TMC driver '{name}' not found")
        driver['req_hold_current'] = hold_current
        vsense, irun, ihold = self._calc_current(run_current, hold_current, name)
        if vsense != driver['fields'].get_field("vsense"):
            val = driver['fields'].set_field("vsense", vsense)
            driver['mcu_tmc'].set_register("CHOPCONF", val, print_time)
        driver['fields'].set_field("ihold", ihold)
        val = driver['fields'].set_field("irun", irun)
        driver['mcu_tmc'].set_register("IHOLD_IRUN", val, print_time)

class Dock:
    def __init__(self, config):
        self.config = config
        self.printer= config.get_printer()
        self.reactor = self.printer.get_reactor()

        # Read config section
        self.name = config.get_name().split()[1]
        self.extruder_name = config.get('extruder', 'extruder')
        self.filament_sensor = config.get('filament_sensor', None)
        self.toolhead_detect = config.getboolean('toolhead_detect', False)

        self.docking_axis = config.getboolean('docking_axis', False)
        if self.docking_axis:
            self.axis_mode = config.getchoice(
                'axis_mode', axis_modes, 'balanced')
        else:
            self.axis_mode = axis_modes['static']

        self.restore_axes = config.getlist(
            'restore_axes', ['y', 'x', 'z', 'docking_axis'])

        self.docking_speed = config.getfloat('docking_speed', 20., above=0.)
        self.engage_speed = config.getfloat('engage_speed', 10., above=0.)
        self.disengage_speed = config.getfloat('disengage_speed', 10., above=0.)
        self.cut_speed = config.getfloat('cut_speed', 5., above=0.)
        self.travel_speed = config.getfloat('travel_speed', 100., above=0.)

        # Docked positions define the 'parked' position of the toolhead when docked
        self.docked_position_x = config.getfloat('docked_position_x')
        self.docked_position_y = config.getfloat('docked_position_y')
        if self.axis_mode == 'static':
            self.docked_position_z = config.getfloat('docked_position_z')
        else:
            # Offset is from toolhead relative to docking axis, where docking axis is considered as z=0
            self.docked_offset_z = config.getfloat('docked_offset_z')   

        # Docking sequence parameters
        self.safe_position_y = config.getfloat(
            'safe_position_y', 80., minval=0.)       # Position to clear all toolheads
        self.safe_offset_z = config.getfloat(
            'safe_offset_z', 20., minval=0.)         # Offset above dock to enter safely
        self.slide_distance_y = config.getfloat(
            'slide_distance_y', 3., minval=0.)       # Distance to slide horizontally into dock
        self.disengage_offset_z = config.getfloat(
            'disengage_offset_z', 8., minval=0.)     # Offset to drop towards dock when disengaging
        self.disengage_offset_y = config.getfloat(
            'disengage_offset_y', 20., minval=0.)    # Offset to back away from dock when disengaging
        
        # Cutting sequence parameters
        self.filament_cutter = config.getboolean('filament_cutter', False)
        self.cutter_position_x = config.getfloat(
            'cutter_position_x', None, minval=0.)
        self.cutter_position_y = config.getfloat( 
            'cutter_position_y', None, minval=0.)
        self.cutter_offset_z = config.getfloat(
            'cutter_offset_z', None)
        self.cutter_retract_y = config.getfloat(
            'cutter_retract_y', None)
        
        self.cutting_current = config.getfloat(
            'cutting_current', None, minval=0.)
        self.current_helper = None
        if self.cutting_current is not None:
            self.current_helper = TMCCurrentHelper(config)

        # Optional custom g-code templates
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.docking_gcode = None
        if config.get('docking_gcode', None) is not None:
            self.docking_gcode = gcode_macro.load_template(config, 'docking_gcode', '')

        self.undocking_gcode = None
        if config.get('undocking_gcode', None) is not None:
            self.undocking_gcode = gcode_macro.load_template(config, 'undocking_gcode', '')

        self.cut_gcode = None
        if config.get('cut_gcode', None) is not None:
            self.cut_gcode = gcode_macro.load_template(config, 'cut_gcode', '')         

        # Initial state
        self.prev_currents = None

        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        # Register required objects
        self.gcode = self.printer.lookup_object('gcode')

        # Register g-code commands
        self.gcode.register_mux_command('CUT_FILAMENT', 'DOCK', self.name,
                                        self.cmd_CUT_FILAMENT,
                                        desc="Cut filament using dock cutter")

        self.gcode.register_mux_command('SET_AXIS_MODE', 'DOCK', self.name, 
                                        self.cmd_SET_AXIS_MODE,
                                        desc="Set docking axis mode")
    
    def handle_connect(self):
        self.extruder = self.printer.lookup_object(self.extruder_name)
        self.toolhead = self.printer.lookup_object('toolhead')
        if self.travel_speed is None:
            self.travel_speed = self.toolhead.get_max_velocity()[0]

        if self.filament_sensor is not None:
            for instance in self.printer.lookup_objects('filament_switch_sensor'):
                if instance[1].runout_helper.name == self.filament_sensor:
                    self.filament_sensor = instance[1]
                    break
            if self.filament_sensor is None:
                raise self.config.error("Missing required filament_sensor object")
            
        if self.docking_axis is True:
            self.docking_axis = self.printer.lookup_object('docking_axis')

        if self.toolhead_detect is True:
            self.toolhead_detect = None
            for instance in self.printer.lookup_objects('toolhead_detect'):
                if instance[1].get_extruder_name() == self.extruder_name:
                    self.toolhead_detect = instance[1]
                    break
            if self.toolhead_detect is None:
                raise self.config.error("Missing required toolhead_detect object")   
    
    def cmd_CUT_FILAMENT(self, gcmd):
        self.cut_filament()
    
    def cmd_SET_AXIS_MODE(self, gcmd):
        mode = gcmd.get('MODE')
        if mode not in axis_modes:
            raise self.printer.command_error(
                f"Invalid axis mode '{mode}', valid modes are: {', '.join(axis_modes.keys())}")
        self.axis_mode = mode
        gcmd.respond_info(f"Dock {self.name} axis mode set to {self.axis_mode}")
        
    def cut_filament(self):
        # Check if modules are homed and ready for motion
        self._enabled_check()

        # Save current positions for restore after operation
        self._save_init_pos()

        # Check if filament cutter is configured
        if not self.filament_cutter:
            raise self.printer.command_error(
                f"Filament cutter not configured for dock {self.name}")
        
        # Check if toolhead is mounted
        if self.toolhead_detect:
            if not self.toolhead_detect.query_state_blocking():
                raise self.printer.command_error(
                    f"Toolhead not mounted, cannot cut filament on dock {self.name}")
        
        # Check if toolhead has filament loaded
        if self.filament_sensor:
            if not self.filament_sensor.runout_helper.filament_present:
                raise self.printer.command_error(
                    f"No filament detected, cannot cut filament on dock {self.name}")
        
        # Determine z-axis and docking axis positions for cutting
        movepos_z = self._determine_movepos_z(self.cutter_offset_z)
        logging.info(f"Determined cut movepos_z: {movepos_z}")
        if movepos_z is [None, None]:
            raise self.printer.command_error(
                f"Cannot achieve cutter offset z={self.cutter_offset_z}mm with current positions")

        # Get current toolhead position
        pos = self.toolhead.get_position()

        # Move to safe xy position in front of dock
        pos[0] = self.docked_position_x
        pos[1] = self.safe_position_y
        self.toolhead.move(pos, self.travel_speed)
        self.toolhead.wait_moves()
        
        # Move z-axis and possibly docking axis to desired height
        if movepos_z[1] is not None:
            speed = min(self.travel_speed, self.docking_axis.stepper.velocity)
            self.docking_axis.stepper.do_move(
                movepos_z[1], speed, self.docking_axis.stepper.accel, sync=False if self.axis_mode == 'balanced' else True
            )
        if movepos_z[0] is not None:
            pos[2] = movepos_z[0]
            self.toolhead.move(pos, self.travel_speed)
        self.toolhead.wait_moves()
        
        # Check if custom cut g-code is defined
        if self.cut_gcode is not None:
            pass # Needs to be programmed still
        else:
            # Move to cutter xy position
            pos = self.toolhead.get_position()
            pos[0] = self.cutter_position_x
            pos[1] = self.cutter_position_y + self.cutter_retract_y
            self.toolhead.move(pos, self.travel_speed)      
            self.toolhead.wait_moves()

            # Raise tmc driver currents
            if self.cutting_current is not None:
                self.prev_currents = {}
                for name, driver in self.current_helper.tmc_drivers.items():
                    run_current, hold_current, _, max_current = self.current_helper.get_current(name)
                    self.prev_currents[name] = (run_current, hold_current)
                    if self.cutting_current > run_current:
                        cutting_current = min(self.cutting_current, max_current)
                        print_time = self.toolhead.get_last_move_time()
                        self.current_helper.set_current(cutting_current, hold_current, print_time, name)
                        logging.info(f"Set cutting current for driver {name} to {cutting_current}A at time {print_time}")

            # Move into cutter
            pos[1] = self.cutter_position_y
            self.toolhead.move(pos, self.cut_speed)

            # Restore tmc driver currents
            if self.prev_currents is not None:
                for name, _ in self.current_helper.tmc_drivers.items():
                    if name in self.prev_currents:
                        run_current, hold_current = self.prev_currents[name]
                        print_time = self.toolhead.get_last_move_time()
                        self.current_helper.set_current(run_current, hold_current, print_time, name)
                        logging.info(f"Restored current for driver {name} to {run_current}A at time {print_time}")
                self.prev_currents = None
            
            # Retract from cutter
            pos[1] = self.cutter_position_y + self.cutter_retract_y
            self.toolhead.move(pos, self.travel_speed)
            self.toolhead.wait_moves()

        # Restore previous positions
        self._restore_last_pos(restore_axes=True)
        
    def _save_init_pos(self):
        # Save current toolhead and docking_axis positions for restore after operation
        self.last_toolhead_pos = self.toolhead.get_position()
        if self.docking_axis:
            self.last_docking_axis_pos = self.docking_axis.get_position()
    
    def _restore_last_pos(self, restore_axes=False):
        # Ensure all moves are done before restoring and cleanup position from toolhead
        self.toolhead.wait_moves()
        self.toolhead.set_position(self.toolhead.get_position())

        # Restore requested axes to last position
        if restore_axes is False:
            return
        elif restore_axes is True:
            restore_axes = self.restore_axes
        
        if self.last_toolhead_pos is None:
                raise self.printer.command_error(
                    f"Cannot restore toolhead position, no saved position found")
    
        if self.docking_axis:
            if self.last_docking_axis_pos is None:
                raise self.printer.command_error(
                    f"Cannot restore docking axis position, no saved position found")

        for axis_group in restore_axes:
            axes = axis_group.split()
            if 'docking_axis' in axes and self.docking_axis:
                speed = min(self.travel_speed, self.docking_axis.stepper.velocity)
                self.docking_axis.stepper.do_move(
                    self.last_docking_axis_pos, speed, 
                    self.docking_axis.stepper.accel, sync=False
                )
            pos = self.toolhead.get_position()
            if 'x' in axes:
                pos[0] = self.last_toolhead_pos[0]
            if 'y' in axes:
                pos[1] = self.last_toolhead_pos[1]
            if 'z' in axes:
                pos[2] = self.last_toolhead_pos[2]
            self.toolhead.move(pos, self.travel_speed)
            self.toolhead.wait_moves()
                
        # Set init positions to None
        self.toolhead.set_position(self.toolhead.get_position()) # Cleanup
        self.last_toolhead_pos = None
        self.last_docking_axis_pos = None

    def _enabled_check(self):
        curtime = self.reactor.monotonic()
        if self.docking_axis:
            # Check if docking axis is homed (also means enabled)
            if not self.docking_axis.get_status(curtime)['homed']:
                raise self.printer.command_error(
                    f"Docking axis not homed, please home before performing motions")
        if 'xyz' not in self.toolhead.get_kinematics().get_status(curtime)['homed_axes']:
            raise self.printer.command_error(
                f"Toolhead not homed, please home before performing motions")
    
    def _determine_movepos_z(self, target_z_offset):
        # Check movepos z based on axis mode and target offset
        toolhead_max_z = self.toolhead.get_kinematics().axes_max[2]
        toolhead_min_z = self.toolhead.get_kinematics().axes_min[2]
        if self.docking_axis:
            docking_axis_max_z = self.docking_axis.stepper.pos_max
            docking_axis_min_z = self.docking_axis.stepper.pos_min
        
        if self.axis_mode == 'static':
            # Only toolhead moves, docking axis is static
            if self.docking_axis:
                toolhead_movepos_z = self.docking_axis.get_position() + target_z_offset
            else:
                toolhead_movepos_z = self.docked_position_z + target_z_offset
            if toolhead_movepos_z > toolhead_max_z or toolhead_movepos_z < toolhead_min_z:
                raise self.printer.command_error(
                    f"Required toolhead Z position {toolhead_movepos_z} out of bounds")
            logging.info(f"Determined cut movepos_z: {toolhead_movepos_z}")
            movepos = [toolhead_movepos_z, None]
            return movepos
        else:
            toolhead_pos_z = self.toolhead.get_position()[2]
            docking_axis_pos_z = self.docking_axis.get_position()
            logging.info(f"Current positions: toolhead Z {toolhead_pos_z}, docking axis Z {docking_axis_pos_z}")
            logging.info(f"Target offset z: {target_z_offset}")
            if self.axis_mode == 'balanced':
                # Split the difference equally
                delta_z = (docking_axis_pos_z - toolhead_pos_z) + target_z_offset
                toolhead_movepos_z = toolhead_pos_z + (delta_z / 2.)
                docking_axis_movepos_z = docking_axis_pos_z - (delta_z / 2.)                                    
            elif self.axis_mode == 'minimize_z':
                # Minimize toolhead movement, docking axis does most of the work
                docking_axis_movepos_z = toolhead_pos_z + target_z_offset
                toolhead_movepos_z = toolhead_pos_z
        
        logging.info(f"Initial calculated movepos_z: toolhead {toolhead_movepos_z}, docking axis {docking_axis_movepos_z}")
        
        # Correct for axes bounds if needed
        for _ in range(2):
            # Check 1: Toolhead may never move below current position
            if toolhead_movepos_z < self.last_toolhead_pos[2]:
                toolhead_movepos_z = self.last_toolhead_pos[2]
                docking_axis_movepos_z = toolhead_movepos_z - target_z_offset
            
            # Check 2: Toolhead may never move above max position
            elif toolhead_movepos_z > toolhead_max_z:
                toolhead_movepos_z = toolhead_max_z
                docking_axis_movepos_z = toolhead_movepos_z - target_z_offset

            # Check 3: Docking axis may never move below min position
            if docking_axis_movepos_z < docking_axis_min_z:
                docking_axis_movepos_z = docking_axis_min_z
                toolhead_movepos_z = docking_axis_movepos_z + target_z_offset
            
            # Check 4: Docking axis may never move above max position
            elif docking_axis_movepos_z > docking_axis_max_z:
                docking_axis_movepos_z = docking_axis_max_z
                toolhead_movepos_z = docking_axis_movepos_z + target_z_offset
        
        # Final validation - if still out of bounds, it's impossible
        if (toolhead_movepos_z > toolhead_max_z or 
            toolhead_movepos_z < toolhead_min_z or
            toolhead_movepos_z < toolhead_pos_z):
            raise self.printer.command_error(
                f"Cannot achieve {target_z_offset:.2f}mm offset: "
                f"toolhead Z would be {toolhead_movepos_z:.2f}mm "
                f"(limits: {toolhead_min_z:.2f} to {toolhead_max_z:.2f}mm, current: {toolhead_pos_z:.2f}mm)"
            )
        
        if (docking_axis_movepos_z > docking_axis_max_z or 
            docking_axis_movepos_z < docking_axis_min_z):
            raise self.printer.command_error(
                f"Cannot achieve {target_z_offset:.2f}mm offset: "
                f"docking axis would be {docking_axis_movepos_z:.2f}mm "
                f"(limits: {docking_axis_min_z:.2f} to {docking_axis_max_z:.2f}mm)"
            )
        
        logging.info(f"Final calculated movepos_z: toolhead {toolhead_movepos_z}, docking axis {docking_axis_movepos_z}")
        
        movepos = [toolhead_movepos_z, docking_axis_movepos_z]
        return movepos

def load_config_prefix(config):
    return Dock(config)