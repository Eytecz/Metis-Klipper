# Toolhead docking module - manages complete pickup/dropoff sequences
# Can optionally use docking_axis for dynamic dock positioning
#
# Copyright (C) 2025 Eytecz Engineering
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import traceback
from extras import tmc2130, tmc2240, tmc2660, tmc5160
import configparser
from configfile import ConfigWrapper # type: ignore
from .led_effect import ledEffect # type: ignore

STATUS_UNINITIALIZED    = 'uninitialized'            # Dock not yet initialized, or indeterminate state
STATUS_INITIALIZING     = 'initializing'             # Dock is initializing
STATUS_UNDOCKED         = 'undocked'                 # Toolhead is undocked, but not on carriage
STATUS_ENGAGED          = 'engaged'                  # Toolhead is docked on carriage
STATUS_DOCKED           = 'docked'                   # Toolhead is docked in the dock
STATUS_UNDOCKING        = 'undocking'                # Toolhead is in the process of undocking
STATUS_DOCKING          = 'docking'                  # Toolhead is in the process of docking
STATUS_CUT_FILAMENT     = 'cut_filament'             # Filament is being cut
STATUS_ERROR            = 'error'                    # Dock is in error state

# Docking axis behaviour options
axis_modes = {
    'static': 'static',         # No docking axis movement
    'balanced': 'balanced',     # Split motion, docking axis and z-axis meet halfway
    'minimize_z': 'minimize_z', # Minimize z-axis movement, docking axis does most of the work
}

class TMCCurrentHelper:
    def __init__(self, config):
        self.config = config
        self.printer= config.get_printer()
        self.tmc_helpers = {}
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
    
    def handle_connect(self):
        tmc_modules = {
            'tmc2130': tmc2130.TMCCurrentHelper,
            'tmc2208': tmc2130.TMCCurrentHelper,
            'tmc2209': tmc2130.TMCCurrentHelper,
            'tmc2240': tmc2240.TMC2240CurrentHelper,
            'tmc2660': tmc2660.TMC2660CurrentHelper,
            'tmc5160': tmc5160.TMC5160CurrentHelper,
        }

        for name, obj in self.printer.lookup_objects():
            if name.startswith('tmc'):
                stepper_name = name.split()[1] if len(name.split()) > 1 else None
                if stepper_name and (stepper_name.startswith('stepper_x') or
                                     stepper_name.startswith('stepper_y')):
                    tmc_type = name.split()[0]
                    if tmc_type in tmc_modules:
                        section = self.config.getsection(name)
                        current_helper = tmc_modules[tmc_type](section, obj.mcu_tmc)
                        self.tmc_helpers[stepper_name] = current_helper
        if 'stepper_x' not in self.tmc_helpers and 'stepper_y' not in self.tmc_helpers:
            raise self.config.error("No TMC stepper drivers found for X or Y axes")

    def get_current(self, stepper_name):
        if stepper_name in self.tmc_helpers:
            return self.tmc_helpers[stepper_name].get_current()
        else:
            raise self.config.error(f"No TMC current helper found for stepper '{stepper_name}'")
    
    def set_current(self, run_current, hold_current, print_time, stepper_name):
        if stepper_name in self.tmc_helpers:
            self.tmc_helpers[stepper_name].set_current(
                run_current, hold_current, print_time)
        else:
            raise self.config.error(f"No TMC current helper found for stepper '{stepper_name}'")

class LEDHelper:
    def __init__(self, config, dock_unit):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[1]
        self.dock_unit = dock_unit

        # Initial state
        self.state_effects = {}
        self.nozzle_effects = {}

        # State layer default presets for state LED's
        LAYER_UNINITIALIZED = """static        0  0     top        (0.0, 0.0, 0.0, 0.0)"""
        LAYER_INITIALIZING  = """breathing     2  1     top        (0.0, 0.0, 0.0, 1.0)"""
        LAYER_UNDOCKED    = """static        0  0     top        (1.0, 0.0, 0.0, 0.0)"""
        LAYER_ENGAGED       = """static        0  0     top        (0.0, 0.0, 0.0, 1.0)"""
        LAYER_DOCKED        = """static        0  0     top        (0.0, 0.0, 1.0, 0.0)"""
        LAYER_UNDOCKING     = """breathing     2  1     top        (0.0, 1.0, 0.0, 0.0)"""
        LAYER_DOCKING       = """breathing     2  1     top        (1.0, 0.0, 0.0, 0.0)"""
        LAYER_CUT_FILAMENT  = """breathing     2  1     top        (1.0, 0.5, 0.0, 0.0)"""
        LAYER_ERROR         = """strobe        1  1.5   add        (1.0, 1.0, 1.0, 1.0)
                                 breathing     2  0     difference (1.0, 0.0, 0.0, 0.0)
                                 static        1  0     top        (1.0, 0.0, 0.0, 0.0)"""
        
        # State layer default presets for nozzle LED's
        LAYER_NOZZLE_ENGAGED  = """static        0  0     top     (0.0, 0.0, 0.0, 1.0)"""
        LAYER_NOZZLE_DOCKED   = """temperature   40 180   top     (0.0, 0.0, 1.0, 0.0),(1.0, 1.0, 0.0, 0.0),(1.0, 0.0, 0.0, 0.0)""" 

        # Read config section
        self.state_layers = {
            STATUS_UNINITIALIZED: config.get('state_layer_uninitialized', LAYER_UNINITIALIZED),
            STATUS_INITIALIZING:  config.get('state_layer_initializing', LAYER_INITIALIZING),
            STATUS_UNDOCKED:      config.get('state_layer_undocked', LAYER_UNDOCKED),
            STATUS_ENGAGED:       config.get('state_layer_engaged', LAYER_ENGAGED),
            STATUS_DOCKED:        config.get('state_layer_docked', LAYER_DOCKED),
            STATUS_UNDOCKING:     config.get('state_layer_undocking', LAYER_UNDOCKING),
            STATUS_DOCKING:       config.get('state_layer_docking', LAYER_DOCKING),
            STATUS_CUT_FILAMENT:  config.get('state_layer_cut_filament', LAYER_CUT_FILAMENT),
            STATUS_ERROR:         config.get('state_layer_error', LAYER_ERROR),
        }

        self.nozzle_layers = {
            STATUS_ENGAGED:   config.get('nozzle_layer_engaged', LAYER_NOZZLE_ENGAGED),
            STATUS_DOCKED:    config.get('nozzle_layer_docked', LAYER_NOZZLE_DOCKED)
        }

        self.frame_rate = config.getfloat('frame_rate', default=24, minval=1, maxval=60)
        self.status_leds = config.get('status_leds')
        self.nozzle_leds = config.get('nozzle_leds', None)

        # Lookup required objects
        self.configfile = self.printer.lookup_object('configfile')

        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def handle_connect(self):
        self._create_led_configs()
    
    def handle_ready(self):
        self.dock_unit.led_ready = True

    def _create_led_configs(self):
        for state, state_layer in self.state_layers.items():
            effect_name = f'{self.name}_{state}'
            effect_config = {
                'auto_start':   'False',
                'frame_rate':   str(self.frame_rate),
                'layers':       state_layer,
                'leds':         self.status_leds
            }
            config, section = self._create_led_config(effect_name, effect_config)

            try:
                self.printer.add_object(section, self.printer.load_object(config, 'led_effect'))
                self.state_effects[state] = ledEffect(config)
            except Exception as e:
                logging.error(f"Error creating LED effect '{effect_name}': {e}")
        
        if self.nozzle_leds is not None:
            for state, nozzle_layer in self.nozzle_layers.items():
                effect_name = f'{self.name}_nozzle_{state}'
                effect_config = {
                    'auto_start':   'False',
                    'frame_rate':   str(self.frame_rate),
                    'layers':       nozzle_layer,
                    'leds':         self.nozzle_leds
                }

                # Inject heater when using temperature layer
                if nozzle_layer and 'temperature' in nozzle_layer:
                    effect_config['heater'] = self.dock_unit.get_extruder_name()

                config, section = self._create_led_config(effect_name, effect_config)

                try:
                    self.printer.add_object(section, self.printer.load_object(config, 'led_effect'))
                    self.nozzle_effects[state] = ledEffect(config)
                except Exception as e:
                    logging.error(f"Error creating nozzle LED effect '{effect_name}': {e}")
                   
    def _create_led_config(self, effect_name, effect_config):
        # Create a new configparser with the led effect configuration
        fileconfig = configparser.RawConfigParser()
        section_name = f"led_effect {effect_name}"
        fileconfig.add_section(section_name)
        
        # Add configuration options
        for key, value in effect_config.items():
            fileconfig.set(section_name, key, str(value))
        
        # Create ConfigWrapper
        config_wrapper = ConfigWrapper(self.printer, fileconfig, self.configfile.validate.access_tracking, section_name)
        return config_wrapper, section_name

    def set_state(self, state):
        if state not in self.state_effects:
            logging.warning(f"LED effect for state '{state}' not found")
            return
        led_effect = self.state_effects[state]
        for led in led_effect.leds:
            for effect in led_effect.handler.effects:
                if effect is not led_effect and led in effect.leds:
                    effect.set_enabled(False)
        led_effect.set_enabled(True)
        if self.nozzle_leds is not None:
            if state in self.nozzle_effects:
                nozzle_effect = self.nozzle_effects[state]
                for led in nozzle_effect.leds:
                    for effect in nozzle_effect.handler.effects:
                        if effect is not nozzle_effect and led in effect.leds:
                            effect.set_enabled(False)
                nozzle_effect.set_enabled(True)

class DockUnit:
    def __init__(self, config):
        self.config = config
        self.printer= config.get_printer()
        self.reactor = self.printer.get_reactor()

        # Read config section
        self.name = config.get_name().split()[1]
        self.extruder_name = config.get('extruder', 'extruder')
        self.filament_sensor = config.get('filament_sensor', None)
        self.toolhead_detect_shuttle = config.get('toolhead_detect_shuttle', None)
        self.toolhead_detect_dock = config.get('toolhead_detect_dock', None)

        self.retract_length = config.getfloat('retract_length', 30., minval=0.)
        self.unretract_length = config.getfloat('unretract_length', 30., minval=0.)
        self.retract_speed = config.getfloat('retract_speed', 100., above=0.)
        self.unretract_speed = config.getfloat('unretract_speed', 20., above=0.)
        
        status_leds = config.get('status_leds', None)
        self.led_helper = None
        if status_leds:
            self.led_helper = LEDHelper(config, self)

        self.docking_axis = config.getboolean('docking_axis', False)
        if self.docking_axis:
            self.axis_mode = config.getchoice(
                'axis_mode', axis_modes, 'balanced')
        else:
            self.axis_mode = axis_modes['static']

        self.restore_axes = config.getlist(
            'restore_axes', ['y', 'x', 'z', 'docking_axis'])

        self.docking_speed = config.getfloat('docking_speed', 50., above=0.)
        self.engage_speed = config.getfloat('engage_speed', 20., above=0.)
        self.disengage_speed = config.getfloat('disengage_speed', 20., above=0.)
        self.cut_speed = config.getfloat('cut_speed', 10., above=0.)
        self.travel_speed = config.getfloat('travel_speed', 400., above=0.)

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
            'safe_offset_z', 6., minval=0.)         # Offset above dock to enter safely
        self.slide_distance_y = config.getfloat(
            'slide_distance_y', 3., minval=0.)       # Distance to slide horizontally into dock
        self.disengage_offset_z = config.getfloat(
            'disengage_offset_z', 8., minval=0.)     # Offset to drop towards dock when disengaging
        self.disengage_offset_y = config.getfloat(
            'disengage_offset_y', 20., minval=0.)    # Offset to back away from dock when disengaging
        
        # Cutting sequence parameters
        self.filament_cutter = config.getboolean('filament_cutter', False)
        self.toolhead_sensor_cutter_distance = config.getfloat('toolhead_sensor_cutter_distance', None)
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

        self.exception_pause = config.getboolean('pause_on_exception', True)
        if self.exception_pause or config.get('exception_gcode', None) is not None:
            self.exception_gcode = gcode_macro.load_template(config, 'exception_gcode', '')       

        # Initial state
        self.status = STATUS_UNINITIALIZED
        self.prev_currents = None
        self.led_ready = False
        self.previous_extruder = None
        self.enable_filament_cutter = True
        self.last_toolhead_pos = None
        self.last_docking_axis_pos = None

        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        # Register required objects
        self.gcode = self.printer.lookup_object('gcode')
                    

        # Register g-code commands
        self.gcode.register_mux_command('INITIALIZE_DOCK', 'DOCK', self.name,
                                        self.cmd_INITIALIZE_DOCK,
                                        desc="Initialize the dock unit")
        self.gcode.register_mux_command('SET_DOCK_STATUS', 'DOCK', self.name,
                                        self.cmd_SET_DOCK_STATUS,
                                        desc="Set the dock unit status")
        self.gcode.register_mux_command('CUT_FILAMENT', 'DOCK', self.name,
                                        self.cmd_CUT_FILAMENT,
                                        desc="Cut filament using dock cutter")
        self.gcode.register_mux_command('DROPOFF_EXTRUDER', 'EXTRUDER', self.extruder_name,
                                        self.cmd_DROPOFF_EXTRUDER,
                                        desc="Drop the specified extruder")
        self.gcode.register_mux_command('PICKUP_EXTRUDER', 'EXTRUDER', self.extruder_name,
                                        self.cmd_PICKUP_EXTRUDER,
                                        desc="Pickup the specified extruder")
        self.gcode.register_mux_command('SET_AXIS_MODE', 'DOCK', self.name, 
                                        self.cmd_SET_AXIS_MODE,
                                        desc="Set docking axis mode")
        self.gcode.register_mux_command('SET_CUTTER', 'DOCK', self.name,
                                        self.cmd_SET_CUTTER,
                                        desc="Enable/disable the filament cutter")
        self.gcode.register_mux_command('CALIBRATE_CUTTER_POSITION', 'DOCK', self.name,
                                        self.cmd_CALIBRATE_CUTTER_POSITION,
                                        desc="Calibrate the cutter position based on toolhead sensor")
        self.gcode.register_mux_command('SET_RETRACT_LENGTH', 'DOCK', self.name,
                                        self.cmd_SET_RETRACT_LENGTH,
                                        desc="Set the filament retract and unretract lengths")
        self.gcode.register_mux_command('SET_UNRETRACT_LENGTH', 'DOCK', self.name,
                                        self.cmd_SET_UNRETRACT_LENGTH,
                                        desc="Set the filament unretract length")
            
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

        if self.toolhead_detect_shuttle is not None:
            found = False
            for name, instance in self.printer.lookup_objects('toolhead_detect'):
                if instance.get_name() == self.toolhead_detect_shuttle:
                    logging.info(f"Found toolhead_detect shuttle: {instance.name}")
                    self.toolhead_detect_shuttle = instance
                    found = True
                    break
            if not found:
                raise self.config.error("Missing required toolhead_detect shuttle object")
        
        if self.toolhead_detect_dock is not None:
            found = False
            for name, instance in self.printer.lookup_objects('toolhead_detect'):
                if instance.get_name() == self.toolhead_detect_dock:
                    self.toolhead_detect_dock = instance
                    found = True
                    break
            if not found:
                raise self.config.error("Missing required toolhead_detect dock object")
        
        # Register state change callbacks for toolhead_detect sensors
        if self.toolhead_detect_shuttle is not None:
            self.toolhead_detect_shuttle.register_callback(
                self.toolhead_detect_callback)
        if self.toolhead_detect_dock is not None:
            self.toolhead_detect_dock.register_callback(
                self.toolhead_detect_callback)
    
    def handle_ready(self):
        self.reactor.register_timer(self.initialize_dock_unit, self.reactor.monotonic() + 2.0)

    def handle_exception(self, exception, context, pause_on_error=True):
        # Log the error with context
        logging.error(f"Error in {context} for toolhead {self.name}: {exception}")
        logging.error(traceback.format_exc())

        # Set status to ERROR
        self.set_status(STATUS_ERROR)

        # Ensure all moves are done and update position
        self.toolhead.wait_moves()
        self.toolhead.set_position(self.toolhead.get_position())

        # Check printer status and pause if required
        if pause_on_error:
            idle_timeout = self.printer.lookup_object('idle_timeout')
            is_printing = idle_timeout.get_status(self.reactor.monotonic())['state'] == "Printing"
            if is_printing:
                pause_prefix = ""
                if self.exception_pause:
                    pause_resume = self.printer.lookup_object('pause_resume')
                    pause_resume.send_pause_command()
                    pause_prefix = "PAUSE\n"
                    self.reactor.pause(self.reactor.monotonic() + 0.5)
                self._exec_gcode(pause_prefix, self.exception_gcode)
                msg = (
                    f"{str(exception)} Printing has been paused. Please resolve the issue before resuming.")
                if context == "initializing":
                    return self.reactor.NEVER
                else:
                    raise Exception(msg)
        
        msg = (
            f"{str(exception)} Printing has not been paused. Please resolve the issue before continuing."
        )

        if context == "initializing":
            return self.reactor.NEVER
        else:
            raise Exception(msg)

    def _exec_gcode(self, prefix, template):
        try:
            self.gcode.run_script_from_command(prefix + template.render() + "\nM400")
        except Exception as e:
           self.handle_exception(e, "_exec_gcode", pause_on_error=True)

    def set_status(self, status):
        if self.status == status:
            return
        self.status = status
        if self.led_helper:
            if self.led_ready:
                self.led_helper.set_state(self.status)

    def initialize_dock_unit(self, eventtime=None):
        self.set_status(STATUS_INITIALIZING)
        # If toolhead detection is not configured, set to uninitialized
        if not self.toolhead_detect_shuttle:
            self.set_status(STATUS_UNINITIALIZED)
            logging.info(f"Dock {self.name} initialized without toolhead detection, status set to uninitialized.")
            return self.reactor.NEVER
        
        # Query toolhead detect state and dock sensor if configured
        if self.toolhead_detect_shuttle.query_state_blocking():
            if self.toolhead_detect_dock:
                if self.toolhead_detect_dock.query_state_blocking():
                    self.set_status(STATUS_ERROR)
                    logging.error(f"Dock {self.name} initialization error: Toolhead detected as mounted but dock sensor indicates docked")
                    return self.reactor.NEVER
            self.set_status(STATUS_ENGAGED)
            logging.info(f"Dock {self.name} initialized, toolhead detected as mounted, status set to engaged")
        else:
            if self.toolhead_detect_dock:
                if self.toolhead_detect_dock.query_state_blocking():
                    self.set_status(STATUS_DOCKED)
                    logging.info(f"Dock {self.name} initialized, dock sensor indicates docked, status set to docked")
                    return self.reactor.NEVER
                else:
                    self.set_status(STATUS_UNDOCKED)
                    logging.info(f"Dock {self.name} initialized, toolhead not mounted and dock sensor indicates undocked, status set to undocked")
            else:
                self.set_status(STATUS_DOCKED)
                logging.info(f"Dock {self.name} initialized, toolhead not mounted (assuming docked), status set to docked")             
        return self.reactor.NEVER

    def toolhead_detect_callback(self, state):
        if self.status not in [STATUS_CUT_FILAMENT, STATUS_DOCKING, STATUS_UNDOCKING]:
            try:
                # Figure out what the new toolhead status is
                if self.toolhead_detect_shuttle.query_state_blocking():
                    if self.toolhead_detect_dock:
                        if self.toolhead_detect_dock.query_state_blocking():
                            self.set_status(STATUS_ERROR)
                    self.set_status(STATUS_ENGAGED)
                else:
                    if self.toolhead_detect_dock:
                        if self.toolhead_detect_dock.query_state_blocking():
                            self.set_status(STATUS_DOCKED)
                        else:
                            self.set_status(STATUS_UNDOCKED)
                    else:
                        self.set_status(STATUS_DOCKED)
            except Exception as e:
                self.handle_exception(e, "toolhead_detect_callback", pause_on_error=False)

    def cmd_INITIALIZE_DOCK(self, gcmd):
        try:
            self.initialize_dock_unit()
            gcmd.respond_info(f"Dock {self.name} initialized, status: {self.status}")
        except Exception as e:
            raise gcmd.error(f"Error initializing dock: {e}")

    def cmd_SET_DOCK_STATUS(self, gcmd):
        status = gcmd.get('STATUS')
        valid_statuses = [
            STATUS_UNINITIALIZED,
            STATUS_INITIALIZING,
            STATUS_UNDOCKED,
            STATUS_ENGAGED,
            STATUS_DOCKED,
            STATUS_UNDOCKING,
            STATUS_DOCKING,
            STATUS_CUT_FILAMENT,
            STATUS_ERROR
        ]
        if status not in valid_statuses:
            raise gcmd.error(
                f"Invalid dock status '{status}', valid statuses are: {', '.join(valid_statuses)}")
        self.set_status(status)
        gcmd.respond_info(f"Dock {self.name} status set to {self.status}")

    def cmd_SET_CUTTER(self, gcmd):
        enable = gcmd.get_int('ENABLE', 1)
        self.enable_filament_cutter = bool(enable)
        status = "enabled" if self.enable_filament_cutter else "disabled"
        gcmd.respond_info(f"Dock {self.name} filament cutter {status}")   

    def cmd_CUT_FILAMENT(self, gcmd):
        restore_pos = gcmd.get_int('RESTORE_POS', 1)
        try:
            self.save_init_pos()
            self.cut_filament(bool(restore_pos))
        except Exception as e:
            raise gcmd.error(f"Error cutting filament: {e}")
       
    def cmd_DROPOFF_EXTRUDER(self, gcmd):
        try:
            self.save_init_pos()
            cut_filament = gcmd.get_int('CUT_FILAMENT', 1)
            self.retract_filament()
            if cut_filament:
                self.cut_filament(restore_pos=False)
            self.dock_toolhead(restore_pos=False)
        except Exception as e:
            raise gcmd.error(f"Error dropping off extruder: {e}")
    
    def cmd_PICKUP_EXTRUDER(self, gcmd):
        try:
            self.undock_toolhead(restore_pos=False)
            if self.last_toolhead_pos is not None:
                self.restore_last_pos(restore_axes=True)
            self.unretract_filament()
        except Exception as e:
            raise gcmd.error(f"Error picking up extruder: {e}")

    def cmd_SET_AXIS_MODE(self, gcmd):
        mode = gcmd.get('MODE')
        if mode not in axis_modes:
            raise gcmd.error(
                f"Invalid axis mode '{mode}', valid modes are: {', '.join(axis_modes.keys())}.")
        self.axis_mode = mode
        gcmd.respond_info(f"Dock {self.name} axis mode set to {self.axis_mode}.")
    
    def cmd_CALIBRATE_CUTTER_POSITION(self, gcmd):
        try:
            self.save_init_pos()
            self.calibrate_cutter_position()
        except Exception as e:
            raise gcmd.error(f"Error calibrating cutter position: {e}")
    
    def cmd_SET_RETRACT_LENGTH(self, gcmd):
        length = gcmd.get_float('LENGTH', 0.)
        if length < 0.:
            raise gcmd.error("Retract length must be non-negative")
        self.retract_length = length

        # Save config values
        configfile = self.printer.lookup_object('configfile')
        configfile.set(f'dock_unit {self.name}', 'retract_length', f'{self.retract_length:.3f}')

        def format_macro(macro: str) -> str:
                return f'<a class="command">{macro}</a>'
        
        self.gcode.respond_info(
            f"Retract length updated for extruder {self.extruder_name} successfully to {self.retract_length} mm.\n"
            f"Please use {format_macro(f'SAVE_CONFIG')} to save the calibration values."
            )
    
    def cmd_SET_UNRETRACT_LENGTH(self, gcmd):
        length = gcmd.get_float('LENGTH', 0.)
        if length < 0.:
            raise gcmd.error("Unretract length must be non-negative")
        self.unretract_length = length

        # Save config values
        configfile = self.printer.lookup_object('configfile')
        configfile.set(f'dock_unit {self.name}', 'unretract_length', f'{self.unretract_length:.3f}')

        def format_macro(macro: str) -> str:
                return f'<a class="command">{macro}</a>'
        
        self.gcode.respond_info(
            f"Unretract length updated for extruder {self.extruder_name} successfully to {self.unretract_length} mm.\n"
            f"Please use {format_macro(f'SAVE_CONFIG')} to save the calibration values."
            )

    def check_set_extruder_temp(self, wait=False):
        logging.info(f"Checking extruder temperature for dock {self.name} before cutting filament.")
        pheaters = self.printer.lookup_object('heaters')
        min_temp = self.extruder.heater.min_extrude_temp
        target_temp = self.extruder.heater.target_temp
        
        if not self.extruder.heater.can_extrude:
            logging.info(f"Extruder heater cannot extrude")
            # Extruder is too cold, heat it up
            if target_temp < min_temp:
                logging.info(f"Extruder target temp {target_temp}C is below min extrude temp {min_temp}C, setting to {min_temp + 10.}C")
                # No valid target set, use min_extrude_temp + margin
                pheaters.set_temperature(self.extruder.get_heater(), min_temp + 10., wait)
            else:
                logging.info(f"Extruder target temp {target_temp}C is above min extrude temp {min_temp}C")
                # Target already set appropriately, wait if requested
                if wait:
                    logging.info(f"Waiting for extruder to reach target temp {target_temp}C")
                    pheaters.set_temperature(self.extruder.get_heater(), target_temp, wait)
        else:
            logging.info(f"Extruder heater can extrude")
            # Extruder is hot enough now, but check if target would cool it below threshold
            if target_temp < min_temp:
                logging.info(f"Extruder target temp {target_temp}C is below min extrude temp {min_temp}C,")
                # Target is too low, would cool down - prevent this
                pheaters.set_temperature(self.extruder.get_heater(), min_temp + 10., wait=False)

    def activate_extruder(self):
        if self.toolhead.get_extruder() == self.extruder:
            return
        self.previous_extruder = self.toolhead.get_extruder()
        self.toolhead.flush_step_generation()
        self.toolhead.set_extruder(self.extruder, self.extruder.last_position)
        self.printer.send_event("extruder:activate_extruder")
        #self.toolhead.wait_moves()

    def restore_extruder(self):
        if self.previous_extruder is None or self.previous_extruder == self.extruder:
            return
        self.toolhead.flush_step_generation()
        self.toolhead.set_extruder(self.previous_extruder, self.previous_extruder.last_position)
        self.printer.send_event("extruder:activate_extruder")
        #self.toolhead.wait_moves()

    def calibrate_cutter_position(self):
        if self.status not in [STATUS_ENGAGED]:
            raise Exception("Cutter position calibration can only be performed when dock unit is ENGAGED")
        try:
            # Check if cutter is available
            if not self.filament_cutter:
                raise Exception("Cutter position calibration requires a filament cutter to configured in the dock unit")

            # Check if this dock/toolhead has a loaded spool unit
            spool_unit = None
            for _, unit in self.printer.lookup_objects('spool_unit'):
                if unit.get_extruder_name() == self.extruder_name:  # Also add () - it's a method
                    if unit.get_status(self.reactor.monotonic())['status'] == 'loaded':
                        spool_unit = unit
                        break
            if spool_unit is None:
                raise Exception("Cutter position calibration requires an active spool unit loaded on the dock's extruder")
            
            # Update spool unit state
            spool_unit.set_status('calibrating')

            # Ensure axes are homed
            self.enabled_check()

            # Prepare extruder
            spool_unit.sync_to_extruder(True)
            spool_unit.activate_extruder()
            spool_unit.check_set_extruder_temp(wait=True)

            # Purge filament to ensure it's past the cutter
            pos = self.toolhead.get_position()
            pos[3] += 50.
            extrude_speed = 5.0
            self.toolhead.move(pos, extrude_speed)
            #self.toolhead.wait_moves()
            
            # Perform cut
            self.cut_filament(restore_pos=True)

            # Iteratively retract filament until toolhead sensor is triggered
            speed = spool_unit.stepper_helper.homing_speed
            step_size = 0.2
            dist_max = 50.
            dist = 0.
            pos[3] -= step_size
            while bool(spool_unit.toolhead_sensor.runout_helper.filament_present):
                self.toolhead.move(pos, speed)
                #self.toolhead.wait_moves()
                dist += step_size
                if dist >= dist_max:
                    raise Exception("Toolhead sensor not triggered within expected range during cutter position calibration.")
                pos[3] -= step_size
            self.toolhead_sensor_cutter_distance = dist
            pos[3] += self.toolhead_sensor_cutter_distance # Move back to starting position
            self.toolhead.move(pos, speed)

            # Restore extruder state
            spool_unit.restore_extruder()

            # Restore spool unit state
            spool_unit.set_status('loaded')

            # Save config values
            configfile = self.printer.lookup_object('configfile')
            configfile.set(f'dock_unit {self.name}', 'toolhead_sensor_cutter_distance', f'{self.toolhead_sensor_cutter_distance:.3f}')

            def format_macro(macro: str) -> str:
                    return f'<a class="command">{macro}</a>'
            
            self.gcode.respond_info(
                f"Cutter position calibration for extruder {self.extruder_name} completed successfully.\n"
                f"Please use {format_macro(f'SAVE_CONFIG')} to save the calibration values."
            )
        except Exception as e:
            raise Exception(f"Cutter position calibration failed: {e}")

    def cut_filament(self, restore_pos=True, restore_axes=None):
        if not self.enable_filament_cutter:
            logging.info(f"Filament cutter disabled for dock {self.name}, skipping cut.")
            return
        try:
            # Check dock status and set to cutting state
            if self.status not in STATUS_ENGAGED:
                raise Exception(
                    f"Cannot cut filament, dock {self.name} status is '{self.status}', must be '{STATUS_ENGAGED}'.")
            self.set_status(STATUS_CUT_FILAMENT)

            # Check if modules are homed and ready for motion
            self.enabled_check()

            # Check if filament cutter is configured
            if not self.filament_cutter:
                raise Exception(
                    f"Filament cutter not configured for dock {self.name}.")
            
            # Check if toolhead is mounted
            if self.toolhead_detect_shuttle and self.toolhead_detect_shuttle.get_enabled():
                if not self.toolhead_detect_shuttle.query_state_blocking():
                    raise Exception(
                        f"Toolhead not mounted, cannot cut filament on dock {self.name}.")
            
            # Check if toolhead has filament loaded
            if self.filament_sensor:
                if not self.filament_sensor.runout_helper.filament_present:
                    raise Exception(
                        f"No filament detected, cannot cut filament on dock {self.name}.")
            
            self.check_set_extruder_temp(wait=False)

            # Check if custom cut g-code is defined else perform standard cut sequence
            if self.cut_gcode is not None:
                try:
                    self.gcode.run_script_from_command(self.cut_gcode.render() + "\nM400")
                except Exception as e:
                    raise Exception(f"Error executing cut gcode: {e}")
            else:
                # Determine z-axis and docking axis positions for cutting
                movepos_z = self._determine_movepos_z(self.cutter_offset_z)
                if movepos_z is [None, None]:
                    raise Exception(
                        f"Cannot achieve cutter offset z={self.cutter_offset_z}mm with current positions.")

                # Get current toolhead position
                pos = self.toolhead.get_position()

                # Move to safe xy position in front of dock
                pos[0] = self.docked_position_x
                pos[1] = self.safe_position_y
                self.toolhead.move(pos, self.travel_speed)
                
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
                
                # Move to cutter xy position
                pos = self.toolhead.get_position()
                pos[0] = self.cutter_position_x
                pos[1] = self.cutter_position_y + self.cutter_retract_y
                self.toolhead.move(pos, self.travel_speed)      

                # Raise tmc driver currents
                if self.cutting_current is not None:
                    self.prev_currents = {}
                    for name, _ in self.current_helper.tmc_helpers.items():
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
                    for name, _ in self.current_helper.tmc_helpers.items():
                        if name in self.prev_currents:
                            run_current, hold_current = self.prev_currents[name]
                            print_time = self.toolhead.get_last_move_time()
                            self.current_helper.set_current(run_current, hold_current, print_time, name)
                            logging.info(f"Restored current for driver {name} to {run_current}A at time {print_time}")
                    self.prev_currents = None
                            
                # Retract from cutter
                pos[1] = self.cutter_position_y + self.cutter_retract_y
                self.toolhead.move(pos, self.travel_speed)

                # Ensure extruder is still heated for retraction
                self.check_set_extruder_temp(wait=True)

                # Release tension on blade so it can retract (temporarily bypass extrude check)
                try:
                    self.activate_extruder()
                    pos = self.toolhead.get_position()
                    pos[3] -= 2.0
                    self.toolhead.move(pos, 10.0)
                    pos[3] += 2.0
                    self.toolhead.move(pos, 10.0)
                    #self.toolhead.wait_moves()
                    self.restore_extruder()
                except Exception as e:
                    raise Exception(f"Error retracting filament after cutting: {e}")

            # Restore previous positions
            if restore_pos:
                if restore_axes is None:
                    restore_axes = self.restore_axes
                self.restore_last_pos(restore_axes=restore_axes)
            else:
                # Cleanup current position from toolhead
                #self.toolhead.wait_moves()
                self.toolhead.set_position(self.toolhead.get_position())
            
            # Verify that toolhead is still engaged
            if self.toolhead_detect_shuttle and self.toolhead_detect_shuttle.get_enabled():
                self.toolhead.wait_moves()
                if not self.toolhead_detect_shuttle.query_state_blocking():
                    raise Exception(
                        f"Toolhead no longer detected as mounted after cutting filament on dock {self.name}.")
            self.set_status(STATUS_ENGAGED)
        except Exception as e:
            self.handle_exception(e, "cutting", pause_on_error=self.exception_pause)

    def dock_toolhead(self, restore_pos=True, restore_axes=None):
        try:
            # Check dock status and set to docking state
            if self.status not in STATUS_ENGAGED:
                raise Exception(
                    f"Cannot dock toolhead, dock {self.name} status is '{self.status}', must be '{STATUS_ENGAGED}'.")
            self.set_status(STATUS_DOCKING)
            # Check if modules are homed and ready for motion
            self.enabled_check()
            
            # Check if toolhead is mounted
            if self.toolhead_detect_shuttle and self.toolhead_detect_shuttle.get_enabled():
                if not self.toolhead_detect_shuttle.query_state_blocking():
                    raise Exception(
                        f"Toolhead not mounted, cannot dock on dock {self.name}.")
            
            # Check if custom docking g-code is defined else perform standard docking sequence
            if self.docking_gcode is not None:
                try:
                    self.gcode.run_script_from_command(self.docking_gcode.render() + "\nM400")
                except Exception as e:
                    raise Exception(f"Error executing docking gcode: {e}")
            else:
                if self.last_toolhead_pos is not None:
                    # Move 5mm above last toolhead position to ensure safe docking
                    pos = self.last_toolhead_pos.copy()
                    pos[2] += 5.0
                    self.toolhead.move(pos, self.travel_speed)
                    #self.toolhead.wait_moves()
                # Determine z-axis and docking axis positions for docking
                if self.axis_mode == 'static' and self.docking_axis is False:
                    target_z_offset = self.docked_position_z + self.safe_offset_z
                else:
                    target_z_offset = self.docked_offset_z + self.safe_offset_z
                movepos_z = self._determine_movepos_z(target_z_offset)
                if movepos_z is [None, None]:
                    raise Exception(
                        f"Cannot achieve docked offset z={target_z_offset}mm with current positions.")
                
                # Get current toolhead position
                pos = self.toolhead.get_position()

                # Move to safe xy position in front of dock
                pos[0] = self.docked_position_x
                if self.cutter_position_y is not None and self.cutter_retract_y is not None:
                    pos[1] = self.cutter_position_y + self.cutter_retract_y
                else:
                    pos[1] = self.safe_position_y
                self.toolhead.move(pos, self.travel_speed)

                # Move z-axis and possibly docking axis to desired height (slightly above dock)
                if movepos_z[1] is not None:
                    speed = min(self.travel_speed, self.docking_axis.stepper.velocity)
                    self.docking_axis.stepper.do_move(
                        movepos_z[1], speed, self.docking_axis.stepper.accel, sync=False if self.axis_mode == 'balanced' else True
                    )
                if movepos_z[0] is not None:
                    pos[2] = movepos_z[0]
                    self.toolhead.move(pos, self.travel_speed)
                #self.toolhead.wait_moves()

                # Move into dock position (before slide step)
                pos = self.toolhead.get_position()
                pos[0] = self.docked_position_x
                pos[1] = self.docked_position_y + self.slide_distance_y
                self.toolhead.move(pos, self.docking_speed)

                # Lower to docked position
                pos = self.toolhead.get_position()
                pos[2] -= self.safe_offset_z
                self.toolhead.move(pos, self.docking_speed)

                # Slide into dock
                pos[1] = self.docked_position_y
                self.toolhead.move(pos, self.docking_speed)

                # Disengage toolhead
                pos[2] -= self.disengage_offset_z
                self.toolhead.move(pos, self.disengage_speed)

                # Back away from coupling
                pos[1] += self.disengage_offset_y
                self.toolhead.move(pos, self.disengage_speed)

                # Verify that toolhead is no longer mounted
                if self.toolhead_detect_shuttle and self.toolhead_detect_shuttle.get_enabled():
                    self.toolhead.wait_moves()
                    if self.toolhead_detect_shuttle.query_state_blocking():
                        raise Exception(
                            f"Toolhead still detected as mounted after docking on dock {self.name}.")

                # Verify dock sensor if configured
                if self.toolhead_detect_dock and self.toolhead_detect_dock.get_enabled():
                    if not self.toolhead_detect_dock.query_state_blocking():
                        raise Exception(
                            f"Dock sensor did not indicate docked state after docking on dock {self.name}.")

            # Restore previous positions
            if restore_pos:
                pos[2] = self.safe_position_y
                self.toolhead.move(pos, self.travel_speed)
                if restore_axes is None:
                    restore_axes = self.restore_axes
                self.restore_last_pos(restore_axes=restore_axes)
            else:
                # Cleanup current position from toolhead
                #self.toolhead.wait_moves()
                self.toolhead.set_position(self.toolhead.get_position())
            self.set_status(STATUS_DOCKED)
        except Exception as e:
            self.handle_exception(e, "docking", pause_on_error=self.exception_pause)
    
    def undock_toolhead(self, restore_pos=True, restore_axes=None):
        try:
            # Check dock status and set to undocking state
            if self.status not in STATUS_DOCKED:
                raise Exception(
                    f"Cannot undock toolhead, dock {self.name} status is '{self.status}', must be '{STATUS_DOCKED}'.")
            self.set_status(STATUS_UNDOCKING)

            # Check if modules are homed and ready for motion
            self.enabled_check()

            # Check if toolhead is unmounted
            if self.toolhead_detect_shuttle and self.toolhead_detect_shuttle.get_enabled():
                if self.toolhead_detect_shuttle.query_state_blocking():
                    raise Exception(
                        f"Toolhead still detected as mounted, cannot undock on dock {self.name}.")
            
            # Check if custom undocking g-code is defined else perform standard undocking sequence
            if self.undocking_gcode is not None:
                try:
                    self.gcode.run_script_from_command(self.undocking_gcode.render() + "\nM400")
                except Exception as e:
                    raise Exception(f"Error executing undocking gcode: {e}")
            else:
                # Determine z-axis and undocking axis positions for undocking
                if self.axis_mode == 'static' and self.docking_axis is False:
                    target_z_offset = self.docked_position_z - self.disengage_offset_z
                else:
                    target_z_offset = self.docked_offset_z - self.disengage_offset_z
                movepos_z = self._determine_movepos_z(target_z_offset)
                if movepos_z is [None, None]:
                    raise Exception(
                        f"Cannot achieve docked offset z={target_z_offset}mm with current positions.")
                
                # Get current toolhead position
                pos = self.toolhead.get_position()

                # Move z-axis and possibly docking axis to desired height (slightly above dock)
                if movepos_z[1] is not None:
                    speed = min(self.travel_speed, self.docking_axis.stepper.velocity)
                    self.docking_axis.stepper.do_move(
                        movepos_z[1], speed, self.docking_axis.stepper.accel, sync=False if self.axis_mode == 'balanced' else True
                    )
                if movepos_z[0] is not None:
                    pos[2] = movepos_z[0]
                    self.toolhead.move(pos, self.travel_speed)
                self.toolhead.wait_moves()

                # Move to safe xy position in front of dock
                pos[0] = self.docked_position_x
                pos[1] = self.docked_position_y + self.disengage_offset_y
                self.toolhead.move(pos, self.travel_speed)

                # Advance into dock position
                pos[1] = self.docked_position_y
                self.toolhead.move(pos, self.engage_speed)

                # Raise to docked height
                pos[2] += self.disengage_offset_z
                self.toolhead.move(pos, self.engage_speed)

                # Slide out of dock
                pos[1] = self.docked_position_y + self.slide_distance_y
                self.toolhead.move(pos, self.docking_speed)

                # Raise to safe height
                pos[2] += self.safe_offset_z
                self.toolhead.move(pos, self.docking_speed)

                # Move to safe xy position in front of dock
                pos[1] = self.safe_position_y
                self.toolhead.move(pos, self.travel_speed)
            
                # Verify that toolhead is now mounted
                if self.toolhead_detect_shuttle and self.toolhead_detect_shuttle.get_enabled():
                    self.toolhead.wait_moves()
                    if not self.toolhead_detect_shuttle.query_state_blocking():
                        raise Exception(
                            f"Toolhead not detected as mounted after undocking on dock {self.name}.")

                # Verify dock sensor if configured
                if self.toolhead_detect_dock and self.toolhead_detect_dock.get_enabled():
                    if self.toolhead_detect_dock.query_state_blocking():
                        raise Exception(
                            f"Dock sensor still indicated docked state after undocking on dock {self.name}.")

            # Restore previous positions
            if restore_pos:
                if restore_axes is None:
                    restore_axes = self.restore_axes
                self.restore_last_pos(restore_axes=restore_axes)
            else:
                # Cleanup current position from toolhead
                self.toolhead.wait_moves()
                self.toolhead.set_position(self.toolhead.get_position())
            self.set_status(STATUS_ENGAGED)
        except Exception as e:
            self.handle_exception(e, "undocking", pause_on_error=self.exception_pause)

    def retract_filament(self):
        try:
            logging.info(f"Retracting filament on dock {self.name}")
            self.activate_extruder()
            self.check_set_extruder_temp(wait=True)
            pos = self.toolhead.get_position()
            pos[3] -= self.retract_length
            self.toolhead.move(pos, self.retract_speed)
            #self.toolhead.wait_moves()
            self.toolhead.set_position(self.toolhead.get_position())
            self.restore_extruder()
        except Exception as e:
            self.handle_exception(e, "retracting filament", pause_on_error=self.exception_pause)

    def unretract_filament(self):
        try:
            logging.info(f"Unretracting filament on dock {self.name}")
            #self.activate_extruder()
            self.check_set_extruder_temp(wait=True)
            pos = self.toolhead.get_position()
            pos[3] += self.unretract_length
            self.toolhead.move(pos, self.unretract_speed)
            #self.toolhead.wait_moves()
            self.toolhead.set_position(self.toolhead.get_position())
            #self.restore_extruder()
        except Exception as e:
            self.handle_exception(e, "unretracting filament", pause_on_error=self.exception_pause)
    
    def finalize_load_to_cutter(self):
        if self.toolhead_sensor_cutter_distance is None:
            logging.info(f"No toolhead sensor to cutter distance calibrated for dock {self.name}, skipping finalize load to cutter")
            return
        try:
            logging.info(f"Finalizing load to cutter on dock {self.name}")
            pos = self.toolhead.get_position()
            pos[3] += self.toolhead_sensor_cutter_distance
            speed = 5.
            self.toolhead.move(pos, speed)
            #self.toolhead.wait_moves()
            self.toolhead.set_position(self.toolhead.get_position())
        except Exception as e:
            self.handle_exception(e, "finalizing load to cutter", pause_on_error=self.exception_pause)

    def save_init_pos(self, save_axes=None):
        self.toolhead.set_position(self.toolhead.get_position())
        # Verify if custom save axes provided
        # axes is to be a string list of axes to save, e.g. ['x', 'y', 'z', 'docking_axis']
        if save_axes is not None:
            for axis in save_axes:
                if axis not in ['x', 'y', 'z', 'e', 'docking_axis']:
                    raise self.printer.command_error(
                        f"Invalid axis '{axis}' for saving position, valid axes are: x, y, z, e, docking_axis.")
                curpos = self.toolhead.get_position()
                if self.last_toolhead_pos is None:
                    self.last_toolhead_pos = curpos
                elif axis == 'x':
                    self.last_toolhead_pos[0] = curpos[0]
                elif axis == 'y':
                    self.last_toolhead_pos[1] = curpos[1]
                elif axis == 'z':
                    self.last_toolhead_pos[2] = curpos[2]
                elif axis == 'e':
                    self.last_toolhead_pos[3] = curpos[3]
                if axis == 'docking_axis' and self.docking_axis:
                    docking_axis_pos = self.docking_axis.get_position()
                    self.last_docking_axis_pos = docking_axis_pos
        else:
            # Save current toolhead and docking_axis positions for restore after operation
            self.last_toolhead_pos = self.toolhead.get_position()
            if self.docking_axis:
                self.last_docking_axis_pos = self.docking_axis.get_position()
    
    def restore_last_pos(self, restore_axes=False):
        try:
            # Ensure all moves are done before restoring and cleanup position from toolhead
            #self.toolhead.wait_moves()
            self.toolhead.set_position(self.toolhead.get_position())

            # Restore requested axes to last position
            if restore_axes is False:
                return
            elif restore_axes is True:
                restore_axes = self.restore_axes
            else:
                restore_axes = restore_axes
            
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
                if 'docking_axis' in axes and self.docking_axis:
                    self.toolhead.wait_moves()
                    
            # Set init positions to None
            #self.toolhead.wait_moves()
            self.toolhead.set_position(self.toolhead.get_position()) # Cleanup
            self.last_toolhead_pos = None
            self.last_docking_axis_pos = None
        except Exception as e:
            self.handle_exception(e, "restoring position", pause_on_error=self.exception_pause)

    def enabled_check(self):
        curtime = self.reactor.monotonic()
        if self.docking_axis:
            # Check if docking axis is homed (also means enabled)
            if not self.docking_axis.get_status(curtime)['homed']:
                raise self.printer.command_error(
                    f"Docking axis not homed, please home before performing motions.")
        if 'xyz' not in self.toolhead.get_kinematics().get_status(curtime)['homed_axes']:
            raise self.printer.command_error(
                f"Toolhead not homed, please home before performing motions")
    
    def _determine_movepos_z(self, target_z_offset):
        try:
            # Get current toolhead position or use last position
            curpos = self.toolhead.get_position() if self.last_toolhead_pos is None else self.last_toolhead_pos

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
                    raise Exception(
                        f"Required toolhead Z position {toolhead_movepos_z} out of bounds.")
                movepos = [toolhead_movepos_z, None]
                return movepos
            else:
                toolhead_pos_z = self.toolhead.get_position()[2] if self.last_toolhead_pos is None else self.last_toolhead_pos[2]
                docking_axis_pos_z = self.docking_axis.get_position()
                if self.axis_mode == 'balanced':
                    # Split the difference equally
                    delta_z = (docking_axis_pos_z - toolhead_pos_z) + target_z_offset
                    toolhead_movepos_z = toolhead_pos_z + (delta_z / 2.)
                    docking_axis_movepos_z = docking_axis_pos_z - (delta_z / 2.)                                    
                elif self.axis_mode == 'minimize_z':
                    # Minimize toolhead movement, docking axis does most of the work
                    docking_axis_movepos_z = toolhead_pos_z + target_z_offset
                    toolhead_movepos_z = toolhead_pos_z

            # Correct for axes bounds if needed
            for _ in range(2):
                # Check 1: Toolhead may never move below current position
                if toolhead_movepos_z < curpos[2]:
                    toolhead_movepos_z = curpos[2]
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
                raise Exception(
                    f"Cannot achieve {target_z_offset:.2f}mm offset: "
                    f"toolhead Z would be {toolhead_movepos_z:.2f}mm "
                    f"(limits: {toolhead_min_z:.2f} to {toolhead_max_z:.2f}mm, current: {toolhead_pos_z:.2f}mm)"
                )
            
            if (docking_axis_movepos_z > docking_axis_max_z or 
                docking_axis_movepos_z < docking_axis_min_z):
                raise Exception(
                    f"Cannot achieve {target_z_offset:.2f}mm offset: "
                    f"docking axis would be {docking_axis_movepos_z:.2f}mm "
                    f"(limits: {docking_axis_min_z:.2f} to {docking_axis_max_z:.2f}mm)"
                )
                       
            movepos = [toolhead_movepos_z, docking_axis_movepos_z]
            return movepos

        except Exception as e:
            self.handle_exception(e, "determining movepos z", pause_on_error=self.exception_pause)
    
    def get_extruder_name(self):
        return self.extruder_name
    
    def get_name(self):
        return self.name
    
    def get_status(self, eventtime=None):
        return{
            'status': self.status,
            'axis_mode': self.axis_mode
        }

def load_config_prefix(config):
    return DockUnit(config)