# Collection of spool unit related functions
#
# Copyright (C) 2025 Eytecz Engineering
#
# This file may be distributed under the terms of the GNU GPLv3 license

import math
import logging
import traceback
import configparser
from configfile import ConfigWrapper # type: ignore
from .led_effect import ledEffect # type: ignore

STATUS_UNINITIALIZED    = 'uninitialized'   # Initial state before determining actual status
STATUS_INITIALIZING     = 'initializing'    # State detection to find initial state
STATUS_EMPTY            = 'empty'           # No filament present in the pre-gate sensor
STATUS_HOMING           = 'homing'          # Automated loading assist of filament until post-gear sensor
STATUS_IDLE             = 'idle'            # Filament available in spool unit extruder gear
STATUS_LOADING          = 'loading'         # Automated loading as effect of request event until toolhead
STATUS_LOADED           = 'loaded'          # Filament available in toolhead and ready for printing
STATUS_UNLOADING        = 'unloading'       # Automated unloading as effect of exchange/remove request event
STATUS_RUNOUT           = 'runout'          # Filament runout detected during printing on pre-gate sensor
STATUS_ERROR            = 'error'           # Exception occurred during homing/loading/unloading
STATUS_CALIBRATING      = 'calibrating'     # Automated calibration processes ongoing
STATUS_EJECTING         = 'ejecting'        # Automated ejection of filament from spool unit

class InsertHelper:
    def __init__(self, config, spool_unit):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.spool_unit = spool_unit

        # Read config
        self.insert_load = config.getboolean('load_on_insert', True)
        insert_pin = config.get('insert_pin', None)
        self.debounce_time = config.getfloat('debounce_time', 2.0)

        # Initial state
        self.min_event_systime = self.reactor.NEVER
        self.filament_present = False
        self.sensor_enabled = True
        self.homing_state = False

        # Register required objects
        self.gcode = self.printer.lookup_object('gcode')
        buttons = self.printer.load_object(config, 'buttons')

        # Register commands and event handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        buttons.register_debounce_button(insert_pin, self._event_handler, config)

    def handle_ready(self):
        self.min_event_systime = self.reactor.monotonic() + self.debounce_time

    def _event_handler(self, eventtime, state):
        self.note_filament_present(eventtime, state)

    def _insert_event_handler(self, eventtime):
        if self.insert_load and not self.homing_state:
            try:
                self.spool_unit.set_status(STATUS_HOMING)
                self.spool_unit.sync_to_extruder(False)
                self.homing_state = True
                self.spool_unit.stepper_helper.do_set_position(0.)
                self.spool_unit.stepper_helper.do_homing_move(movepos=500.0, triggered=True)
                self.spool_unit.stepper_helper.do_set_position(0.)
                self.spool_unit.stepper_helper.do_move(movepos=-10.0)
                self.homing_state = False
                self.spool_unit.set_status(STATUS_IDLE)
            except Exception as e:
                self.homing_state = False
                return self.spool_unit.handle_exception(e, "insert event handling", pause_on_error=False)

    def _runout_event_handler(self, eventtime):
        logging.info(f"Runout event triggered at {eventtime}")
        self.spool_unit.runout_event_handler(eventtime)
        
    def query_endstop(self):
        return self.filament_present

    def note_filament_present(self, eventtime, state):
        if state == self.filament_present:
            return
        self.filament_present = state
        if eventtime < self.min_event_systime or not self.sensor_enabled:
            return
        self.min_event_systime = eventtime + self.debounce_time
        if self.filament_present:
            self._insert_event_handler(eventtime)
        else:
            self._runout_event_handler(eventtime)

class StepperHelper:
    def __init__(self, config, spool_unit):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.spool_unit = spool_unit
        self.stepper = None

        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        
        # Optional config sections
        self.load_speed = config.getfloat('load_speed', 150.0, minval=1.0)
        self.unload_speed = config.getfloat('unload_speed', 150.0, minval=1.0)
        self.homing_speed = config.getfloat('homing_speed', 50.0, minval=1.0)
        self.accel = config.getfloat('accel', 500.0, minval=1.0)

    def handle_connect(self):
        stepper_name = self.config.get('stepper', None)
        for manual_stepper in self.printer.lookup_objects('manual_stepper'):
            name = manual_stepper[1].get_steppers()[0].get_name()
            if name == stepper_name:
                self.stepper = manual_stepper[1]
        if self.stepper is None:
            raise self.config.error("Could not find stepper '%s'" % stepper_name)
        
        self.toolhead = self.printer.lookup_object('toolhead')
        
        # Intercept stepper trapq_append and wipe_trapq function to extract motion data
        self.trapq_append_original = self.stepper.trapq_append
        self.stepper.trapq_append = self._trapq_append_intercept
        self.wipe_trapq_original = self.stepper.motion_queuing.wipe_trapq
        self.stepper.motion_queuing.wipe_trapq = self._wipe_trapq_intercept

    def _trapq_append_intercept(self, *args):
        self.trapq_append_original(*args)
        self.spool_unit.motion_extraction(*args)

    def _wipe_trapq_intercept(self, *args):
        self.wipe_trapq_original(*args)
        self.spool_unit.hbridge_motor.abort_async_motion()
    
    def query_endstop(self, print_time=None):
        if print_time is None:
            print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        qe = self.printer.lookup_object('query_endstops')
        for mcu_endstop, name in qe.endstops:
            if name == self.stepper.get_steppers()[0].get_name():
                state = mcu_endstop.query_endstop(print_time)
                break
            state = None
        if state is None:
            logging.warning(f'No endstop found for stepper {self.stepper.get_steppers()[0].get_name()}')
            return None
        return bool(state)

    def do_homing_move(self, movepos, triggered, homing_speed=None):
        if homing_speed is None:
            homing_speed = self.homing_speed
        self.stepper.do_homing_move(movepos, homing_speed, self.accel, triggered, check_trigger=True)
    
    def do_move(self, movepos):
        if movepos >=0:
            speed = self.load_speed
        else:
            speed = self.unload_speed
        self.stepper.do_move(movepos, speed, self.accel, sync=True)
    
    def get_position(self):
        return self.stepper.get_position()
    
    def do_set_position(self, position):
        self.stepper.do_set_position(position)
    
class LEDHelper:
    def __init__(self, config, spool_unit):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = spool_unit.get_name()

        # Initial state
        self.state_effects = {}

        # State layer default presets
        LAYER_INITIALIZING = """breathing     2  1     top        (0.0, 0.0, 0.0, 1.0)"""
        LAYER_EMPTY        = """static        0  0     top        (0.0, 0.0, 0.0, 1.0)"""
        LAYER_HOMING       = """breathing     2  1     top        (0.0, 0.0, 1.0, 0.0)"""
        LAYER_IDLE         = """static        0  0     top        (0.0, 0.0, 1.0, 0.0)"""
        LAYER_LOADING      = """breathing     2  1     top        (0.0, 1.0, 0.0, 0.0)"""
        LAYER_LOADED       = """static        0  0     top        (0.0, 1.0, 0.0, 0.0)"""
        LAYER_UNLOADING    = """breathing     2  1     top        (1.0, 0.0, 0.0, 0.0)"""
        LAYER_RUNOUT       = """breathing     2  1     top        (1.0, 0.5, 0.0, 0.0)"""
        LAYER_ERROR        = """strobe        1  1.5   add        (1.0, 1.0, 1.0, 1.0)
                                breathing     2  0     difference (1.0, 0.0, 0.0, 0.0)
                                static        1  0     top        (1.0, 0.0, 0.0, 0.0)"""
        LAYER_CALIBRATING  = """breathing     2  1     top        (1.0, 1.0, 0.0, 0.0)"""
        LAYER_EJECTING     = """breathing     2  1     top        (0.0, 0.0, 0.0, 1.0)"""
        
        # Read config section
        self.state_layers = {
            STATUS_INITIALIZING:    config.get('state_layer_initializing', LAYER_INITIALIZING),
            STATUS_EMPTY:           config.get('state_layer_empty', LAYER_EMPTY),
            STATUS_HOMING:          config.get('state_layer_homing', LAYER_HOMING),
            STATUS_IDLE:            config.get('state_layer_idle', LAYER_IDLE),
            STATUS_LOADING:         config.get('state_layer_loading', LAYER_LOADING),
            STATUS_LOADED:          config.get('state_layer_loaded', LAYER_LOADED),
            STATUS_UNLOADING:       config.get('state_layer_unloading', LAYER_UNLOADING),
            STATUS_RUNOUT:          config.get('state_layer_runout', LAYER_RUNOUT),
            STATUS_ERROR:           config.get('state_layer_error', LAYER_ERROR),
            STATUS_CALIBRATING:     config.get('state_layer_calibrating', LAYER_CALIBRATING),
            STATUS_EJECTING:        config.get('state_layer_ejecting', LAYER_EJECTING)
            }
        self.frame_rate = config.getfloat('frame_rate', default=24, minval=1, maxval=60)
        self.status_leds = config.get('status_leds')
        
        # Lookup required objects
        self.configfile = self.printer.lookup_object('configfile')

        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
    
    def handle_connect(self):
        self._create_led_configs()
    
    def _create_led_configs(self):
        configs = {}
        for state, state_layer in self.state_layers.items():
            effect_name = f'{self.name}_{state}'
            effect_config = {
                'auto_start':   'False',
                'frame_rate':   str(self.frame_rate),
                'layers':       state_layer,
                'leds':         self.status_leds
            }
            config, section = self._create_led_config(effect_name, effect_config)
            configs[state] = config

            try:
                self.printer.add_object(section, self.printer.load_object(config, 'led_effect'))
                self.state_effects[state] = ledEffect(config)
            except Exception as e:
                logging.error(f"Error creating LED effect '{effect_name}': {e}")
                   
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
         
class SpoolUnit:
    _instances: list["SpoolUnit"] = []
    _ready_report_timer = None

    def __init__(self, config):
        SpoolUnit._instances.append(self)
        self.config = config
        self.printer = self.config.get_printer()
        self.reactor = self.printer.get_reactor()

        # MCU Tracking
        self.all_mcus = [m for n, m in self.printer.lookup_objects(module='mcu')]
        self.mcu = self.all_mcus[0]

        # Initial state
        self.status = STATUS_UNINITIALIZED
        self.sensor_states = {}
        self.spool_measurement = False
        self.assist_forward = False
        self.assist_reverse = True
        self.enable_tracking = True
        self.moved_distance = 0.
        self.can_extrude_original = None
        self.synced = False
        self.previous_extruder = None

        # Register handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        # Register required objects
        self.gcode = self.printer.lookup_object('gcode')
        self.configfile = self.printer.lookup_object('configfile')      

        # Read config section
        self.name = config.get_name().split()[1]
        self.spool_diameter = [config.getfloat('spool_diameter_min', 90.0),
                               config.getfloat('spool_diameter_max', 200.0)]
        self.poll_interval = config.getfloat('poll_interval', 0.5, minval=0.01)
        self.assist_threshold = config.getfloat('assist_threshold', 20.0, minval=0.0)
        self.hbridge_motor_name = config.get('hbridge_motor', None)
        self.filament_hub_name = config.get('filament_hub', None)
        self.toolhead_sensor_name = config.get('toolhead_sensor', None)
        self.extruder_name = config.get('extruder', 'extruder')
        self.hub_toolhead_distance_max = config.getfloat('hub_toolhead_distance_max', None)
        self.hub_toolhead_distance_min = config.getfloat('hub_toolhead_distance_min', None)
        self.park_hub_distance = config.getfloat('park_hub_distance', None)
        self.gear_entry_toolhead_distance = config.getfloat('gear_entry_toolhead_distance', None)
        
        self.runout_pause = config.getboolean('pause_on_runout', True)
        self.exception_pause = config.getboolean('pause_on_exception', True)
        self.runout_eject = config.getboolean('eject_on_runout', False)
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        if self.runout_pause or config.get('runout_gcode', None) is not None:
            self.runout_gcode = gcode_macro.load_template(
                config, 'runout_gcode', '')
        if self.exception_pause or config.get('exception_gcode', None) is not None:
            self.exception_gcode = gcode_macro.load_template(
                config, 'exception_gcode', '')
        
        self.vl6180_name = config.get('vl6180_sensor', None)
        if self.vl6180_name:
            self.vl6180_center_distance = config.getfloat('vl6180_center_distance', 125.0, minval=0.0)
            self.measurement_samples = config.getint('measurement_samples', 10, minval=1, maxval=20)
        
        status_leds = config.get('status_leds', None)
        if status_leds:
            self.led_helper = LEDHelper(config, self)

        insert_pin = config.get('insert_pin', None)
        if insert_pin:
            self.insert_helper = InsertHelper(config, self)

        self.stepper_name = config.get('stepper', None)
        if self.stepper_name:
            self.stepper_helper = StepperHelper(config, self)
        
        # Material and spool properties for content estimation
        self.material_density = config.getfloat('material_density', 1.05, minval=0.1) 
        self.filament_diameter = config.getfloat('filament_diameter', 1.75, minval=0.1)
        self.spool_width = config.getfloat('spool_width', 56.0, minval=1.0)
        self.packing_efficiency = config.getfloat('packing_efficiency', 0.80, minval=0.5, maxval=1.0)

        # Register g-code commands
        self.gcode.register_mux_command('SPOOL_MOTION_CONTROL', 'SPOOL', self.name,
                                        self.cmd_SPOOL_MOTION_CONTROL,
                                        desc="Control spool motion functionality.")
        self.gcode.register_mux_command('MEASURE_SPOOL_DIAMETER', 'SPOOL', self.name,
                                        self.cmd_MEASURE_SPOOL_DIAMETER,
                                        desc="Measure and report the current spool diameter.")
        self.gcode.register_mux_command('ESTIMATE_SPOOL_CONTENT', 'SPOOL', self.name,
                                        self.cmd_ESTIMATE_SPOOL_CONTENT,
                                        desc="Estimate the remaining filament content on the spool.")
        self.gcode.register_mux_command('SET_STATUS', 'SPOOL', self.name,
                                        self.cmd_SET_STATUS,
                                        desc="Set the status of the spool unit for LED indication.")
        self.gcode.register_mux_command('QUERY_SPOOL_UNIT', 'SPOOL', self.name,
                                        self.cmd_QUERY_SPOOL_UNIT,
                                        desc="Query the current status and parameters of the spool unit.")
        self.gcode.register_mux_command('CLEAR_ERROR', 'SPOOL', self.name,
                                        self.cmd_CLEAR_ERROR,
                                        desc="Clear error state and re-initialize the spool unit.")
        self.gcode.register_mux_command('SPOOL_LOAD', 'SPOOL', self.name,
                                        self.cmd_SPOOL_LOAD,
                                        desc="Load filament from spool unit to toolhead.")
        self.gcode.register_mux_command('SPOOL_UNLOAD', 'SPOOL', self.name,
                                        self.cmd_SPOOL_UNLOAD,
                                        desc="Unload filament from toolhead to spool unit.")
        self.gcode.register_mux_command('SPOOL_EJECT', 'SPOOL', self.name,
                                        self.cmd_SPOOL_EJECT,
                                        desc="Eject filament completely from spool unit.")
        self.gcode.register_mux_command('CALIBRATE_BOWDEN_LENGTH', 'SPOOL', self.name,
                                        self.cmd_CALIBRATE_BOWDEN_LENGTH,
                                        desc="Calibrate the bowden length from spool unit to toolhead.")


    def handle_ready(self):
        self.reactor.register_timer(self.initialize_spool_unit, self.reactor.monotonic() + 2.0)

        def _report_status(eventtime):
                states = []
                for spool_unit in sorted(SpoolUnit._instances, key=lambda su: su.name):
                    status = spool_unit.get_status(eventtime)
                    states.append(
                        f"{spool_unit.name}: <b>{status['status'].upper()}</b>"
                    )
                header = "<b><span style='color: #FF4444;'>Spool unit states</span></b>"
                body = ", ".join(states)
                self.gcode.respond_info(f"{header}\n{body}")
                SpoolUnit._ready_report_timer = None
                return self.reactor.NEVER

        if SpoolUnit._ready_report_timer is None:
            waketime = self.reactor.monotonic() + 6.0
            SpoolUnit._ready_report_timer = self.reactor.register_timer(
                _report_status, waketime
            )          

    def handle_connect(self):
        self.axis_sync = self.printer.lookup_object('axis_sync')
        self.toolhead = self.printer.lookup_object('toolhead')

        # Connect extruder
        self.extruder = self.printer.lookup_object(self.extruder_name)

        # Connect filament hub modules
        if self.filament_hub_name:
            for filament_hub in self.printer.lookup_objects('filament_hub'):
                name = filament_hub[1].get_name()
                if name == self.filament_hub_name:
                    self.filament_hub = filament_hub[1]
            if self.filament_hub is None:
                raise self.config.error("Could not find filament_hub '%s'" % self.filament_hub_name)
        
        # Connect toolhead sensor modules
        if self.toolhead_sensor_name:
            self.toolhead_sensor = None
            for toolhead_sensor in self.printer.lookup_objects('filament_switch_sensor'):
                name = toolhead_sensor[1].runout_helper.name
                if name == self.toolhead_sensor_name:
                    self.toolhead_sensor = toolhead_sensor[1]
            if self.toolhead_sensor is None:
                raise self.config.error("Could not find toolhead_sensor '%s'" % self.toolhead_sensor_name)

        # Connect hbridge_motor modules 
        if self.hbridge_motor_name:
            for hbridge_motor in self.printer.lookup_objects('hbridge_motor'):
                name = hbridge_motor[1].get_name()
                if name == self.hbridge_motor_name:
                    self.hbridge_motor = hbridge_motor[1]
            if self.hbridge_motor is None:
                raise self.config.error("Could not find hbridge_motor '%s'" % self.hbridge_motor_name)
        else:
            raise self.config.error("Missing required 'hbridge_motor' config option") 

        # Connect vl6180 sensor
        if self.vl6180_name:
            for vl6180 in self.printer.lookup_objects('vl6180'):
                name = vl6180[1].get_name()
                if name == self.vl6180_name:
                    self.vl6180 = vl6180[1]
            if self.vl6180 is None:
                raise self.config.error("Could not find vl6180 '%s'" % self.vl6180_name)
            self.spool_measurement = self.config.getboolean('spool_measurement', True)

    def handle_exception(self, exception, context, pause_on_error=True):
        # Log the error with context
        logging.error(f"Error in {context} for spool {self.name}: {exception}")
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
                    f"{str(exception)} Printing has been paused. Please resolve the issue before resuming."
                )
                if context == "initializing":
                    return self.reactor.NEVER
                else:
                    raise Exception(msg)
        
        msg = (
            f"{str(exception)} Printing has not been paused. Please resolve the issue before continuing.")
        if context == "initializing":
            return self.reactor.NEVER
        else:
            raise Exception(msg)

    def cmd_SET_STATUS(self, gcmd):
        self.set_status(gcmd.get('STATUS'))
    
    def cmd_QUERY_SPOOL_UNIT(self, gcmd):
        # Get sensor states safely
        pre_gate = bool(self.insert_helper.query_endstop()) if hasattr(self, 'insert_helper') else None
        post_gear = bool(self.stepper_helper.query_endstop()) if hasattr(self, 'stepper_helper') else None
        hub = bool(self.filament_hub.query_hub_endstop()) if hasattr(self, 'filament_hub') else None
        toolhead = bool(self.toolhead_sensor.runout_helper.filament_present) if hasattr(self, 'toolhead_sensor') else None
        
        # Get spool diameter
        diameter = self.estimate_spool_diameter()
        diameter_str = f"{diameter:.1f}mm" if diameter else "n/a"
        
        # Get spool content
        content = self.estimate_spool_content()
        if content:
            length_m, mass_g = content
            content_str = f"{length_m:.1f}m / {mass_g:.0f}g"
        else:
            content_str = "n/a"
        
        # Format sensor states with color
        def fmt_sensor(val):
            if val is True:
                return "<span style='color: #00FF00;'>triggered</span>"
            elif val is False:
                return "<span style='color: #FF0000;'>open</span>"
            else:
                return "n/a"
        
        # Format enabled/disabled with color
        def fmt_state(enabled):
            if enabled:
                return "<span style='color: #00FF00;'>enabled</span>"
            else:
                return "<span style='color: #FF0000;'>disabled</span>"
        
        gcmd.respond_info(
            f"<b><span style='color: #FF4444;'>Query results for spool unit {self.name}</span></b>\n"
            f"  Status: <b>{self.status.upper()}</b>\n"
            f"  Synced to extruder: {fmt_state(bool(self.synced))}\n"
            f"  Measured spool diameter: {diameter_str}\n"
            f"  Estimated spool content: {content_str}\n"
            f"  Pre-gate sensor: {fmt_sensor(pre_gate)}\n"
            f"  Post-gear sensor: {fmt_sensor(post_gear)}\n"
            f"  Hub sensor: {fmt_sensor(hub)}\n"
            f"  Toolhead sensor: {fmt_sensor(toolhead)}\n"
            f"  tracking: {fmt_state(self.enable_tracking)}\n"
            f"  forward_assist: {fmt_state(self.assist_forward)}\n"
            f"  reverse_assist: {fmt_state(self.assist_reverse)}\n"
            f"  assist_threshold: {self.assist_threshold:.1f}mm"
        )
            
    def cmd_SPOOL_LOAD(self, gcmd):
        try:
            self.spool_load()
        except Exception as e:
            raise gcmd.error(f"Spool load error for spool unit {self.name}: {e}")

    def cmd_SPOOL_UNLOAD(self, gcmd):
        try:
            self.spool_unload()
        except Exception as e:
            raise gcmd.error(f"Spool unload error for spool unit {self.name}: {e}")

    def cmd_SPOOL_EJECT(self, gcmd):
        try:
            self.spool_eject()
        except Exception as e:
            raise gcmd.error(f"Spool eject error for spool unit {self.name}: {e}")
        
    def cmd_CLEAR_ERROR(self, gcmd):
        if self.status != STATUS_ERROR:
            gcmd.respond_info(f'Spool unit {self.name} not in error state, no action taken.')
            return
        gcmd.respond_info(f'Clearing error state and re-initializing spool unit {self.name}.')
        self.initialize_spool_unit()

    def set_status(self, status):
        if self.status == status:
            return
        self.status = status
        if self.led_helper:
            self.led_helper.set_state(self.status)
        
    def initialize_spool_unit(self, eventtime=None):
        self.set_status(STATUS_INITIALIZING)
        self.sensor_states = {
            'pre_gate_sensor': bool(self.insert_helper.query_endstop()) if hasattr(self, 'insert_helper') else None,
            'post_gear_sensor': bool(self.stepper_helper.query_endstop()) if hasattr(self, 'stepper_helper') else None,
            'hub_sensor': bool(self.filament_hub.query_hub_endstop()) if hasattr(self, 'filament_hub') else None,
            'toolhead_sensor': bool(self.toolhead_sensor.runout_helper.filament_present) if hasattr(self, 'toolhead_sensor') else None
        }
        
        # Catch inconsistent sensor states
        if self.sensor_states.get('hub_sensor') == True and self.sensor_states.get('toolhead_sensor') == False:
            e = f"Inconsistent sensor states detected during initialization for spool unit {self.name}."
            self.handle_exception(e, "initializing", False)
        elif self.sensor_states.get('hub_sensor') == False and self.sensor_states.get('toolhead_sensor') == True:
            e = f"Inconsistent sensor states detected during initialization for spool unit {self.name}."
            self.handle_exception(e, "initializing", False)
        
        # Determine status based on sensor states
        if self.sensor_states.get('pre_gate_sensor') == False:
            if self.sensor_states.get('post_gear_sensor') == False:
                self.set_status(STATUS_EMPTY)
            else:
                self.set_status(STATUS_ERROR)
        elif self.sensor_states.get('pre_gate_sensor') == True:
            if self.sensor_states.get('post_gear_sensor') == False:
                try:
                    self.stepper_helper.do_set_position(0.)
                    self.stepper_helper.do_homing_move(movepos=50.0, triggered=True)
                    self.stepper_helper.do_set_position(0.)
                    self.stepper_helper.do_move(movepos=-10.0)
                    self.stepper_helper.do_set_position(0.)
                    self.set_status(STATUS_IDLE)
                except Exception as e:
                    self.handle_exception(e, "initializing", pause_on_error=False) 
            else:
                if self.sensor_states.get('hub_sensor') == True and self.sensor_states.get('toolhead_sensor') == True:
                    try:
                        loaded_spool = self.filament_hub.get_loaded_spool_unit()
                        if loaded_spool is not None and loaded_spool is not self:
                            # Delayed start required to allow other spool unit to finish initialization first
                            def error_timer(eventtime):
                                e = (
                                f"Cannot initialize spool unit {self.name} and {loaded_spool.name}, sensor conflict detected.\n"
                                f"Manual intervention required to resolve conflict."
                                )
                                self.gcode.respond_info(e)
                                self.filament_hub.set_loaded_spool_unit(None)
                                self.set_status(STATUS_ERROR)
                                loaded_spool.set_status(STATUS_ERROR)
                                return self.reactor.NEVER
                            self.reactor.register_timer(error_timer, self.reactor.monotonic() + 2.0)
                        elif loaded_spool is None:
                            self.filament_hub.set_loaded_spool_unit(self)
                            self.sync_to_extruder(True)
                            self.set_status(STATUS_LOADED)
                    except Exception as e:
                        self.handle_exception(e, "initializing", pause_on_error=False)
                else:
                    self.set_status(STATUS_ERROR)
        return self.reactor.NEVER        
    
    def cmd_CALIBRATE_BOWDEN_LENGTH(self, gcmd):
        try:
            self.calibrate_bowden_length()
        except Exception as e:
            raise gcmd.error(f"Bowden length calibration error for spool unit {self.name}: {e}")

    def calibrate_bowden_length(self):
        if self.status == STATUS_IDLE or self.status == STATUS_LOADED:
            # Check shared hub sensor state and unload
            loaded_spool = self.filament_hub.get_loaded_spool_unit()
            if loaded_spool is self:
                self.spool_unload()
            elif loaded_spool is not None:
                loaded_spool.spool_unload()
            self.set_status(STATUS_CALIBRATING)
            try:
                # Preheat extruder if needed
                self.check_set_extruder_temp(wait=False)

                # Reposition filament at correct distance from post-gear sensor
                self.stepper_helper.do_set_position(0.)
                self.stepper_helper.do_homing_move(movepos=50.0, triggered=True)
                self.stepper_helper.do_set_position(0.)
                self.stepper_helper.do_move(movepos=-10.0)
                self.stepper_helper.do_set_position(0.)
                
                # Estimate filament hub sensor distance
                step_size = 1.0
                movepos = step_size 
                while not self.filament_hub.query_hub_endstop():
                    self.stepper_helper.do_move(movepos)
                    self.toolhead.wait_moves()
                    movepos += step_size
                    if movepos > self.park_hub_distance + 20.0:
                        raise Exception("Filament hub endstop not triggered within expected range during calibration.")
                hub_distance = self.stepper_helper.stepper.get_position()[0]

                # Estimate toolhead sensor distance
                self.sync_to_extruder(True)
                self.activate_extruder()
                self.check_set_extruder_temp(wait=True) # Ensure extruder is hot enough to allow motion
                speed = self.stepper_helper.homing_speed
                step_size = 10.0    # Coarse step size due to large distance
                pos = self.toolhead.get_position()
                init_pos = pos[3]
                pos[3] += step_size
                while not bool(self.toolhead_sensor.runout_helper.filament_present):
                    self.toolhead.move(pos, speed)
                    self.toolhead.wait_moves()
                    pos[3] += step_size
                    if pos[3] - init_pos > self.hub_toolhead_distance_max + 100.0:
                        raise Exception("Toolhead sensor not triggered within expected range during calibration.")
                # Back off and do fine steps
                pos[3] -= 2 * step_size 
                self.toolhead.move(pos, speed)
                self.toolhead.wait_moves()
                step_size = 1.0    # Fine step size for accurate measurement
                inter_pos = pos[3]
                pos[3] += step_size
                while not bool(self.toolhead_sensor.runout_helper.filament_present):
                    self.toolhead.move(pos, speed)
                    self.toolhead.wait_moves()
                    pos[3] += step_size
                    if pos[3] - inter_pos > 50.0:
                        raise Exception("Toolhead sensor not triggered within expected range during calibration.")
                hub_toolhead_distance = pos[3] - init_pos - step_size
                self.restore_extruder()
                self.sync_to_extruder(False)
                
                # Estimate buffer length and calculate min/max positions for toolhead sensor
                advance_state = bool(self.filament_hub.query_advancing_endstop())
                trailing_state = bool(self.filament_hub.query_trailing_endstop())
                step_size = 1.0
                movepos = self.stepper_helper.stepper.get_position()[0]
                init_pos = movepos
                if advance_state and trailing_state:        # Invalid state
                    raise Exception("Both buffer endstops are triggered! Cannot calibrate buffer length.")
                elif advance_state and not trailing_state:  # Fully extended state
                    movepos -= step_size     
                    while not bool(self.filament_hub.query_trailing_endstop()):     # Retract until trailing endstop triggered
                        self.stepper_helper.do_move(movepos)
                        self.toolhead.wait_moves()
                        movepos -= step_size
                        if movepos - init_pos < -100.0:
                            raise Exception("Trailing endstop not triggered within expected range during calibration.")
                    trailing_trigger_pos = self.stepper_helper.stepper.get_position()[0]
                    movepos += 2 * step_size
                    while not bool(self.filament_hub.query_advancing_endstop()):    # Advance until advancing endstop triggered
                        self.stepper_helper.do_move(movepos)
                        self.toolhead.wait_moves()
                        movepos += step_size
                        if movepos - init_pos > 100.0:
                            raise Exception("Advancing endstop not triggered within expected range during calibration.")
                    advancing_trigger_pos = self.stepper_helper.stepper.get_position()[0]
                    buffer_length = advancing_trigger_pos - trailing_trigger_pos
                    correction_length = advancing_trigger_pos - init_pos
                    mean_pos = (advancing_trigger_pos + trailing_trigger_pos) / 2
                    hub_toolhead_distance_max = hub_toolhead_distance + correction_length
                    hub_toolhead_distance_min = hub_toolhead_distance_max - buffer_length
                elif not advance_state and trailing_state:  # Fully retracted state
                    movepos += step_size
                    while not bool(self.filament_hub.query_advancing_endstop()):    # Advance until advancing endstop triggered
                        self.stepper_helper.do_move(movepos)
                        self.toolhead.wait_moves()
                        movepos += step_size
                        if movepos - init_pos > 100.0:
                            raise Exception("Advancing endstop not triggered within expected range during calibration.")
                    advancing_trigger_pos = self.stepper_helper.stepper.get_position()[0]
                    movepos -= 2 * step_size     
                    while not bool(self.filament_hub.query_trailing_endstop()):     # Retract until trailing endstop triggered
                        self.stepper_helper.do_move(movepos)
                        self.toolhead.wait_moves()
                        movepos -= step_size
                        if movepos - init_pos < -100.0:
                            raise Exception("Trailing endstop not triggered within expected range during calibration.")
                    trailing_trigger_pos = self.stepper_helper.stepper.get_position()[0]
                    buffer_length = advancing_trigger_pos - trailing_trigger_pos
                    correction_length = trailing_trigger_pos - init_pos
                    mean_pos = (advancing_trigger_pos + trailing_trigger_pos) / 2
                    hub_toolhead_distance_min = hub_toolhead_distance + correction_length
                    hub_toolhead_distance_max = hub_toolhead_distance_min + buffer_length
                else:                                       # Somewhere in between  
                    movepos -= step_size     
                    while not bool(self.filament_hub.query_trailing_endstop()):     # Retract until trailing endstop triggered
                        self.stepper_helper.do_move(movepos)
                        self.toolhead.wait_moves()
                        movepos -= step_size
                        if movepos - init_pos < -100.0:
                            raise Exception("Trailing endstop not triggered within expected range during calibration.")
                    trailing_trigger_pos = self.stepper_helper.stepper.get_position()[0]
                    movepos += 2 * step_size
                    while not bool(self.filament_hub.query_advancing_endstop()):    # Advance until advancing endstop triggered
                        self.stepper_helper.do_move(movepos)
                        self.toolhead.wait_moves()
                        movepos += step_size
                        if movepos - init_pos > 100.0:
                            raise Exception("Advancing endstop not triggered within expected range during calibration.")
                    advancing_trigger_pos = self.stepper_helper.stepper.get_position()[0]
                    buffer_length = advancing_trigger_pos - trailing_trigger_pos
                    mean_pos = (advancing_trigger_pos + trailing_trigger_pos) / 2
                    hub_toolhead_distance_max = hub_toolhead_distance + (advancing_trigger_pos - init_pos)
                    hub_toolhead_distance_min = hub_toolhead_distance_max - buffer_length
                
                # Estimate position of extruder gear entry
                self.stepper_helper.do_move(mean_pos)   # Move to mean position on buffer
                self.toolhead.wait_moves()
                trigger_pos_mean = self.stepper_helper.get_position()[0]
                self.sync_to_extruder(True)
                self.activate_extruder()
                pos = self.toolhead.get_position()
                retract_dist = 50.0
                pos[3] -= retract_dist    # Move out of the extruder gears
                self.toolhead.move(pos, speed)
                self.toolhead.wait_moves()
                self.sync_to_extruder(False)
                self.restore_extruder()
                start_pos_mean = trigger_pos_mean - retract_dist
                self.stepper_helper.do_set_position(start_pos_mean)
                step_size = 1.0
                movepos = step_size + start_pos_mean
                while not bool(self.filament_hub.query_advancing_endstop()):    # Expand against gears until advancing endstop triggered
                    self.stepper_helper.do_move(movepos)
                    self.toolhead.wait_moves()
                    movepos += step_size
                    if self.stepper_helper.get_position()[0] > trigger_pos_mean:
                        raise Exception("Could not reach trailing endstop when estimating extruder gear entry position.")
                trigger_pos = self.stepper_helper.get_position()[0]
                travel_corrected = trigger_pos - start_pos_mean - (buffer_length / 2)
                gear_entry_toolhead_distance = retract_dist - travel_corrected

                # Cleanup
                self.toolhead.set_position(self.toolhead.get_position())
                self.toolhead.wait_moves()
                self.hub_toolhead_distance_max = hub_toolhead_distance_max
                self.hub_toolhead_distance_min = hub_toolhead_distance_min 
                self.park_hub_distance = hub_distance
                self.gear_entry_toolhead_distance = gear_entry_toolhead_distance
                self.spool_unload()

                # Save config values
                self.configfile.set(f'spool_unit {self.name}', 'hub_toolhead_distance_max', str(hub_toolhead_distance_max))
                self.configfile.set(f'spool_unit {self.name}', 'hub_toolhead_distance_min', str(hub_toolhead_distance_min))
                self.configfile.set(f'spool_unit {self.name}', 'park_hub_distance', str(hub_distance))
                self.configfile.set(f'spool_unit {self.name}', 'gear_entry_toolhead_distance', str(gear_entry_toolhead_distance))

                def format_macro(macro: str) -> str:
                    return f'<a class="command">{macro}</a>'
                
                self.gcode.respond_info(
                    f"Estimated park_hub_distance: {int(round(hub_distance))} mm\n"
                    f"Estimated hub_toolhead_distance_min: {int(round(hub_toolhead_distance_min))} mm\n"
                    f"Estimated hub_toolhead_distance_max: {int(round(hub_toolhead_distance_max))} mm\n"
                    f"Estimated gear_entry_toolhead_distance: {int(round(gear_entry_toolhead_distance))} mm"
                )

                self.gcode.respond_info(
                    f"Calibration completed successfully for spool_unit {self.name}.\n"
                    f"Please use {format_macro('SAVE_CONFIG')} to save the calibration values."
                )

                self.set_status(STATUS_IDLE)

            except Exception as e:
                self.handle_exception(e, "calibrate", pause_on_error=True)
        else:
            e = f"Spool unit {self.name} must be in IDLE or LOADED state to perform calibration"
            self.handle_exception(e, "calibrate", pause_on_error=True)

    def sync_to_extruder(self, state):
        if self.synced == state:
            return
        try:
            if state:
                self.toolhead.wait_moves()
                self.axis_sync.sync_stepper_to_extruder(self.stepper_name, self.extruder_name)
            else:
                self.toolhead.wait_moves()
                self.axis_sync.sync_stepper_to_extruder(self.stepper_name, None)
            self.synced = state
        except Exception as e:
            self.handle_exception(e, "sync_to_extruder", pause_on_error=True)
           
    def activate_extruder(self):
        if self.toolhead.get_extruder() == self.extruder:
            return
        self.previous_extruder = self.toolhead.get_extruder()
        self.toolhead.flush_step_generation()
        self.toolhead.set_extruder(self.extruder, self.extruder.last_position)
        self.printer.send_event("extruder:activate_extruder")
        self.toolhead.wait_moves()
    
    def restore_extruder(self):
        if self.previous_extruder is None or self.previous_extruder == self.extruder:
            return
        self.toolhead.flush_step_generation()
        self.toolhead.set_extruder(self.previous_extruder, self.previous_extruder.last_position)
        self.printer.send_event("extruder:activate_extruder")
        self.toolhead.wait_moves()

    def check_set_extruder_temp(self, wait=False):
        pheaters = self.printer.lookup_object('heaters')
        min_temp = self.extruder.heater.min_extrude_temp
        target_temp = self.extruder.heater.target_temp
        
        if not self.extruder.heater.can_extrude:
            # Extruder is too cold, heat it up
            if target_temp < min_temp:
                # No valid target set, use min_extrude_temp + margin
                pheaters.set_temperature(self.extruder.get_heater(), min_temp + 10., wait)
            else:
                # Target already set appropriately, wait if requested
                if wait:
                    pheaters.set_temperature(self.extruder.get_heater(), target_temp, wait)
        else:
            # Extruder is hot enough now, but check if target would cool it below threshold
            if target_temp < min_temp:
                # Target is too low, would cool down - prevent this
                pheaters.set_temperature(self.extruder.get_heater(), min_temp + 10., wait=False)

    def spool_unload(self):
        if self.status == STATUS_LOADED or self.status == STATUS_ERROR or self.status == STATUS_CALIBRATING or self.status == STATUS_RUNOUT:
            try:
                if self.status != STATUS_CALIBRATING:
                    self.set_status(STATUS_UNLOADING)
                self.check_set_extruder_temp(wait=True)
                # Cut filament here potentially? Ideally cut before parking?
                self.hbridge_motor.scheduled_motion(pwm_value=-1.0) # Reverse spool to keep tension
                self.sync_to_extruder(True)
                self.activate_extruder()
                speed = self.stepper_helper.unload_speed
                movepos = self.toolhead.get_position()
                movepos[3] -= 100.0     # Ideally get filament_cutter - end_of_bowden distance
                self.toolhead.move(movepos, speed)
                self.toolhead.wait_moves()
                #self.hbridge_motor.scheduled_motion(pwm_value=0)
                self.toolhead.set_position(self.toolhead.get_position()) # Cleanup position
                self.restore_extruder()
                self.sync_to_extruder(False)
                if self.hub_toolhead_distance_max and self.park_hub_distance:
                    movepos = -(self.hub_toolhead_distance_max + self.park_hub_distance)
                else:
                    movepos = -2000.0
                self.stepper_helper.do_set_position(0.)
                self.stepper_helper.do_homing_move(movepos=movepos, triggered=False, homing_speed=self.stepper_helper.unload_speed)
                movepos -= 10.0
                self.stepper_helper.do_move(movepos)
                self.hbridge_motor.scheduled_motion(pwm_value=-1.0, runtime=5.0) # Override interception
                self.stepper_helper.do_set_position(0.)
                self.filament_hub.set_loaded_spool_unit(None)

                # Final state verification
                final_check_failed = []
                if bool(self.stepper_helper.query_endstop()):
                    final_check_failed.append("post_gear_sensor")
                if bool(self.filament_hub.query_hub_endstop()):
                    final_check_failed.append("hub_sensor")
                if bool(self.toolhead_sensor.runout_helper.filament_present):
                    final_check_failed.append("toolhead_sensor")

                if final_check_failed:
                    raise Exception(f"Final unload checks failed for sensors: {', '.join(final_check_failed)}")
                
                logging.info(f'Spool unit {self.name} unloaded successfully.')
                if self.status != STATUS_CALIBRATING:
                    self.set_status(STATUS_IDLE)

            except Exception as e:
                self.handle_exception(e, "spool_unload", pause_on_error=True)
        else:
            e = f"Spool {self.name} is not in LOADED or ERROR state, cannot perform unload."
            self.handle_exception(e, "spool_unload", pause_on_error=True)
    
    def spool_load(self):
        # Check if already loaded
        loaded_spool = self.filament_hub.get_loaded_spool_unit()
        if loaded_spool is self:
            return
        # Unload if another spool is loaded
        elif loaded_spool is not None:
            loaded_spool.spool_unload()
        
        if self.status == STATUS_IDLE:
            # Load filament to toolhead
            try:
                self.set_status(STATUS_LOADING)
                self.check_set_extruder_temp(wait=False)
                self.stepper_helper.do_set_position(0.)
                movepos = (self.hub_toolhead_distance_min - self.gear_entry_toolhead_distance +
                           self.park_hub_distance)
                self.stepper_helper.do_move(movepos)    # Move to (almost) touching extruder gears
                self.toolhead.wait_moves()
                self.sync_to_extruder(True)
                self.activate_extruder()
                self.check_set_extruder_temp(wait=True) # Ensure extruder is hot enough to allow motion
                dist_max = (self.hub_toolhead_distance_max - self.hub_toolhead_distance_min +
                             self.gear_entry_toolhead_distance) + 100.0
                speed = self.stepper_helper.load_speed
                step_size = 1.0
                dist = 0.0
                pos = self.toolhead.get_position()
                pos[3] += step_size
                while not bool(self.toolhead_sensor.runout_helper.filament_present):
                    self.toolhead.move(pos, speed)
                    self.toolhead.wait_moves()
                    pos[3] += step_size
                    dist += step_size
                    if dist > dist_max:
                        raise Exception("Toolhead sensor not triggered within expected range during spool load.")
                buffer_mean_pos = ((self.hub_toolhead_distance_max + self.hub_toolhead_distance_min)/2 +
                                   self.park_hub_distance)
                correction_dist = buffer_mean_pos - movepos - dist
                self.sync_to_extruder(False)
                movepos = self.stepper_helper.get_position()[0] + correction_dist
                self.stepper_helper.do_move(movepos)
                self.toolhead.wait_moves()
                self.sync_to_extruder(True)
                ### Finalize loading until park position here
                self.toolhead.set_position(self.toolhead.get_position())
                self.restore_extruder()
                self.filament_hub.set_loaded_spool_unit(self)

                # Final state verification
                final_check_failed = []
                if not bool(self.insert_helper.query_endstop()):
                    final_check_failed.append("pre_gate_sensor")
                if not bool(self.stepper_helper.query_endstop()):
                    final_check_failed.append("post_gear_sensor")
                if not bool(self.filament_hub.query_hub_endstop()):
                    final_check_failed.append("hub_sensor")
                if not bool(self.toolhead_sensor.runout_helper.filament_present):
                    final_check_failed.append("toolhead_sensor")
                
                if final_check_failed:
                    raise Exception(f"Final load checks failed for sensors: {', '.join(final_check_failed)}")     

                logging.info(f'Spool unit {self.name} loaded successfully.')
                self.set_status(STATUS_LOADED)

            except Exception as e:
                self.handle_exception(e, "spool_load", pause_on_error=True)
            
        else:
            e = f"Spool {self.name} is not in IDLE state, cannot perform load."
            self.handle_exception(e, "spool_load", pause_on_error=True)           

    def spool_eject(self):
        if self.status in [STATUS_IDLE, STATUS_LOADED, STATUS_ERROR, STATUS_RUNOUT]:
            if self.status == STATUS_LOADED or self.status == STATUS_ERROR or self.status == STATUS_RUNOUT:
                try:
                    self.spool_unload()
                except Exception as e:
                    self.handle_exception(e, "spool_eject", pause_on_error=True)
            try:
                self.set_status(STATUS_EJECTING)
                self.stepper_helper.do_set_position(0.)
                self.stepper_helper.do_move(movepos=-20.0) # Move out of gears
                self.toolhead.wait_moves()

                def check_for_runout(eventtime):
                    if self.status != STATUS_EMPTY:
                        self.set_status(STATUS_ERROR)
                        self.gcode.respond_info(f"Runout is not detected on spool {self.name} during ejection!\n"
                                                f"Please manually clear the filament path.")
                    return self.reactor.NEVER
                
                runtime = 20.0
                self.eject_timer = self.reactor.register_timer(check_for_runout, self.reactor.monotonic() + runtime)
                self.hbridge_motor.scheduled_motion(pwm_value=-1.0, runtime=runtime) # Reverse spool to eject

            except Exception as e:
                self.handle_exception(e, "spool_eject", pause_on_error=True)
    
    def runout_event_handler(self, eventtime):
        if self.status == STATUS_EJECTING:
            self.hbridge_motor.scheduled_motion(pwm_value=0)
            if self.eject_timer:
                self.reactor.unregister_timer(self.eject_timer)
                self.eject_timer = None
            self.set_status(STATUS_EMPTY)
        else:
            # Check if this tool was actually loaded
            if self.status != STATUS_LOADED:
                return  # This is not the actual spool engaged on this extruder - ignore
            # Check printer state
            idle_timeout = self.printer.lookup_object('idle_timeout')
            is_printing = idle_timeout.get_status(self.reactor.monotonic())['state'] == "Printing"

            if is_printing:
                # Check if this toolhead is active
                if self.toolhead.get_extruder() == self.extruder:   # This toolhead is active
                    self.set_status(STATUS_RUNOUT)
                    self.gcode.respond_info(f"Runout detected on spool unit {self.name} during printing.\n"
                                            f"Executing runout gcode and pausing print.")
                    pause_prefix = ""
                    if self.runout_pause:
                        pause_resume = self.printer.lookup_object('pause_resume')
                        pause_resume.send_pause_command()
                        pause_prefix = "PAUSE\n"
                        self.reactor.pause(eventtime + 0.5)
                        if self.runout_eject:
                            self.spool_eject()
                    self._exec_gcode(pause_prefix, self.runout_gcode)
            else:                                               # Not active
                self.set_status(STATUS_EMPTY)

    def _exec_gcode(self, prefix, template):
        try:
            self.gcode.run_script_from_command(prefix + template.render() + "\nM400")
        except Exception as e:
            self.handle_exception(e, "_exec_gcode", pause_on_error=True)
            
    def motion_extraction(self, *args):
        print_time = args[1]
        runtime = args[2] + args[3] + args[4]
        move_distance = (1/2 * args[13] * (args[2]**2) + args[12] * args[3] + 1/2 * args[13] * (args[4]**2))* args[8]
        cruise_v = args[12]
        move_dir = args[8]

        if self.enable_tracking:
            if abs(move_distance) >= self.assist_threshold:
                self.moved_distance = 0.
                self._motion_planning(cruise_v, move_dir, print_time, runtime)
            else:
                self.moved_distance += move_distance
                if abs(self.moved_distance) >= self.assist_threshold:
                    self._assist_threshold_motion_planning(self.moved_distance)
                    self.moved_distance = 0.

    def _motion_planning(self, cruise_v, move_dir, print_time, runtime):
        if move_dir == 1:
            if self.assist_forward:
                pwm_value = move_dir * self._get_scaling_factor(cruise_v)
                self.hbridge_motor.scheduled_async_motion(pwm_value, print_time, runtime)
        else:
            if self.assist_reverse:
                pwm_value = move_dir * self._get_scaling_factor(cruise_v)
                self.hbridge_motor.scheduled_async_motion(pwm_value, print_time, runtime)

    def _assist_threshold_motion_planning(self, distance):
        pass # Calculate required pwm and scaling to perform distance move for assist motions             

    def _get_scaling_factor(self, cruise_v):
        if self.spool_measurement:
            scaling_factor = 1.0
            return scaling_factor
        return 1.0

    def schedule_async_motion(self, cruise_v, move_dir, print_time, runtime):
        pwm_value = 1.0 * move_dir
        self.hbridge_motor.scheduled_async_motion(pwm_value, print_time, runtime)

    def estimate_spool_diameter(self):
        if not self.spool_measurement or not hasattr(self, 'vl6180'):
            return None
        
        try:
            valid_measurements = []
            
            # Take multiple measurements
            for i in range(self.measurement_samples):
                range_value, error_description = self.vl6180.single_shot_measurement()
                
                if error_description is not None:
                    logging.warning(f'VL6180 {self.name} measurement {i+1}/{self.measurement_samples} error: {error_description}')
                    continue
                
                calculated_diameter = (self.vl6180_center_distance - range_value) * 2
                
                # Only include measurements within valid range
                if self.spool_diameter[0] <= calculated_diameter <= self.spool_diameter[1]:
                    valid_measurements.append(calculated_diameter)
            
            if not valid_measurements:
                logging.warning(f'No valid measurements obtained for spool {self.name}')
                return None
            
            # Calculate mean of all valid measurements
            mean_diameter = sum(valid_measurements) / len(valid_measurements)
            
            # Clamp to valid range and return as integer
            spool_diameter = max(self.spool_diameter[0], 
                               min(mean_diameter, self.spool_diameter[1]))
            
            return int(spool_diameter)
            
        except Exception as e:
            logging.error(f'Failed to measure spool diameter for {self.name}: {e}')
            return None

    def estimate_spool_content(self, density=None, width=None, filament_diameter=None, packing_efficiency=None):
        # Get current spool diameter
        current_diameter = self.estimate_spool_diameter()
        if current_diameter is None:
            return None
        
        # Use provided parameters or fall back to config defaults
        if density is None:
            density = self.material_density
        if width is None:
            width = self.spool_width
        if filament_diameter is None:
            filament_diameter = self.filament_diameter
        if packing_efficiency is None:
            packing_efficiency = self.packing_efficiency
        
        # Calculate volumes in mm³
        core_radius = self.spool_diameter[0] / 2.0  # spool_diameter_min
        current_radius = current_diameter / 2.0
        
        # Volume of filament wound area = (current cylinder volume) - (core cylinder volume)
        core_volume = math.pi * (core_radius ** 2) * width
        current_volume = math.pi * (current_radius ** 2) * width
        wound_area_volume = current_volume - core_volume
        
        if wound_area_volume <= 0:
            return 0.0, 0.0
        
        # Apply packing efficiency to account for voids between filament layers
        actual_filament_volume_mm3 = wound_area_volume * packing_efficiency
        
        # Calculate filament length
        filament_cross_section = math.pi * ((filament_diameter / 2.0) ** 2)
        filament_length_mm = actual_filament_volume_mm3 / filament_cross_section
        filament_length_m = filament_length_mm / 1000.0
        
        # Calculate mass
        filament_volume_cm3 = actual_filament_volume_mm3 / 1000.0  # Convert mm³ to cm³
        mass_g = filament_volume_cm3 * density
        
        return filament_length_m, mass_g

    def cmd_SPOOL_MOTION_CONTROL(self, gcmd):
        # Parse parameters
        enable = gcmd.get('ENABLE', None)
        assist_forward = gcmd.get('ASSIST_FORWARD', None)
        assist_reverse = gcmd.get('ASSIST_REVERSE', None)
        threshold = gcmd.get('THRESHOLD', None)
        
        # Update tracking state
        if enable is not None:
            self.enable_tracking = gcmd.get_int('ENABLE', self.enable_tracking, minval=0, maxval=1)
            gcmd.respond_info(f"Spool tracking {'enabled' if self.enable_tracking else 'disabled'}")
        
        # Update assist directions
        if assist_forward is not None:
            self.assist_forward = gcmd.get_int('ASSIST_FORWARD', self.assist_forward, minval=0, maxval=1)
            gcmd.respond_info(f"Forward assist {'enabled' if self.assist_forward else 'disabled'}")
            
        if assist_reverse is not None:
            self.assist_reverse = gcmd.get_int('ASSIST_REVERSE', self.assist_reverse, minval=0, maxval=1)
            gcmd.respond_info(f"Reverse assist {'enabled' if self.assist_reverse else 'disabled'}")
        
        # Update threshold
        if threshold is not None:
            self.assist_threshold = gcmd.get_float('THRESHOLD', self.assist_threshold, minval=0.0)
            gcmd.respond_info(f"Assist threshold set to {self.assist_threshold:.1f}mm")
        
        # If no parameters provided, show current status
        if all(param is None for param in [enable, assist_forward, assist_reverse, threshold]):
            status_msg = (f"Motion control for {self.name}: "
                         f"tracking={'enabled' if self.enable_tracking else 'disabled'}, "
                         f"forward_assist={'enabled' if self.assist_forward else 'disabled'}, "
                         f"reverse_assist={'enabled' if self.assist_reverse else 'disabled'}, "
                         f"assist_threshold={self.assist_threshold:.1f}mm")
            gcmd.respond_info(status_msg)

    def cmd_MEASURE_SPOOL_DIAMETER(self, gcmd):
        if not self.spool_measurement or not hasattr(self, 'vl6180'):
            gcmd.error("Spool measurement or VL6180 sensor not enabled.")
            return
        diameter = self.estimate_spool_diameter()
        if diameter is None:
            gcmd.error("Failed to measure spool diameter - sensor error or no spool detected")
        else:
            gcmd.respond_info(f"Estimated spool diameter: {diameter:.2f}mm")

    def cmd_ESTIMATE_SPOOL_CONTENT(self, gcmd):
        # Parse optional parameters
        density = gcmd.get_float('rho', self.material_density, minval=0.1)
        width = gcmd.get_float('w', self.spool_width, minval=1.0)
        filament_dia = gcmd.get_float('d', self.filament_diameter, minval=0.1)
        packing_eff = gcmd.get_float('eta', self.packing_efficiency, minval=0.5, maxval=1.0)
        
        content = self.estimate_spool_content(density, width, filament_dia, packing_eff)
        
        if content is None:
            gcmd.respond_info("Failed to estimate spool content - unable to measure spool diameter")
            return
        
        length_m, mass_g = content
        gcmd.respond_info(
            f"Estimated spool content: {length_m:.2f}m / {mass_g:.1f}g\n"
            f"Parameters: D={self.estimate_spool_diameter():.2f}mm, w={width:.1f}mm, d={filament_dia:.2f}mm, rho={density:.2f}g/cm³,  eta={packing_eff:.2f}"
            )
    
    def get_name(self):
        return self.name

    def get_extruder_name(self):
        return self.extruder_name
    
    def get_status(self, eventtime):
        return {
            'status': self.status,
            'synced': self.synced
        }

def load_config_prefix(config):
    return SpoolUnit(config)