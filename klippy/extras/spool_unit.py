# Collection of spool unit related functions
#
# Copyright (C) 2025 Eytecz
#
# This file may be distributed under the terms of the GNU GPLv3 license

import math
import logging
import configparser
from configfile import ConfigWrapper
from .led_effect import ledEffect

STATUS_INITIALIZING     = 'initializing'    # State detection to find initial state
STATUS_EMPTY            = 'empty'           # No filament present in the pre-gate sensor
STATUS_HOMING           = 'homing'          # Automated loading assist of filament until post-gear sensor
STATUS_IDLE             = 'idle'            # Filament available in spool unit extruder gear
STATUS_LOADING          = 'loading'         # Automated loading as effect of request event until toolhead
STATUS_LOADED           = 'loaded'          # Filament available in toolhead and ready for printing
STATUS_UNLOADING        = 'unloading'       # Automated unloading as effect of exchange/remove request event
STATUS_RUNOUT           = 'runout'          # Filament runout detected during printing on pre-gate sensor
STATUS_ERROR            = 'error'           # Exception occurred during homing/loading/unloading/

class InsertHelper:
    def __init__(self, config, spool_unit):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.spool_unit = spool_unit

        # Read config
        self.insert_load = config.getboolean('load_on_insert', True)
        insert_pin = config.get('insert_pin', None)

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
        self.min_event_systime = self.reactor.monotonic() + 2.

    def _event_handler(self, eventtime, state):
        self.note_filament_present(eventtime, state)

    def _insert_event_handler(self, eventtime):
        if self.insert_load and not self.homing_state:
            try:
                self.homing_state = True
                self.spool_unit.stepper_helper.stepper.do_set_position(0.)
                self.spool_unit.stepper_helper.stepper.do_homing_move(movepos=500, speed=25, accel=1000, triggered=True, check_trigger=True)
                self.spool_unit.stepper_helper.stepper.do_set_position(0.)
                self.homing_state = False
            except Exception as e:
                self.homing_state = False
                logging.error(f"Error during insert event handling: {e}")

    def _runout_event_handler(self, eventtime):
        logging.info(f"Runout event triggered at {eventtime}")
    
    def query_endstop(self):
        return self.filament_present

    def note_filament_present(self, eventtime, state):
        if eventtime < self.min_event_systime or not self.sensor_enabled:
            logging.debug(f"Ignored insert/runout event due to debounce/startup guard: eventtime={eventtime}, min_event_systime={self.min_event_systime}")
            return
        if state == self.filament_present:
            return
        self.filament_present = state
        self.min_event_systime = eventtime + 2.
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
        
    def handle_connect(self):
        stepper_name = self.config.get('stepper', None)
        for manual_stepper in self.printer.lookup_objects('manual_stepper'):
            name = manual_stepper[1].get_steppers()[0].get_name()
            if name == stepper_name:
                self.stepper = manual_stepper[1]
        if self.stepper is None:
            raise self.config.error("Could not find stepper '%s'" % stepper_name)
        
        # Intercept stepper trapq_append and wipe_trapq function to extract motion data
        self.trapq_append_original = self.stepper.trapq_append
        self.stepper.trapq_append = self._trapq_append_intercept
        self.wipe_trapq_original = self.stepper.motion_queuing.wipe_trapq
        self.stepper.motion_queuing.wipe_trapq = self._wipe_trapq_intercept

    def _trapq_append_intercept(self, *args):
        logging.info(f'_trapq_append_intercept with args: {args}')
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
                logging.info(f'Query endstop for stepper {name} at time {print_time}: {state}')
                break
            state = None
        if state is None:
            logging.warning(f'No endstop found for stepper {self.stepper.get_steppers()[0].get_name()}')
            return None
        return bool(state)
    
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
            STATUS_ERROR:           config.get('state_layer_error', LAYER_ERROR)
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
    def __init__(self, config):
        self.config = config
        self.printer = self.config.get_printer()
        self.reactor = self.printer.get_reactor()

        # MCU Tracking
        self.all_mcus = [m for n, m in self.printer.lookup_objects(module='mcu')]
        self.mcu = self.all_mcus[0]

        # Initial state
        self.state = None
        self.spool_measurement = False
        self.assist_forward = False
        self.assist_reverse = True
        self.enable_tracking = True
        self.moved_distance = 0.

        # Register handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        # Register required objects
        self.gcode = self.printer.lookup_object('gcode')      

        # Read config section
        self.name = config.get_name().split()[1]
        self.spool_diameter = [config.getfloat('spool_diameter_min', 90.0),
                               config.getfloat('spool_diameter_max', 200.0)]
        self.poll_interval = config.getfloat('poll_interval', 0.5, minval=0.01)
        self.assist_threshold = config.getfloat('assist_threshold', 20.0, minval=0.0)
        self.hbridge_motor_name = config.get('hbridge_motor', None)
        
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

        stepper_name = config.get('stepper', None)
        if stepper_name:
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
        self.gcode.register_mux_command('QUERY_STEPPER_ENDSTOP', 'SPOOL', self.name,
                                        self.cmd_QUERY_STEPPER_ENDSTOP,
                                        desc="Query the endstop state of the specified stepper.")
        self.gcode.register_mux_command('SET_STATUS', 'SPOOL', self.name,
                                        self.cmd_SET_STATUS,
                                        desc="Set the status of the spool unit for LED indication.")

    def handle_ready(self):
        self.reactor.register_timer(self.initialize_spool_unit, self.reactor.monotonic() + 2)

    def handle_connect(self):
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
            self.toolhead = self.printer.lookup_object('toolhead')
        
    def cmd_SET_STATUS(self, gcmd):
        self.led_helper.set_state(gcmd.get('STATUS'))

    def cmd_QUERY_STEPPER_ENDSTOP(self, gcmd):
        state = self.stepper_helper.query_endstop()
        if state is None:
            gcmd.error(f"Could not query endstop for stepper")
        else:
            gcmd.respond_info(f"Endstop state for stepper: {'triggered' if state else 'open'}")

    def initialize_spool_unit(self, eventtime):
        self.led_helper.set_state(STATUS_INITIALIZING)
        return self.reactor.NEVER
    
    def motion_extraction(self, *args):
        print_time = args[1]
        runtime = args[2] + args[3] + args[4]
        move_distance = (1/2 * args[13] * (args[2]**2) + args[12] * args[3] + 1/2 * args[13] * (args[4]**2))* args[8]
        cruise_v = args[12]
        move_dir = args[8]
        logging.info(f'_motion_extraction: print_time={print_time}, runtime={runtime}, move_distance={move_distance}, cruise_v={cruise_v}, move_dir={move_dir}')

        if self.enable_tracking:
            logging.info('tracking is True, proceeding with move motion_planning')
            if abs(move_distance) >= self.assist_threshold:
                logging.info(f'move_distance={move_distance} >= self.assist_threshold={self.assist_threshold}')
                self.moved_distance = 0.
                self._motion_planning(cruise_v, move_dir, print_time, runtime)
            else:
                logging.info(f'self.moved_distance={self.moved_distance} += {move_distance}')
                self.moved_distance += move_distance
                logging.info(f'self.moved_distance={self.moved_distance}')
                if abs(self.moved_distance) >= self.assist_threshold:
                    logging.info(f'move_distance={move_distance} >= self.assist_threshold={self.assist_threshold}')
                    self._assist_threshold_motion_planning(self.moved_distance)
                    self.moved_distance = 0.

    def _motion_planning(self, cruise_v, move_dir, print_time, runtime):
        logging.info(f'_motion_planning cruise_v={cruise_v}, move_dir={move_dir}, print_time={print_time}, runtime={runtime}')
        if move_dir == 1:
            logging.info(f'self.assist_forward={self.assist_forward}')
            if self.assist_forward:
                logging.info('_motion_planning assist_forward')
                pwm_value = move_dir * self._get_scaling_factor(cruise_v)
                self.hbridge_motor.scheduled_async_motion(pwm_value, print_time, runtime)
        else:
            logging.info(f'self.assist_reverse={self.assist_reverse}')
            if self.assist_reverse:
                logging.info('_motion_planning assist_reverse')
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

    def get_name(self):
        return self.name

    def get_status_leds(self):
        return self.status_leds

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
            gcmd.respond_info(f"Estimated spool diameter: {diameter:.2f} mm")

    def cmd_ESTIMATE_SPOOL_CONTENT(self, gcmd):
        # Parse optional parameters
        density = gcmd.get_float('DENSITY', self.material_density, minval=0.1)
        width = gcmd.get_float('WIDTH', self.spool_width, minval=1.0)
        filament_dia = gcmd.get_float('FILAMENT_DIAMETER', self.filament_diameter, minval=0.1)
        packing_eff = gcmd.get_float('PACKING_EFFICIENCY', self.packing_efficiency, minval=0.5, maxval=1.0)
        
        content = self.estimate_spool_content(density, width, filament_dia, packing_eff)
        
        if content is None:
            gcmd.respond_info("Failed to estimate spool content - unable to measure spool diameter")
            return
        
        length_m, mass_g = content
        gcmd.respond_info(f"Estimated spool content:")
        gcmd.respond_info(f"  Filament length: {length_m:.2f} meters")
        gcmd.respond_info(f"  Filament mass: {mass_g:.1f} grams")
        gcmd.respond_info(f"  Parameters used: density={density:.2f}g/cm³, width={width:.1f}mm, filament_dia={filament_dia:.2f}mm, packing_eff={packing_eff:.2f}")

def load_config_prefix(config):
    return SpoolUnit(config)