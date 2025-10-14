# Automated control of spool motion functionality
#
# Copyright (C) 2025 Eytecz
#
# This file may be distributed under the terms of the GNU GPLv3 license

import math
import logging

class SpoolMotionControl:
    def __init__(self, config):
        self.config = config
        self.printer = self.config.get_printer()
        self.reactor = self.printer.get_reactor()

        # MCU Tracking
        self.all_mcus = [m for n, m in self.printer.lookup_objects(module='mcu')]
        self.mcu = self.all_mcus[0]

        # Initial state
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
        self.spool_diameter = [config.getfloat('spool_diameter_min', 80.0),
                               config.getfloat('spool_diameter_max', 200.0)]
        self.poll_interval = config.getfloat('poll_interval', 0.5, minval=0.01)
        self.assist_threshold = config.getfloat('assist_threshold', 20.0, minval=0.0)
        self.hbridge_motor_name = config.get('hbridge_motor', None)
        self.stepper_name = config.get('stepper', None)
        self.vl6180_name = config.get('vl6180_sensor', None)
        if self.vl6180_name:
            self.vl6180_center_distance = config.getfloat('vl6180_center_distance', 125.0, minval=0.0)


        # Register g-code commands
        self.gcode.register_mux_command('SPOOL_MOTION_CONTROL', 'SPOOL', self.name,
                                        self.cmd_SPOOL_MOTION_CONTROL,
                                        desc="Control spool motion functionality.")
        self.gcode.register_mux_command('MEASURE_SPOOL_DIAMETER', 'SPOOL', self.name,
                                        self.cmd_MEASURE_SPOOL_DIAMETER,
                                        desc="Measure and report the current spool diameter.")

    def handle_ready(self):
        pass

    def handle_connect(self):
        # Connect hbridge_motor modules 
        if self.hbridge_motor_name:
            for hbridge_motor in self.printer.lookup_objects('hbridge_motor'):
                name = hbridge_motor[1].get_name()
                if name == self.hbridge_motor_name:
                    self.hbridge_motor = hbridge_motor[1]
            if self.hbridge_motor is None:
                raise self.config.error("Could not find hbridge_motor '%s'" % hbridge_motor_name)
        else:
            raise self.config.error("Missing required 'hbridge_motor' config option")

        # Connect tracked manual_stepper
        for manual_stepper in self.printer.lookup_objects('manual_stepper'):
            name = manual_stepper[1].get_steppers()[0].get_name()
            if name == self.stepper_name:
                self.stepper = manual_stepper[1]
                self.enable_tracking = self.config.getboolean('enable_tracking', True)
                logging.info(f'stepper {name} connected')
        if self.stepper is None:
            raise self.config.error("Could not find stepper '%s'" % self.stepper_name)  

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

        # Intercept stepper trapq_append and wipe_trapq function to extract motion data
        self.trapq_append_original = self.stepper.trapq_append
        self.stepper.trapq_append = self._trapq_append_intercept
        self.wipe_trapq_original = self.stepper.motion_queuing.wipe_trapq
        self.stepper.motion_queuing.wipe_trapq = self._wipe_trapq_intercept
   
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
            gcmd.respond_info(f"Spool Motion Control Status:")
            gcmd.respond_info(f"  Tracking: {'enabled' if self.enable_tracking else 'disabled'}")
            gcmd.respond_info(f"  Forward assist: {'enabled' if self.assist_forward else 'disabled'}")
            gcmd.respond_info(f"  Reverse assist: {'enabled' if self.assist_reverse else 'disabled'}")
            gcmd.respond_info(f"  Assist threshold: {self.assist_threshold:.1f}mm")
            gcmd.respond_info(f"  Moved distance: {self.moved_distance:.1f}mm")

    def cmd_MEASURE_SPOOL_DIAMETER(self, gcmd):
        if not self.spool_measurement or not hasattr(self, 'vl6180'):
            gcmd.respond_info("Spool measurement or VL6180 sensor not enabled.")
            return
        diameter = self.estimate_spool_diameter()
        if diameter is None:
            gcmd.respond_info("Failed to measure spool diameter - sensor error or no spool detected")
        else:
            gcmd.respond_info(f"Estimated spool diameter: {diameter:.2f} mm")

    def _trapq_append_intercept(self, *args):
        logging.info(f'_trapq_append_intercept with args: {args}')
        self.trapq_append_original(*args)
        self._motion_extraction(*args)

    def _wipe_trapq_intercept(self, *args):
        self.wipe_trapq_original(*args)
        self.hbridge_motor.abort_async_motion()

    def _motion_extraction(self, *args):
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
            range_value, error_description = self.vl6180.single_shot_measurement()
            
            if error_description is not None:
                logging.warning(f'VL6180 {self.name} measurement error: {error_description}')
                if error_description == "Max Convergence":
                    logging.info(f'No spool detected for spool slot {self.name}')
                return None
            
            calculated_diameter = (self.vl6180_center_distance - range_value) * 2
            spool_diameter = max(self.spool_diameter[0], 
                               min(calculated_diameter, self.spool_diameter[1]))
            
            return spool_diameter
            
        except Exception as e:
            logging.error(f'Failed to measure spool diameter for {self.name}: {e}')
            return None


def load_config_prefix(config):
    return SpoolMotionControl(config)