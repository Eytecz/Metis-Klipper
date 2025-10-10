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

        # Register g-code commands
        self.gcode.register_mux_command('SPOOL_MOTION_CONTROL', 'SPOOL', self.name,
                                        self.cmd_SPOOL_MOTION_CONTROL,
                                        desc="Control spool motion functionality.")

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
                    self.vl6180 = vl6180
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
        self.enable_tracking = not self.enable_tracking
        logging.info(f'self.enable_tracking set to {self.enable_tracking}')
    
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

def load_config_prefix(config):
    return SpoolMotionControl(config)


        

    
        

