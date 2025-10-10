# Manual or automated control of a DC motor via Low-Voltage H-Bridge Drivers
#
# Copyright (C) 2025 Eytecz
#
# This file may be distributed under the terms of the GNU GPLv3 license

import logging
from . import output_pin
from bisect import insort

class HBridgeMotor:
    def __init__(self, config):
        self.config = config
        self.printer = self.config.get_printer()
        self.reactor = self.printer.get_reactor()

        # Initial state
        self.last_pwm_value = 0.
        self.start_timer = None
        self.stop_timer = None

        # Queue for async motions
        self.cmd_queue = []      # list of (print_time, pwm_value)
        self.queue_timer = None  # single timer processing the queue

        # Register handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.register_event_handler("gcode:request_restart", self.handle_restart)

        # Register required objects
        self.gcode = self.printer.lookup_object('gcode')

        # Read config section
        self.name = config.get_name().split()[1]
        self.max_power = config.getfloat('max_power', 1., above=0., maxval=1.)
        self.kick_start_time = config.getfloat('kick_start_time', 0.1, minval=0.)
        self.brake_time = config.getfloat('brake_time', 0.1, minval=0.)
        self.off_below = config.getfloat('off_below', 0., minval=0., maxval=1.)
        cycle_time = config.getfloat('cycle_time', 0.1, minval=0.01)
        hardware_pwm = config.getboolean('hardware_pwm', False)
        dir_invert = config.getboolean('dir_invert', False)

        # Setup pin objects
        ppins = self.printer.lookup_object('pins')
        pin_pairs = [('in1_pin', config.get('in2_pin') if dir_invert else config.get('in1_pin')),
                     ('in2_pin', config.get('in1_pin') if dir_invert else config.get('in2_pin'))]

        for attr, pin_name in pin_pairs:
            pin = ppins.setup_pin('pwm', pin_name)
            pin.setup_max_duration(0.)
            pin.setup_cycle_time(cycle_time, hardware_pwm)
            pin.setup_start_value(0., 0.)
            setattr(self, attr, pin)
        
        self.mcu = self.in1_pin.get_mcu()

        enable_pin = config.get('enable_pin', None)
        if enable_pin:
            self.enable_pin = ppins.setup_pin('digital_out', enable_pin)
            self.enable_pin.setup_max_duration(0.)

        # Create g-code request queue
        if self.in1_pin.get_mcu() != self.in2_pin.get_mcu():
            raise config.error("in1_pin and in2_pin must be on the same MCU")
        self.motion_request = output_pin.GCodeRequestQueue(config, self.in1_pin.get_mcu(),
                                                self.execute_controlled_drive)

        # Register g-code commands
        self.cmd_HBRIDGE_MOTOR_help = "Set the speed of the H-Bridge motor. Usage: HBRIDGE_MOTOR MOTOR=<motor> VALUE=<-1..1> RUNTIME=<seconds> DELAY=<seconds>"
        self.gcode.register_mux_command('HBRIDGE_MOTOR', 'MOTOR', self.name,
                                        self.cmd_HBRIDGE_MOTOR, desc=self.cmd_HBRIDGE_MOTOR_help)

    def handle_ready(self):
        pass

    def handle_connect(self):
        pass

    def handle_restart(self, print_time):
        self.motion_request.queue_gcode_request(0.0)
    
    def execute_controlled_drive(self, print_time, pwm_value):
        if abs(pwm_value) < self.off_below:
            pwm_value = 0.0        
        
        # Kick-start
        if self.kick_start_time > 0 and pwm_value != 0.0:
            kick_required = (
                self.last_pwm_value == 0.0 or                                   # Starting from stop
                pwm_value * self.last_pwm_value < 0 or                          # Changing direction
                (abs(pwm_value) > abs(self.last_pwm_value)                      # Increasing speed
                    and abs(abs(pwm_value) - abs(self.last_pwm_value)) > 0.2)
            )
            if kick_required:
                mode = 'forward' if pwm_value > 0 else 'reverse'
                self.set_drv_mode(print_time, mode, self.max_power)
                return "delay", self.kick_start_time
        
        # Braking
        if self.brake_time > 0 and self.last_pwm_value != 0.0 and pwm_value == 0.0:
            self.set_drv_mode(print_time, 'brake')
            return "delay", self.brake_time

        # Normal forward/reverse/sleep
        mode = 'forward' if pwm_value > 0 else 'reverse' if pwm_value < 0 else 'sleep'
        self.set_drv_mode(print_time, mode, abs(pwm_value))   
    
    def set_drv_mode(self, print_time, mode, pwm_value=0.0):
        valid_modes = {"sleep", "forward", "reverse", "brake", "coast"}
        if mode not in valid_modes:
            raise ValueError(f"Invalid driver mode: {mode}")

        pwm_value = max(0., min(self.max_power, pwm_value))
    
        if mode == 'sleep':
            self.enable_pin.set_digital(print_time, 0)
            self.in1_pin.set_pwm(print_time, 0.0)
            self.in2_pin.set_pwm(print_time, 0.0)
            self.last_pwm_value = 0.0
            return
        
        self.enable_pin.set_digital(print_time, 1)
        
        if mode == 'forward':
            self.in1_pin.set_pwm(print_time, pwm_value)
            self.in2_pin.set_pwm(print_time, 0.0)
            self.last_pwm_value = pwm_value
                   
        elif mode == 'reverse':
            self.in1_pin.set_pwm(print_time, 0.0)
            self.in2_pin.set_pwm(print_time, pwm_value)
            self.last_pwm_value = -pwm_value
            
        elif mode == 'brake':
            self.in1_pin.set_pwm(print_time, 1.0)
            self.in2_pin.set_pwm(print_time, 1.0)
            self.last_pwm_value = 0.0
            
        elif mode == 'coast':
            self.in1_pin.set_pwm(print_time, 0.0)
            self.in2_pin.set_pwm(print_time, 0.0)
            self.last_pwm_value = 0.0   

    def scheduled_motion(self, pwm_value, runtime=None, eventtime=None):
        def start_motion(eventtime):
            self.motion_request.queue_gcode_request(pwm_value)

            if self.stop_timer:
                self.reactor.unregister_timer(self.stop_timer)
                self.stop_timer = None
                
            if runtime is not None:
                def end_motion(eventtime):
                    self.motion_request.queue_gcode_request(0.0)
                    return self.reactor.NEVER
                stop_time = self.reactor.monotonic() + runtime
                self.stop_timer = self.reactor.register_timer(end_motion, stop_time)
            return self.reactor.NEVER

        if eventtime is None:
            start_motion(self.reactor.monotonic())
        else:
            self.start_timer = self.reactor.register_timer(start_motion, eventtime)

    def scheduled_async_motion(self, pwm_value, print_time, runtime=None):
        insort(self.cmd_queue, (print_time, pwm_value))

        if runtime is not None:
            stop_time = print_time + runtime - 1e-6
            insort(self.cmd_queue, (stop_time, 0.0))

        if not self.queue_timer:
            self._start_next()

    def abort_async_motion(self):
        if self.queue_timer or self.cmd_queue:
            current_pt = self.get_print_time(self.reactor.monotonic()) + self.mcu.min_schedule_time()
            self.motion_request.send_async_request(0.0, current_pt)

        self.cmd_queue.clear()
        if self.queue_timer:
            self.reactor.update_timer(self.queue_timer, self.reactor.NEVER)
            self.queue_timer = None

    def _start_next(self):
        if not self.cmd_queue:
            self.queue_timer = None
            return

        pt, pwm = self.cmd_queue.pop(0)
        sys_time = self.get_sys_time(pt) - self.mcu.min_schedule_time()

        def _runner(eventtime):
            self.motion_request.send_async_request(pwm, pt)
            self.queue_timer = None
            self._start_next()
            return self.reactor.NEVER

        self.queue_timer = self.reactor.register_timer(_runner, sys_time)

    def get_print_time(self, systime):
        print_time = self.mcu.estimated_print_time(systime)
        return print_time
    
    def get_sys_time(self, print_time):
        sys_time = self.mcu._clocksync.estimate_clock_systime(self.mcu.print_time_to_clock(print_time))
        return sys_time

    def get_name(self):
        return self.name
    
    def cmd_HBRIDGE_MOTOR(self, gcmd):
        value = gcmd.get_float('VALUE', 0., minval=-1., maxval=1.)
        runtime = gcmd.get_float('RUNTIME', None, minval=0.)
        delay = gcmd.get_float('DELAY', None)
        eventtime = None if delay is None else self.reactor.monotonic() + delay
        self.scheduled_motion(value, runtime, eventtime)
  
def load_config_prefix(config):
    return HBridgeMotor(config)
