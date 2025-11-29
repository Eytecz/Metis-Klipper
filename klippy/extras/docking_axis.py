# Dynamic docking axis support for Klipper
#
# Copyright (C) 2025 Eytecz
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

class StepperHelper:
    def __init__(self, config):
        self.config = config
        self.printer= config.get_printer()

        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        # Read config section
        self.stepper_names = config.getlist('steppers', ['stepper_a', 'stepper_b'])
        self.homing_velocity = config.getfloat('homing_speed', 10., above=0.)
        self.velocity = config.getfloat('velocity', 20., above=0.)
        self.accel = config.getfloat('accel', 1000., minval=0.)
        self.pos_min = config.getfloat('position_min', None)
        self.pos_max = config.getfloat('position_max', None)
        self.pos_endstop = config.getfloat('position_endstop', 0.)

        # Internal state
        self.commanded_pos = 0.
    
    def handle_connect(self):
        stepper_objects = {}
        for stepper in self.printer.lookup_objects('manual_stepper'):
            name = stepper[1].get_steppers()[0].get_name()
            if name in self.stepper_names:
                stepper_objects[name] = stepper[1]
        
        self.steppers = []
        for name in self.stepper_names:
            if name in stepper_objects:
                self.steppers.append(stepper_objects[name])
                logging.info(f"Found stepper '{name}' and added to steppers list")
            else:
                raise self.config.error("Could not find stepper '%s'" % name)
    
    def do_enable(self, enable):
        for s in self.steppers:
            s.do_enable(enable)
            logging.info(f"stepper {s.get_steppers()[0].get_name()} enable set to {enable}")


    def do_set_position(self, setpos):
        for s in self.steppers:
            s.do_set_position(setpos)
            logging.info(f"stepper {s.get_steppers()[0].get_name()} position set to {setpos}")
        self.commanded_pos = setpos
    
    def do_move(self, movepos, speed, accel, sync=True):
        # Check if move is in bounds
        if movepos < self.pos_endstop or movepos > self.pos_max:
            raise self.printer.command_error("Move to %.3f out of bounds (min: %.3f, max: %.3f)"
                                             % (movepos, self.pos_endstop, self.pos_max))
        for s in self.steppers[:-1]:
            s.do_move(movepos, speed, accel, sync=False)
            logging.info(f"stepper {s.get_steppers()[0].get_name()} moving to {movepos} at speed {speed} accel {accel} sync False")
        self.steppers[-1].do_move(movepos, speed, accel, sync=sync)
        logging.info(f"stepper {self.steppers[-1].get_steppers()[0].get_name()} moving to {movepos} at speed {speed} accel {accel} sync {sync}")
        self.commanded_pos = movepos

    def do_homing_move(self, movepos, speed, accel, triggered, check_trigger):
        for s in self.steppers:
            if not s.can_home:
                raise self.printer.command_error("Stepper '%s' cannot home"
                                                 % s.get_steppers()[0].get_name())
        phoming = self.printer.lookup_object('homing')
        for s in self.steppers:
            endstops = s.rail.get_endstops()
            pos = [movepos, 0., 0., 0.]
            s.homing_accel = accel
            phoming.manual_home(s, endstops, pos, speed, triggered, check_trigger)
            logging.info(f"stepper {s.get_steppers()[0].get_name()} homing to {movepos} at speed {speed} accel {accel}")
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.wait_moves()
        self.commanded_pos = movepos
                
class DockingAxis:
    def __init__(self, config):
        self.config = config
        self.printer= config.get_printer()

        self.stepper = StepperHelper(config)

        # Register g-code commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('RUN', self.cmd_RUN,
                                    desc = "Execute a method command within Python context")
        
    def cmd_RUN(self, gcmd):
        method_name = gcmd.get('METHOD')
        
        # Get the method from stepper helper
        if not hasattr(self.stepper, method_name):
            raise gcmd.error(f"Method '{method_name}' not found in StepperHelper")
        
        method = getattr(self.stepper, method_name)
        
        # Ensure it's callable
        if not callable(method):
            raise gcmd.error(f"'{method_name}' is not a callable method")
        
        # Parse arguments - convert strings to appropriate types
        kwargs = {}
        for param in gcmd.get_command_parameters():
            if param in ['METHOD']:  # Skip the method name itself
                continue
            value = gcmd.get(param)
            param_lower = param.lower()
            
            # Try to convert to appropriate type
            try:
                if value.lower() in ['true', '1']:
                    kwargs[param_lower] = True
                elif value.lower() in ['false', '0']:
                    kwargs[param_lower] = False
                else:
                    # Try float conversion
                    kwargs[param_lower] = gcmd.get_float(param)
            except:
                # Keep as string if conversion fails
                kwargs[param_lower] = value
        
        # Call the method
        try:
            method(**kwargs)
            gcmd.respond_info(f"Executed {method_name} with args: {kwargs}")
        except TypeError as e:
            raise gcmd.error(f"Invalid arguments for {method_name}: {str(e)}")
        except Exception as e:
            raise gcmd.error(f"Error executing {method_name}: {str(e)}")


def load_config(config):
    return DockingAxis(config)