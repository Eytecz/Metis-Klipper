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
        self.printer.register_event_handler("stepper_enable:motor_off", self.stepper_enable_motor_off)

        # Read config section
        self.stepper_names = config.getlist('steppers', ['stepper_a', 'stepper_b'])
        self.homing_velocity = config.getfloat('homing_speed', 50., above=0.)
        self.velocity = config.getfloat('velocity', 100., above=0.)
        self.accel = config.getfloat('accel', 1500., minval=0.)
        self.pos_min = config.getfloat('position_min', 38.)
        self.pos_max = config.getfloat('position_max', 450.)
        self.pos_endstop = config.getfloat('position_endstop', 38.)
        self.pos_park = config.getfloat('position_park', 400.)

        # Internal state
        self.commanded_pos = 0.
        self.homed = False

    def handle_connect(self):
        self.axis_sync = self.printer.lookup_object('axis_sync')
        stepper_objects = {}
        for stepper in self.printer.lookup_objects('manual_stepper'):
            name = stepper[1].get_steppers()[0].get_name()
            if name in self.stepper_names:
                stepper_objects[name] = stepper[1]
        
        self.steppers = []
        for name in self.stepper_names:
            if name in stepper_objects:
                self.steppers.append(stepper_objects[name])
            else:
                raise self.config.error("Could not find stepper '%s'" % name)

    def stepper_enable_motor_off(self):
        se = self.printer.lookup_object('stepper_enable')
        for s in self.steppers:
            name = s.get_steppers()[0].get_name()
            state = se.lookup_enable(name).is_enabled
            if not state:
                self.homed = False

    def do_enable(self, enable):
        for s in self.steppers:
            s.do_enable(enable)

    def do_set_position(self, setpos):
        for s in self.steppers:
            s.do_set_position(setpos)
        self.commanded_pos = setpos
    
    def do_move(self, movepos, speed, accel, sync=True):
        if not self.homed:
            raise self.printer.command_error("Must home docking axis before move")
        # Check if move is in bounds
        if movepos < self.pos_endstop or movepos > self.pos_max:
            raise self.printer.command_error("Move to %.3f out of bounds (min: %.3f, max: %.3f)"
                                             % (movepos, self.pos_endstop, self.pos_max))
        for s in self.steppers[:-1]:
            s.do_move(movepos, speed, accel, sync=False)
        self.steppers[-1].do_move(movepos, speed, accel, sync=sync)
        self.commanded_pos = movepos

    def do_homing_move(self, movepos, speed, accel, triggered, check_trigger):
        for s in self.steppers:
            if not s.can_home:
                raise self.printer.command_error("Stepper '%s' cannot home"
                                                 % s.get_steppers()[0].get_name())
            s.do_set_position(self.pos_max)
        phoming = self.printer.lookup_object('homing')

        # Collect endstops to stop on any endstop
        endstops = []
        for s in self.steppers:
            s.homing_accel = accel
            endstops.extend(s.rail.get_endstops())
        
        # Sync all steppers to the first stepper
        for s in self.steppers[1:]:
            self.axis_sync.sync_stepper_to_manual_stepper(
                s.get_steppers()[0].get_name(),
                self.steppers[0].get_steppers()[0].get_name()
            )
        
        # Perform synced homing move
        try:
            pos = [movepos, 0., 0., 0.]
            phoming.manual_home(self.steppers[0], endstops, pos, speed, triggered, check_trigger)
        
            # Unsync all steppers
            for s in self.steppers[1:]:
                self.axis_sync.sync_stepper_to_manual_stepper(
                    s.get_steppers()[0].get_name(),
                    None
                )

            # Perform seperate homing moves on all steppers to align them
            for s in self.steppers:
                s.do_set_position(self.pos_max)
                endstops = s.rail.get_endstops()
                s.homing_accel = accel
                pos = [movepos, 0., 0., 0.]
                phoming.manual_home(s, endstops, pos, speed, triggered, check_trigger)
                s.do_set_position(self.pos_endstop)
            
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.wait_moves()
            self.commanded_pos = self.pos_endstop
            self.homed = True

        except Exception as e:
            for s in self.steppers[1:]:
                self.axis_sync.sync_stepper_to_manual_stepper(
                    s.get_steppers()[0].get_name(),
                    None
                )
            raise self.printer.command_error("Homing move failed: %s" % str(e))

    def get_position(self):
        return self.commanded_pos
                 
class DockingAxis:
    def __init__(self, config):
        self.config = config
        self.printer= config.get_printer()

        self.stepper = StepperHelper(config)

        # Register g-code commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('RUN', self.cmd_RUN,
                                    desc = "Execute a method command within Python context")
        self.gcode.register_command('DOCKING_AXIS', self.cmd_DOCKING_AXIS,
                                    desc = "Command the docking axis module")
        
    def cmd_RUN(self, gcmd):
        method_name = gcmd.get('METHOD')
        
        # Get the method from stepper helper
        if not hasattr(self.stepper, method_name):
            raise gcmd.error(f"Method '{method_name}' not found in StepperHelper")
        
        method = getattr(self.stepper, method_name)
        
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
        except Exception as e:
            raise gcmd.error(f"Error executing {method_name}: {str(e)}")

    def cmd_DOCKING_AXIS(self, gcmd):
        enable = gcmd.get_int('ENABLE', None)
        if enable is not None:
            self.stepper.do_enable(bool(enable))
        setpos = gcmd.get_float('SET_POSITION', None)
        if setpos is not None:
            self.stepper.do_set_position(setpos)
        speed = gcmd.get_float('SPEED', self.stepper.velocity, above=0.)
        accel = gcmd.get_float('ACCEL', self.stepper.accel, minval=0.)
        homing_move = gcmd.get_int('STOP_ON_ENDSTOP', 0)
        if homing_move:
            movepos = gcmd.get_float('MOVE')
            if ((self.stepper.pos_min is not None and movepos < self.stepper.pos_min)
                or (self.stepper.pos_max is not None and movepos > self.stepper.pos_max)):
                raise gcmd.error("Move out of range")
            speed = gcmd.get_float('SPEED', self.stepper.homing_velocity, above=0.)
            self.stepper.do_homing_move(movepos, speed, accel,
                                        homing_move > 0, abs(homing_move) == 1)
        elif gcmd.get_float('MOVE', None) is not None:
            movepos = gcmd.get_float('MOVE')
            if ((self.stepper.pos_min is not None and movepos < self.stepper.pos_min)
                or (self.stepper.pos_max is not None and movepos > self.stepper.pos_max)):
                raise gcmd.error("Move out of range")
            sync = gcmd.get_int('SYNC', 1)
            self.stepper.do_move(movepos, speed, accel, sync)
        
    def get_status(self, eventtime):
        return {
            'position': self.stepper.get_position(),
            'homed': bool(self.stepper.homed)
        }


def load_config(config):
    return DockingAxis(config)