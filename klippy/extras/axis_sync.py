# Synchronize manual_stepper objects with extruders
#
# Copyright (C) 2025 Eytecz
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

class AxisSync:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        
        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        
        # Register commands
        self.gcode.register_command('SYNC_EXTRUDER_STEPPER', self.cmd_SYNC_EXTRUDER_STEPPER,
                                  desc=self.cmd_SYNC_EXTRUDER_STEPPER_help)

    def handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')

    def sync_stepper_to_extruder(self, stepper_name, extruder_name=None):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        motion_queuing = self.printer.lookup_object('motion_queuing')
        
        # Get manual stepper
        stepper = self.printer.lookup_object(stepper_name, None)
        if stepper is None or stepper.__class__.__name__ != 'ManualStepper':
            raise self.printer.command_error("'%s' is not a manual stepper" % stepper_name)
        
        # Unsync if no extruder specified
        if not extruder_name:
            orig_trapq = getattr(stepper, '_sync_orig_trapq', None)
            stepper.rail.set_trapq(orig_trapq)
            if hasattr(stepper, '_sync_orig_trapq'):
                delattr(stepper, '_sync_orig_trapq')
            motion_queuing.check_step_generation_scan_windows()
            return
        
        # Get extruder
        extruder = self.printer.lookup_object(extruder_name, None)
        if extruder is None or not hasattr(extruder, 'get_trapq'):
            raise self.printer.command_error("'%s' is not a valid extruder" % extruder_name)
        
        # Store original trapq and sync
        if not hasattr(stepper, '_sync_orig_trapq'):
            stepper._sync_orig_trapq = stepper.get_trapq()
        stepper.do_set_position(extruder.last_position)
        stepper.rail.set_trapq(extruder.get_trapq())
        motion_queuing.check_step_generation_scan_windows()

    cmd_SYNC_EXTRUDER_STEPPER_help = "Sync manual stepper with extruder motion queue"
    def cmd_SYNC_EXTRUDER_STEPPER(self, gcmd):
        stepper = gcmd.get('STEPPER')
        extruder = gcmd.get('EXTRUDER', None)
        
        try:
            self.sync_stepper_to_extruder(stepper, extruder)
            if extruder:
                gcmd.respond_info("Stepper '%s' synced to '%s'" % (stepper, extruder))
            else:
                gcmd.respond_info("Stepper '%s' unsynced" % stepper)
        except Exception as e:
            raise self.gcode.error("SYNC_EXTRUDER_STEPPER error: %s" % str(e))

def load_config(config):
    return AxisSync(config)