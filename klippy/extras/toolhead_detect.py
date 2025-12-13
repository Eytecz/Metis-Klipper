# Code to detect toolchanger toolhead engagement on shuttle
#
# Copyright (C) 2025 Eytecz Engineering
#
# This file may be distributed under the terms of the GNU GPLv3 license

import logging

class ToolheadDetect:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        # Read config
        self.name = config.get_name().split()[1]
        pin = config.get('pin')
        self.debounce_time = config.getfloat('debounce_time', 0.5, minval=0.)
        self.lost_timeout = config.getfloat('lost_timeout', 5.0, minval=0.)
        self.pause_on_lost = config.getboolean('pause_on_lost', True)
        self.extruder_name = config.get('extruder', 'extruder')
        self.state_logging = config.getboolean('state_logging', True)
        
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.lost_gcode = None
        if self.pause_on_lost or config.get('lost_gcode', None) is not None:
            self.lost_gcode = gcode_macro.load_template(config, 'lost_gcode', '')

        # Initial state
        self.toolhead_present = False
        self.pin_state = False
        self.pending_state = None
        self.confirm_timer = None
        self.lost_timer = None
        self.enabled = True
        self.action_enabled = False
        self.callbacks = []

        # Register required objects
        buttons = self.printer.load_object(config, 'buttons')
        buttons.register_debounce_button(pin, self._trigger_callback, config)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        # Register g-code commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command('QUERY_TOOLHEAD_ENGAGEMENT', 'TOOLHEAD', self.name,
                                        self.cmd_QUERY_TOOLHEAD_ENGAGEMENT,
                                        desc="Query toolhead engagement state")
        self.gcode.register_mux_command('ENABLE_TOOLHEAD_DETECT', 'TOOLHEAD', self.name,
                                        self.cmd_ENABLE_TOOLHEAD_DETECT,
                                        desc="Enable toolhead engagement detection")
        self.gcode.register_mux_command('ENABLE_ACTION_TOOLHEAD_DETECT', 'TOOLHEAD', self.name,
                                        self.cmd_ENABLE_ACTION_TOOLHEAD_DETECT,
                                        desc="Enable action on toolhead engagement detection")

    def handle_ready(self):
        pass
    
    def _trigger_callback(self, eventtime, state):
        if state == self.pin_state:
            return
        self.pin_state = state

        # If state changed from pending, cancel confirmation
        if self.pending_state is not None and state != self.pending_state:
            if self.confirm_timer:
                self.reactor.unregister_timer(self.confirm_timer)
                self.confirm_timer = None
            if self.state_logging:
                logging.info(f"Toolhead {self.name} state bounced, ignoring")
        
        def _confirm_state_change(eventtime):
            if self.pending_state == state:
                self.pending_state = None
                if self.toolhead_present == state:
                    if self.state_logging:
                        logging.info(f"Toolhead {self.name} state unchanged after confirmation")
                    return self.reactor.NEVER
                self.toolhead_present = state
                if self.toolhead_present:
                    if self.state_logging:
                        logging.info(f"Toolhead {self.name} engaged confirmed")
                    self._engage_event_handler()
                else:
                    if self.state_logging:
                        logging.info(f"Toolhead {self.name} disengaged confirmed")
                    self._disengage_event_handler()
            self.confirm_timer = None
            return self.reactor.NEVER

        # Start confirmation timer
        self.pending_state = state
        self.confirm_timer = self.reactor.register_timer(
            _confirm_state_change, self.reactor.monotonic() + self.debounce_time
        )
        if self.state_logging:
            logging.info(f"Toolhead {self.name} state change detected, confirming...")

    def _engage_event_handler(self):
        if not self.enabled:
            return
        for callback in self.callbacks:
            callback(True)
        if not self.action_enabled:
            return
        # Cancel lost timer if running
        if self.lost_timer is not None:
            self.reactor.unregister_timer(self.lost_timer)
            self.lost_timer = None
            if self.state_logging:
                logging.info(f"Toolhead {self.name} re-engaged, cancelling lost timer")
        

    def _disengage_event_handler(self):
        if not self.enabled:
            return
        for callback in self.callbacks:
            callback(False)
        if not self.action_enabled:
            return
        if self.pause_on_lost or self.lost_gcode is not None:
            # Check printer state, return if not printing
            idle_timeout = self.printer.lookup_object('idle_timeout')
            is_printing = idle_timeout.get_status(self.reactor.monotonic())['state'] == "Printing"
            if not is_printing:
                if self.state_logging:
                    logging.info(f"Not pausing print due to toolhead {self.name} disengagement (not printing)")
                return

            # Verify active extruder is this toolhead
            toolhead = self.printer.lookup_object('toolhead')
            active_extruder = toolhead.get_extruder().get_name()
            if active_extruder != self.extruder_name:
                if self.state_logging:
                    logging.info(f"Not pausing print due to toolhead {self.name} disengagement (active extruder is {active_extruder})")
                return
            
            # Define pause print callback
            def _pause_print(eventtime):
                logging.info(f"Toolhead {self.name} lost timeout reached, executing lost gcode and/or pausing...")
                self.lost_timer = None
                pause_prefix = ""
                if self.pause_on_lost:
                    pause_resume = self.printer.lookup_object('pause_resume')
                    pause_resume.send_pause_command()
                    pause_prefix = "PAUSE\n"
                    self.reactor.pause(eventtime + 0.5)
                self._exec_gcode(pause_prefix, self.lost_gcode)
                return self.reactor.NEVER
            
            # Check for existing lost timer
            if self.lost_timer is not None:
                self.reactor.unregister_timer(self.lost_timer)
                self.lost_timer = None

            # Start lost timer
            self.lost_timer = self.reactor.register_timer(
                _pause_print, self.reactor.monotonic() + self.lost_timeout
            )
            if self.state_logging:
                logging.info(f"Toolhead {self.name} disengaged, starting lost timer...")

    def _exec_gcode(self, prefix, template):
        try:
            self.gcode.run_script(prefix + template.render() + "\nM400")
        except Exception as e:
            raise self.printer.command_error(f"Failed to execute toolhead {self.name} pause_on_lost gcode: {str(e)}")
    
    def cmd_QUERY_TOOLHEAD_ENGAGEMENT(self, gcmd):
        try:
            state = self.query_state_blocking()
            gcmd.respond_info(f"Toolhead {self.name} engaged: {state}")
        except Exception as e:
            raise self.printer.command_error(str(e))
        
    def cmd_ENABLE_TOOLHEAD_DETECT(self, gcmd):
        enable = bool(gcmd.get_int('ENABLE', 1))
        self.enable(enable)
        gcmd.respond_info(f"Toolhead {self.name} detection {'enabled' if enable else 'disabled'}")

    def cmd_ENABLE_ACTION_TOOLHEAD_DETECT(self, gcmd):
        enable = bool(gcmd.get_int('ENABLE', 1))
        self.action_enabled = enable
        gcmd.respond_info(f"Toolhead {self.name} detection action {'enabled' if enable else 'disabled'}")

    def register_callback(self, callback):
        self.callbacks.append(callback)

    def enable(self, state=True):
        if self.enabled == state:
            return
        self.enabled = state
        if self.state_logging:
            logging.info(f"Toolhead {self.name} detection {'enabled' if state else 'disabled'}")

    def query_state(self):
        return self.toolhead_present
    
    def query_state_blocking(self):
        if self.pending_state is None:
            return bool(self.toolhead_present)
        
        # Wait until confirmation timer completes
        curtime = self.reactor.monotonic()
        endtime = curtime + 5.  # 5 second timeout

        while self.pending_state is not None:
            curtime = self.reactor.monotonic()
            if curtime > endtime:
                raise Exception(f"Timeout waiting for toolhead {self.name} state confirmation")
            self.reactor.pause(curtime + 0.05)

        return bool(self.toolhead_present)
    
    def query_pending_state(self):
        return bool(self.pending_state)

    def get_enabled(self):
        return self.enabled
    
    def get_action_enabled(self):
        return self.action_enabled

    def get_name(self):
        return self.name

    def get_extruder_name(self):
        return self.extruder_name

    def get_status(self, eventtime):
        return {
            'toolhead_engaged': bool(self.toolhead_present),
            'toolhead_pending_state': bool(self.pending_state is not None),
            'toolhead_detect_enabled': bool(self.enabled),
        }        

def load_config_prefix(config):
    return ToolheadDetect(config)