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

        # Initial state
        self.toolhead_present = False
        self.pin_state = False
        self.pending_state = None
        self.confirm_timer = None
        self.lost_timer = None
        self.debounce_time = config.getfloat('debounce_time', 0.5, minval=0.)
        self.lost_timeout = config.getfloat('lost_timeout', 5.0, minval=0.)
        self.pause_on_lost = config.getboolean('pause_on_lost', True)
        self.extruder_name = config.get('extruder', 'extruder')

        # Register required objects
        buttons = self.printer.load_object(config, 'buttons')
        buttons.register_debounce_button(pin, self._trigger_callback, config)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        # Register g-code commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command('QUERY_TOOLHEAD_ENGAGEMENT', 'TOOLHEAD', self.name,
                                        self.cmd_QUERY_TOOLHEAD_ENGAGEMENT,
                                        desc="Query toolhead engagement state")

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
            logging.info("Toolhead state bounced, ignoring")
        
        def _confirm_state_change(eventtime):
            if self.pending_state == state:
                self.pending_state = None
                if self.toolhead_present == state:
                    logging.info("Toolhead state unchanged after confirmation")
                    return self.reactor.NEVER
                self.toolhead_present = state
                if self.toolhead_present:
                    logging.info("Toolhead engaged confirmed")
                    self._engage_event_handler()
                else:
                    logging.info("Toolhead disengaged confirmed")
                    self._disengage_event_handler()
            self.confirm_timer = None
            return self.reactor.NEVER

        # Start confirmation timer
        self.pending_state = state
        self.confirm_timer = self.reactor.register_timer(
            _confirm_state_change, self.reactor.monotonic() + self.debounce_time
        )
        logging.info("Toolhead state change detected, confirming...")

    def _engage_event_handler(self):
        pass

    def _disengage_event_handler(self):
        if self.pause_on_lost:
            toolhead = self.printer.lookup_object('toolhead')
            active_extruder = toolhead.get_extruder().get_name()
            if active_extruder == self.extruder_name:
                logging.info("Pausing print due to toolhead disengagement")
                # Add more code here

    
    def cmd_QUERY_TOOLHEAD_ENGAGEMENT(self, gcmd):
        try:
            state = self.query_state_blocking()
            gcmd.respond_info("Toolhead engaged: %s" % state)
        except Exception as e:
            raise self.printer.command_error(str(e))

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
                raise Exception("Timeout waiting for toolhead state confirmation")
            self.reactor.pause(curtime + 0.05)

        return bool(self.toolhead_present)
    
    def query_pending_state(self):
        return bool(self.pending_state)

    def get_name(self):
        return self.name

    def get_status(self, eventtime):
        return {
            'toolhead_engaged': bool(self.toolhead_present),
            'toolhead_pending_state': bool(self.pending_state is not None),
        }        

def load_config_prefix(config):
    return ToolheadDetect(config)