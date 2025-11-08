# Collection of spool unit related functions
#
# Copyright (C) 2025 Eytecz Engineering
#
# This file may be distributed under the terms of the GNU GPLv3 license

import logging

class FilamentHub:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        # Read config
        self.name = config.get_name().split()[1]
        switch_pin = config.get('switch_pin')
        self.debounce_time = config.getfloat('debounce_time', 0.5)

        # Initial state
        self.filament_present = False
        self.min_event_systime = self.reactor.NEVER

        # Register required objects
        buttons = self.printer.load_object(config, 'buttons')
        buttons.register_debounce_button(switch_pin, self._event_handler, config)

    def _event_handler(self, eventtime, state):
        if eventtime < self.min_event_systime:
            return
        if state == self.filament_present:
            return
        self.filament_present = state
        self.min_event_systime = eventtime + self.debounce_time
        if self.filament_present:
            self._insert_event_handler(eventtime)
        else:
            self._runout_event_handler(eventtime)

    def _insert_event_handler(self, eventtime):
        logging.info(f"Insert event triggered on filament hub {self.name} at {eventtime}")
    
    def _runout_event_handler(self, eventtime):
        logging.info(f"Runout event triggered on filament hub {self.name} at {eventtime}")
    
    def query_endstop(self):
        return self.filament_present
    
    def get_name(self):
        return self.name
        
    def get_status(self, eventtime):
        return {'filament_present': bool(self.filament_present)}

def load_config_prefix(config):
    return FilamentHub(config)