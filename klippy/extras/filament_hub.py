# Collection of spool unit related functions
#
# Copyright (C) 2025 Eytecz Engineering
#
# This file may be distributed under the terms of the GNU GPLv3 license

import logging

BUFFER_STATE_DISABLED   = 'disabled'
BUFFER_STATE_TRAILING   = 'trailing'        # Compressed buffer
BUFFER_STATE_ADVANCING  = 'advancing'       # Extended buffer

class FilamentHub:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        # Read config
        self.name = config.get_name().split()[1]
        hub_pin = config.get('hub_pin')
        advance_pin = config.get('advance_pin', None)
        trailing_pin = config.get('trailing_pin', None)
        self.multiplier_high = config.getfloat('multiplier_high', 1.1, minval=1.0)
        self.multiplier_low = config.getfloat('multiplier_low', 0.9, minval=0.0, maxval=1.0)
        self.debounce_time = config.getfloat('debounce_time', 2.0)

        # Initial state
        self.filament_present = False
        self.advance_state = False
        self.trailing_state = False
        self.loaded_spool_unit = None
        self.enable_buffer = False
        self.buffer_state = BUFFER_STATE_DISABLED
        self.min_event_systime = {
            'hub': self.reactor.NEVER,
            'advance': self.reactor.NEVER,
            'trailing': self.reactor.NEVER
        }

        # Register required objects
        buttons = self.printer.load_object(config, 'buttons')
        buttons.register_debounce_button(hub_pin, self._hub_callback, config)
        if advance_pin:
            buttons.register_debounce_button(advance_pin, self._advance_callback, config)
        if trailing_pin:
            buttons.register_debounce_button(trailing_pin, self._trailing_callback, config)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def handle_ready(self):
        self.min_event_systime = {
            'hub': self.reactor.monotonic() + self.debounce_time,
            'advance': self.reactor.monotonic() + self.debounce_time,
            'trailing': self.reactor.monotonic() + self.debounce_time
        }

    def _hub_callback(self, eventtime, state):
        if state == self.filament_present:
            return
        self.filament_present = state
        if eventtime < self.min_event_systime['hub']:
            return
        self.min_event_systime['hub'] = eventtime + self.debounce_time
        if self.filament_present:
            self._insert_event_handler(eventtime)
        else:
            self._runout_event_handler(eventtime)

    def _insert_event_handler(self, eventtime):
        logging.info(f"Insert event triggered on filament hub {self.name} at {eventtime}")
    
    def _runout_event_handler(self, eventtime):
        logging.info(f"Runout event triggered on filament hub {self.name} at {eventtime}")
    
    def _advance_callback(self, eventtime, state):
        if state == self.advance_state:
            return
        self.advance_state = state
        if eventtime < self.min_event_systime['advance']:
            return
        self.min_event_systime['advance'] = eventtime + self.debounce_time
        logging.info(f"Advance event triggered on filament hub {self.name} at {eventtime}, state: {state}")
    
    def _trailing_callback(self, eventtime, state):
        if state == self.trailing_state:
            return
        self.trailing_state = state
        if eventtime < self.min_event_systime['trailing']:
            return
        self.min_event_systime['trailing'] = eventtime + self.debounce_time
        logging.info(f"Trailing event triggered on filament hub {self.name} at {eventtime}, state: {state}")

    def set_loaded_spool_unit(self, spool_unit):
        self.loaded_spool_unit = spool_unit
    
    def get_loaded_spool_unit(self):
        return self.loaded_spool_unit
    
    def get_bowden_length_to_toolhead(self):
        return self.bowden_length_to_toolhead

    def query_hub_endstop(self):
        return self.filament_present

    def query_trailing_endstop(self):
        return self.trailing_state
    
    def query_advancing_endstop(self):
        return self.advance_state
    
    def get_name(self):
        return self.name
        
    def get_status(self, eventtime):
        return {
            'filament_present': bool(self.filament_present),
            'advance_state': bool(self.advance_state),
            'trailing_state': bool(self.trailing_state),
            'buffer_state': self.buffer_state,
            'loaded_spool_unit': self.loaded_spool_unit.get_name() if self.loaded_spool_unit else None
        }

def load_config_prefix(config):
    return FilamentHub(config)