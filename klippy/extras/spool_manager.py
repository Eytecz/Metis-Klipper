# Higher level spool management functions for AFC-TC system
#
# Copyright (C) 2025 Eytecz Engineering
#
# This file may be distributed under the terms of the GNU GPLv3 license

import logging

class SpoolManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()        

        # Initial state
        self.state = None
        self.status_led = False

        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        # Register required objects
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_macro = self.printer.load_object(config, 'gcode_macro')
       
        # Read config section
        if config.getboolean('status_leds', True):
            self.status_led = True
            self.frame_rate = config.getfloat('frame_rate', default=24, minval=1, maxval=60)
            self.state_layer_empty = config.get('state_layer_empty', None)
            self.state_layer_ready = config.get('state_layer_ready', None)
            self.state_layer_changing = config.get('state_layer_changing', None)
            self.state_layer_loaded = config.get('state_layer_loaded', None)
            self.state_layer_error = config.get('state_layer_error', None)

        


        # Register commands
    

    def handle_connect(self):
        for object in self.printer.lookup_objects('spool_unit'):
            name = object.name
            self.spool_units[name] = object

    def handle_ready(self):
        pass
    
def load_config(config):
  return SpoolManager(config)