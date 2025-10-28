# Higher level spool management functions for AFC-TC system
#
# Copyright (C) 2025 Eytecz Engineering
#
# This file may be distributed under the terms of the GNU GPLv3 license

import logging
import configparser
from configfile import ConfigWrapper
from .led_effect import ledEffect

class SpoolManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()        

        # Initial state
        self.state = None
        self.status_led = False
        self.spool_units = {}

        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        # Register required objects
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_macro = self.printer.load_object(config, 'gcode_macro')
        
        # Get configfile object for creating config wrappers
        self.configfile = self.printer.lookup_object('configfile')
       
        # Read config section
        if config.getboolean('status_leds', True):
            self.status_led = True
            self.frame_rate = config.getfloat('frame_rate', default=24, minval=1, maxval=60)
            self.state_layers = {
                'empty': config.get('state_layer_empty', None),
                'ready': config.get('state_layer_ready', None),
                'changing': config.get('state_layer_changing', None),
                'loaded': config.get('state_layer_loaded', None),
                'error': config.get('state_layer_error', None)
            }
        
        # Register commands
    
    def get_state_layer(self, state):
        return self.state_layers.get(state, None)

    def handle_connect(self):
        for object in self.printer.lookup_objects('spool_unit'):
            name = object[1].get_name()
            status_leds = object[1].get_status_leds()   
            self.spool_units[name] = object[1]
        logging.info("SpoolManager: Found the following spool units: %s", list(self.spool_units.keys()))

        for name in list(self.spool_units.keys()):
            self.create_state_led_configs(name)

    def handle_ready(self):
        pass
    
    def create_led_effect_config(self, effect_name, effect_config):
        # Create a new configparser with the led effect configuration
        fileconfig = configparser.RawConfigParser()
        section_name = f"led_effect {effect_name}"
        fileconfig.add_section(section_name)
        
        # Add configuration options
        for key, value in effect_config.items():
            fileconfig.set(section_name, key, str(value))
        
        # Create ConfigWrapper using the configfile's access tracking
        config_wrapper = ConfigWrapper(
            self.printer, 
            fileconfig, 
            self.configfile.validate.access_tracking,
            section_name
        )
        
        return config_wrapper
        
    def create_state_led_configs(self, spool_unit_name):
        if not self.status_led or spool_unit_name not in self.spool_units:
            return
        spool_unit = self.spool_units[spool_unit_name]
        led_pins = spool_unit.get_status_leds()

        configs = {}
    
        # Create config for each state that has a defined layer
        for state, state_layer in self.state_layers.items():
            if state_layer:
                effect_config = {
                    'auto_start': 'False',
                    'frame_rate': str(self.frame_rate),
                    'layers': state_layer,
                    'leds': led_pins
                }
                
                effect_name = f"{spool_unit_name}_{state}"
                config = self.create_led_effect_config(effect_name, effect_config)
                configs[state] = config
                
                try:
                    led_effect_obj = self.printer.load_object(config, 'led_effect')
                    full_name = f"led_effect {effect_name}"
                    self.printer.add_object(full_name, led_effect_obj)
                    ledEffect(config)
                except Exception as e:
                    logging.error("SpoolManager: Failed to create LED effect '%s': %s", effect_name, e)
        
        return configs

def load_config(config):
  return SpoolManager(config)