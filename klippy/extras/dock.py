# Toolhead docking module - manages complete pickup/dropoff sequences
# Can optionally use docking_axis for dynamic dock positioning
#
# Copyright (C) 2025 Eytecz
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

class Dock:
    def __init__(self, config):
        self.config = config
        self.printer= config.get_printer()

        # Read config section
        self.name = config.get_name().split()[1]
        self.extruder_name = config.get('extruder', 'extruder')
        self.docking_axis = config.getboolean('docking_axis', False)
        self.toolhead_detect = config.getboolean('toolhead_detect', False)

        self.docking_speed = config.getfloat('docking_speed', 20., above=0.)
        self.engage_speed = config.getfloat('engage_speed', 10., above=0.)
        self.disengage_speed = config.getfloat('disengage_speed', 10., above=0.)

        # Docked positions define the 'parked' position of the toolhead when docked
        self.docked_position_x = config.getfloat('docked_position_x')
        self.docked_position_y = config.getfloat('docked_position_y')
        self.docked_offset_z = config.getfloat('docked_offset_z')

        # Docking sequence distances/positions
        self.safe_position_y = config.getfloat(
            'safe_position_y', 80., above=0.)       # Position to clear all toolheads
        self.safe_offset_z = config.getfloat(
            'safe_offset_z', 20., above=0.)         # Offset above dock to enter safely
        self.slide_distance_y = config.getfloat(
            'slide_distance_y', 3., above=0.)       # Distance to slide horizontally into dock
        self.disengage_offset_z = config.getfloat(
            'disengage_offset_z', 8., above=0.)     # Offset to drop towards dock when disengaging
        self.disengage_offset_y = config.getfloat(
            'disengage_offset_y', 20., above=0.)    # Offset to back away from dock when disengaging

        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.docking_gcode = gcode_macro.load_template(config, 'docking_gcode', '')
        self.undocking_gcode = gcode_macro.load_template(config, 'undocking_gcode', '')
        
        self.filament_cutter = False
        if config.get('cut_gcode', None) is not None:
            self.cut_gcode = gcode_macro.load_template(config, 'cut_gcode', '')
            self.filament_cutter = True
        
        # Filament cutter position when the filament is cut
        if self.filament_cutter:
            self.cutter_position_x = config.getfloat('cutter_position_x', None)
            self.cutter_position_y = config.getfloat('cutter_position_y', None)
            self.cutter_position_z = config.getfloat('cutter_position_z', None)
            self.cutter_retract_y = config.getfloat('cutter_retract_y', None)

        # Initial state

        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        # Register reuired objects
        self.gcode = self.printer.lookup_object('gcode')

        # Register g-code commands
    
    def handle_connect(self):
        self.extruder = self.printer.lookup_object(self.extruder_name)
        if self.docking_axis is True:
            self.docking_axis = self.printer.lookup_object('docking_axis')
        if self.toolhead_detect is True:
            self.toolhead_detect = None
            for instance in self.printer.lookup_objects('toolhead_detect'):
                if instance[1].get_extruder_name() == self.extruder_name:
                    self.toolhead_detect = instance[1]
                    break
            if self.toolhead_detect is None:
                raise self.config.error("Missing required toolhead_detect object")
        
    
def load_config_prefix(config):
    return Dock(config)