# Additional module to support toolchanger functionality
#
# Copyright (C) 2025 Eytecz Engineering
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

class ToolchangerHelper:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        
        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        # Register g-code commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('TOOL_REQUEST', self.cmd_TOOL_REQUEST,
                                    desc="Request a tool change to the specified tool.")
    
    def handle_connect(self):
        # Step 1: Collect all tool objects
        self.tools = {}
        for tool_name, tool_obj in self.printer.lookup_objects('tool'):
            tool = tool_name.split()[1]
            self.tools[tool] = {
                'tool': tool_obj,
                'extruder_name': tool_obj.extruder_name,
                'toolhead_detect': None,
                'spool_unit': None,
                'filament_hub': None,
                'dock': None
            }
        
        # Step 2: Match toolhead_detect by extruder_name
        for detect_name, toolhead_detect in self.printer.lookup_objects('toolhead_detect'):
            extruder_name = toolhead_detect.get_extruder_name()
            for tool, tool_contents in self.tools.items():
                if tool_contents['extruder_name'] == extruder_name:
                    tool_contents['toolhead_detect'] = toolhead_detect
                    logging.info(f"Matched {detect_name} to tool {tool}")
        
        # Step 3: Match spool_unit by name
        for spool_name, spool_unit in self.printer.lookup_objects('spool_unit'):
            tool = spool_name.split()[1]
            if tool in self.tools:
                self.tools[tool]['spool_unit'] = spool_unit
                self.tools[tool]['filament_hub'] = spool_unit.filament_hub
                logging.info(f"Matched {spool_name} to tool {tool}")

        # Step 4: Match dock by extruder_name
        for dock_name, dock in self.printer.lookup_objects('dock'):
            extruder_name = dock.extruder_name  # Direct attribute, not a method
            for tool, tool_contents in self.tools.items():
                if tool_contents['extruder_name'] == extruder_name:
                    tool_contents['dock'] = dock
                    logging.info(f"Matched {dock_name} to tool {tool}")
    
    def cmd_TOOL_REQUEST(self, gcmd):
            pass

def load_config(config):
    return ToolchangerHelper(config)