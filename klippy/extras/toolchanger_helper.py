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
        self.reactor = self.printer.get_reactor()
        
        # Read config
        self.docking_axis = config.getboolean('docking_axis', True)
        self.restore_axes = config.getlist(
            'restore_axes', ['y', 'x', 'z', 'docking_axis'])

        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        # Register g-code commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('TOOL_REQUEST', self.cmd_TOOL_REQUEST,
                                    desc="Request a tool change to the specified tool")
        self.gcode.register_command('INIT_DOCKS', self.cmd_INIT_DOCKS,
                                    desc="Initialize all dock units")
        self.gcode.register_command('SET_DOCKING_AXIS_MODE', self.cmd_SET_DOCKING_AXIS_MODE,
                                    desc="Set docking axis mode")
        self.gcode.register_command('RESTORE_TOOLCHANGER', self.cmd_RESTORE_TOOLCHANGER,
                                    desc="Restore toolchanger to ready state after error")
        
        # Optional gcodes
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.pre_dock_gcode = None
        if config.get('pre_dock_gcode', None) is not None:
            self.pre_dock_gcode = gcode_macro.load_template(config, 'pre_dock_gcode', '')
        self.pre_change_gcode = None
        if config.get('pre_change_gcode', None) is not None:
            self.pre_change_gcode = gcode_macro.load_template(config, 'pre_change_gcode', '')
        self.post_undock_gcode = None
        if config.get('post_undock_gcode', None) is not None:
            self.post_undock_gcode = gcode_macro.load_template(config, 'post_undock_gcode', '')
        self.post_change_gcode = None
        if config.get('post_change_gcode', None) is not None:
            self.post_change_gcode = gcode_macro.load_template(config, 'post_change_gcode', '')
        self.post_replace_gcode = None
        if config.get('post_replace_gcode', None) is not None:
            self.post_replace_gcode = gcode_macro.load_template(config, 'post_replace_gcode', '')
    
    def handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.toolchanger = self.printer.lookup_object('toolchanger')
        self.docking_axis = self.printer.lookup_object('docking_axis', None)

        # Step 1: Collect all tool objects
        self.tools = {}
        for tool_name, tool_obj in self.printer.lookup_objects('tool'):
            tool = tool_name.split()[1]
            self.tools[tool] = {
                'tool': tool_obj,
                'tool_number': tool_obj.tool_number,
                'extruder_name': tool_obj.extruder_name,
                'toolhead_detect': None,
                'spool_unit': None,
                'filament_hub': None,
                'dock_unit': None
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
        self.dock_units = {}
        for dock_name, dock_unit in self.printer.lookup_objects('dock_unit'):
            extruder_name = dock_unit.extruder_name  # Direct attribute, not a method
            for tool, tool_contents in self.tools.items():
                if tool_contents['extruder_name'] == extruder_name:
                    tool_contents['dock_unit'] = dock_unit
                    logging.info(f"Matched {dock_name} to tool {tool}")
                self.dock_units[dock_name] = dock_unit
    
    def cmd_TOOL_REQUEST(self, gcmd):
        tool_param = gcmd.get('TOOL', None)
        if tool_param is None:
            tool_param = gcmd.get('TOOL_NUMBER', None)
        # Normalize tool parameter: accept "t0", "T0", "tool0", "Tool0", or just "0"
        if tool_param.lower().startswith('t'):
            tool_number = int(tool_param[1:])
        elif tool_param.lower().startswith('tool'):
            tool_number = int(tool_param[4:])
        else:
            tool_number = int(tool_param)

        try:      
            self.handle_tool_request(tool_number)
        except Exception as e:
            error_msg = f"Tool request failed: {str(e)}"
            logging.error(error_msg)
            
            # Set toolchanger to ERROR status so it won't cancel the print
            self.toolchanger.status = 'error'
            self.toolchanger.error_message = error_msg
            
            self.gcode.run_script_from_command("PAUSE")
            gcmd.respond_info(error_msg)

    def cmd_SET_DOCKING_AXIS_MODE(self, gcmd):
        mode = gcmd.get('MODE', None).lower()
        if mode is None:
            raise gcmd.error("MODE parameter is required")
        if self.docking_axis is None:
            raise gcmd.error("No docking_axis object found")
        if mode not in ['static', 'balanced', 'minimize_z']:
            raise gcmd.error("Invalid MODE parameter, must be one of: static, balanced, minimize_z")
        
        for _, dock_unit in self.dock_units.items():
            dock_unit.axis_mode = mode
            gcmd.respond_info(f"Set docking axis mode for dock unit {dock_unit.name} to {mode}")

    def get_tool_name(self, tool_number):
        for tool, tool_contents in self.tools.items():
            if tool_contents['tool_number'] == tool_number:
                return tool
        return None

    def get_active_dock(self):
        active_docks = []
        for _, dock_unit in self.dock_units.items():
            status = dock_unit.get_status(self.reactor.monotonic())['status']
            if status == 'engaged':
                active_docks.append(dock_unit)
        if len(active_docks) == 0:
            return None
        elif len(active_docks) == 1:
            return active_docks[0]
        else:
            raise Exception("Multiple dock units are currently engaged, manual intervention required.")

    def handle_tool_request(self, tool_number):
        # Get the tool name
        tool_name = self.get_tool_name(tool_number)
        if tool_name is None:
            raise Exception(f"Tool number {tool_number} not found")
        tool = self.tools[tool_name]

        # Check if the spool_unit can be used
        tool_state = tool['spool_unit'].get_status(self.reactor.monotonic())['status']
        if tool_state not in ['loaded', 'idle']:
            raise Exception(f"Spool unit for tool {tool_name} not ready (status: {tool_state}).")

        # Check if the toolhead can be used
        dock_state = tool['dock_unit'].get_status(self.reactor.monotonic())['status']
        if dock_state not in ['engaged', 'docked']:
            raise Exception(f"Dock unit for tool {tool_name} not ready (status: {dock_state}).")

        # Perform required actions
        if tool_state == 'loaded' and dock_state == 'engaged': # Already usable
            logging.info(f"Tool {tool_name} already loaded and docked, no action required")
            return
        
        elif tool_state == 'loaded' and dock_state == 'docked': # Change toolhead
            logging.info(f"Changing to tool {tool_name}")
            active_dock = self.get_active_dock()
            if active_dock is not None:
                active_dock.retract_filament()
            tool['dock_unit'].save_init_pos()
            if self.pre_dock_gcode is not None:
                try:
                    self.gcode.run_script_from_command(self.pre_dock_gcode.render() + "\nM400")
                except Exception as e:
                    raise Exception(f"Pre-dock gcode failed: {str(e)}")
            tool['dock_unit'].save_init_pos(save_axes=['x', 'y', 'e'])
            if active_dock is not None:
                if active_dock.filament_cutter and active_dock.filament_sensor.runout_helper.filament_present:
                    active_dock.cut_filament(restore_pos=False)
                active_dock.dock_toolhead(restore_pos=False)
            tool['dock_unit'].undock_toolhead(restore_pos=True, restore_axes=self.restore_axes)
            if tool['dock_unit'].filament_cutter:
                tool['dock_unit'].finalize_load_to_cutter()
            tool['dock_unit'].unretract_filament()
            if self.post_undock_gcode is not None:
                try:
                    self.gcode.run_script_from_command(self.post_undock_gcode.render() + "\nM400")
                except Exception as e:
                    raise Exception(f"Post-undock gcode failed: {str(e)}")
                
        elif tool_state == 'idle' and dock_state == 'engaged': # Change filament
            logging.info(f"Changing filament to tool {tool_name}")
            active_dock = self.get_active_dock()
            if active_dock is not None:
                active_dock.retract_filament()
            tool['dock_unit'].save_init_pos()
            if self.pre_change_gcode is not None:
                try:
                    self.gcode.run_script_from_command(self.pre_change_gcode.render() + "\nM400")
                except Exception as e:
                    raise Exception(f"Pre-change gcode failed: {str(e)}")
            tool['dock_unit'].save_init_pos(save_axes=['x', 'y', 'e'])
            if active_dock is not None:
                if active_dock.filament_cutter and active_dock.filament_sensor.runout_helper.filament_present:
                    active_dock.cut_filament(restore_pos=True, restore_axes=self.restore_axes)
            tool['spool_unit'].spool_load()
            if tool['dock_unit'].filament_cutter:
                tool['dock_unit'].finalize_load_to_cutter()
            tool['dock_unit'].unretract_filament()
            if self.post_change_gcode is not None:
                try:
                    self.gcode.run_script_from_command(self.post_change_gcode.render() + "\nM400")
                except Exception as e:
                    raise Exception(f"Post-change gcode failed: {str(e)}")
                
        elif tool_state == 'idle' and dock_state == 'docked': # Replace toolhead and filament
            logging.info(f"Replacing toolhead and filament to tool {tool_name}")
            active_dock = self.get_active_dock()
            if active_dock is not None:
                active_dock.retract_filament()
            tool['dock_unit'].save_init_pos()
            if self.pre_change_gcode is not None:
                try:
                    self.gcode.run_script_from_command(self.pre_change_gcode.render() + "\nM400")
                except Exception as e:
                    raise Exception(f"Pre-change gcode failed: {str(e)}")
            tool['spool_unit'].spool_load()
            if self.pre_dock_gcode is not None:
                try:
                    self.gcode.run_script_from_command(self.pre_dock_gcode.render() + "\nM400")
                except Exception as e:
                    raise Exception(f"Pre-dock gcode failed: {str(e)}")
            if tool['dock_unit'].filament_cutter:
                tool['dock_unit'].finalize_load_to_cutter()
            tool['dock_unit'].save_init_pos(save_axes=['x', 'y', 'e'])
            if active_dock is not None:
                if active_dock.filament_cutter and active_dock.filament_sensor.runout_helper.filament_present:
                    active_dock.cut_filament(restore_pos=False)
                active_dock.dock_toolhead(restore_pos=False)
            tool['dock_unit'].undock_toolhead(restore_pos=True, restore_axes=self.restore_axes)
            if tool['dock_unit'].filament_cutter:
                tool['dock_unit'].finalize_load_to_cutter()
            tool['dock_unit'].unretract_filament()
            if self.post_replace_gcode is not None:
                try:
                    self.gcode.run_script_from_command(self.post_replace_gcode.render() + "\nM400")
                except Exception as e:
                    raise Exception(f"Post-replace gcode failed: {str(e)}")

    def cmd_INIT_DOCKS(self, gcmd):
        for _, dock_unit in self.dock_units.items():
            dock_unit.initialize_dock_unit()
            gcmd.respond_info(f"Initialized dock unit: {dock_unit.name}, to status: {dock_unit.status}.")
        
    def cmd_RESTORE_TOOLCHANGER(self, gcmd):
        try:
            self.restore_toolchanger()
        except Exception as e:
            raise gcmd.error("Restore toolchanger failed: %s" % str(e))
        
    def restore_toolchanger(self):
        try:
            self.toolchanger.status = 'ready'
        except Exception:
            raise


def load_config(config):
    return ToolchangerHelper(config)