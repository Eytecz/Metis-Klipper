# VL6180 Proximity sensing module
#
# Copyright (C) 2023 Eytecz
#
# This file may be distributed under the terms of the GNU GPLv2 license.

from . import bus
import codecs
import logging

class EnableHelper:
  def __init__(self, mcu, pin_desc, cmd_queue=None, value=0):
      self.enable_pin = bus.MCU_bus_digital_out(mcu, pin_desc, cmd_queue, value)

  def init(self):
      mcu = self.enable_pin.get_mcu()
      reactor = mcu.get_printer().get_reactor()
      curtime = reactor.monotonic()
      print_time = mcu.estimated_print_time(curtime)

      # Ensure chip is powered off
      minclock = mcu.print_time_to_clock(print_time + mcu.min_schedule_time())
      self.enable_pin.update_digital_out(value=0, minclock=minclock)

      # Enable chip
      minclock = mcu.print_time_to_clock(print_time + 2 * mcu.min_schedule_time())
      self.enable_pin.update_digital_out(value=1, minclock=minclock)

      # Force a delay for any subsequent commands on the command queue
      waketime = curtime + 5 * mcu.min_schedule_time()
      reactor.pause(waketime)
 
class vl6180:
  IDENTIFICATION__MODEL_ID              = 0x0000
  IDENTIFICATION__MODEL_REV_MAJOR       = 0x0001
  IDENTIFICATION__MODEL_REV_MINOR       = 0x0002
  IDENTIFICATION__MODULE_REV_MAJOR      = 0x0003
  IDENTIFICATION__MODULE_REV_MINOR      = 0x0004
  IDENTIFICATION__DATE_HI               = 0x0006
  IDENTIFICATION__DATE_LO               = 0x0007
  IDENTIFICATION__TIME                  = 0x0008  # 0x0008:0x0009

  SYSTEM__MODE_GPIO0                    = 0x0010
  SYSTEM__MODE_GPIO1                    = 0x0011
  SYSTEM__HISTORY_CTRL                  = 0x0012
  SYSTEM__INTERRUPT_CONFIG_GPIO         = 0x0014
  SYSTEM__INTERRUPT_CLEAR               = 0x0015
  SYSTEM__FRESH_OUT_OF_RESET            = 0x0016
  SYSTEM__GROUPED_PARAMETER_HOLD        = 0x0017

  SYSRANGE__START                       = 0x0018
  SYSRANGE__THRESH_HIGH                 = 0x0019
  SYSRANGE__THRESH_LOW                  = 0x001a
  SYSRANGE__INTERMEASUREMENT_PERIOD     = 0x001b
  SYSRANGE__MAX_CONVERGENCE_TIME        = 0x001c
  SYSRANGE__CROSSTALK_COMPENSATION_RATE = 0x001e
  SYSRANGE__CROSSTALK_VALID_HEIGHT      = 0x0021
  SYSRANGE__EARLY_CONVERGENCE_ESTIMATE  = 0x0022
  SYSRANGE__PART_TO_PART_RANGE_OFFSET   = 0x0024
  SYSRANGE__RANGE_IGNORE_VALID_HEIGHT   = 0x0025
  SYSRANGE__RANGE_IGNORE_THRESHOLD      = 0x0026
  SYSRANGE__MAX_AMBIENT_LEVEL_MULT      = 0x002c
  SYSRANGE__RANGE_CHECK_ENABLES         = 0x002d
  SYSRANGE__VHV_RECALIBRATE             = 0x002e
  SYSRANGE__VHV_REPEAT_RATE             = 0x0031

  RESULT__RANGE_STATUS                  = 0x004d
  RESULT__INTERRUPT_STATUS_GPIO         = 0x004f
  RESULT__HISTORY_BUFFER_x              = 0x0052  # 0x0052:0x0060 (0x2)
  RESULT__RANGE_VAL                     = 0x0062
  RESULT__RANGE_RAW                     = 0x0064
  RESULT__RANGE_RETURN_RATE             = 0x0066
  RESULT__RANGE_REFERENCE_RATE          = 0x0068
  RESULT__RANGE_RETURN_SIGNAL_COUNT     = 0x006c
  RESULT__RANGE_REFERENCE_SIGNAL_COUNT  = 0x0070
  RESULT__RANGE_RETURN_AMB_COUNT        = 0x0074
  RESULT__RANGE_REFERENCE_AMB_COUNT     = 0x0078
  RESULT__RANGE_RETURN_CONV_TIME        = 0x007c
  RESULT__RANGE_REFERENCE_CONV_TIME     = 0x0080

  READOUT__AVERAGING_SAMPLE_PERIOD      = 0x010a
  FIRMWARE__BOOTUP                      = 0x0119
  I2C_SLAVE__DEVICE_ADDRESS             = 0x0212

  MEASUREMENT_TIMEOUT_MS = 1000
  POLLING_DELAY_MS = 1

  def __init__(self, config):
    self.config = config
    self.printer = config.get_printer()
    self.reactor = self.printer.get_reactor()
    self.name = config.get_name().split()[1]
    
    self.i2c_default = bus.MCU_I2C_from_config(config, default_addr=0x29, default_speed=100000)
    mcu = self.i2c_default.get_mcu()
    self.enable_helper = EnableHelper(mcu, self.config.get('enable_pin'))
    self.i2c_slave_address = config.getint('i2c_slave_address', None)
    if self.i2c_slave_address:
      self.i2c = bus.MCU_I2C_from_config(self.config, self.i2c_slave_address, default_speed=100000)
    self.part_to_part_range_offset = config.getint('part_to_part_range_offset', None)

    self.gcode = self.printer.lookup_object('gcode')
    self.gcode.register_mux_command('SINGLE_SHOT_MEASUREMENT', 'SENSOR', self.name,
                                    self.cmd_SINGLE_SHOT_MEASUREMENT,
                                    desc = "Performs a single shot measurement with proper error handling")
    self.gcode.register_mux_command('DIAG_VL_SENSOR', 'SENSOR', self.name,
                                    self.cmd_DIAG_VL_SENSOR,
                                    desc = "Returns sensor diagnostics")
    self.gcode.register_mux_command('GET_VL_OFFSET', 'SENSOR', self.name,
                                    self.cmd_GET_VL_OFFSET,
                                    desc = "Read current VL sensor offset calibration value")
    self.gcode.register_mux_command('SET_VL_OFFSET', 'SENSOR', self.name,
                                    self.cmd_SET_VL_OFFSET,
                                    desc = "Set VL sensor offset calibration value")
    self.gcode.register_mux_command('CALIBRATE_VL_OFFSET', 'SENSOR', self.name,
                                   self.cmd_CALIBRATE_VL_OFFSET,
                                   desc = "Perform automated offset calibration at specified distance")
    
    self.printer.register_event_handler('klippy:connect', self.handle_connect)
  
  def handle_connect(self):
    self.enable_helper.init()
    if self.i2c_slave_address is not None:
      register = 0x0212
      addr = self.i2c_slave_address
      reg_high = (register >> 8) & 0xFF
      reg_low = register & 0xFF
      self.i2c_default.i2c_write_noack([reg_high, reg_low, (addr & 0xFF)])
    else:
      self.i2c = self.i2c_default
      self.i2c_default = None
    self.set_init_reg()
    self.set_register(0x0016, 0x00)     # Change fresh out of set satus to 0
    logging.info(f'successfully connected VL6180 {self.name} on address {self.i2c.get_i2c_address()}')

  def set_init_reg(self):
    # Recommended settings required to be loaded onto the VL6180 during the initialisation of the device
    # https://www.st.com/resource/en/application_note/an4545-vl6180x-basic-ranging-application-note-stmicroelectronics.pdf

    # Mandatory private registers
    self.set_register(0x0207, 0x01)
    self.set_register(0x0208, 0x01)
    self.set_register(0x0096, 0x00)
    self.set_register(0x0097, 0xfd)
    self.set_register(0x00e3, 0x01)
    self.set_register(0x00e4, 0x03)
    self.set_register(0x00e5, 0x02)
    self.set_register(0x00e6, 0x01)
    self.set_register(0x00e7, 0x03)
    self.set_register(0x00f5, 0x02)
    self.set_register(0x00d9, 0x05)
    self.set_register(0x00db, 0xce)
    self.set_register(0x00dc, 0x03)
    self.set_register(0x00dd, 0xf8)
    self.set_register(0x009f, 0x00)
    self.set_register(0x00a3, 0x3c)
    self.set_register(0x00b7, 0x00)
    self.set_register(0x00bb, 0x3c)
    self.set_register(0x00b2, 0x09)
    self.set_register(0x00ca, 0x09)
    self.set_register(0x0198, 0x01)
    self.set_register(0x01b0, 0x17)
    self.set_register(0x01ad, 0x00)
    self.set_register(0x00ff, 0x05)
    self.set_register(0x0100, 0x05)
    self.set_register(0x0199, 0x05)
    self.set_register(0x01a6, 0x1b)
    self.set_register(0x01ac, 0x3e)
    self.set_register(0x01a7, 0x1f)
    self.set_register(0x0030, 0x00)

    # Recommended public registers
    self.set_register(0x0011, 0x10)   # Enables polling for 'New Sample ready' when measurement completes
    self.set_register(0x010a, 0x30)   # Set the averaging sample period (compromise between lower noise and increased execution time)
    self.set_register(0x0031, 0xFF)   # Sets the # of range measurements after which auto calibration of system is performed
    self.set_register(0x002e, 0x01)   # Perform a single temperature calibration of the ranging sensor

    # Optional public registers
    self.set_register(0x001b, 0x09)   # Set default ranging inter-measurement period to 100ms
    self.set_register(0x0014, 0x24)   # Configures interrupt on 'New Sample Ready threshold event'

    # Set offset value correctly
    if self.part_to_part_range_offset:
      self.set_register(self.SYSRANGE__PART_TO_PART_RANGE_OFFSET, self.part_to_part_range_offset & 0xFF)

  def set_register(self, register, data):
    reg_high = (register >> 8) & 0xFF
    reg_low = register & 0xFF
    self.i2c.i2c_write([reg_high, reg_low, (data & 0xFF)])

  def set_register_16bit(self, register, data):
    reg_high = (register >> 8) & 0xFF
    reg_low = register & 0xFF
    data_high = (data >> 8) & 0xFF
    data_low = data & 0xFF
    self.i2c.i2c_write([reg_high, reg_low, data_high, data_low])

  def get_register(self, register):
    register_high = (register >> 8) & 0xFF
    register_low = register & 0xFF
    val = self.i2c.i2c_read([register_high, register_low], 1)
    return int(codecs.encode(val['response'], 'hex'), 16)
  
  def delay_ms(self, ms):
    curtime = self.reactor.monotonic()
    waketime = curtime + ms * 0.001
    self.reactor.pause(waketime)

  def format_macro(self, macro: str) -> str:
    return f'<a class="command">{macro}</a>'

  def single_shot_measurement(self):
    try:
      # Step 1: Check device is ready to start a range measurement (Optional)
      range_status = self.get_register(self.RESULT__RANGE_STATUS)
      if not (range_status & 0x01):
          logging.info('Device not ready for range measurement')
          return None, "Device not ready"
      
      # Step 2: Start a range measurement
      self.set_register(self.SYSRANGE__START, 0x01)
      
      # Step 3: Wait for range measurement to complete
      start_time = self.reactor.monotonic()
      timeout_seconds = self.MEASUREMENT_TIMEOUT_MS * 0.001  # Convert to seconds
      
      while True:
          interrupt_status = self.get_register(self.RESULT__INTERRUPT_STATUS_GPIO)
          
          # Check for errors first
          error_value = (interrupt_status >> 6) & 0b11
          if error_value != 0:
              error_description, _ = self.interrupt_status_lookup(interrupt_status)
              logging.info(f'Error during measurement: {error_description}')
              self.set_register(self.SYSTEM__INTERRUPT_CLEAR, 0x07)
              return None, error_description
          
          # Check for threshold events
          range_event = interrupt_status & 0b111
          if range_event == 4:  # New Sample Ready threshold event
              break
          
          # Check timeout using reactor time
          current_time = self.reactor.monotonic()
          if (current_time - start_time) > timeout_seconds:
              logging.info('Measurement timeout')
              self.set_register(self.SYSTEM__INTERRUPT_CLEAR, 0x07)
              return None, "Measurement timeout"
          
          # Use reactor delay
          self.delay_ms(self.POLLING_DELAY_MS)
      
      # Step 4: Reading range result
      range_value = self.get_register(self.RESULT__RANGE_VAL)

      # Step 5: Check result range status for any warnings or errors
      error_description = None
      range_status = self.get_register(self.RESULT__RANGE_STATUS)
      if (range_status >> 4) & 0x0F != 0:
          error_description = self.result_range_status_lookup(range_status)
          logging.info(f'Warning in range status: {error_description}')

      # Step 6: Clear the Interrupt status
      self.set_register(self.SYSTEM__INTERRUPT_CLEAR, 0x07)
      
      return range_value, error_description

    except Exception as e:
        logging.error(f'Exception in single_shot_measurement: {e}')
        try:
            self.set_register(self.SYSTEM__INTERRUPT_CLEAR, 0x07)
        except:
            pass 
        return None, f"Communication error: {str(e)}"

  def interrupt_status_lookup(self, status):
    error_value = (status >> 6) & 0b11
    range_value = status & 0b111

    interrupt_error_descriptions = {
      0: "No error reported",
      1: "Laser Safety Error",
      2: "PLL error (either PLL1 or PLL2)"
    }
    interrupt_range_descriptions = {
      0: "No threshold events reported",
      1: "Level Low threshold event",
      2: "Level High threshold event",
      3: "Out Of Window threshold event",
      4: "New Sample Ready threshold event"
    }

    error_description = interrupt_error_descriptions.get(error_value, "Unknown error")
    range_description = interrupt_range_descriptions.get(range_value, "Unknown range event")

    return error_description, range_description
  
  def result_range_status_lookup(self, status):
    error_code = (status >> 4) & 0x0F
    error_descriptions = {
      0x0: "No error",                           # 0000
      0x1: "VCSEL Continuity Test",              # 0001
      0x2: "VCSEL Watchdog Test",                # 0010
      0x3: "VCSEL Watchdog",                     # 0011
      0x4: "PLL1 Lock",                          # 0100
      0x5: "PLL2 Lock",                          # 0101
      0x6: "Early Convergence Estimate",         # 0110
      0x7: "Max Convergence",                    # 0111
      0x8: "No Target Ignore",                   # 1000
      0x9: "Not used",                           # 1001
      0xA: "Not used",                           # 1010
      0xB: "Max Signal To Noise Ratio",          # 1011
      0xC: "Raw Ranging Algo Underflow",         # 1100
      0xD: "Raw Ranging Algo Overflow",          # 1101
      0xE: "Ranging Algo Underflow",             # 1110
      0xF: "Ranging Algo Overflow"               # 1111
    }
    return error_descriptions.get(error_code, "Unknown error code")

  def get_offset_calibration_data(self):
        return self.get_register(self.SYSRANGE__PART_TO_PART_RANGE_OFFSET)
    
  def set_offset_calibration(self, offset_value):
        self.set_register(self.SYSRANGE__PART_TO_PART_RANGE_OFFSET, offset_value & 0xFF)

  def get_name(self):
        return self.name
  
  def cmd_SINGLE_SHOT_MEASUREMENT(self, gcmd):
    range_value, error_description = self.single_shot_measurement()
    
    if range_value is None:
        self.gcode.respond_info(f'Single shot measurement failed: {error_description}')
    elif error_description:
        self.gcode.respond_info(f'Single shot measurement: {range_value} mm (Warning: {error_description})')
    else:
        self.gcode.respond_info(f'Single shot measurement: {range_value} mm')

  def cmd_DIAG_VL_SENSOR(self, gcmd):
    # Get only register constants (those with double underscore in the name)
    registers = {name: value for name, value in vars(self.__class__).items() 
                if isinstance(value, int) and '__' in name}
    
    # Sort by register address for logical output order
    sorted_registers = sorted(registers.items(), key=lambda x: x[1])
    
    for reg_name, reg_addr in sorted_registers:
      try:
        reg_value = self.get_register(reg_addr)
        self.gcode.respond_info('%s: %s (%d)' % (reg_name, hex(reg_value), reg_value))
      except Exception as e:
        self.gcode.respond_info('%s: Error reading register - %s' % (reg_name, str(e)))

  def cmd_GET_VL_OFFSET(self, gcmd):
    offset = self.get_offset_calibration_data()
    # Convert from 2s complement to signed value for display
    offset_signed = offset if offset < 128 else offset - 256
    self.gcode.respond_info(f'Current VL sensor offset: {offset_signed}mm (register: 0x{offset:02x})')
        
  def cmd_SET_VL_OFFSET(self, gcmd):
    offset = gcmd.get_int('OFFSET', 0)
    if offset > 127 or offset < -128:
        self.gcode.respond_info('Error: Offset must be between -128 and +127')
        return
    # Convert to register value (2s complement if negative)
    register_value = offset if offset >= 0 else 256 + offset
    self.set_offset_calibration(register_value)
    
    configfile = self.printer.lookup_object('configfile')
    configfile.set(f'vl6180 {self.name}', 'part_to_part_range_offset', str(register_value))
    gcmd.respond_info(f"Offset set to {offset}mm, with register value 0x{register_value:02x}! Please use {self.format_macro('SAVE_CONFIG')} to save the calibration value.")

  def cmd_CALIBRATE_VL_OFFSET(self, gcmd):
    target_distance = gcmd.get_float('DISTANCE', 50.0)
    num_samples = gcmd.get_int('SAMPLES', 10)
    
    if num_samples < 1:
        self.gcode.respond_info('Error: SAMPLES must be at least 1')
        return
        
    self.gcode.respond_info(f'Starting offset calibration at {target_distance}mm with {num_samples} samples')
    self.gcode.respond_info('Make sure target is placed at the specified distance with 17%+ reflectance')
    
    # Step 1: Get and clear the current system offset
    old_offset_reg = self.get_offset_calibration_data()
    # Convert from 2s complement to signed value for display
    old_offset_signed = old_offset_reg if old_offset_reg < 128 else old_offset_reg - 256
    self.gcode.respond_info(f'Current offset: {old_offset_signed}mm (register: 0x{old_offset_reg:02x})')
    
    self.set_offset_calibration(0)
    self.gcode.respond_info('Cleared existing offset calibration')
    
    # Step 2 & 3: Collect measurements and calculate mean
    measurements = []
    failed_measurements = 0
    
    for i in range(num_samples):
        range_value, error_description = self.single_shot_measurement()
        
        if range_value is not None:
            measurements.append(range_value)
            self.gcode.respond_info(f'Sample {i+1}/{num_samples}: {range_value}mm')
        else:
            failed_measurements += 1
            self.gcode.respond_info(f'Sample {i+1}/{num_samples}: Failed - {error_description}')
    
    if len(measurements) == 0:
        self.gcode.respond_info('Error: All measurements failed. Check sensor and target setup.')
        return
    
    if failed_measurements > 0:
        self.gcode.respond_info(f'Warning: {failed_measurements} measurements failed')
    
    # Calculate mean
    mean_measurement = sum(measurements) / len(measurements)
    self.gcode.respond_info(f'Mean measurement: {mean_measurement:.2f}mm')
    
    # Step 4: Calculate the offset required
    offset_required = target_distance - mean_measurement
    self.gcode.respond_info(f'Calculated offset: {offset_required:.2f}mm')
    
    # Convert to integer and handle 2's complement representation
    offset_int = int(round(offset_required))
    
    # Clamp to valid range (-128 to +127)
    if offset_int > 127:
        offset_int = 127
        self.gcode.respond_info('Warning: Offset clamped to maximum value of +127')
    elif offset_int < -128:
        offset_int = -128
        self.gcode.respond_info('Warning: Offset clamped to minimum value of -128')
    
    # Convert to 2's complement representation for register
    if offset_int >= 0:
        register_value = offset_int
    else:
        register_value = 256 + offset_int  # Convert negative to 2's complement
    
    # Step 5: Apply offset and show the change
    self.set_offset_calibration(register_value)
    self.part_to_part_range_offset = register_value
    offset_change = offset_int - old_offset_signed
    self.gcode.respond_info(f'Offset changed: {old_offset_signed}mm -> {offset_int}mm (change: {offset_change:+d}mm)')
    self.gcode.respond_info(f'Register value: 0x{old_offset_reg:02x} -> 0x{register_value:02x}')
    
    # Verify calibration with a test measurement
    self.gcode.respond_info('Verifying calibration...')
    test_range, test_error = self.single_shot_measurement()
    if test_range is not None:
        error_after_cal = abs(test_range - target_distance)
        self.gcode.respond_info(f'Verification measurement: {test_range}mm (error: {error_after_cal:.2f}mm)')
        if error_after_cal <= 2.0:  # Within 2mm tolerance
            self.gcode.respond_info('Calibration completed successfully!')
            configfile = self.printer.lookup_object('configfile')
            configfile.set(f'vl6180 {self.name}', 'part_to_part_range_offset', str(register_value))
            gcmd.respond_info(
                f"Calibration completed successfully! Please use {self.format_macro('SAVE_CONFIG')} to save the calibration value."
            )
        else:
            self.gcode.respond_info('Warning: Calibration may need adjustment or target repositioning')
    else:
        self.gcode.respond_info(f'Verification failed: {test_error}')

def load_config_prefix(config):
    return vl6180(config)