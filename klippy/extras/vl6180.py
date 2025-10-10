# VL6180 Proximity sensing module
#
# Copyright (C) 2023 Eytecz
#
# This file may be distributed under the terms of the GNU GPLv2 license.

from . import bus
import codecs
import timeit
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

    self.gcode = self.printer.lookup_object('gcode')
    self.gcode.register_mux_command('MEASURE_DISTANCE', 'SENSOR', self.name,
                                    self.cmd_MEASURE_DISTANCE,
                                    desc = "Returns the measured value on vl6180 sensor")
    self.gcode.register_mux_command('DIAG_VL_SENSOR', 'SENSOR', self.name,
                                    self.cmd_DIAG_VL_SENSOR,
                                    desc = "Returns sensor diagnostics")
    self.gcode.register_mux_command('SET_VL_INIT_REG', 'SENSOR', self.name,
                                    self.cmd_SET_VL_INIT_REG,
                                    desc = "Sets the initial registers as recommended")
    self.gcode.register_mux_command('CONTINUOUS_RANGE_MEASUREMENT', 'SENSOR', self.name,
                                    self.cmd_CONTINUOUS_RANGE_MEASUREMENT,
                                    desc = "Starts a continuous range measurement and returns the values for an n amount of samples")
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

  def cmd_SET_VL_INIT_REG(self, gcmd):
    self.set_init_reg()

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
  
  def delay(self, us):
    start_time = timeit.default_timer()
    while (timeit.default_timer() - start_time) * 1e6 < us:
      pass

  def vl6180_start_range(self):
    self.set_register(self.SYSRANGE__START, 0x01)

  def vl6180_poll_range(self):
    status = self.get_register(self.RESULT__INTERRUPT_STATUS_GPIO)
    range_status = status & 0x07
    while range_status != 0x04:
      status = self.get_register(self.RESULT__INTERRUPT_STATUS_GPIO)
      range_status = status & 0x07
      self.delay(1000)

  def vl6180_read_range(self):
    range = self.get_register(self.RESULT__RANGE_VAL)
    return range
  
  def vl6180_clear_interrupts(self):
    self.set_register(self.SYSTEM__INTERRUPT_CLEAR, 0x07)

  def vl6180_single_range_measurement(self):
    self.vl6180_start_range()
    self.vl6180_poll_range()
    value = self.vl6180_read_range()
    self.vl6180_clear_interrupts()
    return value

  def interrupt_status_lookup(self, status):
    self.gcode.respond_info('Status: %s' % status)
    # Get error values
    error_value = (status >> 6) & 0b11
    range_value = status & 0b111

    # Define dictionaries for error and range descriptions
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

    # Read out dictionary with error and range values
    error_description = interrupt_error_descriptions.get(error_value, "Unknown error")
    range_description = interrupt_range_descriptions.get(range_value, "Unknown range event")

    return error_description, range_description
  
  def get_name(self):
        return self.name

  def cmd_MEASURE_DISTANCE(self, gcmd):
    self.gcode.respond_info('Measured distance: %i' % self.vl6180_single_range_measurement())
  
  def cmd_CONTINUOUS_RANGE_MEASUREMENT(self, gcmd):
    samples = gcmd.get_int('SAMPLES', 10)
    self.set_register(self.SYSRANGE__START, 0x03)   # Start Ranging Mode Continuous
    self.delay(5e5)
    for _ in range(samples):
      if self.get_register(self.RESULT__INTERRUPT_STATUS_GPIO) == 0x04:
        value = self.get_register(self.RESULT__RANGE_VAL)
        self.set_register(self.SYSTEM__INTERRUPT_CLEAR, 0x07)
        self.gcode.respond_info('RESULT__RANGE_VAL: %i mm' % value)
        self.delay(1e6)
      else:
        interrupt_status = self.get_register(self.RESULT__INTERRUPT_STATUS_GPIO)
        interrupt_status_descriptions = self.interrupt_status_lookup(interrupt_status)
        error_description = interrupt_status_descriptions[0][0]
        range_description = interrupt_status_descriptions[0][1]
        self.gcode.respond_info('Error status: %s' % error_description)
        self.gcode.respond_info('Range status: %s' % range_description)
        return
    self.set_register(self.SYSRANGE__START, 0x01)   # Stop ranging

  def cmd_DIAG_VL_SENSOR(self, gcmd):

    # Identification registers
    self.gcode.respond_info('IDENTIFICATION__MODEL_ID: %s' % hex(int(self.get_register(self.IDENTIFICATION__MODEL_ID))))
    self.gcode.respond_info('IDENTIFICATION__MODEL_REV_MAJOR: %s' % hex(int(self.get_register(self.IDENTIFICATION__MODEL_REV_MAJOR))))
    self.gcode.respond_info('IDENTIFICATION__MODEL_REV_MINOR: %s' % hex(int(self.get_register(self.IDENTIFICATION__MODEL_REV_MINOR))))
    self.gcode.respond_info('IDENTIFICATION__MODULE_REV_MAJOR: %s' % hex(int(self.get_register(self.IDENTIFICATION__MODULE_REV_MAJOR))))
    self.gcode.respond_info('IDENTIFICATION__MODULE_REV_MINOR %s' % hex(int(self.get_register(self.IDENTIFICATION__MODULE_REV_MINOR))))
    self.gcode.respond_info('IDENTIFICATION__DATE_HI: %s' % hex(int(self.get_register(self.IDENTIFICATION__DATE_HI))))
    self.gcode.respond_info('IDENTIFICATION__DATE_LO: %s' % hex(int(self.get_register(self.IDENTIFICATION__DATE_LO))))
    self.gcode.respond_info('IDENTIFICATION__TIME: %s' % hex(int(self.get_register(self.IDENTIFICATION__TIME))))

    # System registers
    self.gcode.respond_info('SYSTEM__MODE_GPIO0: %s' % hex(int(self.get_register(self.SYSTEM__MODE_GPIO0))))
    self.gcode.respond_info('SYSTEM__MODE_GPIO1: %s' % hex(int(self.get_register(self.SYSTEM__MODE_GPIO1))))
    self.gcode.respond_info('SYSTEM__HISTORY_CTRL: %s' % hex(int(self.get_register(self.SYSTEM__HISTORY_CTRL))))
    self.gcode.respond_info('SYSTEM__INTERRUPT_CONFIG_GPIO: %s' % hex(int(self.get_register(self.SYSTEM__INTERRUPT_CONFIG_GPIO))))
    self.gcode.respond_info('SYSTEM__INTERRUPT_CLEAR: %s' % hex(int(self.get_register(self.SYSTEM__INTERRUPT_CLEAR))))
    self.gcode.respond_info('SYSTEM__FRESH_OUT_OF_RESET: %s' % hex(int(self.get_register(self.SYSTEM__FRESH_OUT_OF_RESET))))
    self.gcode.respond_info('SYSTEM__GROUPED_PARAMETER_HOLD: %s' % hex(int(self.get_register(self.SYSTEM__GROUPED_PARAMETER_HOLD))))
    
    # Measurement registers
    self.gcode.respond_info('SYSRANGE__START: %s' % hex(int(self.get_register(self.SYSRANGE__START))))
    self.gcode.respond_info('SYSRANGE__THRESH_HIGH: %s' % hex(int(self.get_register(self.SYSRANGE__THRESH_HIGH))))
    self.gcode.respond_info('SYSRANGE__THRESH_LOW: %s' % hex(int(self.get_register(self.SYSRANGE__THRESH_LOW))))
    self.gcode.respond_info('SYSRANGE__INTERMEASUREMENT_PERIOD: %s' % hex(int(self.get_register(self.SYSRANGE__INTERMEASUREMENT_PERIOD))))
    self.gcode.respond_info('SYSRANGE__MAX_CONVERGENCE_TIME: %s' % hex(int(self.get_register(self.SYSRANGE__MAX_CONVERGENCE_TIME))))
    self.gcode.respond_info('SYSRANGE__CROSSTALK_COMPENSATION_RATE: %s' % hex(int(self.get_register(self.SYSRANGE__CROSSTALK_COMPENSATION_RATE))))
    self.gcode.respond_info('SYSRANGE__CROSSTALK_VALID_HEIGHT: %s' % hex(int(self.get_register(self.SYSRANGE__CROSSTALK_VALID_HEIGHT))))
    self.gcode.respond_info('SYSRANGE__EARLY_CONVERGENCE_ESTIMATE: %s' % hex(int(self.get_register(self.SYSRANGE__EARLY_CONVERGENCE_ESTIMATE))))
    self.gcode.respond_info('SYSRANGE__PART_TO_PART_RANGE_OFFSET: %s' % hex(int(self.get_register(self.SYSRANGE__PART_TO_PART_RANGE_OFFSET))))
    self.gcode.respond_info('SYSRANGE__RANGE_IGNORE_VALID_HEIGHT: %s' % hex(int(self.get_register(self.SYSRANGE__RANGE_IGNORE_VALID_HEIGHT))))
    self.gcode.respond_info('SYSRANGE__RANGE_IGNORE_THRESHOLD: %s' % hex(int(self.get_register(self.SYSRANGE__RANGE_IGNORE_THRESHOLD))))
    self.gcode.respond_info('SYSRANGE__MAX_AMBIENT_LEVEL_MULT: %s' % hex(int(self.get_register(self.SYSRANGE__MAX_AMBIENT_LEVEL_MULT))))
    self.gcode.respond_info('SYSRANGE__RANGE_CHECK_ENABLES: %s' % hex(int(self.get_register(self.SYSRANGE__RANGE_CHECK_ENABLES))))
    self.gcode.respond_info('SYSRANGE__VHV_RECALIBRATE: %s' % hex(int(self.get_register(self.SYSRANGE__VHV_RECALIBRATE))))
    self.gcode.respond_info('SYSRANGE__VHV_REPEAT_RATE: %s' % hex(int(self.get_register(self.SYSRANGE__VHV_REPEAT_RATE))))
    self.gcode.respond_info('SYSRANGE__MAX_CONVERGENCE_TIME: %s' % hex(int(self.get_register(self.SYSRANGE__MAX_CONVERGENCE_TIME))))
    self.gcode.respond_info('SYSRANGE__CROSSTALK_COMPENSATION_RATE: %s' % hex(int(self.get_register(self.SYSRANGE__CROSSTALK_COMPENSATION_RATE))))
    self.gcode.respond_info('SYSRANGE__CROSSTALK_VALID_HEIGHT: %s' % hex(int(self.get_register(self.SYSRANGE__CROSSTALK_VALID_HEIGHT))))
    self.gcode.respond_info('SYSRANGE__EARLY_CONVERGENCE_ESTIMATE: %s' % hex(int(self.get_register(self.SYSRANGE__EARLY_CONVERGENCE_ESTIMATE))))
    self.gcode.respond_info('SYSRANGE__PART_TO_PART_RANGE_OFFSET: %s' % hex(int(self.get_register(self.SYSRANGE__PART_TO_PART_RANGE_OFFSET))))
    self.gcode.respond_info('SYSRANGE__RANGE_IGNORE_VALID_HEIGHT: %s' % hex(int(self.get_register(self.SYSRANGE__RANGE_IGNORE_VALID_HEIGHT))))
    self.gcode.respond_info('SYSRANGE__RANGE_IGNORE_THRESHOLD: %s' % hex(int(self.get_register(self.SYSRANGE__RANGE_IGNORE_THRESHOLD))))
    self.gcode.respond_info('SYSRANGE__MAX_AMBIENT_LEVEL_MULT: %s' % hex(int(self.get_register(self.SYSRANGE__MAX_AMBIENT_LEVEL_MULT))))
    self.gcode.respond_info('SYSRANGE__RANGE_CHECK_ENABLES: %s' % hex(int(self.get_register(self.SYSRANGE__RANGE_CHECK_ENABLES))))
    
    # Result registers
    self.gcode.respond_info('RESULT__RANGE_STATUS: %s' % hex(int(self.get_register(self.RESULT__RANGE_STATUS))))
    self.gcode.respond_info('RESULT__INTERRUPT_STATUS_GPIO: %s' % hex(int(self.get_register(self.RESULT__INTERRUPT_STATUS_GPIO))))
    self.gcode.respond_info('RESULT__HISTORY_BUFFER_x: %s' % hex(int(self.get_register(self.RESULT__HISTORY_BUFFER_x))))
    self.gcode.respond_info('RESULT__RANGE_VAL: %s' % hex(int(self.get_register(self.RESULT__RANGE_VAL))))
    self.gcode.respond_info('RESULT__RANGE_RAW: %s' % hex(int(self.get_register(self.RESULT__RANGE_RAW))))
    self.gcode.respond_info('RESULT__RANGE_RETURN_RATE: %s' % hex(int(self.get_register(self.RESULT__RANGE_RETURN_RATE))))
    self.gcode.respond_info('RESULT__RANGE_REFERENCE_RATE: %s' % hex(int(self.get_register(self.RESULT__RANGE_REFERENCE_RATE))))
    self.gcode.respond_info('RESULT__RANGE_RETURN_SIGNAL_COUNT: %s' % hex(int(self.get_register(self.RESULT__RANGE_RETURN_SIGNAL_COUNT))))
    self.gcode.respond_info('RESULT__RANGE_REFERENCE_SIGNAL_COUNT: %s' % hex(int(self.get_register(self.RESULT__RANGE_REFERENCE_SIGNAL_COUNT))))
    self.gcode.respond_info('RESULT__RANGE_RETURN_AMB_COUNT: %s' % hex(int(self.get_register(self.RESULT__RANGE_RETURN_AMB_COUNT))))
    self.gcode.respond_info('RESULT__RANGE_REFERENCE_AMB_COUNT: %s' % hex(int(self.get_register(self.RESULT__RANGE_REFERENCE_AMB_COUNT))))
    self.gcode.respond_info('RESULT__RANGE_RETURN_CONV_TIME: %s' % hex(int(self.get_register(self.RESULT__RANGE_RETURN_CONV_TIME))))
    self.gcode.respond_info('RESULT__RANGE_REFERENCE_CONV_TIME: %s' % hex(int(self.get_register(self.RESULT__RANGE_REFERENCE_CONV_TIME))))
    
    # Other registers
    self.gcode.respond_info('READOUT__AVERAGING_SAMPLE_PERIOD: %s' % hex(int(self.get_register(self.READOUT__AVERAGING_SAMPLE_PERIOD))))
    self.gcode.respond_info('FIRMWARE__BOOTUP: %s' % hex(int(self.get_register(self.FIRMWARE__BOOTUP))))
    self.gcode.respond_info('I2C_SLAVE__DEVICE_ADDRESS: %s' % hex(int(self.get_register(self.I2C_SLAVE__DEVICE_ADDRESS))))


def load_config_prefix(config):
    return vl6180(config)
