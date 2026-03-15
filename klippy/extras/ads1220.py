# ADS1220 Support
#
# Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bulk_sensor, bus

#
# Constants
#
BYTES_PER_SAMPLE = 4  # samples are 4 byte wide unsigned integers
MAX_SAMPLES_PER_MESSAGE = bulk_sensor.MAX_BULK_MSG_SIZE // BYTES_PER_SAMPLE
UPDATE_INTERVAL = 0.10
RESET_CMD = 0x06
START_SYNC_CMD = 0x08
RREG_CMD = 0x20
WREG_CMD = 0x40
NOOP_CMD = 0x0
RESET_STATE = bytearray([0x0, 0x0, 0x0, 0x0])

# ADS1220 MUX value for shorted inputs (V(AVDD)+V(AVSS))/2
MUX_SHORTED = 0b1110

# Temperature sensor resolution: 0.03125 °C per LSB (14-bit)
TEMP_RESOLUTION = 0.03125

# turn bytearrays into pretty hex strings: [0xff, 0x1]
def hexify(byte_array):
    return "[%s]" % (", ".join([hex(b) for b in byte_array]))


class ADS1220:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.last_error_count = 0
        self.consecutive_fails = 0
        # Chip options
        # Gain
        self.gain_options = {'1': 0x0, '2': 0x1, '4': 0x2, '8': 0x3, '16': 0x4,
                             '32': 0x5, '64': 0x6, '128': 0x7}
        self.gain = config.getchoice('gain', self.gain_options, default='128')
        # Sample rate
        self.sps_normal = {'20': 20, '45': 45, '90': 90, '175': 175,
                           '330': 330, '600': 600, '1000': 1000}
        self.sps_turbo = {'40': 40, '90': 90, '180': 180, '350': 350,
                          '660': 660, '1200': 1200, '2000': 2000}
        self.sps_options = self.sps_normal.copy()
        self.sps_options.update(self.sps_turbo)
        self.sps = config.getchoice('sample_rate', self.sps_options,
                                    default='660')
        self.is_turbo = str(self.sps) in self.sps_turbo
        # Input multiplexer: AINP and AINN
        mux_options = {'AIN0_AIN1': 0b0000, 'AIN0_AIN2': 0b0001,
                       'AIN0_AIN3': 0b0010, 'AIN1_AIN2': 0b0011,
                       'AIN1_AIN3': 0b0100, 'AIN2_AIN3': 0b0101,
                       'AIN1_AIN0': 0b0110, 'AIN3_AIN2': 0b0111,
                       'AIN0_AVSS': 0b1000, 'AIN1_AVSS': 0b1001,
                       'AIN2_AVSS': 0b1010, 'AIN3_AVSS': 0b1011}
        self.mux = config.getchoice('input_mux', mux_options,
                                    default='AIN0_AIN1')
        # PGA Bypass
        self.pga_bypass = config.getboolean('pga_bypass', default=False)
        # bypass PGA when AVSS is the negative input
        force_pga_bypass = self.mux >= 0b1000
        self.pga_bypass = force_pga_bypass or self.pga_bypass
        # Voltage Reference
        self.vref_options = {'internal': 0b0, 'REF0': 0b01, 'REF1': 0b10,
                             'analog_supply': 0b11}
        self.vref = config.getchoice('vref', self.vref_options,
                                     default='internal')
        # check for conflict between REF1 and AIN0/AIN3
        mux_conflict = [0b0000, 0b0001, 0b0010, 0b0100, 0b0101, 0b0110, 0b0111,
                        0b1000, 0b1011]
        if self.vref == 0b10 and self.mux in mux_conflict:
            raise config.error("ADS1220 config error: AIN0/REFP1 and AIN3/REFN1"
                               " cant be used as a voltage reference and"
                               " an input at the same time")
        # Temperature compensation options
        self.temp_compensation_enabled = config.getboolean(
            'temperature_compensation', default=False)
        self.calibration_interval = config.getfloat(
            'calibration_interval', default=5.0, above=0.5)
        self.offset_calibration_enabled = config.getboolean(
            'offset_calibration', default=False)
        # Temperature compensation state
        self.chip_temperature = 0.
        self.offset_counts = 0
        self.last_calibration_time = 0.
        self.temperature_callback = None
        self.min_temp = self.max_temp = 0.
        # SPI Setup
        spi_speed = 512000 if self.is_turbo else 256000
        self.spi = bus.MCU_SPI_from_config(config, 1, default_speed=spi_speed)
        self.mcu = mcu = self.spi.get_mcu()
        self.oid = mcu.create_oid()
        # Data Ready (DRDY) Pin
        drdy_pin = config.get('data_ready_pin')
        ppins = printer.lookup_object('pins')
        drdy_ppin = ppins.lookup_pin(drdy_pin)
        self.data_ready_pin = drdy_ppin['pin']
        drdy_pin_mcu = drdy_ppin['chip']
        if drdy_pin_mcu != self.mcu:
            raise config.error("ADS1220 config error: SPI communication and"
                               " data_ready_pin must be on the same MCU")
        # Clock tracking
        chip_smooth = self.sps * UPDATE_INTERVAL * 2
        # Measurement conversion
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth, "<i")
        # Process messages in batches
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch, self._start_measurements,
            self._finish_measurements, UPDATE_INTERVAL)
        # Command Configuration
        mcu.add_config_cmd(
            "config_ads1220 oid=%d spi_oid=%d data_ready_pin=%s"
            % (self.oid, self.spi.get_oid(), self.data_ready_pin))
        mcu.add_config_cmd("query_ads1220 oid=%d rest_ticks=0"
                           % (self.oid,), on_restart=True)
        mcu.register_config_callback(self._build_config)
        self.query_ads1220_cmd = None
        # Register as temperature sensor if compensation enabled
        if self.temp_compensation_enabled:
            pheaters = printer.load_object(config, 'heaters')
            pheaters.register_sensor(config, self)

    def setup_trigger_analog(self, trigger_analog_oid):
        self.mcu.add_config_cmd(
            "ads1220_attach_trigger_analog oid=%d trigger_analog_oid=%d"
            % (self.oid, trigger_analog_oid), is_init=True)

    def _build_config(self):
        cmdqueue = self.spi.get_command_queue()
        self.query_ads1220_cmd = self.mcu.lookup_command(
            "query_ads1220 oid=%c rest_ticks=%u", cq=cmdqueue)
        self.ffreader.setup_query_command("query_ads1220_status oid=%c",
                                          oid=self.oid, cq=cmdqueue)

    def get_mcu(self):
        return self.mcu

    def get_samples_per_second(self):
        return self.sps

    def lookup_sensor_error(self, error_code):
        return "Unknown ads1220 error" % (error_code,)

    # returns a tuple of the minimum and maximum value of the sensor, used to
    # detect if a data value is saturated
    def get_range(self):
        return -0x800000, 0x7FFFFF

    # add_client interface, direct pass through to bulk_sensor API
    def add_client(self, callback):
        self.batch_bulk.add_client(callback)

    # Temperature sensor interface (for Klipper heaters system)
    def setup_callback(self, temperature_callback):
        self.temperature_callback = temperature_callback

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def get_report_time_delta(self):
        return self.calibration_interval

    def get_temp(self, eventtime):
        return self.chip_temperature, 0.

    def get_status(self, eventtime):
        return {'temperature': round(self.chip_temperature, 2)}

    # Measurement decoding
    def _convert_samples(self, samples):
        adc_factor = 1. / (1 << 23)
        offset = self.offset_counts
        count = 0
        for ptime, val in samples:
            corrected = val - offset
            samples[count] = (round(ptime, 6), corrected,
                              round(corrected * adc_factor, 9))
            count += 1
        del samples[count:]

    # Read a single 24-bit conversion result from the ADC
    def _read_adc_value(self):
        # Send 3 NOP bytes and read back 3 data bytes
        params = self.spi.spi_transfer([NOOP_CMD, NOOP_CMD, NOOP_CMD])
        response = bytearray(params['response'])
        counts = (response[0] << 16) | (response[1] << 8) | response[2]
        # Sign-extend 24-bit to 32-bit
        if counts & 0x800000:
            counts -= 0x1000000
        return counts

    # Build register 0 value
    def _build_reg0(self, mux=None):
        if mux is None:
            mux = self.mux
        return (mux << 4) | (self.gain << 1) | int(self.pga_bypass)

    # Build register 1 value
    def _build_reg1(self, temp_sensor=False):
        continuous = 0x1
        mode = 0x2 if self.is_turbo else 0x0
        sps_list = self.sps_turbo if self.is_turbo else self.sps_normal
        data_rate = list(sps_list.keys()).index(str(self.sps))
        ts_bit = 0x1 if temp_sensor else 0x0
        return (data_rate << 5) | (mode << 3) | (continuous << 2) | (ts_bit << 1)

    # Read the ADS1220 internal temperature sensor
    def _read_chip_temperature(self):
        # Set TS=1 in register 1 to enable temperature sensor mode
        self.write_reg(0x1, [self._build_reg1(temp_sensor=True)])
        # Start a single-shot conversion
        self.send_command(START_SYNC_CMD)
        # Wait for conversion to complete
        reactor = self.printer.get_reactor()
        reactor.pause(reactor.monotonic() + 1. / self.sps + 0.002)
        # Read the 24-bit result
        raw = self._read_adc_value()
        # Restore normal mode (TS=0)
        self.write_reg(0x1, [self._build_reg1(temp_sensor=False)])
        # Convert: 14-bit value is left-aligned in 24-bit word
        # Shift right by 10 to get the signed 14-bit temperature code
        temp_code = raw >> 10
        return temp_code * TEMP_RESOLUTION

    # Read the ADC offset by shorting inputs internally
    def _read_offset(self):
        # Set MUX to shorted inputs: (V(AVDD) + V(AVSS)) / 2
        self.write_reg(0x0, [self._build_reg0(mux=MUX_SHORTED)])
        # Start a single-shot conversion
        self.send_command(START_SYNC_CMD)
        # Wait for conversion to complete
        reactor = self.printer.get_reactor()
        reactor.pause(reactor.monotonic() + 1. / self.sps + 0.002)
        # Read the offset value
        offset = self._read_adc_value()
        # Restore original MUX setting
        self.write_reg(0x0, [self._build_reg0()])
        return offset

    # Perform a calibration cycle: read temperature and optionally offset
    def _do_calibration_cycle(self):
        # Stop the MCU-side streaming
        self.query_ads1220_cmd.send([self.oid, 0])
        self.ffreader.note_end()
        try:
            # Read chip temperature
            self.chip_temperature = self._read_chip_temperature()
            logging.info("ADS1220 '%s' chip temperature: %.2f C",
                         self.name, self.chip_temperature)
            # Optionally read offset
            if self.offset_calibration_enabled:
                self.offset_counts = self._read_offset()
                logging.info("ADS1220 '%s' offset: %d counts",
                             self.name, self.offset_counts)
            # Fire temperature callback for heaters system
            if self.temperature_callback is not None:
                mcu_clock = self.mcu.estimated_print_time(
                    self.printer.get_reactor().monotonic())
                self.temperature_callback(mcu_clock, self.chip_temperature)
        except self.printer.command_error as e:
            logging.exception("ADS1220 '%s' calibration cycle error", self.name)
        # Restore full chip configuration and restart streaming
        self.setup_chip()
        rest_ticks = self.mcu.seconds_to_clock(1. / (10. * self.sps))
        self.query_ads1220_cmd.send([self.oid, rest_ticks])
        self.ffreader.note_start()
        self.last_calibration_time = self.printer.get_reactor().monotonic()

    # Start, stop, and process message batches
    def _start_measurements(self):
        self.last_error_count = 0
        self.consecutive_fails = 0
        self.last_calibration_time = self.printer.get_reactor().monotonic()
        # Start bulk reading
        self.reset_chip()
        self.setup_chip()
        rest_ticks = self.mcu.seconds_to_clock(1. / (10. * self.sps))
        self.query_ads1220_cmd.send([self.oid, rest_ticks])
        logging.info("ADS1220 starting '%s' measurements", self.name)
        # Initialize clock tracking
        self.ffreader.note_start()

    def _finish_measurements(self):
        # don't use serial connection after shutdown
        if self.printer.is_shutdown():
            return
        # Halt bulk reading
        self.query_ads1220_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info("ADS1220 finished '%s' measurements", self.name)

    def _process_batch(self, eventtime):
        # Run calibration cycle if interval has elapsed
        if (self.temp_compensation_enabled
                and eventtime - self.last_calibration_time
                    >= self.calibration_interval):
            self._do_calibration_cycle()
        samples = self.ffreader.pull_samples()
        self._convert_samples(samples)
        return {'data': samples, 'errors': self.last_error_count,
                'overflows': self.ffreader.get_last_overflows()}

    def reset_chip(self):
        # the reset command takes 50us to complete
        self.send_command(RESET_CMD)
        # read startup register state and validate
        val = self.read_reg(0x0, 4)
        if val != RESET_STATE:
            if self.mcu.is_fileoutput():
                return
            raise self.printer.command_error(
                "Invalid ads1220 reset state (got %s vs %s).\n"
                "This is generally indicative of connection problems\n"
                "(e.g. faulty wiring) or a faulty ADS1220 chip."
                % (hexify(val), hexify(RESET_STATE)))

    def setup_chip(self):
        reg_values = [self._build_reg0(),
                      self._build_reg1(),
                      (self.vref << 6),
                      0x0]
        self.write_reg(0x0, reg_values)
        # start measurements immediately
        self.send_command(START_SYNC_CMD)

    def read_reg(self, reg, byte_count):
        read_command = [RREG_CMD | (reg << 2) | (byte_count - 1)]
        read_command += [NOOP_CMD] * byte_count
        params = self.spi.spi_transfer(read_command)
        return bytearray(params['response'][1:])

    def send_command(self, cmd):
        self.spi.spi_send([cmd])

    def write_reg(self, reg, register_bytes):
        write_command = [WREG_CMD | (reg << 2) | (len(register_bytes) - 1)]
        write_command.extend(register_bytes)
        self.spi.spi_send(write_command)
        stored_val = self.read_reg(reg, len(register_bytes))
        if bytearray(register_bytes) != stored_val:
            if self.mcu.is_fileoutput():
                return
            raise self.printer.command_error(
                "Failed to set ADS1220 register [0x%x] to %s: got %s. "
                "This may be a connection problem (e.g. faulty wiring)" % (
                    reg, hexify(register_bytes), hexify(stored_val)))


ADS1220_SENSOR_TYPE = {"ads1220": ADS1220}
