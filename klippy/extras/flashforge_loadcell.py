import logging
import mcu

from enum import Enum

MCU_FLASHFORGE_RESPONSE = "flashforge_loadcell_response"

MCU_CMD_FLASHFORGE_H1 = "flashforge_loadcell_h1"
MCU_CMD_FLASHFORGE_H2 = "flashforge_loadcell_h2 weight=%u"
MCU_CMD_FLASHFORGE_H3 = "flashforge_loadcell_h3"
MCU_CMD_FLASHFORGE_H7 = "flashforge_loadcell_h7"
MCU_CMD_FLASHFORGE_TEST = "flashforge_loadcell_test_cmd cmd=%*s"


class Commands(Enum):
    H1 = 'H1'
    H2 = 'H2'
    H3 = 'H3'
    H7 = 'H7'
    TEST = 'TEST'

    @classmethod
    def from_mcu_command(cls, cmd):
        if cmd==MCU_CMD_FLASHFORGE_H1:
            return cls.H1
        if cmd==MCU_CMD_FLASHFORGE_H2:
            return cls.H2
        if cmd==MCU_CMD_FLASHFORGE_H3:
            return cls.H3
        if cmd==MCU_CMD_FLASHFORGE_H7:
            return cls.H7
        if cmd==MCU_CMD_FLASHFORGE_TEST:
            return cls.TEST
        raise ValueError(f"Unknown MCU command for loadcell: {cmd}")

class FlashforgeLoadCell:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.name = config.get_name().split()[-1]
        self.mcu = mcu.get_printer_mcu(self.printer, config.get('mcu'))
        self.mcu.register_response(self._handle_flashforge_response, MCU_FLASHFORGE_RESPONSE)
        self.active_command = None
        self.logger = logging.getLogger('klippy')
        self.last_weight_grams = 0
        self.tare_threshold = config.getint('tare_threshold', 5, 0)
        self.tare_timeout = config.getfloat('tare_timeout', 10.0, 0.)
        self.supported_cmds = {}
        self.gcode.register_command(
            "FLASHFORGE_LOAD_CELL_TARE",
            self.cmd_LOAD_CELL_TARE,
            desc="Starts the weight tare (zeroing) procedure"
        )
        self.gcode.register_command(
            "FLASHFORGE_LOAD_CELL_CALIBRATE",
            self.cmd_LOAD_CELL_CALIBRATE,
            desc="Sends a calibration command (H2 S500)"
        )
        self.gcode.register_command(
            "FLASHFORGE_LOAD_CELL_SAVE_CALIBRATION",
            self.cmd_LOAD_CELL_SAVE_CALIBRATION,
            desc="Sends calibration save command (H3 S200)"
        )
        self.gcode.register_command(
            "FLASHFORGE_GET_LOAD_CELL_WEIGHT",
            self.cmd_GET_LOAD_CELL_WEIGHT,
            desc="Queries and displays the current weight (H7)"
        )
        self.gcode.register_command(
            "FLASHFORGE_LOAD_CELL_TEST",
            self.cmd_LOAD_CELL_TEST,
            desc="Sends an arbitrary command to the loadcell."
        )
        self.printer.register_event_handler("klippy:connect", self._handle_connect)

    def _handle_connect(self):
        for cmd_name in (
            MCU_CMD_FLASHFORGE_H1,
            MCU_CMD_FLASHFORGE_H2,
            MCU_CMD_FLASHFORGE_H3,
            MCU_CMD_FLASHFORGE_H7,
            MCU_CMD_FLASHFORGE_TEST
        ):
            cmd = self.mcu.try_lookup_command(cmd_name)
            if not cmd:
                raise Exception(
                    f"{self.name}: Required MCU command '{cmd_name}' is not available. Check your firmware."
                )
            self.supported_cmds[cmd_name] = cmd

    def _handle_flashforge_response(self, params):
        raw_cmd = params.get('command')
        if raw_cmd is None:
            self.logger.warning(f"{self.name}: Response without command field: {params}")
            return
        try:
            cmd_str = raw_cmd.decode('utf-8')
        except Exception:
            self.logger.warning(f"{self.name}: Cannot decode command in response: {raw_cmd}")
            return

        try:
            response_cmd = Commands(cmd_str)
        except ValueError:
            self.logger.info(f"{self.name}: Unrecognized response command '{cmd_str}'")
            return

        status_raw = params.get('status')
        status = None
        if status_raw is not None:
            try:
                status = status_raw.decode('utf-8')
            except Exception:
                status = None

        # self.logger.info(f"{self.name}: Received response: {response_cmd} {params}")

        if self.active_command:
            expected = self.active_command.get('cmd')
            if response_cmd == expected:
                completion = self.active_command.get('completion')
                if not completion.test():
                    completion.complete(params)
                self.active_command = None
                return

        if response_cmd == Commands.H7 and status == 'ok':
            value_raw = params.get('value')
            try:
                self.last_weight_grams = int(value_raw or 0)
            except Exception:
                self.logger.warning(f"{self.name}: Invalid weight value: {value_raw}")

    def _send_and_wait(self, command_name, params_list=None):
        if self.active_command:
            raise self.printer.command_error(f"{self.name}: Another G-Code command is already in progress.")
        cmd_obj = self.mcu.try_lookup_command(command_name)
        if not cmd_obj:
            raise self.printer.command_error(f"{self.name}: MCU command '{command_name}' not found. Check firmware.")
        try:
            cmd_enum = Commands.from_mcu_command(command_name)
        except ValueError as e:
            raise self.printer.command_error(f"{self.name}: {e}")
        completion = self.reactor.completion()
        self.active_command = {'cmd': cmd_enum, 'completion': completion}
        if params_list:
            cmd_obj.send(params_list)
        else:
            cmd_obj.send()
        try:
            response = completion.wait(self.reactor.monotonic() + 0.6)
        except self.reactor.TimeoutError:
            self.active_command = None
            raise self.printer.command_error(f"{self.name}: MCU command '{command_name}' timed out.")
        finally:
            self.active_command = None
        if (not response):
            self.logger.warning(f"{self.name}: Response is none!")
            response={}
        status_raw = response.get('status','unknown')
        try:
            status = status_raw.decode('utf-8') if status_raw is not None else None
        except Exception:
            status = None
        if status != 'ok':
            cmd_field = response.get('command')
            try:
                cmd_field_str = cmd_field.decode('utf-8') if cmd_field is not None else command_name
            except Exception:
                cmd_field_str = command_name
            raw_resp = response.get('raw_response')
            try:
                raw_resp_str = raw_resp.decode('utf-8') if raw_resp is not None else 'no details'
            except Exception:
                raw_resp_str = 'no details'
            raise self.printer.command_error(
                f"{self.name}: Command '{cmd_field_str}' failed with status '{status}'. MCU Response: '{raw_resp_str}'"
            )
        return response

    def cmd_GET_LOAD_CELL_WEIGHT(self, gcmd):
        response = self._send_and_wait(MCU_CMD_FLASHFORGE_H7)
        value_raw = response.get('value')
        try:
            weight = int(value_raw or 0)
        except Exception:
            weight = 0
        self.last_weight_grams = weight
        gcmd.respond_info(f"{self.name}: Weight: {weight:.2f} grams")

    def cmd_LOAD_CELL_TARE(self, gcmd):
        gcmd.respond_info(f"{self.name}: Starting tare procedure...")
        deadline = self.reactor.monotonic() + self.tare_timeout
        while self.reactor.monotonic() < deadline:
            try:
                self._send_and_wait(MCU_CMD_FLASHFORGE_H1)
                response = self._send_and_wait(MCU_CMD_FLASHFORGE_H7)
            except Exception as e:
                raise gcmd.error(f"{self.name}: Tare step failed: {e}")
            value_raw = response.get('value')
            try:
                current_weight = int(value_raw or 0)
            except Exception:
                current_weight = self.last_weight_grams
            if abs(current_weight) <= self.tare_threshold:
                gcmd.respond_info(f"{self.name}: Tare successful. Final weight: {current_weight:.2f}g")
                return
            gcmd.respond_info(f"{self.name}: Weight is {current_weight:.2f}g, retrying...")
            self.reactor.pause(self.reactor.monotonic() + 0.2)
        raise gcmd.error(f"{self.name}: Tare failed to complete within {self.tare_timeout}s.")

    def cmd_LOAD_CELL_CALIBRATE(self, gcmd):
        weight = gcmd.get_int('WEIGHT', 500, 0)
        self._send_and_wait(MCU_CMD_FLASHFORGE_H2, params_list=[weight])
        gcmd.respond_info(f"{self.name}: Calibrate command sent.")

    def cmd_LOAD_CELL_SAVE_CALIBRATION(self, gcmd):
        self._send_and_wait(MCU_CMD_FLASHFORGE_H3)
        gcmd.respond_info(f"{self.name}: Save calibration command sent.")

    def cmd_LOAD_CELL_TEST(self, gcmd):
        cmd_str = gcmd.get('CMD')
        if cmd_str is None:
            raise gcmd.error(f"{self.name}: No CMD parameter provided.")
        try:
            cmd_bytes = cmd_str.encode()
        except Exception as e:
            raise gcmd.error(f"{self.name}: Invalid CMD parameter: {e}")
        response = self._send_and_wait(MCU_CMD_FLASHFORGE_TEST, params_list=[cmd_bytes])
        raw_resp = response.get('raw_response')
        try:
            raw_resp_str = raw_resp.decode('utf-8') if raw_resp is not None else ''
        except Exception:
            raw_resp_str = ''
        gcmd.respond_info(f"{self.name}: Response: {raw_resp_str}")


class LoadCellSensor:
    def __init__(self, config, loadcell):
        self.loadcell = loadcell
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.logger = logging.getLogger('klippy')
        self.sample_interval = config.getfloat('sample_interval', 0.2, 0.1)
        self.check_only_when_printing = config.getboolean('check_only_when_printing', True)
        self.vc = self.printer.lookup_object("virtual_sdcard")
        self.max_force = config.getint('max_force', 900, 0)
        self.overload_action = config.getchoice("overload_action", ["shutdown", "pause"], default="shutdown")
        self.sample_timer = self.reactor.register_timer(self._sample)
        self._callback = None
        self.printer.register_event_handler("klippy:connect", self._handle_connect)

    def _handle_connect(self):
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def _sample(self, eventtime):
        try:
            cmd_obj = self.loadcell.supported_cmds.get(MCU_CMD_FLASHFORGE_H7)
            if cmd_obj and not self.loadcell.active_command:
                cmd_obj.send()
        except Exception as e:
            self.logger.warning(f"{self.name}: Could not send H7 poll: {e}")
        weight = self.loadcell.last_weight_grams
        if weight > self.max_force:
            if not self.check_only_when_printing or self.vc.is_active():
                msg = f"{self.name}: Max force exceeded. Last weight was: {weight:.0f}g"
                if self.overload_action == "shutdown":
                    self.printer.invoke_shutdown(msg)
                    return self.reactor.NEVER
                else:
                    self.logger.warning(msg)
                    self.vc.do_pause()
        measured_time = self.reactor.monotonic()
        if self._callback:
            try:
                estimated = self.loadcell.mcu.estimated_print_time(measured_time)
            except Exception:
                estimated = None
            self._callback(estimated, weight)
        return measured_time + self.sample_interval

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.sample_interval

    def setup_minmax(self, min_temp, max_temp):
        pass

    def get_temp(self, eventtime):
        return self.loadcell.last_weight_grams, 0

    def get_status(self, eventtime):
        return {'temperature': self.loadcell.last_weight_grams}


def load_config(config):
    loadcell = FlashforgeLoadCell(config)
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory(
        "flashforge_loadcell",
        lambda cfg: LoadCellSensor(cfg, loadcell)
    )
    return loadcell
