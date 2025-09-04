import logging
import mcu

from enum import Enum

MCU_FLASHFORGE_RESPONSE = "flashforge_loadcell_response"

MCU_CMD_FLASHFORGE_H1 = "flashforge_loadcell_h1"
MCU_CMD_FLASHFORGE_H2 = "flashforge_loadcell_h2 weight=%u"
MCU_CMD_FLASHFORGE_H3 = "flashforge_loadcell_h3 weight=%u"
MCU_CMD_FLASHFORGE_H7 = "flashforge_loadcell_h7"
MCU_CMD_FLASHFORGE_TEST = "flashforge_loadcell_test_cmd cmd=%*s"


class Commands(Enum):
    H1 = 'H1'
    H2 = 'H2'
    H3 = 'H3'
    H7 = 'H7'
    TEST = 'TEST'

_MCU_CMD_MAP = {
    MCU_CMD_FLASHFORGE_H1: Commands.H1,
    MCU_CMD_FLASHFORGE_H2: Commands.H2,
    MCU_CMD_FLASHFORGE_H3: Commands.H3,
    MCU_CMD_FLASHFORGE_H7: Commands.H7,
    MCU_CMD_FLASHFORGE_TEST: Commands.TEST,
}

class MCUResponse:
    def __init__(self, params):
        self.params = params
        self.command_name = self._decode(params.get('command'))
        self.status = self._decode(params.get('status'), 'unknown')
        self.raw_response = self._decode(params.get('raw_response'))
        try:
            self.value = int(params.get('value', 0))
        except (ValueError, TypeError):
            self.value = 0

    def _decode(self, value, default=''):
        if value is None: return default
        if isinstance(value, str): return value
        try: return value.decode('utf-8')
        except: return default


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
        self.tare_threshold = config.getint('tare_threshold', 50, 0)
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
            desc="Sends a calibration command"
        )
        self.gcode.register_command(
            "FLASHFORGE_LOAD_CELL_SAVE_CALIBRATION",
            self.cmd_LOAD_CELL_SAVE_CALIBRATION,
            desc="Sends calibration save command"
        )
        self.gcode.register_command(
            "FLASHFORGE_GET_LOAD_CELL_WEIGHT",
            self.cmd_GET_LOAD_CELL_WEIGHT,
            desc="Queries and displays the current weight"
        )
        self.gcode.register_command(
            "FLASHFORGE_LOAD_CELL_TEST",
            self.cmd_LOAD_CELL_TEST,
            desc="Sends an arbitrary command to the loadcell"
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
        response = MCUResponse(params)
        self.logger.debug(f"{self.name}: Received response: {response.command_name}, status: {response.status}")

        if self.active_command:
            expected_cmd = self.active_command.get('cmd')
            if response.command_name == expected_cmd.value:
                completion = self.active_command.get('completion')
                if not completion.test():
                    completion.complete(response)
                return

        if response.command_name == Commands.H7.value and response.status == 'ok':
            self.last_weight_grams = response.value

    def _send_and_wait(self, command_name, params_list=None):
        if self.active_command:
            raise self.printer.command_error(f"{self.name}: Another G-Code command is already in progress.")

        cmd_obj = self.supported_cmds.get(command_name)
        cmd_enum = _MCU_CMD_MAP.get(command_name)
        if not cmd_obj or not cmd_enum:
            raise self.printer.command_error(f"{self.name}: MCU command '{command_name}' not found.")
            
        completion = self.reactor.completion()
        self.active_command = {'cmd': cmd_enum, 'completion': completion}

        try:
            if params_list:
                cmd_obj.send(params_list)
            else:
                cmd_obj.send()

            response = completion.wait(self.reactor.monotonic() + 0.6)

            if response is None:
                raise self.printer.command_error(f"{self.name}: MCU command '{cmd_enum.value}' timed out.")
            
            if response.status != 'ok':
                self._handle_mcu_error(response, command_name)
            
            return response
        finally:
            self.active_command = None
        
    def _handle_mcu_error(self, response: MCUResponse, cmd_name_fallback):
        cmd_name = response.command_name or cmd_name_fallback
        raise self.printer.command_error(
            f"{self.name}: Command '{cmd_name}' failed with status "
            f"'{response.status}'. MCU Response: '{response.raw_response}'")

    def cmd_GET_LOAD_CELL_WEIGHT(self, gcmd):
        response = self._send_and_wait(MCU_CMD_FLASHFORGE_H7)
        self.last_weight_grams = response.value
        gcmd.respond_info(f"{self.name}: Weight: {response.value} grams")

    def cmd_LOAD_CELL_TARE(self, gcmd):
        gcmd.respond_info(f"{self.name}: Starting tare procedure...")
        deadline = self.reactor.monotonic() + self.tare_timeout
        while self.reactor.monotonic() < deadline:
            try:
                self._send_and_wait(MCU_CMD_FLASHFORGE_H1)
                response = self._send_and_wait(MCU_CMD_FLASHFORGE_H7)
            except self.printer.command_error as e:
                raise gcmd.error(f"Tare step failed: {e}")

            if abs(response.value) <= self.tare_threshold:
                gcmd.respond_info(f"Tare successful. Final weight: {response.value}g")
                return
            
            gcmd.respond_info(f"Weight is {response.value}g, retrying...")
            self.reactor.pause(self.reactor.monotonic() + 0.2)
        raise gcmd.error(f"Tare failed to complete within {self.tare_timeout}s.")

    def cmd_LOAD_CELL_CALIBRATE(self, gcmd):
        weight = gcmd.get_int('WEIGHT', 500, 0)
        self._send_and_wait(MCU_CMD_FLASHFORGE_H2, params_list=[weight])
        gcmd.respond_info(f"{self.name}: Calibrate command sent.")

    def cmd_LOAD_CELL_SAVE_CALIBRATION(self, gcmd):
        weight = gcmd.get_int('WEIGHT', 200, 100)
        self._send_and_wait(MCU_CMD_FLASHFORGE_H3, params_list=[weight])
        gcmd.respond_info(f"{self.name}: Save calibration command sent.")

    def cmd_LOAD_CELL_TEST(self, gcmd):
        cmd_str = gcmd.get('CMD', None)
        if cmd_str is None:
            raise gcmd.error(f"{self.name}: No CMD parameter provided.")
        
        cmd_bytes = cmd_str.encode('utf-8')
        response = self._send_and_wait(MCU_CMD_FLASHFORGE_TEST, params_list=[cmd_bytes])
        gcmd.respond_info(f"{self.name}: Response: {response.raw_response}")

    def get_status(self, eventtime):
        return {'force_g': self.last_weight_grams}

class LoadCellSensor:
    def __init__(self, config, loadcell):
        self.loadcell = loadcell
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.logger = logging.getLogger('klippy')
        self.sample_interval = config.getfloat('sample_interval', 0.2, 0.1)
        self.check_only_when_printing = config.getboolean('check_only_when_printing', True)
        try:
            self.pause_resume = self.printer.load_object(
                config, "pause_resume")
        except config.error:
            raise self.printer.config_error(
                f"{self.name} requires [pause_resume] to work,"
                " please add it to your config!")
        self.max_force = config.getint('max_force', 900, 0)
        self.default_max_force = self.max_force
        self.overload_action = config.getchoice("overload_action", ["shutdown", "pause"], default="shutdown")
        self.default_overload_action = self.overload_action
        self.sample_timer = self.reactor.register_timer(self._sample)
        self._callback = None

        self.gcode.register_command("FLASHFORGE_LOAD_CELL_SET_MAX_FORCE", 
                                   self.cmd_SET_LOADCELL_MAX_FORCE,
                                   desc=f'Set max force and overload action for loadcell sensor')

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
            idle_timeout = self.printer.lookup_object("idle_timeout")
            is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
            if not self.check_only_when_printing or is_printing:
                msg = f"{self.name}: Max force exceeded. Last weight was: {weight}g"
                if self.overload_action == "shutdown":
                    self.printer.invoke_shutdown(msg)
                    return self.reactor.NEVER
                else:
                    if not self.pause_resume.is_paused:
                        self.logger.warning(msg)
                        self.pause_resume.send_pause_command()
                        self.gcode.run_script_from_command("PAUSE\nM400\n")
        measured_time = self.reactor.monotonic()
        if self._callback:
            self._callback(self.loadcell.mcu.estimated_print_time(measured_time), weight)

        return measured_time + self.sample_interval

    def cmd_SET_LOADCELL_MAX_FORCE(self, gcmd):
        reset = gcmd.get_int('RESET', 0, 0, 1)
        
        if reset:
            self.max_force = self.default_max_force
            self.overload_action = self.default_overload_action
            gcmd.respond_info(
                f"{self.name}: Max force reset to default value: {self.max_force}g"
                f"overload action  reset to default value:'{self.overload_action}'"
            )
        else:
            force = gcmd.get_int('FORCE', self.max_force, 0)
            overload_action = gcmd.get('ACTION', self.overload_action, lambda x: str(x) if str(x) in ['shutdown', 'pause'] else None)
            if not isinstance(overload_action, str):
                raise gcmd.error("Must be 'shutdown' or 'pause'")
            old_force = self.max_force
            old_action = self.overload_action
            self.max_force = force
            self.overload_action = overload_action
            gcmd.respond_info(
                f"{self.name}: Max force changed from {old_force}g to {self.max_force}g; "
                f"overload action changed from '{old_action}' to '{self.overload_action}'"
            )

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.sample_interval

    def setup_minmax(self, min_temp, max_temp):
        pass

    def get_temp(self, eventtime):
        return self.loadcell.last_weight_grams, 0

    def get_status(self, eventtime):
        return {'temperature': self.loadcell.last_weight_grams, **self.loadcell.get_status(eventtime)}


def load_config(config):
    loadcell = FlashforgeLoadCell(config)
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory(
        "flashforge_loadcell",
        lambda cfg: LoadCellSensor(cfg, loadcell)
    )
    return loadcell
