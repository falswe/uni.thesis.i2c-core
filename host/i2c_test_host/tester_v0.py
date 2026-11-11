import os
import re
import time

HEX_BYTE_RE_TESTER = re.compile(r"\b(?:0x)?([0-9a-fA-F]{2})\b")


def parse_tester_status(raw: str) -> str:
    for line in raw.splitlines():
        line_upper = line.upper()
        if "ERR: NACK" in line_upper:
            return "AddrNak"
        if "ERR: TIMEOUT" in line_upper:
            return "Timeout"
        if "ERR: ARLO" in line_upper:
            return "ArbitrationLost"
        if "ERR: UNDERFLOW" in line_upper:
            return "Aborted"
        if "ERR:" in line_upper:
            return "Aborted"
        stripped = line_upper.strip()
        if stripped == "OK" or stripped.startswith("OK:"):
            return "Success"
    return "Unknown"


def parse_tester_data(raw: str) -> list[int]:
    for line in raw.splitlines():
        if line.startswith("OK:"):
            if "RX empty" in line:
                return []
            remainder = line[3:]
            return [int(token, 16) for token in HEX_BYTE_RE_TESTER.findall(remainder)]
    return []


def _parse_gpio_pin_list(raw: str | None) -> tuple[int, ...]:
    if raw is None or raw.strip() == "":
        return (6, 7)
    pins = tuple(int(part.strip(), 0) for part in raw.replace(";", ",").split(",") if part.strip())
    if len(pins) != 2:
        raise ValueError("I2C_TESTER_HOLD_PINS must contain exactly two GPIO pins: SCL,SDA")
    return pins


class TesterV0:
    TRANSFER_WAIT_TIMEOUT_S = 2.0

    def __init__(self, backend):
        self.backend = backend
        self.i2c_addr7: int | None = None
        # GPIOs physically wired to the tested I2C lines.
        # Default assumes GP6=SCL, GP7=SDA; override with I2C_TESTER_HOLD_PINS=SCL,SDA.
        self.hold_pins = _parse_gpio_pin_list(os.getenv("I2C_TESTER_HOLD_PINS"))
        self._hold_pins_ready = False

    def _collect_block(self, wait_timeout: float = 1.0) -> str:
        # The tester ends every response with a "> " prompt without a newline.
        take_prompt = getattr(self.backend, "take_prompt", None)
        lines: list[str] = []
        deadline = time.monotonic() + wait_timeout
        while time.monotonic() < deadline:
            line = self.backend.read_line(timeout_s=0.01)
            if line is not None:
                lines.append(line)
                if line.rstrip().endswith(">"):
                    break
            elif callable(take_prompt) and take_prompt():
                break
            else:
                time.sleep(0.005)
        return "\n".join(lines)

    def _run_cmd(self, cmd: str, wait_timeout: float = 1.0) -> str:
        self.backend.write_line(cmd)
        return self._collect_block(wait_timeout=wait_timeout)

    def _run_status_cmds(self, commands: list[str], wait_timeout: float) -> tuple[str, str]:
        raws: list[str] = []
        for command in commands:
            raw = self._run_cmd(command, wait_timeout=wait_timeout)
            raws.append(raw)
            status = parse_tester_status(raw)
            if status != "Success":
                return status, "\n".join(raws)
        return "Success", "\n".join(raws)

    def info(self, wait_timeout: float = 0.5) -> str:
        return self._run_cmd("info", wait_timeout=wait_timeout)

    def write(
        self,
        a7: int,
        payload: bytes,
        wait_timeout: float | None = TRANSFER_WAIT_TIMEOUT_S,
        no_stop: bool = False,
    ) -> tuple[str, str]:
        wait_timeout = self.TRANSFER_WAIT_TIMEOUT_S if wait_timeout is None else wait_timeout
        cmd = f"{'w ns' if no_stop else 'w'} 0x{(a7 & 0x7F):02x}"
        if payload:
            cmd += " " + " ".join(f"0x{byte:02x}" for byte in payload)
        raw = self._run_cmd(cmd, wait_timeout=wait_timeout)
        status = parse_tester_status(raw)
        return status, raw

    def read(
        self,
        a7: int,
        length: int,
        wait_timeout: float | None = TRANSFER_WAIT_TIMEOUT_S,
        no_stop: bool = False,
    ) -> tuple[str, list[int], str]:
        wait_timeout = self.TRANSFER_WAIT_TIMEOUT_S if wait_timeout is None else wait_timeout
        cmd = f"{'r ns' if no_stop else 'r'} 0x{(a7 & 0x7F):02x} {length}"
        raw = self._run_cmd(cmd, wait_timeout=wait_timeout)
        status = parse_tester_status(raw)
        data = parse_tester_data(raw)
        return status, data, raw

    def mem_read(
        self,
        a7: int,
        payload: bytes,
        read_length: int,
        wait_timeout: float | None = TRANSFER_WAIT_TIMEOUT_S,
    ) -> tuple[str, list[int], str]:
        write_status, write_raw = self.write(
            a7,
            payload,
            wait_timeout=wait_timeout,
            no_stop=True,
        )
        if write_status != "Success":
            return write_status, [], write_raw

        read_status, data, read_raw = self.read(
            a7,
            read_length,
            wait_timeout=wait_timeout,
        )
        raw = "\n".join(part for part in (write_raw, read_raw) if part)
        return read_status, data, raw

    def peek(self, wait_timeout: float = 0.5) -> tuple[list[int], str]:
        # Peek maps to target-read on the tester v0
        raw = self._run_cmd("target-read", wait_timeout=wait_timeout)
        data = parse_tester_data(raw)
        return data, raw

    def set_speed(self, hz: int, wait_timeout: float = 1.0) -> tuple[str, str]:
        raw = self._run_cmd(f"speed {int(hz)}", wait_timeout=wait_timeout)
        return parse_tester_status(raw), raw

    def target_write(self, payload: bytes, wait_timeout: float = 1.0) -> tuple[str, str]:
        cmd = "target-write"
        if payload:
            cmd += " " + " ".join(f"0x{byte:02x}" for byte in payload)
        raw = self._run_cmd(cmd, wait_timeout=wait_timeout)
        return parse_tester_status(raw), raw

    def target_txclear(self, wait_timeout: float = 1.0) -> tuple[str, str]:
        raw = self._run_cmd("target-txclear", wait_timeout=wait_timeout)
        return parse_tester_status(raw), raw

    def target_status(self, wait_timeout: float = 1.0) -> tuple[str, str]:
        raw = self._run_cmd("target-status", wait_timeout=wait_timeout)
        return parse_tester_status(raw), raw

    def status(self, wait_timeout: float = 1.0) -> tuple[str, str]:
        raw = self._run_cmd("status", wait_timeout=wait_timeout)
        return parse_tester_status(raw), raw

    def _ensure_hold_pins(self, wait_timeout: float) -> tuple[str, str]:
        if self._hold_pins_ready:
            return "Success", ""
        scl, sda = self.hold_pins
        status, raw = self._run_status_cmds(
            [
                f"gpio-config {scl} out open-drain",
                f"gpio-config {sda} out open-drain",
            ],
            wait_timeout,
        )
        self._hold_pins_ready = status == "Success"
        return status, raw

    def hold_bus(self, wait_timeout: float = 1.0) -> tuple[str, str]:
        status, raw = self._ensure_hold_pins(wait_timeout)
        if status != "Success":
            return status, raw
        scl, sda = self.hold_pins
        commands = [
            f"gpio-write {scl} 1; gpio-write {sda} 1",
            f"gpio-write {sda} 0; gpio-write {scl} 0",
        ]
        return self._run_status_cmds(commands, wait_timeout)

    def release_bus(self, wait_timeout: float = 1.0) -> tuple[str, str]:
        status, raw = self._ensure_hold_pins(wait_timeout)
        if status != "Success":
            return status, raw
        scl, sda = self.hold_pins

        commands = [
            f"gpio-write {sda} 0",
            f"gpio-write {scl} 1; gpio-write {sda} 1",
        ]
        return self._run_status_cmds(commands, wait_timeout)
