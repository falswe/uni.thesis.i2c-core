from .collect import collect_response_block
from .parsing import parse_addr, parse_data, parse_fields, parse_status

_VALID_SEG_TOKENS = frozenset({"np", "s-p", "s-np", "ns-p", "ns-np"})


class SessionV1:
    COLLECT_IDLE_S = 0.15
    COLLECT_MAX_TOTAL_S = 1.0
    COLLECT_POLL_TIMEOUT_S = 0.05
    TRANSFER_WAIT_TIMEOUT_S = 6.0

    def __init__(self, backend):
        self.backend = backend
        self.i2c_addr7: int | None = None

    @staticmethod
    def _with_seg_token(command: str, no_stop: bool, seg: str | None) -> str:
        if seg is not None:
            normalized = seg.lower()
            if normalized not in _VALID_SEG_TOKENS:
                raise ValueError(
                    f"invalid seg token {seg!r}; expected one of {sorted(_VALID_SEG_TOKENS)}"
                )
            return f"{command} {normalized}"
        return f"{command} np" if no_stop else command

    def _collect_block(
        self,
        idle_timeout_s: float = COLLECT_IDLE_S,
        max_total_s: float = COLLECT_MAX_TOTAL_S,
        poll_timeout_s: float = COLLECT_POLL_TIMEOUT_S,
    ) -> str:
        return collect_response_block(
            self.backend,
            idle_timeout_s=idle_timeout_s,
            max_total_s=max_total_s,
            poll_timeout_s=poll_timeout_s,
        )

    def info(self, wait_timeout: float = 0.5) -> str:
        self.backend.write_line("INFO")
        return self._collect_block(max_total_s=wait_timeout)

    def probe_ready(self) -> bool:
        return any(
            "INFO protocol=session_v1" in self.info(wait_timeout=0.5) for _attempt in range(2)
        )

    def write(
        self,
        a7: int,
        payload: bytes,
        wait_timeout: float | None = TRANSFER_WAIT_TIMEOUT_S,
        no_stop: bool = False,
        seg: str | None = None,
    ) -> tuple[str, str]:
        wait_timeout = self.TRANSFER_WAIT_TIMEOUT_S if wait_timeout is None else wait_timeout
        cmd = self._with_seg_token("W", no_stop, seg)
        cmd += f" 0x{(a7 & 0x7F):02x}"
        if payload:
            cmd += " " + " ".join(f"0x{byte:02x}" for byte in payload)
        self.backend.write_line(cmd)
        raw = self._collect_block(max_total_s=wait_timeout)
        status = parse_status(raw) or "Timeout"
        return status, raw

    def read(
        self,
        a7: int,
        length: int,
        wait_timeout: float | None = TRANSFER_WAIT_TIMEOUT_S,
        no_stop: bool = False,
        seg: str | None = None,
    ) -> tuple[str, list[int], str]:
        wait_timeout = self.TRANSFER_WAIT_TIMEOUT_S if wait_timeout is None else wait_timeout
        cmd = self._with_seg_token("R", no_stop, seg)
        cmd += f" 0x{(a7 & 0x7F):02x} {int(length)}"
        self.backend.write_line(cmd)
        raw = self._collect_block(max_total_s=wait_timeout)
        status = parse_status(raw) or "Timeout"
        return status, parse_data(raw), raw

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

    def peek(self, wait_timeout: float = 1.0) -> tuple[list[int], str]:
        self.backend.write_line("P")
        raw = self._collect_block(max_total_s=wait_timeout)
        return parse_data(raw), raw

    def own_addr(self, wait_timeout: float = 1.0) -> tuple[str, int | None, str]:
        return self._addr_command("ADDR", wait_timeout)

    def set_own_addr(self, a7: int, wait_timeout: float = 1.0) -> tuple[str, int | None, str]:
        # Not masked to 7 bits: the firmware validates the address.
        return self._addr_command(f"ADDR 0x{int(a7):02x}", wait_timeout)

    def _addr_command(self, cmd: str, wait_timeout: float) -> tuple[str, int | None, str]:
        self.backend.write_line(cmd)
        raw = self._collect_block(max_total_s=wait_timeout)
        return parse_status(raw) or "Timeout", parse_addr(raw), raw

    def q(
        self,
        kind: str,
        a7: int,
        payload: bytes = b"",
        read_length: int = 0,
        wait_timeout: float | None = TRANSFER_WAIT_TIMEOUT_S,
    ) -> tuple[str, dict[str, str], list[int], str]:
        """One transfer through the transaction layer: kind is W, R or WR (see docs/protocol.md)."""
        wait_timeout = self.TRANSFER_WAIT_TIMEOUT_S if wait_timeout is None else wait_timeout
        kind = kind.upper()
        if kind not in ("W", "R", "WR"):
            raise ValueError(f"invalid Q kind {kind!r}; expected W, R or WR")
        cmd = f"Q {kind} 0x{(a7 & 0x7F):02x}"
        if kind in ("R", "WR"):
            cmd += f" {int(read_length)}"
        if payload:
            cmd += " " + " ".join(f"0x{byte:02x}" for byte in payload)
        self.backend.write_line(cmd)
        raw = self._collect_block(max_total_s=wait_timeout)
        fields = parse_fields(raw, "Q")
        return fields.get("status", "Timeout"), fields, parse_data(raw), raw

    def txn_params(
        self,
        f_ms: int | None = None,
        backoff_ms: int | None = None,
        stuck_ms: int | None = None,
        wait_timeout: float = 1.0,
    ) -> tuple[str, dict[str, int], str]:
        """Report, and with arguments change, the transaction layer parameters."""
        cmd = "TXN"
        for key, value in (("F", f_ms), ("backoff", backoff_ms), ("stuck", stuck_ms)):
            if value is not None:
                cmd += f" {key}={int(value)}"
        self.backend.write_line(cmd)
        raw = self._collect_block(max_total_s=wait_timeout)
        params = {key: int(value) for key, value in parse_fields(raw, "txn").items() if value.isdigit()}
        return parse_status(raw) or "Timeout", params, raw
