import socket
from pathlib import Path

import yaml

from .backend import TcpLineBackend
from .session_v1 import SessionV1
from .tester_v0 import TesterV0


_PROC_NET_ROUTE = Path("/proc/net/route")


def resolve_host(value: object) -> str:
    host = str(value)
    if host != "wsl-host":
        return host

    try:
        route_lines = _PROC_NET_ROUTE.read_text(encoding="ascii").splitlines()[1:]
    except OSError as exc:
        raise ValueError(
            "cannot resolve 'wsl-host'; use an explicit IP or 'localhost' for mirrored WSL"
        ) from exc

    gateways: list[tuple[int, int]] = []
    for line in route_lines:
        fields = line.split()
        if len(fields) < 8 or fields[1] != "00000000":
            continue
        try:
            gateway = int(fields[2], 16)
            flags = int(fields[3], 16)
            metric = int(fields[6])
        except ValueError:
            continue
        if gateway and flags & 0x2:
            gateways.append((metric, gateway))

    if not gateways:
        raise ValueError(
            "cannot resolve 'wsl-host'; use an explicit IP or 'localhost' for mirrored WSL"
        )
    gateway = min(gateways)[1]
    return socket.inet_ntoa(gateway.to_bytes(4, byteorder="little"))


class DutDevice:
    def __init__(self, name: str, i2c_addr7: int | None, backend, dialect, dialect_name: str):
        self.name = name
        self.i2c_addr7 = i2c_addr7
        self.backend = backend
        self.dialect = dialect
        self.dialect_name = dialect_name
        self._peers_by_addr: dict[int, DutDevice] = {}

    def open(self) -> None:
        self.backend.open()
        probe_ready = getattr(self.dialect, "probe_ready", None)
        if not callable(probe_ready) or probe_ready():
            return

        self.backend.close()
        raise ConnectionError(f"peer '{self.name}' did not answer its readiness probe")

    def close(self) -> None:
        self.backend.close()

    def bind_peers_by_addr(self, peers_by_addr: dict[int, "DutDevice"]) -> None:
        self._peers_by_addr = peers_by_addr

    def _prepare_target_peer_for_write(self, i2c_addr7: int) -> None:
        target = self._peers_by_addr.get(int(i2c_addr7) & 0x7F)
        if target is None:
            return

        prepare_target_rx = getattr(target.dialect, "prepare_target_rx", None)
        if callable(prepare_target_rx):
            prepare_target_rx()

    def write(
        self,
        i2c_addr7: int,
        payload: bytes,
        wait_timeout: float | None = None,
        no_stop: bool = False,
    ):
        self._prepare_target_peer_for_write(i2c_addr7)
        return self.dialect.write(
            i2c_addr7,
            payload,
            wait_timeout=wait_timeout,
            no_stop=no_stop,
        )

    def read(
        self,
        i2c_addr7: int,
        length: int,
        wait_timeout: float | None = None,
        no_stop: bool = False,
    ):
        return self.dialect.read(
            i2c_addr7,
            length,
            wait_timeout=wait_timeout,
            no_stop=no_stop,
        )

    def mem_read(
        self,
        i2c_addr7: int,
        payload: bytes,
        read_length: int,
        wait_timeout: float | None = None,
    ):
        self._prepare_target_peer_for_write(i2c_addr7)
        mem_read = getattr(self.dialect, "mem_read", None)
        if not callable(mem_read):
            raise NotImplementedError(f"{self.dialect_name} does not support mem_read")
        return mem_read(
            i2c_addr7,
            payload,
            read_length,
            wait_timeout=wait_timeout,
        )

    def q(
        self,
        kind: str,
        i2c_addr7: int,
        payload: bytes = b"",
        read_length: int = 0,
        wait_timeout: float | None = None,
    ):
        """One transfer through the transaction layer; returns (status, fields, data, raw)."""
        q = getattr(self.dialect, "q", None)
        if not callable(q):
            raise NotImplementedError(f"{self.dialect_name} does not support Q")
        if payload:
            self._prepare_target_peer_for_write(i2c_addr7)
        return q(kind, i2c_addr7, payload, read_length, wait_timeout=wait_timeout)

    def txn_params(self, f_ms=None, backoff_ms=None, stuck_ms=None, wait_timeout: float = 1.0):
        txn_params = getattr(self.dialect, "txn_params", None)
        if not callable(txn_params):
            raise NotImplementedError(f"{self.dialect_name} does not support TXN")
        return txn_params(f_ms, backoff_ms, stuck_ms, wait_timeout=wait_timeout)

    def peek(self, wait_timeout: float = 1.0):
        return self.dialect.peek(wait_timeout=wait_timeout)

    def own_addr(self, wait_timeout: float = 1.0):
        own_addr = getattr(self.dialect, "own_addr", None)
        if not callable(own_addr):
            raise NotImplementedError(f"{self.dialect_name} does not support own_addr")
        return own_addr(wait_timeout=wait_timeout)

    def set_i2c_addr7(self, new_addr7: int, wait_timeout: float = 1.0) -> tuple[str, str]:
        """Change this peer's own target address at runtime.

        The host-side address (and the address map shared with the other peers)
        only changes after the firmware confirms the new address.
        """
        set_own_addr = getattr(self.dialect, "set_own_addr", None)
        if not callable(set_own_addr):
            raise NotImplementedError(f"{self.dialect_name} does not support set_own_addr")

        owner = self._peers_by_addr.get(int(new_addr7))
        if owner is not None and owner is not self:
            raise ValueError(f"address 0x{int(new_addr7):02x} is already used by peer '{owner.name}'")

        status, confirmed, raw = set_own_addr(new_addr7, wait_timeout=wait_timeout)
        if status == "Success":
            if confirmed is None:
                confirmed = int(new_addr7)
            if self._peers_by_addr.get(self.i2c_addr7) is self:
                del self._peers_by_addr[self.i2c_addr7]
            self._peers_by_addr[confirmed] = self
            self.i2c_addr7 = confirmed
            self.dialect.i2c_addr7 = confirmed
        return status, raw


class TesterDevice(DutDevice):
    def hold_bus(self, wait_timeout: float = 1.0):
        return self.dialect.hold_bus(wait_timeout=wait_timeout)

    def release_bus(self, wait_timeout: float = 1.0):
        return self.dialect.release_bus(wait_timeout=wait_timeout)


def _parse_i2c_addr7(value) -> int | None:
    if value is None:
        return None
    if isinstance(value, str):
        return int(value, 0)
    return int(value)


def _build_backend(peer: dict):
    backend_name = peer.get("backend")
    if backend_name != "tcp_line":
        raise ValueError(f"Unsupported backend for host: {backend_name}")

    tcp_cfg = peer.get("tcp", {})
    host = resolve_host(tcp_cfg.get("host", "localhost"))
    port = int(tcp_cfg.get("port", 19021))
    return TcpLineBackend(host=host, port=port)


def _build_dialect(peer: dict, backend):
    dialect_name = peer.get("dialect")
    if dialect_name == "session_v1":
        dialect = SessionV1(backend)
    elif dialect_name == "tester_v0":
        dialect = TesterV0(backend)
    else:
        raise ValueError(f"Unsupported dialect for host: {dialect_name}")
    return dialect_name, dialect


def _build_peer(entry: dict) -> DutDevice | None:
    name = entry.get("name", "unnamed")
    kind = entry.get("kind", "dut")
    if kind == "tester":
        return None
    if kind != "dut":
        raise ValueError(f"Unsupported peer kind for host: {kind}")

    i2c_addr7 = _parse_i2c_addr7(entry.get("i2c_addr7"))
    backend = _build_backend(entry)
    dialect_name, dialect = _build_dialect(entry, backend)
    dialect.i2c_addr7 = i2c_addr7
    return DutDevice(
        name=name,
        i2c_addr7=i2c_addr7,
        backend=backend,
        dialect=dialect,
        dialect_name=dialect_name,
    )


def _build_tester_peer(entry: dict) -> TesterDevice | None:
    name = entry.get("name", "unnamed")
    kind = entry.get("kind", "dut")
    if kind != "tester":
        return None

    i2c_addr7 = _parse_i2c_addr7(entry.get("i2c_addr7"))
    backend = _build_backend(entry)
    dialect_name, dialect = _build_dialect(entry, backend)
    dialect.i2c_addr7 = i2c_addr7
    return TesterDevice(
        name=name,
        i2c_addr7=i2c_addr7,
        backend=backend,
        dialect=dialect,
        dialect_name=dialect_name,
    )


def load_peers(peers_yaml_path: str | Path) -> list[DutDevice]:
    path = Path(peers_yaml_path)
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    peer_entries = data.get("peers") or []

    peers = [peer for entry in peer_entries if (peer := _build_peer(entry)) is not None]
    peers_by_addr = {
        int(peer.i2c_addr7) & 0x7F: peer for peer in peers if peer.i2c_addr7 is not None
    }
    for peer in peers:
        peer.bind_peers_by_addr(peers_by_addr)
    return peers


def load_tester(peers_yaml_path: str | Path) -> TesterDevice | None:
    path = Path(peers_yaml_path)
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    peer_entries = data.get("peers") or []

    for entry in peer_entries:
        if entry.get("kind") == "tester":
            return _build_tester_peer(entry)
    return None
