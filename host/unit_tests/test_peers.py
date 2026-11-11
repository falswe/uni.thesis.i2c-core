from pathlib import Path

import pytest

from i2c_test_host import peers


class DialectWithTransferDefaults:
    def write(self, _a7, _payload, wait_timeout=None, no_stop=False):
        return 1.0 if wait_timeout is None else wait_timeout

    def read(self, _a7, _length, wait_timeout=None, no_stop=False):
        return 2.0 if wait_timeout is None else wait_timeout

    def mem_read(self, _a7, _payload, _read_length, wait_timeout=None):
        return 3.0 if wait_timeout is None else wait_timeout


def test_dut_device_preserves_dialect_transfer_timeout_defaults():
    peer = peers.DutDevice("dut", 0x65, None, DialectWithTransferDefaults(), "test")

    assert peer.write(0x66, b"\x01") == 1.0
    assert peer.read(0x66, 1) == 2.0
    assert peer.mem_read(0x66, b"\x00", 1) == 3.0
    assert peer.write(0x66, b"\x01", wait_timeout=4.0) == 4.0
    assert peer.read(0x66, 1, wait_timeout=5.0) == 5.0
    assert peer.mem_read(0x66, b"\x00", 1, wait_timeout=6.0) == 6.0


def test_resolve_host_uses_lowest_metric_default_gateway(monkeypatch, tmp_path: Path):
    route_path = tmp_path / "route"
    route_path.write_text(
        "Iface Destination Gateway Flags RefCnt Use Metric Mask MTU Window IRTT\n"
        "eth1 00000000 0100000A 0003 0 0 50 00000000 0 0 0\n"
        "eth0 00000000 013018AC 0003 0 0 0 00000000 0 0 0\n",
        encoding="ascii",
    )
    monkeypatch.setattr(peers, "_PROC_NET_ROUTE", route_path)

    assert peers.resolve_host("wsl-host") == "172.24.48.1"


def test_resolve_host_preserves_explicit_host(monkeypatch, tmp_path: Path):
    monkeypatch.setattr(peers, "_PROC_NET_ROUTE", tmp_path / "missing")

    assert peers.resolve_host("192.0.2.10") == "192.0.2.10"

class AddrDialect:
    def __init__(self, status="Success"):
        self.status = status
        self.i2c_addr7 = None
        self.sent = []

    def set_own_addr(self, a7, wait_timeout=None):
        self.sent.append(a7)
        reported = a7 if self.status == "Success" else self.i2c_addr7
        return self.status, reported, f"ADDR status={self.status}"


def _bound_pair(dialect):
    a = peers.DutDevice("a", 0x65, None, dialect, "test")
    b = peers.DutDevice("b", 0x66, None, AddrDialect(), "test")
    dialect.i2c_addr7 = 0x65
    by_addr = {0x65: a, 0x66: b}
    a.bind_peers_by_addr(by_addr)
    b.bind_peers_by_addr(by_addr)
    return a, b, by_addr


def test_set_i2c_addr7_updates_shared_address_map_after_success():
    dialect = AddrDialect()
    a, b, by_addr = _bound_pair(dialect)

    status, _raw = a.set_i2c_addr7(0x42)

    assert status == "Success"
    assert (a.i2c_addr7, dialect.i2c_addr7) == (0x42, 0x42)
    assert by_addr == {0x42: a, 0x66: b}


def test_set_i2c_addr7_keeps_address_map_on_failure():
    dialect = AddrDialect(status="Busy")
    a, b, by_addr = _bound_pair(dialect)

    status, _raw = a.set_i2c_addr7(0x42)

    assert status == "Busy"
    assert a.i2c_addr7 == 0x65
    assert by_addr == {0x65: a, 0x66: b}


def test_set_i2c_addr7_rejects_address_of_another_peer_without_sending():
    dialect = AddrDialect()
    a, _b, _by_addr = _bound_pair(dialect)

    with pytest.raises(ValueError):
        a.set_i2c_addr7(0x66)
    assert dialect.sent == []
