from unittest.mock import Mock

import pytest

from i2c_test_host.tester_v0 import TesterV0


class FakeBackend:
    def __init__(self):
        self.sent = []

    def write_line(self, line):
        self.sent.append(line)

    def read_line(self, timeout_s):
        return "OK\n>"


def test_none_wait_timeout_uses_transfer_default():
    tester = TesterV0(FakeBackend())
    tester._run_cmd = Mock(return_value="OK")

    tester.write(0x65, b"", wait_timeout=None)
    tester._run_cmd.assert_called_once_with(
        "w 0x65", wait_timeout=TesterV0.TRANSFER_WAIT_TIMEOUT_S
    )


def test_hold_bus_configures_pins_then_generates_start(monkeypatch):
    monkeypatch.setenv("I2C_TESTER_HOLD_PINS", "0x06,7")
    backend = FakeBackend()
    tester = TesterV0(backend)

    status, raw = tester.hold_bus(wait_timeout=0.01)

    assert status == "Success", raw
    assert backend.sent == [
        "gpio-config 6 out open-drain",
        "gpio-config 7 out open-drain",
        "gpio-write 6 1; gpio-write 7 1",
        "gpio-write 7 0; gpio-write 6 0",
    ]


def test_release_bus_generates_stop_and_idles_high(monkeypatch):
    monkeypatch.setenv("I2C_TESTER_HOLD_PINS", "6,7")
    backend = FakeBackend()
    tester = TesterV0(backend)
    tester.hold_bus(wait_timeout=0.01)
    backend.sent.clear()

    status, raw = tester.release_bus(wait_timeout=0.01)

    assert status == "Success", raw
    assert backend.sent == [
        "gpio-write 7 0",
        "gpio-write 6 1; gpio-write 7 1",
    ]


def test_hold_pins_configured_only_once(monkeypatch):
    monkeypatch.setenv("I2C_TESTER_HOLD_PINS", "6,7")
    backend = FakeBackend()
    tester = TesterV0(backend)

    tester.hold_bus(wait_timeout=0.01)
    tester.release_bus(wait_timeout=0.01)
    tester.hold_bus(wait_timeout=0.01)

    assert backend.sent.count("gpio-config 6 out open-drain") == 1
    assert backend.sent.count("gpio-config 7 out open-drain") == 1
    assert "gpio-config 6 in" not in backend.sent
    assert "gpio-config 7 in" not in backend.sent


def test_hold_pins_must_be_scl_sda_pair(monkeypatch):
    monkeypatch.setenv("I2C_TESTER_HOLD_PINS", "6,7,8")

    with pytest.raises(ValueError):
        TesterV0(FakeBackend())
