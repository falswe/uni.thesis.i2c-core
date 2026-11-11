"""Runtime change of a bridge's own target address (`ADDR`, docs/protocol.md).

The address is volatile, so every test restores the manifest address before it
finishes; otherwise later tests would address the bridge at the wrong place.
"""

import pytest

# Not used by the rig manifests or the tester (0x55).
_ALT_ADDR_CANDIDATES = (0x42, 0x43, 0x44, 0x45)
_SWITCH_CYCLES = 3
_RESERVED_ADDRS = (0x00, 0x07, 0x78, 0x7F, 0x80)


def _commands(peer) -> set[str]:
    for line in peer.dialect.info().splitlines():
        if "commands=" in line:
            return {cmd.strip().upper() for cmd in line.split("commands=", 1)[1].split(",")}
    return set()


def _require_addr_command(*devices):
    for device in devices:
        if "ADDR" not in _commands(device):
            pytest.skip(f"peer '{device.name}' firmware does not advertise ADDR")


def _alt_addr(target) -> int:
    return next(a for a in _ALT_ADDR_CANDIDATES if a not in target._peers_by_addr)


def _restore(device, home: int) -> None:
    if device.i2c_addr7 != home:
        status, raw = device.set_i2c_addr7(home)
        assert status == "Success", f"could not restore 0x{home:02x}: {raw}"


@pytest.mark.timeout(120)
def test_runtime_address_switch(ordered_pair):
    """After each switch the old address is NAKed and write, read and memory
    read work at the new address."""
    controller, target = ordered_pair
    _require_addr_command(target)
    home = target.i2c_addr7
    alt = _alt_addr(target)

    try:
        for cycle in range(_SWITCH_CYCLES):
            for old, new in ((home, alt), (alt, home)):
                status, raw = target.set_i2c_addr7(new)
                assert status == "Success", raw
                status, addr, raw = target.own_addr()
                assert (status, addr) == ("Success", new), raw

                status, raw = controller.write(old, b"\x5a")
                assert status == "AddrNak", f"old address 0x{old:02x} still answers: {raw}"

                payload = bytes([cycle, new, 0xA5])
                status, raw = controller.write(new, payload)
                assert status == "Success", raw
                status, data, raw = controller.read(new, len(payload))
                assert status == "Success", raw
                assert data == list(payload), raw
                status, data, raw = controller.mem_read(new, b"\x01", 2)
                assert status == "Success", raw
                assert data == list(payload[1:]), raw
    finally:
        _restore(target, home)


@pytest.mark.timeout(60)
def test_runtime_address_rejects_reserved_addresses(ordered_pair):
    """Reserved or out-of-range addresses are rejected and leave the previous
    address active."""
    controller, target = ordered_pair
    _require_addr_command(target)
    home = target.i2c_addr7

    try:
        for bad in _RESERVED_ADDRS:
            status, raw = target.set_i2c_addr7(bad)
            assert status == "InvalidParameter", raw
            assert target.i2c_addr7 == home
            status, addr, raw = target.own_addr()
            assert (status, addr) == ("Success", home), raw

        status, raw = controller.write(home, b"\x01")
        assert status == "Success", raw
    finally:
        _restore(target, home)


@pytest.mark.timeout(60)
def test_runtime_address_busy_while_transaction_held_open(ordered_pair):
    """A controller holding a transaction open (no STOP) refuses to change its
    own address until the transaction is closed."""
    controller, target = ordered_pair
    _require_addr_command(controller)
    home = controller.i2c_addr7
    alt = _alt_addr(controller)

    try:
        status, raw = controller.write(target.i2c_addr7, b"\x00", no_stop=True)
        assert status == "Success", raw

        status, raw = controller.set_i2c_addr7(alt)
        assert status == "Busy", raw
        assert controller.i2c_addr7 == home

        status, _data, raw = controller.read(target.i2c_addr7, 1)
        assert status == "Success", raw
    finally:
        _restore(controller, home)
