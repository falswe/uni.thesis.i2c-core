"""Transfers through the transaction layer (`Q`, `TXN`, docs/protocol.md).

Checks the rules of docs/transaction_layer.md sections 3 and 4 with the
baseline access policy of section 5.1. Bridges that do not advertise `Q` and
`TXN` are skipped. Every test restores the parameters it changes.
"""

import time
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager

import pytest

# Not used by the rig manifests or the tester (0x55).
_ABSENT_ADDR = 0x33


def _commands(peer) -> set[str]:
    for line in peer.dialect.info().splitlines():
        if "commands=" in line:
            return {cmd.strip().upper() for cmd in line.split("commands=", 1)[1].split(",")}
    return set()


def _has_txn(peer) -> bool:
    return {"Q", "TXN"} <= _commands(peer)


def _txn_and_other(dut_pair):
    a, b = dut_pair
    if _has_txn(a):
        return a, b
    if _has_txn(b):
        return b, a
    pytest.skip("no peer firmware advertises Q and TXN")


@contextmanager
def _txn_params(device, **params):
    status, before, raw = device.txn_params()
    assert status == "Success", raw
    status, _params, raw = device.txn_params(**params)
    assert status == "Success", raw
    try:
        yield
    finally:
        status, _params, raw = device.txn_params(
            f_ms=before["F_ms"], backoff_ms=before["backoff_ms"], stuck_ms=before["stuck_ms"]
        )
        assert status == "Success", raw


@pytest.mark.timeout(30)
def test_q_write_read_and_memory_read(ordered_pair):
    controller, target = ordered_pair
    if not _has_txn(controller):
        pytest.skip(f"peer '{controller.name}' firmware does not advertise Q and TXN")
    payload = bytes([0x11, 0x22, 0x33, 0x44])

    status, fields, data, raw = controller.q("W", target.i2c_addr7, payload)
    assert status == "Success", raw
    assert (fields["last"], fields["attempts"], fields["arlo"], fields["addrnak"]) == ("Success", "1", "0", "0"), raw
    assert data == list(payload), raw
    peek, raw = target.peek()
    assert peek == list(payload), raw

    status, _fields, data, raw = controller.q("R", target.i2c_addr7, read_length=3)
    assert (status, data) == ("Success", list(payload[:3])), raw

    status, _fields, data, raw = controller.q("WR", target.i2c_addr7, b"\x02", read_length=2)
    assert (status, data) == ("Success", list(payload[2:])), raw


@pytest.mark.timeout(30)
def test_q_backs_off_after_addr_nak_until_budget(ordered_pair):
    controller, _target = ordered_pair
    if not _has_txn(controller):
        pytest.skip(f"peer '{controller.name}' firmware does not advertise Q and TXN")

    with _txn_params(controller, f_ms=300, backoff_ms=100):
        status, fields, _data, raw = controller.q("W", _ABSENT_ADDR, b"\x5a")

    assert status == "Timeout", raw
    assert fields["last"] == "AddrNak", raw
    # Attempts at about 0, 100 and 200 ms; none is started once F has run out.
    assert fields["attempts"] == fields["addrnak"] == "3", raw
    assert 300_000 <= int(fields["head_us"]) < 310_000, raw


@pytest.mark.both_manifests
@pytest.mark.timeout(30)
def test_q_retries_after_arbitration_loss(dut_pair, tester):
    """Both controllers start when the tester releases the bus, and the lower
    transmitted target address wins. A transaction-layer controller that loses
    serves the winner as target and then tries again (4.2, rule 2)."""
    txn_dev, other = _txn_and_other(dut_pair)
    txn_loses = other.i2c_addr7 > txn_dev.i2c_addr7
    txn_payload = bytes([0xE1, 0xE2])
    other_payload = bytes([0xF1, 0xF2])

    status, raw = tester.hold_bus()
    assert status == "Success", raw
    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            q_future = pool.submit(txn_dev.q, "W", other.i2c_addr7, txn_payload)
            w_future = pool.submit(other.write, txn_dev.i2c_addr7, other_payload)
            time.sleep(0.2)
            status, raw = tester.release_bus()
            assert status == "Success", raw
            q_status, fields, _data, q_raw = q_future.result(timeout=5)
            w_status, w_raw = w_future.result(timeout=5)
    finally:
        tester.release_bus()

    assert q_status == "Success", q_raw
    if txn_loses:
        assert w_status == "Success", w_raw
        assert int(fields["arlo"]) >= 1 and int(fields["attempts"]) >= 2, q_raw
        data, raw = txn_dev.peek()
        assert data == list(other_payload), raw
    else:
        assert w_status == "ArbitrationLost", w_raw
        assert (fields["arlo"], fields["attempts"]) == ("0", "1"), q_raw
    data, raw = other.peek()
    assert data == list(txn_payload), raw


@pytest.mark.timeout(30)
def test_q_times_out_busy_while_the_bus_stays_held(dut_pair, tester):
    """An armed START that the platform does not withdraw (platform_capabilities.md
    C2) waits for the bus until F + stuck; then the transfer ends with Timeout,
    last Busy. The retired attempt still runs once the bus is free, so the
    target may have received the data (4.3), and the bridge refuses new
    transfers until then."""
    txn_dev, other = _txn_and_other(dut_pair)
    payload = bytes([0xC3, 0xC4])

    with _txn_params(txn_dev, f_ms=200, stuck_ms=200):
        status, raw = tester.hold_bus()
        assert status == "Success", raw
        try:
            status, fields, _data, raw = txn_dev.q("W", other.i2c_addr7, payload)
            assert status == "Timeout", raw
            assert (fields["last"], fields["attempts"]) == ("Busy", "1"), raw
            assert 400_000 <= int(fields["head_us"]) < 450_000, raw

            status, _fields, _data, raw = txn_dev.q("W", other.i2c_addr7, b"\x01")
            assert status == "Busy", raw
        finally:
            status, raw = tester.release_bus()
            assert status == "Success", raw

        time.sleep(0.1)
        data, raw = other.peek()
        assert data == list(payload), raw

        status, _fields, _data, raw = txn_dev.q("W", other.i2c_addr7, b"\x01")
        assert status == "Success", raw
