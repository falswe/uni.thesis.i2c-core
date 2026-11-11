import pytest


@pytest.mark.timeout(45)
def test_bidirectional_round_trip(dut_pair):
    """Write then read-back in both directions.

    A plain read returns the target register file from offset 0, which holds the
    payload from the immediately preceding write, so this round trip is
    participant-agnostic: it does not depend on which peer is A vs B. Its value
    is that it exercises state carried across operations, which the per-operation
    matrix in test_operations.py deliberately does not.
    """
    peer_a, peer_b = dut_pair

    # peer_a controller -> peer_b target, then read-back.
    payload_a = [0x61, 0x62]
    status, raw = peer_a.write(peer_b.i2c_addr7, bytes(payload_a))
    assert status == "Success", f"write failed: {raw}"

    data, raw = peer_b.peek()
    assert data == payload_a, f"unexpected peek: {raw}"

    status, data, raw = peer_a.read(peer_b.i2c_addr7, len(payload_a))
    assert status == "Success", f"read failed: {raw}"
    assert data == payload_a, f"read-back mismatch: {data}, raw: {raw}"

    # peer_b controller -> peer_a target, then read-back.
    payload_b = [0x02, 0x03, 0x01]
    status, raw = peer_b.write(peer_a.i2c_addr7, bytes(payload_b))
    assert status == "Success", f"write failed: {raw}"

    data, raw = peer_a.peek()
    assert data == payload_b, f"unexpected peek: {raw}"

    status, data, raw = peer_b.read(peer_a.i2c_addr7, len(payload_b))
    assert status == "Success", f"read failed: {raw}"
    assert data == payload_b, f"read-back mismatch: {data}, raw: {raw}"

