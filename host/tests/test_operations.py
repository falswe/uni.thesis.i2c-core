import pytest


@pytest.mark.timeout(45)
def test_write_read(ordered_pair):
    """A write resets the target register file to 0xFF and stores the payload,
    so a read returns the written bytes followed by 0xFF for the unwritten tail.
    The prior, longer write proves the reset clears bytes the second write does
    not cover (see docs/target_model.md)."""
    controller, target = ordered_pair

    # Prior contents the next write must clear.
    status, raw = controller.write(
        target.i2c_addr7, bytes([1, 2, 3, 4, 5])
    )
    assert status == "Success", raw

    status, raw = controller.write(target.i2c_addr7, bytes([11, 12]))
    assert status == "Success", raw

    # Read past the written bytes: stored bytes, then 0xFF (reset cleared 3,4,5).
    status, data, raw = controller.read(target.i2c_addr7, 4)
    assert status == "Success", raw
    assert data == [11, 12, 0xFF, 0xFF], (
        f"register-file readback mismatch: {data}, raw: {raw}"
    )


@pytest.mark.timeout(45)
def test_mem_read(ordered_pair):
    """A memory read: the no-stop write selects a big-endian byte offset and the
    repeated-start read returns bytes from that offset. Single- and multi-byte
    addresses decode to the same offset (see docs/target_model.md)."""
    controller, target = ordered_pair

    status, raw = controller.write(
        target.i2c_addr7, bytes([1, 2, 3, 4, 5])
    )
    assert status == "Success", raw

    # Offset 2 (big-endian) -> bytes [3, 4].
    for mem_addr in ([0x02], [0x00, 0x02]):
        status, data, raw = controller.mem_read(
            target.i2c_addr7, bytes(mem_addr), 2
        )
        assert status == "Success", raw
        assert data == [3, 4], f"memory read {mem_addr} mismatch: {data}, raw: {raw}"
