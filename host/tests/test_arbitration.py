import time
from concurrent.futures import ThreadPoolExecutor

import pytest

pytestmark = pytest.mark.both_manifests


@pytest.mark.timeout(20)
def test_targeted_controller_loses_write_after_tester_releases_bus(dut_pair, tester):
    dut_a, dut_b = dut_pair

    # Lower transmitted target address wins.
    if dut_a.i2c_addr7 < dut_b.i2c_addr7:
        winner = dut_b   # targets dut_a's lower address
        loser = dut_a
    else:
        winner = dut_a
        loser = dut_b

    winner_target = loser
    loser_target = winner

    winner_payload = bytes([0xA1, 0xA2])
    loser_payload = bytes([0xB1, 0xB2])

    status, raw = tester.hold_bus()
    assert status == "Success", raw

    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            winner_future = pool.submit(
                winner.write,
                winner_target.i2c_addr7,
                winner_payload,
            )
            loser_future = pool.submit(
                loser.write,
                loser_target.i2c_addr7,
                loser_payload,
            )

            time.sleep(0.2)

            status, raw = tester.release_bus()
            assert status == "Success", raw

            winner_status, winner_raw = winner_future.result(timeout=3)
            loser_status, loser_raw = loser_future.result(timeout=3)

            if loser_status == "Success" and winner_status == "Success":
                pytest.skip("Both controllers succeeded unexpectedly.")
    finally:
        tester.release_bus()

    print("WINNER:", winner.name, winner_status, repr(winner_raw))
    print("LOSER:", loser.name, loser_status, repr(loser_raw))

    assert winner_status == "Success", winner_raw
    assert loser_status == "ArbitrationLost", loser_raw

    data, raw = winner_target.peek()
    assert data == list(winner_payload), raw


@pytest.mark.timeout(25)
def test_targeted_controller_loses_read_after_tester_releases_bus(dut_pair, tester):
    dut_a, dut_b = dut_pair

    # The lower transmitted address+R byte wins arbitration.
    if dut_a.i2c_addr7 < dut_b.i2c_addr7:
        winner = dut_b   # reads dut_a's lower address
        loser = dut_a
    else:
        winner = dut_a
        loser = dut_b

    winner_target = loser
    loser_target = winner

    winner_data = bytes([0xC1, 0xC2])
    loser_data = bytes([0xD1, 0xD2])

    # Seed both target register files before either controller is armed.
    status, raw = winner.write(
        winner_target.i2c_addr7,
        winner_data,
    )
    assert status == "Success", raw

    status, raw = loser.write(
        loser_target.i2c_addr7,
        loser_data,
    )
    assert status == "Success", raw

    status, raw = tester.hold_bus()
    assert status == "Success", raw

    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            winner_future = pool.submit(
                winner.read,
                winner_target.i2c_addr7,
                len(winner_data),
            )
            loser_future = pool.submit(
                loser.read,
                loser_target.i2c_addr7,
                len(loser_data),
            )

            time.sleep(0.2)

            status, raw = tester.release_bus()
            assert status == "Success", raw

            winner_status, winner_rx, winner_raw = winner_future.result(timeout=3)
            loser_status, loser_rx, loser_raw = loser_future.result(timeout=3)

            if loser_status == "Success" and winner_status == "Success":
                pytest.skip("Both controllers succeeded unexpectedly.")
    finally:
        tester.release_bus()

    print("WINNER:", winner.name, winner_status, winner_rx, repr(winner_raw))
    print("LOSER:", loser.name, loser_status, loser_rx, repr(loser_raw))

    assert winner_status == "Success", winner_raw
    assert winner_rx == list(winner_data), winner_raw
    assert loser_status == "ArbitrationLost", loser_raw
