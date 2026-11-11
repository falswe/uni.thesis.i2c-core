from unittest.mock import Mock

import pytest
from i2c_test_host.peers import DutDevice
from i2c_test_host.session_v1 import SessionV1


class FakeBackend:
    def __init__(self, responses=()):
        self.responses = [list(response) for response in responses]
        self.active = []
        self.sent = []

    def write_line(self, line):
        self.sent.append(line)
        self.active = self.responses.pop(0) if self.responses else []

    def read_line(self, timeout_s):
        return self.active.pop(0) if self.active else None


def test_session_peer_open_retries_readiness_probe_once():
    backend = Mock()
    session = SessionV1(backend)
    session.info = Mock(side_effect=("", "INFO protocol=session_v1"))
    peer = DutDevice("stm", 0x65, backend, session, "other_name")

    peer.open()

    backend.open.assert_called_once_with()
    assert session.info.call_count == 2


def test_session_peer_default_wait_exceeds_firmware_controller_turn_timeout():
    backend = FakeBackend([["W status=Success"]])
    session = SessionV1(backend)
    session._collect_block = Mock(return_value="W status=Success")
    peer = DutDevice("stm", 0x65, backend, session, "session_v1")

    status, _raw = peer.write(0x66, b"\x01")

    assert SessionV1.TRANSFER_WAIT_TIMEOUT_S > 5.0
    assert status == "Success"
    session._collect_block.assert_called_once_with(
        max_total_s=SessionV1.TRANSFER_WAIT_TIMEOUT_S
    )


def test_mem_read_sends_no_stop_write_then_read():
    backend = FakeBackend([
        ["W status=Success"],
        ["R status=Success", "data: 0xaa 0xbb"],
    ])
    client = SessionV1(backend)

    status, data, raw = client.mem_read(0x4A, bytes([0x10]), 2, wait_timeout=0.01)

    assert backend.sent == ["W np 0x4a 0x10", "R 0x4a 2"]
    assert status == "Success"
    assert data == [0xAA, 0xBB]
    assert "W status=Success" in raw
    assert "R status=Success" in raw


def test_mem_read_stops_after_failed_no_stop_write():
    backend = FakeBackend([["W status=AddrNak"]])
    client = SessionV1(backend)

    status, data, raw = client.mem_read(0x4A, bytes([0x10]), 2, wait_timeout=0.01)

    assert backend.sent == ["W np 0x4a 0x10"]
    assert status == "AddrNak"
    assert data == []
    assert raw == "W status=AddrNak"


def test_write_accepts_np_seg_token():
    backend = FakeBackend([["W status=Success"]])
    client = SessionV1(backend)

    status, _raw = client.write(0x4A, b"\x10", wait_timeout=0.01, seg="np")

    assert backend.sent == ["W np 0x4a 0x10"]
    assert status == "Success"


def test_write_normalizes_mixed_case_explicit_seg_token():
    backend = FakeBackend([["W status=Success"]])
    client = SessionV1(backend)

    status, _raw = client.write(0x4A, b"\x10", wait_timeout=0.01, seg="NS-NP")

    assert backend.sent == ["W ns-np 0x4a 0x10"]
    assert status == "Success"


def test_read_seg_token_overrides_no_stop():
    backend = FakeBackend([["R status=Success", "data: 0x01"]])
    client = SessionV1(backend)

    status, data, _raw = client.read(
        0x4A, 1, wait_timeout=0.01, seg="s-p", no_stop=True
    )

    assert backend.sent == ["R s-p 0x4a 1"]
    assert status == "Success"
    assert data == [0x01]


def test_invalid_seg_token_raises():
    client = SessionV1(FakeBackend([]))

    with pytest.raises(ValueError):
        client.write(0x4A, b"\x10", seg="bogus")


def test_write_and_read_return_canonical_timeout_on_empty_backend():
    backend = FakeBackend()
    session = SessionV1(backend)

    status, raw = session.write(0x12, b"", wait_timeout=0.01)
    assert (status, raw) == ("Timeout", "")

    status, data, raw = session.read(0x12, 1, wait_timeout=0.01)
    assert (status, data, raw) == ("Timeout", [], "")


def test_own_addr_queries_without_argument():
    backend = FakeBackend([["ADDR status=Success", "addr: 0x65"]])
    client = SessionV1(backend)

    status, addr, raw = client.own_addr(wait_timeout=0.01)

    assert backend.sent == ["ADDR"]
    assert (status, addr) == ("Success", 0x65)
    assert "addr: 0x65" in raw


def test_set_own_addr_sends_unmasked_address_and_reports_address_in_effect():
    backend = FakeBackend([["ADDR status=InvalidParameter", "addr: 0x65"]])
    client = SessionV1(backend)

    status, addr, _raw = client.set_own_addr(0x80, wait_timeout=0.01)

    assert backend.sent == ["ADDR 0x80"]
    assert (status, addr) == ("InvalidParameter", 0x65)


def test_q_write_read_formats_command_and_parses_report():
    backend = FakeBackend(
        [
            [
                "Q status=Success last=Success attempts=2 arlo=1 addrnak=0 queue_us=0 head_us=412",
                "data: 0x03 0x04",
            ]
        ]
    )
    client = SessionV1(backend)

    status, fields, data, _raw = client.q("wr", 0x60, b"\x02", read_length=2, wait_timeout=0.01)

    assert backend.sent == ["Q WR 0x60 2 0x02"]
    assert status == "Success"
    assert fields["attempts"] == "2" and fields["arlo"] == "1" and fields["last"] == "Success"
    assert data == [0x03, 0x04]


def test_q_without_report_is_timeout_and_rejects_unknown_kind():
    client = SessionV1(FakeBackend())

    status, fields, data, raw = client.q("W", 0x60, b"\x01", wait_timeout=0.01)
    assert (status, fields, data, raw) == ("Timeout", {}, [], "")

    with pytest.raises(ValueError):
        client.q("X", 0x60)


def test_txn_params_sends_given_parameters_and_parses_report():
    backend = FakeBackend([["TXN status=Success", "txn F_ms=300 backoff_ms=50 stuck_ms=1000 queue=8"]])
    client = SessionV1(backend)

    status, params, _raw = client.txn_params(f_ms=300, backoff_ms=50, wait_timeout=0.01)

    assert backend.sent == ["TXN F=300 backoff=50"]
    assert status == "Success"
    assert params == {"F_ms": 300, "backoff_ms": 50, "stuck_ms": 1000, "queue": 8}
