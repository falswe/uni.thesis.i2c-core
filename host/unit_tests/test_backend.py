import socket
import time

from i2c_test_host.backend import TcpLineBackend
from i2c_test_host.tester_v0 import TesterV0


def test_tcp_line_backend_keeps_partial_line_until_newline():
    reader, writer = socket.socketpair()
    backend = TcpLineBackend()
    backend._sock = reader
    reader.setblocking(False)

    try:
        writer.sendall(b"data:")
        assert backend.read_line(timeout_s=0.01) is None

        writer.sendall(b" 0x03 0x04\n")
        assert backend.read_line(timeout_s=0.01) == "data: 0x03 0x04"
    finally:
        backend.close()
        writer.close()

def test_tester_response_ends_at_prompt_without_newline():
    reader, writer = socket.socketpair()
    backend = TcpLineBackend()
    backend._sock = reader
    reader.setblocking(False)
    tester = TesterV0(backend)

    try:
        writer.sendall(b"gpio-read 6\r\nOK: 1\r\n> ")
        start = time.monotonic()
        raw = tester._collect_block(wait_timeout=1.0)

        assert time.monotonic() - start < 0.5
        assert raw == "gpio-read 6\nOK: 1"
        assert backend.take_prompt() is False
    finally:
        backend.close()
        writer.close()
