import socket
import time


class TcpLineBackend:
    def __init__(self, host: str = "localhost", port: int = 19021):
        self.host = host
        self.port = port
        self._sock: socket.socket | None = None
        self._line_buf = bytearray()

    def open(self) -> None:
        if self._sock is not None:
            return

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._connect_with_retry(timeout=4.0)
        self._sock.setblocking(False)
        time.sleep(0.1)
        self._drain()

    def close(self) -> None:
        if self._sock is None:
            return

        try:
            self._sock.close()
        finally:
            self._sock = None
            self._line_buf.clear()

    def write_line(self, line: str) -> None:
        if self._sock is None:
            raise RuntimeError("TCP socket is not open")

        payload = line if line.endswith("\n") else f"{line}\n"
        encoded = payload.encode("latin-1", errors="ignore")
        try:
            self._sock.sendall(encoded)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            self._reconnect()
            if self._sock is None:
                raise RuntimeError("TCP socket reconnect failed")
            self._sock.sendall(encoded)

    def read_line(self, timeout_s: float) -> str | None:
        if self._sock is None:
            raise RuntimeError("TCP socket is not open")

        buffered = self._pop_line()
        if buffered is not None:
            return buffered

        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            try:
                chunk = self._sock.recv(4096)
                if chunk:
                    self._line_buf.extend(chunk)
                    if b"\n" in self._line_buf:
                        break
            except BlockingIOError:
                time.sleep(0.005)

        buffered = self._pop_line()
        return buffered

    def take_prompt(self, prompt: str = ">") -> bool:
        """Consume an unterminated trailing prompt such as the tester's "> ".

        read_line() only returns complete lines, so a prompt without a newline
        stays buffered. Returns True and discards it if the buffered tail ends
        with `prompt`.
        """
        if b"\n" in self._line_buf:
            return False
        tail = self._line_buf.decode("utf-8", errors="ignore").rstrip()
        if not tail.endswith(prompt):
            return False
        self._line_buf.clear()
        return True

    def _connect_with_retry(self, timeout: float) -> None:
        if self._sock is None:
            raise RuntimeError("TCP socket is not initialized")

        deadline = time.monotonic() + timeout
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                self._sock.connect((self.host, self.port))
                return
            except (ConnectionRefusedError, OSError) as exc:
                last_error = exc
                time.sleep(0.2)
                self._sock.close()
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        raise ConnectionError(
            f"Cannot connect to TCP endpoint at {self.host}:{self.port} after {timeout:.1f}s"
        ) from last_error

    def _reconnect(self) -> None:
        self.close()
        self.open()

    def _pop_line(self) -> str | None:
        if b"\n" not in self._line_buf:
            return None
        line, _, rest = self._line_buf.partition(b"\n")
        self._line_buf[:] = rest
        return line.decode("utf-8", errors="ignore").rstrip("\r")

    def _drain(self) -> None:
        if self._sock is None:
            return

        while True:
            try:
                chunk = self._sock.recv(4096)
                if not chunk:
                    break
            except BlockingIOError:
                break
