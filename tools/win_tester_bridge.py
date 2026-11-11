"""Windows USB-serial <-> TCP bridge for the Pico tester.

Run on Windows; the WSL host connects to tcp://<win-ip>:<port> and speaks the
tester's UART command protocol.

    uv run --with pyserial python win_tester_bridge.py COM21 115200 --port 7000
"""

import argparse
import re
import socket
import threading

import serial


def pump_serial_to_tcp(ser, conn: socket.socket, alive: threading.Event) -> None:
    while alive.is_set():
        try:
            data = ser.read(4096)
            if data:
                conn.sendall(data)
        except (OSError, serial.SerialException):
            return


def _windows_com_path(name: str) -> str:
    match = re.fullmatch(r"COM(\d+)", name, re.IGNORECASE)
    return rf"\\.\{name}" if match and int(match.group(1)) >= 10 else name


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("com")
    parser.add_argument("baud", type=int)
    parser.add_argument("--port", type=int, default=7000)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.baud <= 0:
        parser.error("baud must be positive")

    with serial.Serial(_windows_com_path(args.com), args.baud, timeout=0.1) as ser:
        print("opened", ser.portstr)
        with socket.create_server(("0.0.0.0", args.port), reuse_port=False) as server:
            print(f"{args.com}@{args.baud} -> tcp://0.0.0.0:{args.port}")
            # Note: one client at a time, raw UART only; upgrade path is RFC2217.
            while True:
                conn, addr = server.accept()
                with conn:
                    print("client", addr)
                    ser.reset_input_buffer()
                    alive = threading.Event()
                    alive.set()
                    thread = threading.Thread(
                        target=pump_serial_to_tcp,
                        args=(ser, conn, alive),
                        daemon=True,
                    )
                    thread.start()
                    try:
                        while data := conn.recv(4096):
                            ser.write(data)
                    except (OSError, serial.SerialException):
                        pass
                    finally:
                        alive.clear()
                        thread.join(timeout=0.2)
                        print("client gone", addr)


if __name__ == "__main__":
    main()
