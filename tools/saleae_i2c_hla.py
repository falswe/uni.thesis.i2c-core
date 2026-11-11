import os
from pathlib import Path

from saleae.analyzers import AnalyzerFrame, HighLevelAnalyzer

LOG = Path.home() / "saleae-i2c.log"


class Hla(HighLevelAnalyzer):
    result_types = {
        "transaction": {"format": "{{data.text}}"}
    }

    def __init__(self):
        self.active = False
        self.parts = []
        fd = os.open(LOG, os.O_CREAT | os.O_WRONLY | os.O_TRUNC, 0o644)
        os.close(fd)

    def write(self, text):
        print(text)
        fd = os.open(LOG, os.O_CREAT | os.O_WRONLY | os.O_APPEND, 0o644)
        try:
            os.write(fd, (text + "\n").encode("utf-8"))
        finally:
            os.close(fd)

    def output(self, text, frame):
        self.write(text)
        return AnalyzerFrame(
            "transaction",
            frame.start_time,
            frame.end_time,
            {"text": text},
        )

    def decode(self, frame):
        if frame.type == "start":
            if self.active:
                text = "UNFINISHED: " + " ".join(self.parts)
                self.parts = ["repeated-start"]
                return self.output(text, frame)

            self.active = True
            self.parts = ["start"]
            return None

        if frame.type == "address":
            address = frame.data["address"][0]
            direction = "read" if frame.data["read"] else "write"
            ack = "ack" if frame.data["ack"] else "nak"
            self.parts.append(f"{direction} 0x{address:02x} {ack}")
            return None

        if frame.type == "data":
            value = frame.data["data"][0]
            ack = "ack" if frame.data["ack"] else "nak"
            self.parts.append(f"data 0x{value:02x} {ack}")
            return None

        if frame.type == "stop":
            if not self.active:
                return self.output("stop without start", frame)

            self.parts.append("stop")
            text = " | ".join(self.parts)
            self.active = False
            self.parts = []
            return self.output(text, frame)

        return None