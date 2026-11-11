#!/usr/bin/env python3
"""Generate colour-coded I2C bus timing diagrams for write, read and mem_read.

Each diagram shows the SCL and SDA levels of one session_v1 transaction and
colours every interval by the device that controls the line at that time.

Usage: python3 tools/i2c_timing_diagrams.py
Writes docs/diagrams/td_i2c_write.svg, td_i2c_read.svg and td_i2c_mem_read.svg.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from xml.sax.saxutils import escape

OUT_DIR = Path(__file__).resolve().parents[1] / "docs" / "diagrams"

IDLE, CONTROLLER, TARGET = "idle", "controller", "target"
LINE_COLOURS = {IDLE: "#9aa0a6", CONTROLLER: "#1f6fd1", TARGET: "#e8710a"}
TEXT_COLOURS = {IDLE: "#5f6368", CONTROLLER: "#1a5bb0", TARGET: "#b85600"}
LEGEND = [
    (CONTROLLER, "Controller"),
    (TARGET, "Target"),
    (IDLE, "Nobody (bus idle, pull-up holds the line high)"),
]
OPEN_DRAIN_NOTE = (
    "Open-drain: devices only pull lines low; high = released, pulled up by the resistor. "
    "Colour = device that controls the line, so a 1 bit or NACK is a released line owned by its sender."
)

HIGH, LOW = 1, 0
BIT = 36  # px per SCL period: SCL low for the first half, high for the second
HALF = BIT // 2
QUARTER = BIT // 4  # SDA changes in the middle of SCL low
EDGE = 2  # half the drawn rise/fall width
STRETCH = 3 * BIT
STRETCH_NOTE = (
    "Clock stretch is optional and may occur in any SCL low phase: the target holds "
    "SCL low until it is ready, the controller waits for SCL to rise."
)

FONT = "Helvetica, Arial, sans-serif"
MONO = "Consolas, 'DejaVu Sans Mono', monospace"
MARGIN_L, MARGIN_R = 64, 24
Y_TITLE, Y_SUBTITLE, Y_SPAN = 28, 50, 80
ROWS = {"SCL": (92, 120), "SDA": (150, 178)}  # (high y, low y)
Y_BITS, Y_BRACKET, Y_BYTES = 198, 208, 224
Y_LEGEND, NOTE_PITCH = 256, 18


@dataclass
class Line:
    changes: list[tuple[int, int, str]] = field(default_factory=list)  # (x, level, owner)

    def set(self, x: int, level: int, owner: str) -> None:
        if not self.changes or self.changes[-1][1:] != (level, owner):
            self.changes.append((x, level, owner))


@dataclass
class Transaction:
    title: str
    subtitle: str
    notes: list[str]
    scl: Line = field(default_factory=Line)
    sda: Line = field(default_factory=Line)
    x: int = BIT
    bits: list[tuple[int, str, str]] = field(default_factory=list)  # (x, label, owner)
    groups: list[tuple[int, int, str, str]] = field(default_factory=list)  # (x0, x1, label, owner)
    markers: list[tuple[int, str]] = field(default_factory=list)
    spans: list[tuple[int, int, str, str]] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.scl.set(0, HIGH, IDLE)
        self.sda.set(0, HIGH, IDLE)

    def start(self) -> None:
        x = self.x
        self.scl.set(x, HIGH, CONTROLLER)
        self.sda.set(x, LOW, CONTROLLER)
        self.markers.append((x, "S"))
        self.scl.set(x + HALF, LOW, CONTROLLER)
        self.x = x + HALF

    def repeated_start(self) -> None:
        x = self.x
        self.sda.set(x + QUARTER, HIGH, CONTROLLER)
        self.scl.set(x + HALF, HIGH, CONTROLLER)
        self.sda.set(x + HALF + QUARTER, LOW, CONTROLLER)
        self.markers.append((x + HALF + QUARTER, "Sr"))
        self.scl.set(x + BIT, LOW, CONTROLLER)
        self.x = x + BIT

    def stop(self) -> None:
        x = self.x
        self.sda.set(x + QUARTER, LOW, CONTROLLER)
        self.scl.set(x + HALF, HIGH, CONTROLLER)
        p = x + HALF + QUARTER
        self.sda.set(p, HIGH, IDLE)
        self.scl.set(p, HIGH, IDLE)
        self.markers.append((p, "P"))
        self.x = x + 2 * BIT

    def bit(self, level: int, owner: str, label: str, stretch: int = 0) -> None:
        x = self.x
        self.sda.set(x + QUARTER, level, owner)
        if stretch:
            self.scl.set(x + HALF, LOW, TARGET)
            self.spans.append((x + HALF, x + HALF + stretch, "clock stretch (optional)", TARGET))
        self.scl.set(x + HALF + stretch, HIGH, CONTROLLER)
        self.scl.set(x + BIT + stretch, LOW, CONTROLLER)
        self.bits.append((x + HALF + stretch + QUARTER, label, owner))
        self.x = x + BIT + stretch

    def ack(self, owner: str, ack: bool = True) -> None:
        self.bit(LOW if ack else HIGH, owner, "ACK" if ack else "NACK")

    def address(self, a7: int, read: bool) -> None:
        x0 = self.x
        for i in range(6, -1, -1):
            self.bit((a7 >> i) & 1, CONTROLLER, str((a7 >> i) & 1))
        rw = "R" if read else "W"
        self.bit(int(read), CONTROLLER, rw)
        self.groups.append((x0, self.x, f"address 0x{a7:02x} + {rw}", CONTROLLER))
        self.ack(TARGET)

    def byte(self, value: int, sender: str, label: str, ack: bool = True, stretch: int = 0) -> None:
        x0 = self.x + stretch
        for i in range(7, -1, -1):
            self.bit((value >> i) & 1, sender, str((value >> i) & 1), stretch if i == 7 else 0)
        self.groups.append((x0, self.x, f"{label} 0x{value:02x}", sender))
        self.ack(TARGET if sender == CONTROLLER else CONTROLLER, ack)

    def render(self) -> str:
        width = MARGIN_L + self.x + MARGIN_R
        height = Y_LEGEND + NOTE_PITCH * (len(self.notes) + 1) + 16
        top, bottom = ROWS["SCL"][0] - 6, ROWS["SDA"][1] + 6
        out = [
            (
                f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
                f'viewBox="0 0 {width} {height}" font-family="{FONT}">'
            ),
            f"<title>{escape(self.title)}</title>",
            f'<rect width="{width}" height="{height}" fill="#ffffff"/>',
            _text(16, Y_TITLE, self.title, "#202124", 18, True, "start"),
            _text(16, Y_SUBTITLE, self.subtitle, "#3c4043", 13, anchor="start", family=MONO),
            f'<g transform="translate({MARGIN_L},0)">',
        ]
        for x, _, _ in self.bits:
            out.append(_vline(x, top, bottom, "#e8eaed", "2 3"))
        for x, label in self.markers:
            out.append(_vline(x, top - 8, bottom, "#80868b", "4 3"))
            out.append(_text(x, Y_BITS, label, "#202124", bold=True))
        for x0, x1, label, owner in self.spans:
            out.append(_bracket(x0, x1, Y_SPAN + 4, -4, owner))
            out.append(_text((x0 + x1) / 2, Y_SPAN, label, TEXT_COLOURS[owner], size=11))
        for name, (y_high, y_low) in ROWS.items():
            out.append(_text(-14, (y_high + y_low) / 2 + 5, name, "#202124", 14, True, "end"))
            line = self.scl if name == "SCL" else self.sda
            out.extend(_waveform(line, y_high, y_low, self.x))
        for x, label, owner in self.bits:
            out.append(_text(x, Y_BITS, label, TEXT_COLOURS[owner], size=10 if len(label) > 1 else 12))
        for x0, x1, label, owner in self.groups:
            out.append(_bracket(x0, x1, Y_BRACKET, 4, owner))
            out.append(_text((x0 + x1) / 2, Y_BYTES, label, TEXT_COLOURS[owner], size=12))
        out.append("</g>")
        x = 16
        for owner, label in LEGEND:
            colour = LINE_COLOURS[owner]
            out.append(
                f'<line x1="{x}" y1="{Y_LEGEND - 4}" x2="{x + 26}" y2="{Y_LEGEND - 4}" '
                f'stroke="{colour}" stroke-width="3" stroke-linecap="round"/>'
            )
            out.append(_text(x + 34, Y_LEGEND, label, "#202124", 12, anchor="start"))
            x += 34 + 7 * len(label) + 28
        for i, note in enumerate([OPEN_DRAIN_NOTE, *self.notes], start=1):
            out.append(_text(16, Y_LEGEND + NOTE_PITCH * i, note, "#5f6368", 11, anchor="start"))
        out.append("</svg>")
        return "\n".join(out) + "\n"


def _waveform(line: Line, y_high: int, y_low: int, x_end: int) -> list[str]:
    def y(level: int) -> int:
        return y_high if level else y_low

    pieces: list[tuple[str, list[tuple[float, float]]]] = []
    changes = line.changes
    for i, (x, level, owner) in enumerate(changes):
        start = x
        if i and changes[i - 1][1] != level:
            prev_level, prev_owner = changes[i - 1][1:]
            # A release into the idle bus is still the releasing device's edge.
            edge_owner = prev_owner if owner == IDLE else owner
            pieces.append((edge_owner, [(x - EDGE, y(prev_level)), (x + EDGE, y(level))]))
            start = x + EDGE
        if i + 1 < len(changes):
            nx, next_level, _ = changes[i + 1]
            end = nx - EDGE if next_level != level else nx
        else:
            end = x_end
        pieces.append((owner, [(start, y(level)), (end, y(level))]))

    polylines: list[tuple[str, list[tuple[float, float]]]] = []
    for owner, points in pieces:
        if polylines and polylines[-1][0] == owner and polylines[-1][1][-1] == points[0]:
            polylines[-1][1].append(points[1])
        else:
            polylines.append((owner, list(points)))
    return [
        f'<polyline points="{" ".join(f"{px:g},{py:g}" for px, py in points)}" fill="none" '
        f'stroke="{LINE_COLOURS[owner]}" stroke-width="2.5" stroke-linecap="round" '
        f'stroke-linejoin="round"/>'
        for owner, points in polylines
    ]


def _text(
    x: float,
    y: float,
    text: str,
    colour: str,
    size: int = 12,
    bold: bool = False,
    anchor: str = "middle",
    family: str | None = None,
) -> str:
    weight = ' font-weight="bold"' if bold else ""
    font = f' font-family="{family}"' if family else ""
    return (
        f'<text x="{x:g}" y="{y:g}" font-size="{size}" fill="{colour}" '
        f'text-anchor="{anchor}"{weight}{font}>{escape(text)}</text>'
    )


def _vline(x: float, y0: float, y1: float, colour: str, dash: str) -> str:
    return f'<line x1="{x:g}" y1="{y0}" x2="{x:g}" y2="{y1}" stroke="{colour}" stroke-dasharray="{dash}"/>'


def _bracket(x0: float, x1: float, y: float, tick: int, owner: str) -> str:
    x0, x1 = x0 + 3, x1 - 3
    return (
        f'<polyline points="{x0:g},{y - tick} {x0:g},{y} {x1:g},{y} {x1:g},{y - tick}" '
        f'fill="none" stroke="{LINE_COLOURS[owner]}" stroke-width="1.5"/>'
    )


def write_diagram() -> Transaction:
    t = Transaction(
        "I2C write",
        "W 0x12 0x0b 0x0c   |   host: write(0x12, bytes([0x0b, 0x0c]))",
        [
            (
                "Controller sends address + W and every data byte; target ACKs each byte "
                "by pulling SDA low during the 9th clock."
            ),
            (
                "Here the target stretches after its address ACK while it arms reception; "
                "it can also stretch after any later byte."
            ),
            STRETCH_NOTE,
        ],
    )
    t.start()
    t.address(0x12, read=False)
    t.byte(0x0B, CONTROLLER, "data", stretch=STRETCH)
    t.byte(0x0C, CONTROLLER, "data")
    t.stop()
    return t


def read_diagram() -> Transaction:
    t = Transaction(
        "I2C read",
        "R 0x12 2 → data: 0x0b 0x0c   |   host: read(0x12, 2)",
        [
            (
                "Target sends every data byte after its address ACK; controller ACKs each byte "
                "except the last, which it NACKs before STOP."
            ),
            (
                "Here the target stretches after its address ACK until its first data byte is "
                "ready; it can also stretch before any later byte."
            ),
            STRETCH_NOTE,
        ],
    )
    t.start()
    t.address(0x12, read=True)
    t.byte(0x0B, TARGET, "data", stretch=STRETCH)
    t.byte(0x0C, TARGET, "data", ack=False)
    t.stop()
    return t


def mem_read_diagram() -> Transaction:
    t = Transaction(
        "I2C mem_read",
        "W np 0x12 0x01 + R 0x12 1 → data: 0x0c   |   host: mem_read(0x12, bytes([0x01]), 1)",
        [
            (
                "No-stop write selects the register offset; repeated START (Sr) turns the bus "
                "around for the read; STOP only at the end."
            ),
            STRETCH_NOTE,
        ],
    )
    t.start()
    t.address(0x12, read=False)
    t.byte(0x01, CONTROLLER, "register")
    t.repeated_start()
    t.address(0x12, read=True)
    t.byte(0x0C, TARGET, "data", ack=False, stretch=STRETCH)
    t.stop()
    return t


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name, build in (
        ("td_i2c_write.svg", write_diagram),
        ("td_i2c_read.svg", read_diagram),
        ("td_i2c_mem_read.svg", mem_read_diagram),
    ):
        path = OUT_DIR / name
        path.write_text(build().render(), encoding="utf-8")
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
