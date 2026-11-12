#!/usr/bin/env python3
"""Simulate how bus faults end under two timeout designs for a multi-controller I2C bus.

Four participants share one bus: A sends to the target B, C sends background
traffic to the target D (8 bytes every c_period). Each scenario injects one fault.
A run shows when and how each participant ends its transfers, how long C waits,
who reports a bus failure, which data may be lost or wrong without an error, and
in which state the bus ends.

Designs:
- Layered limits (docs/transaction_layer.md). The flush budget F limits only how
  long a controller keeps trying; it never cuts an attempt on the bus. A target
  releases SCL and SDA after S_max of its own stretch and abandons the transfer.
  A controller ends its transaction with a forced STOP B_hold after its START.
  SCL low, SDA low while SCL high, or a missing STOP (busy, both lines high) for
  t_stuck is a bus failure: the participant reports it and enters failure mode
  (no part in the bus until restart; an owed STOP is still sent). After a reset a
  participant waits T_max before its first access as controller and then reads
  the pins. The controller firmware is the real transaction layer,
  firmware/txn/txn.c, compiled into a shared library and driven through its
  platform interface; the limits above are platform behaviour that the bridges
  do not implement yet (docs/platform_capabilities.md section 5).
- Single supervision timer. One timer of length F per transfer, started at the
  head of the queue, covers bus waits, retries and the attempt on the bus; the
  peripheral's SCL-low timeout is set to the same F and so adds nothing. When the
  timer expires the transfer has failed and the controller terminates it
  "unconditionally", read in two ways: "release" (get off the bus: reset the own
  peripheral, no STOP) and "stop" (STOP as soon as SCL is free). An armed START
  is withdrawn by a peripheral reset in both readings. After an address or data
  NAK the controller retries after F / 10, after an arbitration loss or a bus
  error at once. A target that has stretched for its stretch limit becomes ready
  and keeps acknowledging. No wait after reset, no stuck-bus detection.

Peripheral profiles of the layered design (platform_capabilities.md, documented):
- STM32U375: TIMEOUTB with TEXTEN = S_max bounds the own stretch as target and the
  cumulative own clock hold as controller without firmware (C6, C8); TIMEOUTA
  bounds SCL low, whoever holds it, up to 174.8 ms, also without firmware (C9);
  firmware reads SCL and SDA through the GPIO input register (C9).
- Generic short timeout: one timer of at most 16 ms that counts SCL low, or no
  bus event while the peripheral considers the bus busy, and only interrupts the
  firmware. Every limit is firmware counting this timer, so it needs running
  firmware; the pins cannot be read.
Neither profile can withdraw an armed START (C2).

Model (byte level, integer nanoseconds):
- A controller arms its START when its own busy flag is clear and nobody holds
  SCL or SDA low, waits t_buf and starts; simultaneous STARTs are arbitrated by
  the target address. Every enabled peripheral sets its busy flag on a START and
  clears it on a STOP; a peripheral enabled later has missed the START.
- A byte takes 9 bit-times. At each byte boundary the addressed target may
  stretch and a stalled controller holds SCL; the next byte starts when nobody
  holds SCL. A forced STOP is sent at the next byte boundary (the spec NACKs one
  more byte in reads; one byte time is ignored here).
- A START while another controller owns the bus (a participant that joined
  without waiting) is assumed to cut the running transfer as a bus error. The
  hardware behaviour is not documented (assumption; needs a hardware test).
- Detectors have no timer granularity; participants that started counting at the
  same instant detect together. The main loop runs the transaction layer every
  `loop` while it has work, and at once after an attempt ends.

Every value in Timing carries its evidence (EVIDENCE). A private TOML file can
replace values and the generic profile; see load_params().

Usage:
    python3 tools/bus_timeout_sim.py            write docs/diagrams/timeout_*.svg, print a summary
    python3 tools/bus_timeout_sim.py --check    run self-checks, exit 1 on failure
    python3 tools/bus_timeout_sim.py --params FILE [--out DIR]
        values and generic profile from FILE; writes to DIR (default: out/ next
        to FILE), never into docs/diagrams
"""

from __future__ import annotations

import argparse
import ctypes
import heapq
import math
import shutil
import subprocess
import sys
import tempfile
import tomllib
from collections import deque
from dataclasses import dataclass, fields, replace
from pathlib import Path
from typing import Callable, Iterator

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bus_policy_sim as bps  # noqa: E402  shared SVG helpers and layout checks

US, MS, SECOND = bps.US, bps.MS, bps.SECOND
EDGE, INK, GREY, GRID, ALARM, GOOD = bps.EDGE, bps.INK, bps.GREY, bps.GRID, bps.ALARM, bps.GOOD
_text, _rect, _line, _wrap, _duration = bps._text, bps._rect, bps._line, bps._wrap, bps._duration

REPO = Path(__file__).resolve().parents[1]
TXN_DIR = REPO / "firmware" / "txn"
OUT_DIR = REPO / "docs" / "diagrams"
FOREVER = 1 << 62

# txn_outcome_t of firmware/txn/txn.h
NONE, SUCCESS, ARLO, ADDR_NAK, DATA_NAK, ABORTED, TIMEOUT, BUSY = range(8)
OUTCOME = {NONE: "lost in reset", SUCCESS: "Success", ARLO: "ArbitrationLost", ADDR_NAK: "AddrNak",
           DATA_NAK: "DataNak", ABORTED: "Aborted", TIMEOUT: "Timeout", BUSY: "Busy"}


@dataclass(frozen=True)
class Timing:
    byte: int = 11_250
    t_buf: int = 780
    F: int = SECOND
    backoff: int = 100 * MS
    S_max: int = 5 * MS
    T_max: int = 20 * MS
    B_hold: int = 14_900 * US
    t_stuck: int = 100 * MS
    t_reset: int = 50 * MS
    loop: int = 100 * US
    c_period: int = 25 * MS
    horizon: int = 3 * SECOND


EVIDENCE = {
    "byte": "measured: 9 bit-times at the observed 800 kbit/s",
    "t_buf": "measured",
    "F": "spec default, Open",
    "backoff": "spec default F / 10",
    "S_max": "spec default, Open",
    "T_max": "spec, measured basis",
    "B_hold": "spec: T_max − S_max − one byte",
    "t_stuck": "spec default, Open",
    "t_reset": "assumed, to be measured",
    "loop": "assumed main-loop period",
    "c_period": "assumed background traffic",
    "horizon": "simulated time",
}


@dataclass(frozen=True)
class Profile:
    key: str
    label: str
    timer_max: int  # longest hardware timeout
    hw_limits: bool  # own stretch (target) and own hold (controller) bounded without firmware
    hw_scl_low: bool  # SCL-low timeout acts without firmware
    read_pins: bool  # firmware can read SCL and SDA


STM32 = Profile("stm32", "STM32U375, documented", 174_800 * US, True, True, True)
GENERIC = Profile("generic", "generic short timeout", 16 * MS, False, False, False)


@dataclass(frozen=True)
class Design:
    key: str
    label: str
    short: tuple[str, str]  # two header lines of the summary table
    profile: Profile | None = None  # layered design
    stretch_limit: int = 0  # single timer: target stretch supervision
    terminate: str = ""  # single timer: "release" or "stop"

    @property
    def layered(self) -> bool:
        return self.profile is not None


def layered(profile: Profile) -> Design:
    name = profile.label.split(",")[0]
    return Design(f"layered-{profile.key}", f"Layered limits · {profile.label}", ("Layered", name), profile)


def single(terminate: str, limit: int, T: Timing) -> Design:
    lim = "F" if limit == T.F else bps._param(limit)
    word = "release (get off the bus)" if terminate == "release" else "STOP when possible"
    return Design(f"single-{terminate}-{limit // MS}ms", f"Single timer · terminate = {word} · stretch limit {lim}",
                  (f"Single, {terminate}", f"stretch limit {lim}"), None, limit, terminate)


def figure_designs(T: Timing, generic: Profile) -> tuple[Design, ...]:
    return layered(STM32), layered(generic), single("release", 50 * MS, T), single("stop", 50 * MS, T)


def summary_designs(T: Timing, generic: Profile) -> tuple[Design, ...]:
    return (layered(STM32), layered(generic)) + tuple(
        single(term, limit, T) for term in ("release", "stop") for limit in (5 * MS, 50 * MS, T.F))


# --------------------------------------------------------------------------- event kernel


class Event:
    __slots__ = ("fn", "cancelled")

    def __init__(self, fn: Callable[[], None]) -> None:
        self.fn = fn
        self.cancelled = False

    def cancel(self) -> None:
        self.cancelled = True


@dataclass
class Delay:
    dt: int


@dataclass
class Until:
    pred: Callable[[], bool]
    timeout: int | None = None


class Sim:
    """Discrete events at integer nanoseconds; same time: lower prio first, then FIFO."""

    def __init__(self) -> None:
        self.now = 0
        self._queue: list[tuple[int, int, int, Event]] = []
        self._seq = 0
        self.waiting: list[Proc] = []
        self._wake: Event | None = None

    def at(self, t: int, fn: Callable[[], None], prio: int = 0) -> Event:
        ev = Event(fn)
        heapq.heappush(self._queue, (t, prio, self._seq, ev))
        self._seq += 1
        return ev

    def after(self, dt: int, fn: Callable[[], None], prio: int = 0) -> Event:
        return self.at(self.now + dt, fn, prio)

    def changed(self) -> None:
        """State changed: re-check the processes that wait for a condition."""
        if self._wake is None and self.waiting:
            self._wake = self.at(self.now, self._wake_up)

    def _wake_up(self) -> None:
        self._wake = None
        for proc in list(self.waiting):
            if proc in self.waiting and proc.pred is not None and proc.pred():
                proc.resume(True)

    def run(self, until: int) -> None:
        while self._queue and self._queue[0][0] <= until:
            t, _, _, ev = heapq.heappop(self._queue)
            self.now = t
            if not ev.cancelled:
                ev.fn()
        self.now = until


class Proc:
    """A generator that yields Delay or Until; Until resumes with True, or False on timeout."""

    def __init__(self, sim: Sim, gen: Iterator) -> None:
        self.sim, self.gen = sim, gen
        self.alive = True
        self.pred: Callable[[], bool] | None = None
        self._ev: Event | None = sim.at(sim.now, lambda: self._step(None))

    def _step(self, value: bool | None) -> None:
        self._ev = None
        if not self.alive:
            return
        try:
            cmd = self.gen.send(value)
        except StopIteration:
            self.alive = False
            return
        if isinstance(cmd, Delay):
            self._ev = self.sim.after(cmd.dt, lambda: self._step(None))
        elif cmd.pred():
            self._ev = self.sim.at(self.sim.now, lambda: self._step(True))
        else:
            self.pred = cmd.pred
            self.sim.waiting.append(self)
            if cmd.timeout is not None:
                self._ev = self.sim.after(cmd.timeout, lambda: self.resume(False))

    def resume(self, ok: bool) -> None:
        if self in self.sim.waiting:
            self.sim.waiting.remove(self)
        if self._ev is not None:
            self._ev.cancel()
        self.pred = None
        self._step(ok)

    def kill(self) -> None:
        if not self.alive:
            return
        self.alive = False
        if self in self.sim.waiting:
            self.sim.waiting.remove(self)
        if self._ev is not None:
            self._ev.cancel()
        self.gen.close()


# --------------------------------------------------------------------------- bus and recording


@dataclass(frozen=True)
class Xfer:
    addr: int
    length: int
    read: bool = False


@dataclass(frozen=True)
class Report:
    node: str
    t_submit: int
    t: int
    outcome: int
    last: int
    reason: str = ""


@dataclass(frozen=True)
class Risk:
    t: int
    node: str
    text: str
    short: str


class Recorder:
    def __init__(self) -> None:
        self.spans: list[tuple[str, str, int, int]] = []  # node, kind, t0, t1
        self._open: dict[tuple[str, str], int] = {}
        self.reports: list[Report] = []
        self.submits: list[tuple[str, int]] = []
        self.detections: list[tuple[int, str, str]] = []
        self.risks: list[Risk] = []
        self.notes: list[tuple[int, str, str]] = []
        self.transactions: list[tuple[str, int, int, str]] = []  # controller, START, end, how

    def begin(self, node: str, kind: str, t: int) -> None:
        self._open.setdefault((node, kind), t)

    def end(self, node: str, kind: str, t: int) -> None:
        t0 = self._open.pop((node, kind), None)
        if t0 is not None and t > t0:
            self.spans.append((node, kind, t0, t))

    def close(self, t: int) -> None:
        for (node, kind), t0 in sorted(self._open.items(), key=lambda item: item[1]):
            if t > t0:
                self.spans.append((node, kind, t0, t))
        self._open.clear()


class World:
    NODES = (("A", 0x10), ("B", 0x20), ("C", 0x30), ("D", 0x40))

    def __init__(self, design: Design, T: Timing, lib: ctypes.CDLL) -> None:
        self.sim = Sim()
        self.design, self.T, self.lib = design, T, lib
        self.rec = Recorder()
        self.holds: dict[str, dict[str, str]] = {"SCL": {}, "SDA": {}}  # line -> holder -> span kind
        self.clocking: Node | None = None
        self.owner: Node | None = None
        self.owner_since = 0
        self._starters: list[Node] = []
        self._resolve_ev: Event | None = None
        self._bus = "idle"
        self.nodes = {name: Node(self, name, addr) for name, addr in self.NODES}
        for node in self.nodes.values():
            node.fw = TxnFirmware(node, lib) if design.layered else SingleTimerFirmware(node)

    def target(self, addr: int) -> Node | None:
        return next((n for n in self.nodes.values() if n.addr == addr), None)

    def scl_low(self) -> bool:
        return bool(self.holds["SCL"])

    def sda_low(self) -> bool:
        return bool(self.holds["SDA"])

    def lines_free(self) -> bool:
        return not self.holds["SCL"] and not self.holds["SDA"]

    def changed(self) -> None:
        state = ("scl" if self.scl_low() else "sda" if self.sda_low() else "active" if self.owner or self.clocking
                 else "missing" if any(n.enabled and not n.failed and n.busy for n in self.nodes.values())
                 else "idle")
        if state != self._bus:
            self.rec.end("bus", self._bus, self.sim.now)
            self.rec.begin("bus", state, self.sim.now)
            self._bus = state
        self.sim.changed()
        for node in self.nodes.values():
            node.update_detectors()

    def hold(self, node: Node, line: str, kind: str) -> None:
        if node.name not in self.holds[line]:
            self.holds[line][node.name] = kind
            self.rec.begin(node.name, kind, self.sim.now)
            self.changed()

    def release(self, node: Node, line: str) -> None:
        kind = self.holds[line].pop(node.name, None)
        if kind is not None:
            self.rec.end(node.name, kind, self.sim.now)
            self.changed()

    def request_start(self, node: Node) -> None:
        self._starters.append(node)
        if self._resolve_ev is None:
            self._resolve_ev = self.sim.at(self.sim.now, self._resolve, prio=1)

    def _resolve(self) -> None:
        """All STARTs of this instant: the lowest target address wins arbitration."""
        self._resolve_ev = None
        starters = [n for n in self._starters if n.proc is not None and n.proc.alive and n.enabled]
        self._starters = []
        if not starters:
            return
        now = self.sim.now
        winner = min(starters, key=lambda n: (n.xfer.addr, n.addr))
        victim = self.owner
        if victim is not None and victim is not winner:
            self.rec.risks.append(Risk(now, winner.name, f"starts inside {victim.name}'s transfer, which is cut "
                                       "(assumed bus error)", f"START cuts {victim.name}'s transfer"))
            victim.cut("bus error: START inside the transfer (assumed)")
        for node in self.nodes.values():
            node.on_start(winner)
        self.owner, self.owner_since = winner, now
        self.rec.begin(winner.name, "own", now)
        for node in starters:
            node.arb = "won" if node is winner else "lost"
        self.changed()

    def stop(self, ctl: Node) -> None:
        self.end_ownership(ctl, "STOP")
        for node in self.nodes.values():
            node.on_stop()
        self.changed()

    def end_ownership(self, ctl: Node, how: str) -> None:
        if self.owner is ctl:
            self.rec.end(ctl.name, "own", self.sim.now)
            self.rec.transactions.append((ctl.name, self.owner_since, self.sim.now, how))
            self.owner = None
            self.changed()


# --------------------------------------------------------------------------- participant


class Node:
    """One participant: peripheral (controller and target role), firmware main loop, scenario hooks."""

    def __init__(self, w: World, name: str, addr: int) -> None:
        self.w, self.name, self.addr = w, name, addr
        self.enabled = True
        self.failed = False
        self.fw_alive = True
        self.stall_until = 0
        self.busy = False
        self.boot_until = 0
        self.wait_until = 0
        self.fw: TxnFirmware | SingleTimerFirmware
        self._tick_ev: Event | None = None
        self.det: dict[str, Event] = {}
        self.app_on_activate: list[Xfer] = []
        # controller role
        self.proc: Proc | None = None
        self.result = NONE
        self.reason = ""
        self.on_bus = False
        self.stop_req = ""  # why a STOP at the next byte boundary is requested
        self.arb: str | None = None
        self.xfer: Xfer | None = None
        self.own_hold = 0
        self._b_hold_ev: Event | None = None
        self.stall_plan: dict[int, int] = {}  # data byte index -> firmware stall before it
        self.reset_plan: dict[int, int] = {}  # data byte index -> reset (duration) while clocking it
        # target role
        self.addressed_by: Node | None = None
        self.ignoring = False
        self.rx = 0
        self.ready = True
        self.stretching: Proc | None = None
        self.stretch_plan: dict[int, int] = {}  # data byte index -> stretch before it
        self.hang_at: int | None = None  # data byte index before which the firmware hangs
        self.zero_on_abort = False  # keeps driving a 0 if its controller vanishes in a read byte
        self._ff_noted = False

    @property
    def now(self) -> int:
        return self.w.sim.now

    def set_busy(self, value: bool) -> None:
        if value != self.busy:
            self.busy = value
            (self.w.rec.begin if value else self.w.rec.end)(self.name, "busy", self.now)

    # ----- controller role: platform interface of the firmware

    def attempt_start(self, x: Xfer) -> bool:
        if self.failed and self.enabled:
            # Failure mode: every transfer ends at once as a bus failure (txn.c has no own outcome yet).
            self.result, self.reason, self.on_bus, self.xfer = ABORTED, "bus failure", False, x
            return True
        if not self.enabled or self.now < self.wait_until:
            return False
        if self.proc is not None and self.proc.alive:
            return False
        self.result, self.reason, self.on_bus, self.stop_req = NONE, "", False, ""
        self.arb, self.xfer, self.own_hold = None, x, 0
        self.proc = Proc(self.w.sim, self._attempt(x))
        return True

    def attempt_poll(self) -> tuple[int, bool]:
        return self.result, self.on_bus

    def _finish(self, result: int, reason: str = "") -> None:
        if self._b_hold_ev is not None:
            self._b_hold_ev.cancel()
            self._b_hold_ev = None
        self.on_bus = False
        if self.result == NONE:
            self.result, self.reason = result, reason
        self.w.sim.after(0, self.tick)

    def _attempt(self, x: Xfer) -> Iterator:
        w, T = self.w, self.w.T
        while True:
            yield Until(lambda: self.enabled and not self.busy and w.lines_free())
            yield Delay(T.t_buf)
            if self.enabled and not self.busy and w.lines_free():
                break
        w.request_start(self)
        yield Until(lambda: self.arb is not None)
        if self.arb == "lost":
            yield Delay(T.byte)
            self._finish(ARLO)
            return
        self.on_bus = True
        if w.design.layered:
            self._b_hold_ev = w.sim.after(T.B_hold, self._b_hold)
        tgt = w.target(x.addr)
        yield from self._clock(-1)
        if tgt is None or not tgt.t_address(self):
            yield from self._stop()
            self._finish(ADDR_NAK)
            return
        for i in range(x.length):
            if (yield from self._boundary(tgt, i)):
                return
            if self.stop_req:
                break
            yield from self._clock(i)
            if x.read:
                tgt.t_read(self)
            elif not tgt.t_write(self):
                yield from self._stop()
                self._finish(DATA_NAK)
                return
        reason = self.stop_req
        yield from self._stop()
        self._finish(ABORTED if reason else SUCCESS, reason)

    def _clock(self, i: int) -> Iterator:
        w = self.w
        if i in self.reset_plan:
            duration = self.reset_plan.pop(i)
            w.sim.after(w.T.byte // 2, lambda: self.reset(duration))
        w.clocking = self
        w.changed()
        yield Delay(w.T.byte)
        w.clocking = None
        w.changed()

    def _boundary(self, tgt: Node, i: int) -> Iterator:
        """Byte boundary: target stretch, own firmware stall, then wait until nobody holds SCL."""
        w, T = self.w, self.w.T
        tgt.t_before_byte(self, i)
        if i in self.stall_plan:
            self.stall(self.stall_plan.pop(i))
        if self.stall_until > self.now:
            w.hold(self, "SCL", "hold")
            remaining = self.stall_until - self.now
            hw = w.design.layered and w.design.profile.hw_limits
            limit = T.S_max - self.own_hold if hw else None
            if limit is not None and limit < remaining:
                yield Delay(limit)
                self.own_hold += limit
                w.release(self, "SCL")
                yield from self._stop()
                self._finish(ABORTED, "hardware hold limit")
                return True
            yield Delay(remaining)
            self.own_hold += remaining
            w.release(self, "SCL")
        if w.scl_low():
            yield Until(lambda: not w.scl_low())
        return False

    def _stop(self) -> Iterator:
        if not self.w.lines_free():
            yield Until(self.w.lines_free)
        self.w.stop(self)

    def _b_hold(self) -> None:
        self._b_hold_ev = None
        if not self.on_bus:
            return
        if self.stall_until > self.now:  # the firmware acts when it runs again
            self._b_hold_ev = self.w.sim.at(self.stall_until, self._b_hold)
        elif self.fw_alive and not self.stop_req:
            self.stop_req = "forced STOP at B_hold"

    def request_stop(self, reason: str) -> None:
        if self.proc is not None and self.proc.alive and self.on_bus and not self.stop_req:
            self.stop_req = reason

    def stall(self, duration: int) -> None:
        self.stall_until = max(self.stall_until, self.now + duration)
        self.w.rec.spans.append((self.name, "stall", self.now, self.stall_until))

    def _drop_attempt(self, how: str) -> None:
        """End the own attempt at once, without STOP (peripheral reset)."""
        w = self.w
        if self.proc is None or not self.proc.alive:
            return
        if w.clocking is self:
            w.clocking = None
            tgt = w.target(self.xfer.addr)
            if self.xfer.read and tgt is not None and tgt.zero_on_abort and tgt.addressed_by is self:
                w.hold(tgt, "SDA", "sda")
                w.rec.notes.append((self.now, tgt.name, "keeps driving a 0 on SDA"))
        self.proc.kill()
        if self._b_hold_ev is not None:
            self._b_hold_ev.cancel()
            self._b_hold_ev = None
        self.on_bus = False
        w.end_ownership(self, how)
        w.release(self, "SCL")
        w.changed()

    def cut(self, reason: str) -> None:
        """Another controller's START cut the own transfer."""
        self._drop_attempt("cut")
        self.result, self.reason = ABORTED, reason
        self.w.sim.after(0, self.tick)

    def release_peripheral(self, how: str) -> None:
        """Reset the own peripheral (PE = 0): off the bus at once, no STOP, busy flag cleared."""
        self._drop_attempt(how)
        self._drop_target()
        self.set_busy(False)
        self.w.changed()

    # ----- target role

    def t_address(self, ctl: Node) -> bool:
        if not self.enabled or self.failed or self.now < self.boot_until:
            return False
        self.addressed_by, self.rx, self.ignoring = ctl, 0, False
        return True

    def t_before_byte(self, ctl: Node, i: int) -> None:
        if ctl is not self.addressed_by or self.ignoring:
            return
        if self.hang_at == i:
            self.hang()
        duration = FOREVER if not self.fw_alive else self.stretch_plan.pop(i, 0)
        if duration:
            self.w.hold(self, "SCL", "stretch")
            self.stretching = Proc(self.w.sim, self._stretch(duration))

    def _stretch(self, duration: int) -> Iterator:
        w, T = self.w, self.w.T
        self.ready = False
        if duration < FOREVER:
            w.sim.after(duration, self._become_ready)
        if w.design.layered:
            limit = T.S_max if (w.design.profile.hw_limits or self.fw_alive) else None
        else:
            limit = w.design.stretch_limit if self.fw_alive else None
        ok = yield Until(lambda: self.ready, limit)
        w.release(self, "SCL")
        if ok:
            return
        if w.design.layered:
            w.release(self, "SDA")
            self.ignoring = True
            w.rec.notes.append((self.now, self.name, "releases after S_max and abandons the transfer"))
        else:
            self.ready = True
            w.rec.risks.append(Risk(self.now, self.name, "acknowledges while not ready: the command may be lost",
                                    "ACK while not ready"))

    def _become_ready(self) -> None:
        if self.fw_alive:
            self.ready = True
            self.w.changed()

    def hang(self) -> None:
        self.fw_alive = False
        self.w.rec.begin(self.name, "hung", self.now)
        self.w.rec.notes.append((self.now, self.name, "firmware hangs"))

    def t_write(self, ctl: Node) -> bool:
        if ctl is not self.addressed_by or self.ignoring:
            return False
        self.rx += 1
        return True

    def t_read(self, ctl: Node) -> None:
        if (ctl is not self.addressed_by or self.ignoring) and not self._ff_noted:
            self._ff_noted = True
            self.w.rec.risks.append(Risk(self.now, ctl.name, "reads 0xFF that it cannot tell from data",
                                         "reads 0xFF"))

    def _drop_target(self) -> None:
        if self.stretching is not None:
            self.stretching.kill()
            self.stretching = None
        self.w.release(self, "SCL")
        self.w.release(self, "SDA")
        self.addressed_by, self.ignoring, self.rx, self.ready = None, False, 0, True

    def on_start(self, ctl: Node) -> None:
        if not self.enabled:
            return
        if self.addressed_by is not None and self.rx and self.addressed_by is not ctl:
            self.w.rec.notes.append((self.now, self.name, "drops a partial command (START without STOP)"))
        self.addressed_by, self.ignoring, self.rx = None, False, 0
        self.set_busy(True)

    def on_stop(self) -> None:
        if not self.enabled:
            return
        self.addressed_by, self.ignoring, self.rx = None, False, 0
        self.set_busy(False)

    # ----- reset, wait after reset, failure mode

    def reset(self, duration: int) -> None:
        w = self.w
        self._drop_attempt("reset")
        self._drop_target()
        self.set_busy(False)
        self.enabled, self.stall_until = False, 0
        if self.failed:
            self.failed = False
            w.rec.end(self.name, "failure", self.now)
        for ev in self.det.values():
            ev.cancel()
        self.det.clear()
        self.fw.reset()
        w.rec.begin(self.name, "reset", self.now)
        w.rec.notes.append((self.now, self.name, "resets"))
        w.sim.after(duration, self._activate)
        w.changed()

    def _activate(self) -> None:
        w, T = self.w, self.w.T
        self.enabled = True
        self.set_busy(False)
        w.rec.end(self.name, "reset", self.now)
        if w.design.layered:
            self.wait_until = self.now + T.T_max
            w.rec.spans.append((self.name, "wait", self.now, self.wait_until))
            w.sim.after(T.T_max, self._after_wait)
        for x in self.app_on_activate:
            self.submit(x)
        w.changed()
        self.tick()

    def _after_wait(self) -> None:
        w = self.w
        if (w.design.profile.read_pins and self.fw_alive and not self.failed and not self.busy
                and not w.lines_free()):
            self.bus_failure("line check after the wait: busy flag clear, a line low")
        self.tick()

    def bus_failure(self, why: str) -> None:
        w = self.w
        if self.failed:
            return
        self.failed = True
        w.rec.detections.append((self.now, self.name, why))
        w.rec.begin(self.name, "failure", self.now)
        if self.proc is not None and self.proc.alive:
            if self.on_bus:
                self.stop_req = self.stop_req or "bus failure"  # the owed STOP goes out when SCL is free
            else:
                self._drop_attempt("failure")  # withdraw the armed START (peripheral reset)
            if self.result == NONE:
                self.result, self.reason = ABORTED, "bus failure"
            w.sim.after(0, self.tick)
        for ev in self.det.values():
            ev.cancel()
        self.det.clear()
        # Release the own lines after every detector of this instant has looked at the bus.
        w.sim.at(self.now, self._drop_target, prio=3)
        w.changed()

    def update_detectors(self) -> None:
        w = self.w
        if not w.design.layered:
            return
        for kind in ("scl", "sda", "missing"):
            active = self.enabled and not self.failed and self._stuck(kind)
            ev = self.det.get(kind)
            if active and ev is None:
                self.det[kind] = w.sim.after(w.T.t_stuck, lambda k=kind: self._detect(k), prio=2)
            elif not active and ev is not None:
                ev.cancel()
                del self.det[kind]

    def _stuck(self, kind: str) -> bool:
        w = self.w
        if kind == "scl":
            return w.scl_low()
        if kind == "sda":
            return w.sda_low() and not w.scl_low()
        return self.busy and w.lines_free() and w.clocking is None and w.owner is None

    def _detect(self, kind: str) -> None:
        w, p = self.w, self.w.design.profile
        self.det.pop(kind, None)
        if not self.enabled or self.failed or not self._stuck(kind):
            return
        if self.stall_until > self.now:
            self.det[kind] = w.sim.at(self.stall_until, lambda: self._detect(kind), prio=2)
            return
        if kind == "scl" and p.hw_scl_low and w.T.t_stuck <= p.timer_max and self.name in w.holds["SCL"]:
            self._drop_target()  # TIMEOUTA releases the lines of the target role without firmware
        if not self.fw_alive:
            return
        if kind != "scl" and not p.read_pins and not self.busy:
            return  # the event timer counts only while the peripheral considers the bus busy
        what = {"scl": "SCL low", "sda": "SDA low", "missing": "missing STOP"}[kind]
        self.bus_failure(f"{what} for t_stuck")

    # ----- firmware main loop and application

    def tick(self) -> None:
        if self._tick_ev is not None:
            self._tick_ev.cancel()
            self._tick_ev = None
        if not self.enabled or not self.fw_alive:
            return
        if self.stall_until > self.now:
            self._tick_ev = self.w.sim.at(self.stall_until, self.tick)
            return
        self.fw.step()
        if self.fw.busy():
            self._tick_ev = self.w.sim.after(self.w.T.loop, self.tick)

    def submit(self, x: Xfer) -> None:
        if not self.enabled or self.failed or not self.fw_alive:
            return
        if self.fw.submit(x):
            self.w.rec.submits.append((self.name, self.now))
        else:
            self.w.rec.reports.append(Report(self.name, self.now, self.now, BUSY, BUSY, "queue full"))
        self.tick()

    def report(self, t_submit: int, outcome: int, last: int, reason: str = "") -> None:
        if not reason and outcome == ABORTED:
            reason = self.reason
        self.w.rec.reports.append(Report(self.name, t_submit, self.now, outcome, last, reason))


# --------------------------------------------------------------------------- firmware

SHIM = r"""
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "txn.h"

typedef bool (*sim_start_fn)(void *ctx, uint8_t addr7, uint16_t wr_len, uint16_t rd_len);
typedef int (*sim_poll_fn)(void *ctx, bool *on_bus);
typedef uint32_t (*sim_now_fn)(void *ctx);
typedef void (*sim_done_fn)(void *ctx, uint32_t tag, int outcome, int last);

typedef struct {
  txn_t txn;
  txn_platform_t pf;
  sim_start_fn start;
  sim_poll_fn poll;
  sim_now_fn now;
  sim_done_fn done;
  uint8_t buf[1024];
} sim_t;

typedef struct {
  sim_t *sim;
  uint32_t tag;
} ticket_t;

static bool tr_start(void *c, const txn_xfer_t *x)
{
  sim_t *s = c;
  return s->start(NULL, x->addr7, x->wr_len, x->rd_len);
}

static txn_outcome_t tr_poll(void *c, bool *on_bus)
{
  sim_t *s = c;
  return (txn_outcome_t)s->poll(NULL, on_bus);
}

static uint32_t tr_now(void *c)
{
  sim_t *s = c;
  return s->now(NULL);
}

static void tr_done(void *user, const txn_xfer_t *x, const txn_report_t *r)
{
  ticket_t *t = user;
  sim_t *s = t->sim;
  const uint32_t tag = t->tag;
  (void)x;
  free(t);
  s->done(NULL, tag, (int)r->outcome, (int)r->last);
}

sim_t *sim_new(sim_start_fn start, sim_poll_fn poll, sim_now_fn now, sim_done_fn done,
               uint32_t budget_us, uint32_t backoff_us, uint32_t stuck_us)
{
  sim_t *s = calloc(1, sizeof(*s));
  if (s == NULL) {
    return NULL;
  }
  s->start = start;
  s->poll = poll;
  s->now = now;
  s->done = done;
  s->pf.attempt_start = tr_start;
  s->pf.attempt_poll = tr_poll;
  s->pf.attempt_withdraw = NULL; /* platform_capabilities.md C2 */
  s->pf.now_us = tr_now;
  s->pf.ctx = s;
  const txn_params_t prm = {budget_us, backoff_us, stuck_us};
  txn_init(&s->txn, &s->pf, &prm);
  return s;
}

bool sim_submit(sim_t *s, uint8_t addr7, uint16_t wr_len, uint16_t rd_len, uint32_t tag)
{
  ticket_t *t = malloc(sizeof(*t));
  if (t == NULL) {
    return false;
  }
  t->sim = s;
  t->tag = tag;
  const txn_xfer_t x = {addr7, wr_len != 0u ? s->buf : NULL, wr_len, rd_len != 0u ? s->buf : NULL, rd_len};
  if (!txn_submit(&s->txn, &x, tr_done, t)) {
    free(t);
    return false;
  }
  return true;
}

void sim_step(sim_t *s) { txn_step(&s->txn); }
bool sim_busy(const sim_t *s) { return txn_pending(&s->txn) != 0u || !txn_idle(&s->txn); }
unsigned sim_pending(const sim_t *s) { return txn_pending(&s->txn); }
void sim_free(sim_t *s) { free(s); }
"""

START_FN = ctypes.CFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint16, ctypes.c_uint16)
POLL_FN = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.POINTER(ctypes.c_bool))
NOW_FN = ctypes.CFUNCTYPE(ctypes.c_uint32, ctypes.c_void_p)
DONE_FN = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_int, ctypes.c_int)


def build_txn_lib() -> ctypes.CDLL:
    """Compile firmware/txn/txn.c with a small shim into a shared library and load it."""
    cc = shutil.which("gcc") or shutil.which("cc")
    if cc is None:
        raise SystemExit("bus_timeout_sim needs a C compiler (gcc or cc) to build firmware/txn/txn.c")
    with tempfile.TemporaryDirectory(prefix="txnsim-") as tmp:
        shim, lib = Path(tmp) / "txn_sim_shim.c", Path(tmp) / "libtxnsim.so"
        shim.write_text(SHIM, encoding="utf-8")
        cmd = [cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC", f"-I{TXN_DIR}",
               str(TXN_DIR / "txn.c"), str(shim), "-o", str(lib)]
        build = subprocess.run(cmd, capture_output=True, text=True, check=False)
        if build.returncode != 0:
            raise SystemExit(f"building firmware/txn/txn.c failed:\n{build.stderr}")
        dll = ctypes.CDLL(str(lib))  # stays mapped after the file is removed
    dll.sim_new.restype = ctypes.c_void_p
    dll.sim_new.argtypes = [START_FN, POLL_FN, NOW_FN, DONE_FN, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32]
    dll.sim_submit.restype = ctypes.c_bool
    dll.sim_submit.argtypes = [ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint16, ctypes.c_uint16, ctypes.c_uint32]
    dll.sim_step.argtypes = [ctypes.c_void_p]
    dll.sim_busy.restype = ctypes.c_bool
    dll.sim_busy.argtypes = [ctypes.c_void_p]
    dll.sim_pending.restype = ctypes.c_uint
    dll.sim_pending.argtypes = [ctypes.c_void_p]
    dll.sim_free.argtypes = [ctypes.c_void_p]
    return dll


class TxnFirmware:
    """The real transaction layer (firmware/txn/txn.c) on the simulated peripheral."""

    def __init__(self, node: Node, lib: ctypes.CDLL) -> None:
        self.node, self.lib = node, lib
        self._error: BaseException | None = None
        self._cbs = (START_FN(self._start), POLL_FN(self._poll), NOW_FN(self._now), DONE_FN(self._done))
        self._tags: dict[int, int] = {}  # tag -> submission time
        self._next = 0
        self.handle = self._new()

    def _new(self) -> int:
        T = self.node.w.T
        return self.lib.sim_new(*self._cbs, T.F // US, T.backoff // US, T.T_max // US)

    def reset(self) -> None:
        for t_submit in self._tags.values():
            self.node.report(t_submit, NONE, NONE)
        self._tags.clear()
        self.lib.sim_free(self.handle)
        self.handle = self._new()

    def submit(self, x: Xfer) -> bool:
        self._next += 1
        wr, rd = (0, x.length) if x.read else (x.length, 0)
        if not self.lib.sim_submit(self.handle, x.addr, wr, rd, self._next):
            return False
        self._tags[self._next] = self.node.now
        return True

    def step(self) -> None:
        self.lib.sim_step(self.handle)
        if self._error is not None:
            raise self._error

    def busy(self) -> bool:
        return bool(self.lib.sim_busy(self.handle))

    def pending(self) -> int:
        return int(self.lib.sim_pending(self.handle))

    def _start(self, _ctx: int, addr: int, wr: int, rd: int) -> bool:
        try:
            return self.node.attempt_start(Xfer(addr, rd or wr, rd > 0))
        except BaseException as exc:  # noqa: BLE001  re-raised after the C call returns
            self._error = exc
            return False

    def _poll(self, _ctx: int, on_bus: ctypes._Pointer) -> int:
        try:
            result, on = self.node.attempt_poll()
            on_bus[0] = on
            return result
        except BaseException as exc:  # noqa: BLE001
            self._error = exc
            return ABORTED

    def _now(self, _ctx: int) -> int:
        return (self.node.now // US) & 0xFFFFFFFF

    def _done(self, _ctx: int, tag: int, outcome: int, last: int) -> None:
        try:
            self.node.report(self._tags.pop(tag, self.node.now), outcome, last)
        except BaseException as exc:  # noqa: BLE001
            self._error = exc


class SingleTimerFirmware:
    """One supervision timer F per transfer from the head of the queue; on expiry terminate unconditionally."""

    DEPTH = 8

    def __init__(self, node: Node) -> None:
        self.node = node
        self.queue: deque[tuple[int, Xfer]] = deque()
        self.cur: tuple[int, Xfer] | None = None
        self.deadline = self.next_try = 0
        self.running = False
        self.last = BUSY

    def reset(self) -> None:
        for t_submit, _ in ([self.cur] if self.cur else []) + list(self.queue):
            self.node.report(t_submit, NONE, NONE)
        self.queue.clear()
        self.cur, self.running = None, False

    def submit(self, x: Xfer) -> bool:
        if self.pending() >= self.DEPTH:
            return False
        self.queue.append((self.node.now, x))
        return True

    def busy(self) -> bool:
        return self.cur is not None or bool(self.queue)

    def pending(self) -> int:
        return len(self.queue) + (self.cur is not None)

    def step(self) -> None:
        n, T, now = self.node, self.node.w.T, self.node.now
        if self.cur is None:
            if not self.queue:
                return
            self.cur = self.queue.popleft()
            self.deadline, self.next_try, self.running, self.last = now + T.F, now, False, BUSY
        if self.running:
            result, on_bus = n.attempt_poll()
            if result == NONE:
                if now >= self.deadline:
                    if n.w.design.terminate == "stop" and on_bus:
                        n.request_stop("forced STOP at flush expiry")
                    else:
                        n.release_peripheral("released at flush expiry")
                    self._done(TIMEOUT, "flush time expired")
                return
            self.running, self.last = False, result
            if result == SUCCESS:
                self._done(SUCCESS)
                return
            self.next_try = now + (T.F // 10 if result in (ADDR_NAK, DATA_NAK) else 0)
        if now >= self.deadline:
            self._done(TIMEOUT, "flush time expired")
        elif now >= self.next_try and n.attempt_start(self.cur[1]):
            self.running = True

    def _done(self, outcome: int, reason: str = "") -> None:
        t_submit = self.cur[0]
        self.cur, self.running = None, False
        self.node.report(t_submit, outcome, self.last, reason)


# --------------------------------------------------------------------------- scenarios and runs


@dataclass(frozen=True)
class Scenario:
    key: str
    title: str
    text: str
    setup: Callable[[World], None]


def _a_sends(w: World, x: Xfer, at: int = MS) -> None:
    w.sim.at(at, lambda: w.nodes["A"].submit(x))


def _slow_target(w: World) -> None:
    w.nodes["B"].stretch_plan[3] = 40 * MS
    _a_sends(w, Xfer(0x20, 16))


def _hung_target(w: World) -> None:
    w.nodes["B"].hang_at = 3
    _a_sends(w, Xfer(0x20, 16))


def _controller_stall(w: World) -> None:
    w.nodes["A"].stall_plan[4] = 50 * MS
    _a_sends(w, Xfer(0x20, 16))


def _reset_write(w: World, resend: bool) -> None:
    a = w.nodes["A"]
    a.reset_plan[5] = w.T.t_reset
    if resend:
        a.app_on_activate = [Xfer(0x20, 16)]
    _a_sends(w, Xfer(0x20, 16))


def _sda_stuck(w: World) -> None:
    a, b = w.nodes["A"], w.nodes["B"]
    a.reset_plan[5] = w.T.t_reset
    a.app_on_activate = [Xfer(0x20, 16, read=True)]
    b.zero_on_abort = True
    _a_sends(w, Xfer(0x20, 16, read=True))


def _join(w: World) -> None:
    c = w.nodes["C"]
    c.app_on_activate = [Xfer(0x40, 8)]
    _a_sends(w, Xfer(0x20, 1024, read=True))
    w.sim.at(4 * MS, lambda: c.reset(MS))


def _booting_target(w: World) -> None:
    b = w.nodes["B"]
    b.boot_until = 300 * MS
    w.rec.spans.append(("B", "boot", 0, b.boot_until))
    _a_sends(w, Xfer(0x20, 16))


SCENARIOS = (
    Scenario("slow_target", "Slow target",
             "B is still busy with an earlier command and stretches 40 ms before the fourth data byte "
             "of A's 16-byte write.", _slow_target),
    Scenario("hung_target", "Hung target",
             "B's firmware hangs while B stretches before the fourth data byte of A's 16-byte write.",
             _hung_target),
    Scenario("controller_stall", "Controller firmware stall",
             "A's firmware stalls for 50 ms before the fifth data byte of its own 16-byte write; "
             "A holds SCL meanwhile.", _controller_stall),
    Scenario("reset_resend", "Controller reset, then resend",
             "A resets in the sixth data byte of its write to B (no STOP); once its I2C is active again "
             "(t_reset later) it sends the write again.", lambda w: _reset_write(w, True)),
    Scenario("reset_idle", "Controller reset, nothing to send",
             "A resets in the sixth data byte of its write to B (no STOP) and has nothing to send "
             "afterwards.", lambda w: _reset_write(w, False)),
    Scenario("sda_stuck", "Controller reset during a read",
             "A resets in the sixth byte of a read from B while B sends a 0; B keeps SDA low. "
             "Once active again, A reads again.", _sda_stuck),
    Scenario("join", "Participant joins during a transfer",
             "C resets for 1 ms during A's 1024-byte read from B (11.5 ms) and has a write to D queued "
             "when its I2C is active again.", _join),
    Scenario("booting_target", "Booting target",
             "B's I2C is not active for the first 300 ms; A's write to B is submitted at 1 ms.",
             _booting_target),
)


@dataclass
class Result:
    scenario: Scenario
    design: Design
    T: Timing
    rec: Recorder
    a: list[Report]
    c_worst: int  # longest time a transfer of C waited for its result (unfinished: until the horizon)
    c_failed: int
    detections: list[tuple[int, str, str]]
    risks: list[Risk]
    end: str  # bus state at the horizon
    failed: list[str]  # participants in failure mode at the horizon
    longest: int  # longest bus ownership, START to STOP or abandon
    blocked: int  # longest time the bus was unusable: SCL low, SDA low, missing STOP, or one ownership

    @property
    def category(self) -> str:
        if self.end != "free" and not self.detections:
            return "dead"
        if self.risks:
            return "risk"
        if self.detections:
            return "failure"
        if self.blocked > self.T.T_max:
            return "slow"
        return "ok"


def _end_state(w: World) -> str:
    for line in ("SCL", "SDA"):
        if w.holds[line]:
            return f"{line} held low by {' '.join(sorted(w.holds[line]))}"
    busy = [n.name for n in w.nodes.values() if n.enabled and not n.failed and n.busy]
    if busy and w.owner is None:
        return f"missing STOP (busy: {' '.join(busy)})"
    return "free"


def run(scenario: Scenario, design: Design, T: Timing, lib: ctypes.CDLL) -> Result:
    w = World(design, T, lib)
    c = w.nodes["C"]

    def background() -> None:
        if c.fw.pending() < 2:
            c.submit(Xfer(0x40, 8))
        w.sim.after(T.c_period, background)

    w.sim.at(500 * US, background)
    scenario.setup(w)
    w.sim.run(T.horizon)
    end = _end_state(w)
    longest = max((t1 - t0 for _, t0, t1, _ in w.rec.transactions), default=0)
    if w.owner is not None:
        longest = max(longest, T.horizon - w.owner_since)
    w.rec.close(T.horizon)
    stuck = max((t1 - t0 for n, k, t0, t1 in w.rec.spans if n == "bus" and k in ("scl", "sda", "missing")), default=0)
    reports = sorted(w.rec.reports, key=lambda r: r.t)
    c_reports = [r for r in reports if r.node == "C" and r.reason != "queue full"]
    c_submits = [t for name, t in w.rec.submits if name == "C"]
    waits = [r.t - r.t_submit for r in c_reports] + [T.horizon - t for t in c_submits[len(c_reports):]]
    return Result(scenario, design, T, w.rec, [r for r in reports if r.node == "A"], max(waits, default=0),
                  sum(r.outcome != SUCCESS for r in c_reports), sorted(w.rec.detections), w.rec.risks, end,
                  sorted(n.name for n in w.nodes.values() if n.failed), longest, max(stuck, longest))


def load_params(path: Path) -> tuple[Timing, Profile]:
    """Values in milliseconds under [timing_ms]; the generic profile under [generic_profile]."""
    data = tomllib.loads(path.read_text(encoding="utf-8"))
    unknown = set(data) - {"timing_ms", "generic_profile"}
    names = {f.name for f in fields(Timing)}
    unknown |= {f"timing_ms.{k}" for k in data.get("timing_ms", {}) if k not in names}
    profile = data.get("generic_profile", {})
    unknown |= {f"generic_profile.{k}" for k in profile
                if k not in ("label", "timer_max_ms", "hw_limits", "hw_scl_low", "read_pins")}
    if unknown:
        raise SystemExit(f"{path}: unknown keys {sorted(unknown)}")
    T = replace(Timing(), **{k: round(v * MS) for k, v in data.get("timing_ms", {}).items()})
    generic = replace(GENERIC, **{k: v for k, v in profile.items() if k != "timer_max_ms"})
    if "timer_max_ms" in profile:
        generic = replace(generic, timer_max=round(profile["timer_max_ms"] * MS))
    return T, generic


# --------------------------------------------------------------------------- labels

def _outcome_label(r: Report) -> str:
    if r.outcome == TIMEOUT:
        return "Timeout (flush time)" if r.reason == "flush time expired" else f"Timeout (last {OUTCOME[r.last]})"
    if r.outcome == ABORTED and r.reason:
        return f"Aborted ({r.reason})"
    return OUTCOME[r.outcome]


def _short_outcome(r: Report) -> str:
    if r.outcome == NONE:
        return "lost"
    if r.outcome == ABORTED and r.reason == "bus failure":
        return "bus failure"
    return OUTCOME[r.outcome]


def _params_line(T: Timing) -> str:
    return (f"F = {_duration(T.F)} · S_max = {bps._param(T.S_max)} · B_hold = {bps._param(T.B_hold)} · "
            f"T_max = {bps._param(T.T_max)} · t_stuck = {bps._param(T.t_stuck)} · t_reset = {bps._param(T.t_reset)} "
            f"(assumed) · byte {bps._param(T.byte)} · main loop {bps._param(T.loop)} (assumed)")


# --------------------------------------------------------------------------- figure: timeline per scenario

TAU = 100 * US  # time axis: logarithmic above about TAU
W, PX0, PW = 1240, 64, 800
PX1 = PX0 + PW
RX = PX1 + 24
RW = W - EDGE - RX
LANE_H, LANE_STEP = 14, 18
LANES = ("A", "B", "C", "D", "bus")
STATE_FILLS = {"reset": "#bdc1c6", "boot": "url(#st-wait)", "wait": "url(#st-wait)", "failure": "#f6c4c0",
               "stall": "#fde293", "hung": "#fde293"}
BUS_FILLS = {"active": "#dadce0", "scl": "#5f6368", "sda": INK, "missing": "url(#st-missing)"}
AMBER = "#e37400"
MARKS = {SUCCESS: (GOOD, "dot"), TIMEOUT: (AMBER, "circle"), DATA_NAK: (AMBER, "square"),
         ADDR_NAK: (AMBER, "square"), ABORTED: (ALARM, "square")}


def _x(t: int, T: Timing) -> float:
    return PX0 + PW * math.log10(1 + t / TAU) / math.log10(1 + T.horizon / TAU)


def _ticks(T: Timing) -> list[int]:
    return [t for t in (0, 100 * US, MS, 10 * MS, 100 * MS, SECOND) if t < T.horizon] + [T.horizon]


def _bar(t0: int, t1: int, y: float, h: float, fill: str, T: Timing) -> str:
    x0, x1 = _x(t0, T), _x(t1, T)
    w = max(x1 - x0, 1.5)
    return _rect(min(x0, PX1 - w), y, w, h, fill, cls="data")


def _lane(r: Result, name: str, y: float) -> list[str]:
    T, out = r.T, []
    spans = [(k, t0, t1) for n, k, t0, t1 in r.rec.spans if n == name]
    if name == "bus":
        return [_bar(t0, t1, y + 2, LANE_H - 4, BUS_FILLS[k], T) for k, t0, t1 in spans if k in BUS_FILLS]
    out += [_bar(t0, t1, y, LANE_H, STATE_FILLS[k], T) for k, t0, t1 in spans if k in STATE_FILLS]
    out += [_bar(t0, t1, y + LANE_H - 3, 3, "#80868b", T) for k, t0, t1 in spans if k == "busy"]
    shapes = {"own": (2, LANE_H - 6, bps.COLOURS[name]), "stretch": (1, LANE_H - 5, f"url(#st-{name})"),
              "hold": (1, LANE_H - 5, "url(#st-hold)"), "sda": (4, LANE_H - 8, INK)}
    out += [_bar(t0, t1, y + shapes[k][0], shapes[k][1], shapes[k][2], T) for k, t0, t1 in spans if k in shapes]
    cy = y + LANE_H / 2 - 1

    def at(t: int) -> float:
        return min(max(_x(t, T), PX0 + 4), PX1 - 4)

    for rep in r.rec.reports:
        if rep.node == name:
            colour, shape = MARKS.get(rep.outcome, (GREY, "circle"))
            out.append(bps._marker(at(rep.t), cy, colour, shape))
    out += [bps._cross(at(t), cy, cls="data") for t, n, _ in r.detections if n == name]
    return out


def _panel_lines(r: Result) -> list[str]:
    a = "; ".join(f"{_outcome_label(x)} at {_duration(x.t)}" for x in r.a) or "no result"
    lines = [f"A: {a}",
             f"C: longest wait {_duration(r.c_worst)}" + (f", {r.c_failed} not successful" if r.c_failed else ""),
             "Bus failure reported: " + (", ".join(f"{n} at {_duration(t)}" for t, n, _ in r.detections) or "none")]
    lines += [f"Risk: {x.node} {x.text}" for x in r.risks[:2]]
    end = f"bus {r.end}" + (f"; failure mode: {' '.join(r.failed)}" if r.failed else "")
    return lines + [f"At {_duration(r.T.horizon)}: {end}"]


def _panel(body: list[str], top: float, r: Result) -> float:
    T = r.T
    body.append(_text(16, top, r.design.label, INK, 13, True, "start"))
    lanes_top = top + 10
    height = len(LANES) * LANE_STEP - (LANE_STEP - LANE_H)
    body.append(bps._plot_area(PX0 - 5, lanes_top - 3, PW + 10, height + 6))
    for t in _ticks(T):
        body.append(_line(_x(t, T), lanes_top - 3, _x(t, T), lanes_top + height + 3, GRID))
    for k, name in enumerate(LANES):
        y = lanes_top + k * LANE_STEP
        body.append(_text(PX0 - 12, y + 11, "Bus" if name == "bus" else name, GREY, 11, anchor="end"))
        body.append(_rect(PX0, y, PW, LANE_H, bps.LANE_FILL))
        body += _lane(r, name, y)
    ticks_y = lanes_top + height + 16
    for t in _ticks(T):
        body.append(_text(_x(t, T), ticks_y, _duration(t) if t else "0", GREY, 10))
    y = top + 18
    for line in _panel_lines(r):
        for part in _wrap(line, RW, 11.5):
            body.append(_text(RX, y, part, INK, 11.5, anchor="start"))
            y += 15
    return max(ticks_y, y - 15)


def _legend_items() -> list[tuple[bps.Symbol, str]]:
    sym = bps._sym_rect

    def mark(colour: str, shape: str) -> bps.Symbol:
        return lambda x, y: [bps._marker(x + 12, y - 4, colour, shape, cls="")]

    return [(sym(bps.COLOURS["A"]), "owns the bus, START to STOP"), (sym("url(#st-B)"), "target stretches (holds SCL)"),
            (sym("url(#st-hold)"), "controller holds SCL (firmware stalled)"), (sym(INK), "holds SDA low"),
            (sym("#80868b"), "busy flag set (thin bar)"), (sym("#bdc1c6"), "reset"),
            (sym("url(#st-wait)"), "wait after reset; booting"), (sym("#f6c4c0"), "failure mode"),
            (sym("#fde293"), "firmware stalled or hung"), (sym("#dadce0"), "bus: transfer"),
            (sym("#5f6368"), "bus: SCL low"), (sym("url(#st-missing)"), "bus: missing STOP"),
            (mark(GOOD, "dot"), "Success"), (mark(AMBER, "square"), "AddrNak, DataNak"),
            (mark(AMBER, "circle"), "Timeout"),
            (mark(ALARM, "square"), "Aborted"), (mark(GREY, "circle"), "lost in reset"),
            (lambda x, y: [bps._cross(x + 12, y - 4)], "bus failure reported")]


def _figure_notes(T: Timing) -> list[str]:
    return [
        f"Time axis logarithmic: x ∝ log(1 + t / {bps._param(TAU)}). A submits at 1 ms; C writes 8 bytes to D every "
        f"{bps._param(T.c_period)}. Lanes A to D: one participant each, the thin bar is its busy flag; "
        "Bus: line state.",
        "Layered: the controller firmware is firmware/txn/txn.c, compiled unchanged. The limits (S_max, B_hold, "
        "t_stuck, wait after reset, line check, failure mode) follow the spec and the documented platform "
        "capabilities; the bridges do not implement them yet. txn.c has no bus-failure outcome yet: Aborted.",
        "Single timer: model of one supervision timer per transfer (no implementation). Assumed in both designs: a "
        "START inside a running transfer cuts it as a bus error (not documented; needs a hardware test).",
        "Evidence: " + "; ".join(f"{k}: {EVIDENCE[k]}" for k in ("byte", "t_buf", "t_reset", "loop", "c_period")) + ".",
    ]


def timeline_svg(scenario: Scenario, results: list[Result], T: Timing) -> str:
    defs = [bps._stripes(f"st-{n}", bps.COLOURS[n], "#ffffff") for n in "ABCD"] + [
        bps._stripes("st-wait", "#bdc1c6", "#ffffff"), bps._stripes("st-missing", ALARM, "#ffffff"),
        bps._stripes("st-hold", INK, "#ffffff")]
    body: list[str] = []
    y = bps._header(body, W, f"Timeout designs · {scenario.title}", [scenario.text, _params_line(T)])
    for r in results:
        y = _panel(body, y + 34, r)
    y = bps._legend(body, y + 34, W, _legend_items())
    y = bps._notes(body, y + 26, W, _figure_notes(T))
    return bps._svg_doc(W, y + 16, f"Timeout designs: {scenario.title}", defs, body)


# --------------------------------------------------------------------------- figure and table: summary

CATEGORY = {
    "ok": ("#e6f4ea", "bus usable; nothing blocks or owns it longer than T_max"),
    "slow": ("#fef7e0", "bus blocked or owned longer than T_max, then usable again"),
    "failure": ("#e3e7fb", "bus failure reported; failure mode until restart"),
    "risk": ("#fde7e9", "data may be lost or wrong without an error"),
    "dead": ("#f6c4c0", "bus unusable at the end and nothing reported"),
}
LABEL_W = 196


def _cell_lines(r: Result) -> list[str]:
    last = r.a[-1] if r.a else None
    first = f"A: {_short_outcome(last)} {_duration(last.t)}" if last else "A: no result"
    effect = {
        "dead": r.end,
        "risk": r.risks[0].short if r.risks else "",
        "failure": f"failure mode {' '.join(r.failed)} from {_duration(r.detections[0][0])}" if r.detections else "",
        "slow": f"bus blocked {_duration(r.blocked)}",
        "ok": f"C waits ≤ {_duration(r.c_worst)}",
    }[r.category]
    return [first, effect]


def summary_svg(designs: tuple[Design, ...], results: dict[tuple[str, str], Result], T: Timing) -> str:
    body: list[str] = []
    y = bps._header(body, W, "Timeout designs: how each fault ends",
                    ["A's last result and the effect on the bus, per fault scenario (rows) and design (columns).",
                     _params_line(T)])
    cw = (W - EDGE - LABEL_W) / len(designs)
    head = y + 30
    rows_top = head
    for j, d in enumerate(designs):
        cx = LABEL_W + (j + 0.5) * cw
        body.append(_text(cx, head, d.short[0], INK, 11, True))
        for k, part in enumerate(_wrap(d.short[1], cw - 8, 10.5)[:2]):
            body.append(_text(cx, head + 14 + 13 * k, part, GREY, 10.5))
            rows_top = max(rows_top, head + 14 + 13 * k)
    rows_top += 12
    row_h = 66
    for i, s in enumerate(SCENARIOS):
        top = rows_top + i * row_h
        for k, part in enumerate(_wrap(s.title, LABEL_W - 28, 11.5, bold=True)[:2]):
            body.append(_text(16, top + 20 + 15 * k, part, INK, 11.5, True, "start"))
        for j, d in enumerate(designs):
            r = results[s.key, d.key]
            x = LABEL_W + j * cw
            body.append(_rect(x + 2, top + 2, cw - 4, row_h - 4, CATEGORY[r.category][0]))
            lines = [part for line in _cell_lines(r) for part in _wrap(line, cw - 12, 10.5)]
            assert len(lines) <= 4, lines
            for k, part in enumerate(lines):
                body.append(_text(x + cw / 2, top + 19 + 13 * k, part, INK, 10.5))
    y = rows_top + len(SCENARIOS) * row_h + 26
    y = bps._legend(body, y, W, [(bps._sym_rect(fill, "#c4c7cc"), text) for fill, text in CATEGORY.values()])
    notes = ["Single timer columns differ only in the target stretch limit; figures timeout_<scenario>.svg show the "
             "timelines for 50 ms. " + _figure_notes(T)[1], _figure_notes(T)[2]]
    y = bps._notes(body, y + 26, W, notes)
    return bps._svg_doc(W, y + 16, "Timeout designs: how each fault ends", [], body)


def print_summary(designs: tuple[Design, ...], results: dict[tuple[str, str], Result], T: Timing) -> None:
    print(_params_line(T))
    print("| Scenario | " + " | ".join(" ".join(d.short) for d in designs) + " |")
    print("|---" * (len(designs) + 1) + "|")
    for s in SCENARIOS:
        cells = [" / ".join(_cell_lines(results[s.key, d.key])) + f" ({results[s.key, d.key].category})"
                 for d in designs]
        print(f"| {s.title} | " + " | ".join(cells) + " |")


# --------------------------------------------------------------------------- checks and main


def run_all(T: Timing, generic: Profile, lib: ctypes.CDLL) -> tuple[tuple[Design, ...], dict]:
    designs = summary_designs(T, generic)
    return designs, {(s.key, d.key): run(s, d, T, lib) for s in SCENARIOS for d in designs}


def run_checks() -> int:
    T = Timing()
    lib = build_txn_lib()
    designs, res = run_all(T, GENERIC, lib)
    results: list[tuple[bool, str]] = []

    def check(ok: bool, text: str) -> None:
        results.append((bool(ok), text))

    def near(t: int, ref: int, tol: int = MS) -> bool:
        return abs(t - ref) <= tol

    def who(r: Result) -> list[str]:
        return sorted(n for _, n, _ in r.detections)

    lay = [d for d in designs if d.layered]
    one = [d for d in designs if not d.layered]
    fault = MS  # every fault starts within about 0.1 ms after A's submission at 1 ms
    for d in lay:
        r = res["slow_target", d.key]
        check(r.a[-1].outcome == DATA_NAK and near(r.a[-1].t, fault + T.S_max) and r.category == "ok",
              f"slow target, {d.key}: B releases after S_max, A reports DataNak, bus usable")
        r = res["reset_resend", d.key]
        check(r.a[-1].outcome == SUCCESS and near(r.a[-1].t, fault + T.t_reset + T.T_max) and not r.detections,
              f"reset and resend, {d.key}: A's START after the T_max wait ends the missing STOP before t_stuck")
        r = res["reset_idle", d.key]
        check(who(r) == ["B", "C", "D"] and all(near(t, fault + T.t_stuck) for t, _, _ in r.detections),
              f"reset, nothing to send, {d.key}: B, C and D report the missing STOP after t_stuck")
        r = res["join", d.key]
        check(not r.risks and r.a[-1].outcome == SUCCESS and r.longest <= T.T_max,
              f"join, {d.key}: C waits T_max, so its START cannot cut A's transfer")
    for d in one:
        r = res["slow_target", d.key]
        if d.stretch_limit < 40 * MS:
            check(r.a[-1].outcome == SUCCESS and r.category == "risk",
                  f"slow target, {d.key}: B acknowledges while not ready, A reports Success")
        else:
            check(r.a[-1].outcome == SUCCESS and r.blocked > T.T_max and r.category == "slow",
                  f"slow target, {d.key}: A owns the bus for the whole 40 ms stretch")
        r = res["hung_target", d.key]
        check(r.category == "dead" and r.end == "SCL held low by B" and near(r.a[-1].t, fault + T.F)
              and r.a[-1].outcome == TIMEOUT, f"hung target, {d.key}: A times out after F, the bus stays dead")
        r = res["reset_resend", d.key]
        check(r.a[-1].outcome == SUCCESS and near(r.a[-1].t, fault + T.t_reset),
              f"reset and resend, {d.key}: A starts at once after its reset; its START ends the missing STOP")
        r = res["reset_idle", d.key]
        check(not r.detections and r.end == "free" and r.c_failed >= 1 and r.blocked >= T.F - T.c_period,
              f"reset, nothing to send, {d.key}: C times out once; its peripheral reset clears its busy flag")
        r = res["sda_stuck", d.key]
        check(r.category == "dead" and r.end == "SDA held low by B", f"SDA stuck, {d.key}: bus dead, nothing reported")
        r = res["join", d.key]
        check(r.category == "risk", f"join, {d.key}: C's START lands inside A's transfer")
    r = res["hung_target", "layered-stm32"]
    check(r.a[-1].outcome == DATA_NAK and r.category == "ok",
          "hung target, layered STM32: TEXTEN releases without firmware; A reports DataNak")
    r = res["hung_target", "layered-generic"]
    check(who(r) == ["A", "C", "D"] and r.end == "SCL held low by B" and r.category == "failure",
          "hung target, layered generic: the limit needs firmware; A, C and D report SCL low after t_stuck")
    r = res["controller_stall", "layered-stm32"]
    check(r.a[-1].reason == "hardware hold limit" and r.blocked <= T.T_max,
          "controller stall, layered STM32: the hardware ends A's own hold after S_max")
    r = res["controller_stall", "layered-generic"]
    check(r.a[-1].reason == "forced STOP at B_hold" and r.longest > T.T_max,
          "controller stall, layered generic: B_hold needs firmware, so A holds the bus beyond T_max")
    r = res["sda_stuck", "layered-stm32"]
    check(who(r) == ["A", "B", "C", "D"] and near(r.detections[0][0], fault + T.t_reset + T.T_max)
          and r.a[-1].reason == "bus failure" and near(r.a[-1].t, fault + T.t_reset + T.T_max),
          "SDA stuck, layered STM32: A's line check after the wait reports first, its queued read ends as a "
          "bus failure, the others report after t_stuck")
    r = res["sda_stuck", "layered-generic"]
    check(who(r) == ["B", "C", "D"], "SDA stuck, layered generic: no pin read on A; the busy peripherals report")
    for d in designs:
        r = res["booting_target", d.key]
        check(r.a[-1].outcome == SUCCESS and r.a[-1].t >= 300 * MS and r.category == "ok",
              f"booting target, {d.key}: A succeeds after B's I2C is active")
    for s in SCENARIOS:
        r = res[s.key, "layered-stm32"]
        if not r.detections:
            check(r.longest <= T.T_max, f"{s.key}, layered STM32: no ownership longer than T_max")
    shown = figure_designs(T, GENERIC)
    svgs = [timeline_svg(s, [res[s.key, d.key] for d in shown], T) for s in SCENARIOS]
    svgs.append(summary_svg(designs, res, T))
    for k, svg in enumerate(svgs):
        problems = bps.svg_problems(svg)
        check(not problems, f"figure {k + 1}: layout ({'; '.join(problems[:3])})")
    failed = [text for ok, text in results if not ok]
    for ok, text in results:
        print(f"{'ok  ' if ok else 'FAIL'} {text}")
    print(f"{len(results) - len(failed)} of {len(results)} checks passed")
    return 1 if failed else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="run self-checks and exit non-zero on failure")
    parser.add_argument("--params", type=Path, help="TOML file with private values (see load_params)")
    parser.add_argument("--out", type=Path, help="output directory")
    args = parser.parse_args(argv)
    if args.check:
        return run_checks()
    T, generic = (Timing(), GENERIC) if args.params is None else load_params(args.params)
    out = args.out or (OUT_DIR if args.params is None else args.params.parent / "out")
    if args.params is not None and out.resolve() == OUT_DIR.resolve():
        raise SystemExit("results with private parameters must not be written to docs/diagrams")
    designs, results = run_all(T, generic, build_txn_lib())
    out.mkdir(parents=True, exist_ok=True)
    shown = figure_designs(T, generic)
    figures = [(f"timeout_{s.key}.svg", timeline_svg(s, [results[s.key, d.key] for d in shown], T))
               for s in SCENARIOS]
    figures.append(("timeout_summary.svg", summary_svg(designs, results, T)))
    for name, svg in figures:
        (out / name).write_text(svg, encoding="utf-8", newline="\n")
        print(f"wrote {out / name}")
    print_summary(designs, results, T)
    return 0


if __name__ == "__main__":
    sys.exit(main())
