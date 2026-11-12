#!/usr/bin/env python3
"""Simulate and compare four multi-controller I2C bus access policies.

Several controllers share one I2C bus and each sends packets to its own target
address. Arbitration is decided by the target address sent first: when several
controllers start at the same instant, the one addressing the lowest target
address wins and the losers leave no trace on the bus. The policies differ in
when a controller may compete again after its own transfer (its "turn"):

1. Re-arm and retry (baseline): bus busy -> wait until free; arbitration lost
   -> retry at once; after its turn a controller only needs its re-arm time r
   before it competes again.
2. Fixed post-ownership deferral: after its turn a controller waits
   T_max + margin before it competes again.
3. Idle-window deferral (after the MCTP over SMBus/I2C fairness arbitration,
   DMTF DSP0237 section 6.13): after its turn a controller competes again only
   once the bus has been continuously idle for t_idle after its own STOP.
   Controllers that have not had their turn compete as soon as the bus is free.
4. Probe and random backoff (non-persistent CSMA): before every attempt a
   controller probes the bus instead of arming its START while the bus is busy.
   Bus busy or arbitration lost -> back off a random time, uniform in (0, W],
   and probe again; after its own STOP it waits r plus one such backoff. The
   first probe after a STOP takes the bus, so the order of the waiting
   controllers is random instead of decided by their target addresses.

Parameters (the author's numbers; the simulation runs in integer nanoseconds):
- T_max = 20 ms, the bus time of a maximum-length packet. Bus clock set to
  1 Mbit/s, observed effective rate 800 kbit/s; maximum packet 1024 bytes at
  9 bit-times per byte (8 data bits + ACK): 1024 * 9 / 800 000 bit/s = 11.52 ms;
  plus the longest observed clock stretching of 2.7 ms -> 14.22 ms; rounded up
  to 20 ms for margin. Packet lengths m are fractions of T_max: m = T_max is
  the worst case, "10 % packets" are m = 0.1 T_max = 2 ms.
- t_buf = 0.78 us: observed time from a STOP to the next START when other
  participants are already waiting (less than one clock cycle; 780 ns was seen
  repeatedly).
- t_idle = 10 us (policy 3): DSP0237-style idle detection needs about two clock
  cycles = 2.5 us at 800 kHz; 10 us is generous.
- r = 60 us: re-arm time after the controller's own STOP, measured on the bus
  between back-to-back writes of a 48 MHz microcontroller bridge (STOP ->
  completion interrupt -> next submit -> START: 59-65 us). A peripheral that is
  already armed starts t_buf after the STOP, the previous winner only
  r >> t_buf after it, so the winner cannot take the very next START.
- margin = 10 us (policy 2): must exceed 2 * t_buf, the bus-free gaps before and
  after the one other packet the deferral has to let through.
- W = 100 us (policy 4): backoff window. After every STOP the bus stays idle
  until the first waiting controller probes, so W must be small against the
  packet length; a smaller W means more probes (timer wake-ups) per packet.
  Exploratory S3 runs: the mean access delay grows with W, while the worst case
  stays close to policy 3's for W <= 0.05 m; exponential backoff (doubling W on
  every busy probe) and a fixed, non-random period performed worse.

Model (discrete events):
- Each controller writes to one fixed, distinct target address, so the address
  alone decides arbitration. No NAKs or bus errors; clock stretching is covered
  by T_max.
- Saturated controllers always have a packet: the next one is queued at the
  STOP of the previous one (or at t = 0). Poisson controllers receive packets
  as independent seeded Poisson streams and send them in FIFO order; an arrival
  is the moment the firmware could start the packet. All packets of a run have
  the length m.
- After a STOP the bus accepts a new START after t_buf. Every controller that
  may compete by then starts at exactly that instant and the lowest target
  address wins. A controller that becomes ready while the bus is idle starts at
  once; one that becomes ready while the bus is busy waits for the STOP.
- After its own STOP every controller needs r before it can start again. With
  r < t_buf the last winner is back for the next START and keeps the bus. With
  t_buf < r <= m + 2 * t_buf (the default: r = 60 us, m >= 200 us) it lets one
  transfer through and is back for the one after: with three saturated
  controllers A and B alternate and C starves. C gets turns only for
  r > m + 2 * t_buf; for r >= 2 * (m + t_buf) the three take turns.
- Policy 4 probes are instantaneous and its backoffs come from a generator
  seeded with SEED, the controllers, m and W, so every run is reproducible and a
  shorter run is the start of a longer one.

Scenarios (three controllers A, B, C):
  S1  saturated, m = T_max                                100 T_max = 2 s simulated
  S2  saturated, m = 0.1 T_max                            100 T_max = 2 s
  S3  Poisson, 25 % offered load each, m = 0.1 T_max    2 500 T_max = 50 s
  S4  Poisson, 25 % offered load each, m = T_max       25 000 T_max = 500 s
S3 and S4 have about 6 250 packets per controller; all policies see the same
arrivals.

Timelines: S1 and S2 show their start (10 T_max and 2.5 T_max). S3 starts with
empty queues and little contention, during which policies 1 and 3 give every
transfer to the same controller; its timeline therefore shows the 2.5 T_max of
the full run that end just after policy 1's longest wait of C.

Metrics:
- utilisation: fraction of [0, horizon] during which a packet is on the bus.
- access delay, saturated: from queueing a packet (end of the controller's
  previous turn, or t = 0) to the START of the transfer that wins the bus, i.e.
  including re-arm, deferral and lost arbitrations. Poisson: from the packet's
  arrival to that START, including the wait behind the controller's own earlier
  packets. The worst case also counts the oldest packet still waiting at the
  horizon. A saturated controller without any turn is starved; a Poisson
  controller that serves clearly less than its offered load is overloaded (its
  queue grows without bound). Both have unbounded delay.
- wait from the head of the own queue: from the moment a packet is first in its
  controller's queue (its arrival, or the controller's previous STOP) to its
  START. This is the part an access policy controls; the access delay of a
  Poisson packet adds the wait behind the controller's own earlier packets.
  Policy 3 bounds it by (N - 1 + k)·(m + t_buf) + t_idle + r for a controller
  with k lower target addresses: if its next packet arrives while it defers,
  every other controller may send once before the idle window and the k lower
  addresses once more after it. Saturated controllers stay in a fixed round and
  wait only (N - 1)·m.
- Poisson scenarios, checks against queueing theory: seen as a whole, the bus
  is one queue with Poisson arrivals and equal service times m. The mean access
  delay over all packets is compared with the M/D/1 mean wait
  rho * m / (2 * (1 - rho)) (Pollaczek-Khinchine) at the realised offered load
  rho, also for 20 further arrival seeds. Policies 1 and 3 keep the bus busy
  whenever a packet waits (apart from r and t_idle), so by Kleinrock's
  conservation law they have the same mean over all packets and only
  redistribute delay between the controllers. Policy 4 leaves the bus idle after
  every STOP until the first probe, so its mean is higher. The summary also
  reports how bursty the arrivals are (gaps shorter than m, highest offered load
  in 100 ms) and the probes per second of policy 4.

Figures: hand-written SVG. Text widths are estimated conservatively (per
character 0.60 em, digits 0.68 em, capitals 0.78 em, wide glyphs such as m, w,
%, arrows and maths symbols 1.0 em, monospace 0.62 em, bold +10 %; calibrated
against DejaVu Sans, the widest common fallback for Helvetica/Arial) and long
lines are wrapped so that every text stays at least 8 px inside the figure.
Subscripts use tspan dy offsets and rich text is always start-anchored,
because some renderers anchor every tspan on its own.

Usage:
    python3 tools/bus_policy_sim.py           write the SVGs, print a summary
    python3 tools/bus_policy_sim.py --check   run self-checks, exit 1 on failure
Writes docs/diagrams/policy_timeline.svg (S1), policy_timeline_short.svg (S2),
policy_timeline_load.svg (S3), policy_throughput.svg and
policy_access_delay.svg (S1 to S4).
"""

from __future__ import annotations

import argparse
import functools
import math
import random
import re
import sys
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable
from xml.sax.saxutils import escape

OUT_DIR = Path(__file__).resolve().parents[1] / "docs" / "diagrams"

US = 1_000  # all times in integer nanoseconds
MS = 1_000_000
SECOND = 1_000_000_000

SET_BIT_RATE = 1_000_000  # bit/s
EFFECTIVE_BIT_RATE = 800_000  # bit/s, observed
MAX_PACKET_BYTES = 1024
BITS_PER_BYTE = 9  # 8 data bits + ACK
WIRE_TIME = MAX_PACKET_BYTES * BITS_PER_BYTE * SECOND // EFFECTIVE_BIT_RATE  # 11.52 ms
CLOCK_STRETCH = 2_700 * US  # longest observed clock stretching

T_MAX = 20 * MS  # WIRE_TIME + CLOCK_STRETCH = 14.22 ms, rounded up
T_BUF = 780  # bus-free time after STOP before the next START
T_IDLE = 10 * US  # policy 3: idle window that re-arms controllers after their turn
REARM = 60 * US  # re-arm time after the controller's own STOP (measured, 59-65 us)
MARGIN = 10 * US  # policy 2: deferral is T_MAX + MARGIN
W_BACKOFF = 100 * US  # policy 4: random backoff, uniform in (0, W]
NEVER = -(10**15)
SEED = 2026
REPLICATIONS = 20  # further arrival seeds for the comparison with queueing theory
BURST_WINDOW = 100 * MS  # window for the highest offered load of the arrivals

BASELINE, FIXED, IDLE, PROBE = "baseline", "fixed", "idle", "probe"
POLICIES = (BASELINE, FIXED, IDLE, PROBE)
POLICY_TITLES = {
    BASELINE: "Re-arm and retry (baseline)",
    FIXED: "Fixed post-ownership deferral",
    IDLE: "Idle-window deferral",
    PROBE: "Probe and random backoff",
}
POLICY_RULES = {
    BASELINE: (
        "Bus busy → wait for STOP; arbitration lost → retry at once; after its own STOP a controller "
        "needs its re-arm time r before it competes again."
    ),
    FIXED: "After its own STOP a controller waits T_max + margin before it competes again.",
    IDLE: (
        "After its own STOP a controller waits until the bus has been idle for t_idle; "
        "controllers that have not had their turn compete as soon as the bus is free."
    ),
    PROBE: (
        "Before every attempt a controller probes the bus; bus busy or arbitration lost → back off a random "
        "time in (0, W] and probe again; after its own STOP it waits r plus one such backoff."
    ),
}
POLICY_COLOURS = {BASELINE: "#c5221f", FIXED: "#e37400", IDLE: "#137333", PROBE: "#3f51b5"}

HOLD, WAIT, TX = "hold", "wait", "tx"

CHECK_HORIZON = 1000 * T_MAX
THROUGHPUT_HORIZON = 200 * T_MAX
LENGTHS = (0.01, 0.025, 0.05) + tuple(k / 20 for k in range(2, 21))  # m / T_max
SHORT = 0.1  # "10 % packets"

FONT = "Helvetica, Arial, sans-serif"
MONO = "Consolas, 'DejaVu Sans Mono', monospace"
INK, GREY, MUTED, GRID, GRID_STRONG = "#202124", "#5f6368", "#9aa0a6", "#e8eaed", "#c4c7cc"
IDLE_FILL, LANE_FILL = "#f1f3f4", "#f8f9fa"
ALARM, GOOD = "#c5221f", "#137333"
SUBSCRIPT = re.compile(r"\b([TtSB])_(max|buf|idle|hold|stuck|reset)\b")
EDGE = 8  # every text and shape keeps this distance from the left and right edge


@dataclass(frozen=True)
class Controller:
    name: str
    address: int  # 7-bit target address this controller writes to


CONTROLLERS = (Controller("A", 0x10), Controller("B", 0x20), Controller("C", 0x30))
EXTRA = Controller("D", 0x40)  # only used by the four-controller checks
COLOURS = {"A": "#1f6fd1", "B": "#e8710a", "C": "#9334e6", "D": "#188038"}
TINTS = {"A": "#d2e3fc", "B": "#fde3cc", "C": "#ead9fb", "D": "#ceead6"}


@dataclass(frozen=True)
class Params:
    t_buf: int = T_BUF
    rearm: int = REARM
    margin: int = MARGIN
    t_idle: int = T_IDLE
    backoff: int = W_BACKOFF


@dataclass(frozen=True)
class Scenario:
    key: str
    length: int  # packet length m
    load: float | None  # offered load per controller (share of bus time); None: saturated
    horizon: int
    span: int  # time span of the timeline figure, 0: none

    @property
    def saturated(self) -> bool:
        return self.load is None

    def describe(self) -> str:
        if self.load is None:
            sources = f"{len(CONTROLLERS)} saturated controllers"
        else:
            sources = f"{len(CONTROLLERS)} Poisson sources, {_load(self.load)} offered load each"
        return f"{sources}, {_m_label(self.length)}"


SCENARIOS = (
    Scenario("S1", T_MAX, None, 100 * T_MAX, 10 * T_MAX),
    Scenario("S2", T_MAX // 10, None, 100 * T_MAX, 5 * T_MAX // 2),
    Scenario("S3", T_MAX // 10, 0.25, 2500 * T_MAX, 5 * T_MAX // 2),
    Scenario("S4", T_MAX, 0.25, 25000 * T_MAX, 0),
)
SCENARIO = {s.key: s for s in SCENARIOS}


@dataclass(frozen=True)
class Turn:
    owner: int  # index into Run.controllers
    arrival: int  # packet arrival; saturated: previous own STOP, or 0
    queued: int  # packet at the head of its controller's queue
    eligible: int  # from here on the controller competes (after re-arm/deferral)
    start: int
    end: int
    losers: tuple[int, ...]  # controllers that started at the same instant and lost


@dataclass(frozen=True)
class Pending:
    """State of one controller at the horizon."""

    arrival: int | None  # oldest packet still waiting
    queued: int | None
    eligible: int  # when the controller may compete again
    backlog: int  # packets waiting


@dataclass(frozen=True)
class Run:
    policy: str
    controllers: tuple[Controller, ...]
    length: int
    horizon: int
    params: Params
    arrivals: tuple[tuple[int, ...], ...] | None  # None: saturated
    turns: tuple[Turn, ...]
    pending: tuple[Pending, ...]
    probes: tuple[int, ...] = ()  # policy 4: bus probes per controller

    @property
    def saturated(self) -> bool:
        return self.arrivals is None

    def probe_rate(self, i: int) -> float:
        """Policy 4: probes per second of controller i over the whole run."""
        return self.probes[i] * SECOND / self.horizon if self.probes else 0.0

    @functools.cached_property
    def _busy(self) -> tuple[int, ...]:
        busy = [0] * len(self.controllers)
        for t in self.turns:
            busy[t.owner] += min(t.end, self.horizon) - t.start
        return tuple(busy)

    @functools.cached_property
    def _delays(self) -> tuple[tuple[int, ...], ...]:
        delays: list[list[int]] = [[] for _ in self.controllers]
        for t in self.turns:
            delays[t.owner].append(t.start - t.arrival)
        return tuple(tuple(d) for d in delays)

    def utilisation(self) -> float:
        return sum(self._busy) / self.horizon

    def turn_count(self, i: int) -> int:
        return len(self._delays[i])

    def share(self, i: int) -> float:
        total = sum(self._busy)
        return self._busy[i] / total if total else 0.0

    def served(self, i: int) -> float:
        return self._busy[i] / self.horizon

    def offered(self, i: int) -> float | None:
        return None if self.arrivals is None else len(self.arrivals[i]) * self.length / self.horizon

    def backlog(self, i: int) -> int:
        return self.pending[i].backlog

    def starved(self, i: int) -> bool:
        return self.turn_count(i) == 0 and self.pending[i].arrival is not None

    def overloaded(self, i: int) -> bool:
        offered = self.offered(i)
        return offered is not None and self.served(i) < 0.95 * offered

    def unbounded(self, i: int) -> bool:
        return self.starved(i) or self.overloaded(i)

    def delays(self, i: int) -> tuple[int, ...]:
        return self._delays[i]

    def worst_delay(self, i: int) -> float:
        if self.starved(i):
            return math.inf
        head = self.pending[i].arrival
        return max(self._delays[i] + (() if head is None else (self.horizon - head,)))

    def worst_head_delay(self, i: int) -> float:
        """Longest wait from the head of controller i's own queue (arrival or own previous STOP) to START."""
        if self.starved(i):
            return math.inf
        waits = [t.start - t.queued for t in self.turns if t.owner == i]
        queued = self.pending[i].queued
        if queued is not None:
            waits.append(self.horizon - queued)
        return max(waits, default=0)

    def mean_delay(self, i: int) -> float:
        delays = self._delays[i]
        return sum(delays) / len(delays) if delays else math.inf

    def segments(self, i: int, t0: int = 0, t1: int | None = None) -> list[tuple[int, int, str]]:
        """Hold/wait/transmit intervals of controller i, clipped to [t0, t1]."""
        t1 = self.horizon if t1 is None else t1
        segs: list[tuple[int, int, str]] = []
        for t in self.turns:
            if t.owner == i:
                ready = max(t.queued, t.eligible)
                segs += [(t.queued, ready, HOLD), (ready, t.start, WAIT), (t.start, t.end, TX)]
        p = self.pending[i]
        if p.queued is not None:
            ready = max(p.queued, p.eligible)
            segs += [(p.queued, ready, HOLD), (ready, self.horizon, WAIT)]
        clipped = [(max(a, t0), min(b, t1, self.horizon), s) for a, b, s in segs]
        return [(a, b, s) for a, b, s in clipped if b > a]


def simulate(
    policy: str,
    controllers: tuple[Controller, ...] = CONTROLLERS,
    length: int = T_MAX,
    horizon: int = CHECK_HORIZON,
    params: Params = Params(),
    arrivals: tuple[tuple[int, ...], ...] | None = None,
) -> Run:
    """Run the controllers under one policy until the next START would be past the horizon.

    Without arrivals every controller is saturated; otherwise arrivals[i] are the
    arrival times of controller i's packets, sent in FIFO order.
    """
    n = len(controllers)
    if arrivals is not None:
        arrivals = tuple(tuple(a for a in stream if a < horizon) for stream in arrivals)
        queues = [deque(stream) for stream in arrivals]
    else:
        queues = [deque([0]) for _ in range(n)]
    ready: list[int | None] = [0] * n  # None: waiting for the idle window (policy 3)
    own_stop = [NEVER] * n
    last_stop = NEVER
    turns: list[Turn] = []
    starts: list[int | None] = [None] * n
    # Policy 4: next probe per controller; the seed excludes the horizon, so a short run is the start of a long one.
    probe = [0] * n
    probes = [0] * n
    rng = random.Random(f"{SEED}/probe/{','.join(c.name for c in controllers)}/{length}/{params.backoff}")

    def backoff() -> int:
        return rng.randint(1, params.backoff)

    while True:
        window = last_stop + params.t_idle
        bus_free = last_stop + params.t_buf
        start = None
        for i in range(n):
            if not queues[i]:
                starts[i] = None
                continue
            e = probe[i] if policy == PROBE else ready[i]
            if e is None:
                e = max(window, own_stop[i] + params.rearm)
            s = starts[i] = max(e, queues[i][0], bus_free)
            if start is None or s < start:
                start = s
        if start is None or start >= horizon:
            break
        contenders = [i for i in range(n) if starts[i] == start]
        winner = min(contenders, key=lambda i: controllers[i].address)
        if policy == IDLE and start - last_stop >= params.t_idle:
            # The bus has been idle for t_idle: every deferring controller is re-armed.
            for i in range(n):
                if ready[i] is None:
                    ready[i] = max(window, own_stop[i] + params.rearm)
        eligible = ready[winner]
        assert eligible is not None
        arrival = queues[winner].popleft()
        end = start + length
        losers = tuple(i for i in contenders if i != winner)
        turns.append(Turn(winner, arrival, max(arrival, own_stop[winner]), eligible, start, end, losers))
        own_stop[winner] = last_stop = end
        if arrivals is None:
            queues[winner].append(end)
        if policy == BASELINE:
            ready[winner] = end + params.rearm
        elif policy == FIXED:
            ready[winner] = end + max(params.rearm, T_MAX + params.margin)
        elif policy == PROBE:
            for i in contenders:
                probes[i] += 1  # the probe that found the bus free
            ready[winner] = probe[winner] = end + params.rearm + backoff()
            for i in losers:
                probe[i] = start + backoff()
            for i in range(n):
                if i == winner or not queues[i]:
                    continue
                # Every probe during this transfer finds the bus busy and backs off again.
                t = max(probe[i], queues[i][0])
                while t < end:
                    probes[i] += 1
                    t += backoff()
                probe[i] = t
        else:
            ready[winner] = None
    window = last_stop + params.t_idle
    pending = []
    for i in range(n):
        waiting = [a for a in queues[i] if a < horizon]
        e = ready[i]
        eligible = max(window, own_stop[i] + params.rearm) if e is None else e
        head = waiting[0] if waiting else None
        pending.append(Pending(head, None if head is None else max(head, own_stop[i]), eligible, len(waiting)))
    return Run(policy, controllers, length, horizon, params, arrivals, tuple(turns), tuple(pending),
               tuple(probes) if policy == PROBE else ())


def _poisson_streams(scenario: Scenario, tag: str) -> tuple[tuple[int, ...], ...]:
    """Independent Poisson arrival streams, one per controller, seeded with SEED and tag."""
    assert scenario.load is not None
    rate = scenario.load / scenario.length  # packets per ns
    streams = []
    for c in CONTROLLERS:
        rng = random.Random(f"{SEED}/{tag}/{c.name}")
        t, times = 0.0, []
        while True:
            t += rng.expovariate(rate)
            if t >= scenario.horizon:
                break
            times.append(int(t))
        streams.append(tuple(times))
    return tuple(streams)


@functools.lru_cache(maxsize=None)
def poisson_arrivals(key: str) -> tuple[tuple[int, ...], ...]:
    """Independent, seeded Poisson arrival streams, one per controller."""
    return _poisson_streams(SCENARIO[key], key)


@functools.lru_cache(maxsize=None)
def scenario_run(key: str, policy: str, horizon: int | None = None) -> Run:
    scenario = SCENARIO[key]
    arrivals = None if scenario.saturated else poisson_arrivals(key)
    span = scenario.horizon if horizon is None else horizon
    return simulate(policy, CONTROLLERS, scenario.length, span, Params(), arrivals)


def timeline_window(key: str) -> tuple[int, int]:
    """Time window [t0, t1) of the full run shown in the timeline figure.

    Saturated scenarios show their start. A Poisson scenario starts with empty
    queues and little contention, so policies 1 and 3 begin with the same bus
    owners; its window ends just after the START of policy 1's longest wait of
    the highest-address controller, where the policies differ.
    """
    sc = SCENARIO[key]
    if sc.saturated:
        return 0, sc.span
    base = scenario_run(key, BASELINE)
    last = len(CONTROLLERS) - 1
    worst = max((t for t in base.turns if t.owner == last), key=lambda t: t.start - t.arrival)
    step = 5 * MS
    end = -(-(worst.start + sc.length) // step) * step
    return max(0, end - sc.span), max(sc.span, end)


def first_owner_difference(a: Run, b: Run) -> tuple[int | None, float]:
    """START of the first transfer that a and b give to different controllers, and the share of such transfers."""
    pairs = list(zip(a.turns, b.turns))
    differ = [ta.start for ta, tb in pairs if ta.owner != tb.owner]
    return (differ[0] if differ else None), (len(differ) / len(pairs) if pairs else 0.0)


def mean_delay_all(run: Run) -> float:
    """Mean access delay over all transferred packets of all controllers."""
    delays = [d for i in range(len(run.controllers)) for d in run.delays(i)]
    return sum(delays) / len(delays)


def idle_head_bound(run: Run, i: int) -> int:
    """Policy 3: longest wait of controller i from the head of its own queue to START.

    A controller whose next packet arrives while it defers lets every other
    controller send once before the idle window, and the k controllers with a
    lower target address once more after it: (N - 1 + k) transfers.
    """
    k = sum(c.address < run.controllers[i].address for c in run.controllers)
    p = run.params
    return (len(run.controllers) - 1 + k) * (run.length + p.t_buf) + p.t_idle + p.rearm


def md1_wait(run: Run) -> float:
    """Mean wait of an M/D/1 queue at the run's realised offered load rho (Pollaczek-Khinchine): rho·m / (2(1 − rho))."""
    rho = sum(run.offered(i) or 0.0 for i in range(len(run.controllers)))
    return rho * run.length / (2 * (1 - rho))


@functools.lru_cache(maxsize=None)
def md1_ratios(key: str) -> tuple[float, ...]:
    """Policy 1: mean access delay over all packets / M/D/1 prediction, for REPLICATIONS other arrival seeds."""
    sc = SCENARIO[key]
    ratios = []
    for k in range(REPLICATIONS):
        run = simulate(BASELINE, CONTROLLERS, sc.length, sc.horizon, Params(), _poisson_streams(sc, f"{key}/rep{k}"))
        ratios.append(mean_delay_all(run) / md1_wait(run))
    return tuple(ratios)


@functools.lru_cache(maxsize=None)
def arrival_stats(key: str) -> tuple[float, float, float]:
    """Burstiness of the arrivals: share of gaps < m, share of gaps < mean gap, highest offered load in a BURST_WINDOW."""
    sc = SCENARIO[key]
    streams = poisson_arrivals(key)
    gaps = [b - a for stream in streams for a, b in zip(stream, stream[1:])]
    mean_gap = sum(gaps) / len(gaps)
    times = sorted(a for stream in streams for a in stream)
    peak = j = 0
    for i, t in enumerate(times):
        while times[j] <= t - BURST_WINDOW:
            j += 1
        peak = max(peak, i - j + 1)
    short = sum(g < sc.length for g in gaps) / len(gaps)
    below = sum(g < mean_gap for g in gaps) / len(gaps)
    return short, below, peak * sc.length / BURST_WINDOW


@functools.lru_cache(maxsize=None)
def throughput_data() -> dict[tuple[str, int], tuple[float, ...]]:
    """Utilisation per (policy, active saturated controllers) for every m in LENGTHS."""
    return {
        (policy, active): tuple(
            simulate(policy, CONTROLLERS[:active], round(m * T_MAX), THROUGHPUT_HORIZON).utilisation()
            for m in LENGTHS
        )
        for policy in POLICIES
        for active in range(1, len(CONTROLLERS) + 1)
    }


def _clear_caches() -> None:
    for cached in (poisson_arrivals, scenario_run, throughput_data, md1_ratios, arrival_stats):
        cached.cache_clear()


# --------------------------------------------------------------------------- wording


def _ms(ns: float) -> str:
    """Duration in ms with three significant digits for values below 100 ms."""
    v = ns / MS
    return f"{v:.{2 if v < 10 else 1 if v < 100 else 0}f}"


def _duration(ns: float) -> str:
    for unit, name in ((SECOND, "s"), (MS, "ms")):
        if ns >= unit and ns % unit == 0:
            return f"{ns // unit} {name}"
    if ns >= 10 * SECOND:
        return f"{ns / SECOND:.0f} s"
    if ns >= SECOND:
        return f"{ns / SECOND:.2f} s"
    if ns >= MS:
        return f"{_ms(ns)} ms"
    return f"{ns / US:.3g} µs"


def _param(ns: int) -> str:
    """Exact parameter value: 20 ms, 0.78 µs, 10 µs."""
    return f"{ns / MS:g} ms" if ns >= MS else f"{ns / US:g} µs"


def _pct(u: float) -> str:
    return f"{u * 100:.1f} %"


def _load(u: float) -> str:
    return f"{u * 100:g} %"


def _tmax(ns: float) -> str:
    return f"{ns / T_MAX:.2f} T_max"


def _m_label(length: int) -> str:
    frac = length / T_MAX
    return f"m = {'T_max' if frac == 1 else f'{frac:g} T_max'} = {_param(length)}"


def _policy_label(policy: str) -> str:
    return f"Policy {POLICIES.index(policy) + 1} · {POLICY_TITLES[policy]}"


def _names(names: list[str]) -> str:
    return " and ".join(", ".join(names).rsplit(", ", 1)) if names else "nobody"


def _verb(names: list[str], singular: str, plural: str) -> str:
    return singular if len(names) == 1 else plural


def _capacity(length: int, params: Params = Params()) -> float:
    """Policy 2: largest share of bus time one controller can use."""
    return length / (length + max(params.rearm, T_MAX + params.margin))


def _params_line(p: Params = Params()) -> str:
    return (
        f"T_max = {_param(T_MAX)}, t_buf = {_param(p.t_buf)}, r = {_param(p.rearm)} (measured), "
        f"margin = {_param(p.margin)}, t_idle = {_param(p.t_idle)}, W = {_param(p.backoff)}"
    )


def _derivation_line() -> str:
    return (
        f"T_max: {MAX_PACKET_BYTES} bytes × {BITS_PER_BYTE} bit / {EFFECTIVE_BIT_RATE // 1000} kbit/s "
        f"(set {SET_BIT_RATE // 1_000_000} Mbit/s) = {_param(WIRE_TIME)} + {_param(CLOCK_STRETCH)} "
        f"clock stretching = {_param(WIRE_TIME + CLOCK_STRETCH)}, rounded up to {_param(T_MAX)}"
    )


def _verdict(run: Run) -> tuple[str, str]:
    names = [c.name for c in run.controllers]
    starved = [names[i] for i in range(len(names)) if run.starved(i)]
    owners = [names[i] for i in range(len(names)) if run.turn_count(i)]
    if starved:
        text = f"{_names(starved)} never {_verb(starved, 'gets', 'get')} the bus"
        if len(owners) > 1:
            text += f" · {_names(owners)} alternate"
        elif owners:
            text += f" · {owners[0]} keeps it"
        return text, ALARM
    overloaded = [names[i] for i in range(len(names)) if run.overloaded(i)]
    if overloaded:
        return f"overloaded: the queues of {_names(overloaded)} grow without bound", ALARM
    worst = max(run.worst_delay(i) for i in range(len(names)))
    since = "" if run.saturated else " from arrival"
    return f"every controller gets the bus · worst access delay{since} {_ms(worst)} ms = {_tmax(worst)}", GOOD


# --------------------------------------------------------------------------- text and SVG helpers

SANS_EM, DIGIT_EM, UPPER_EM, WIDE_EM, MONO_EM, BOLD_FACTOR = 0.60, 0.68, 0.78, 1.0, 0.62, 1.10
WIDE_CHARS = frozenset("mwMW%@—→←∞≈≤≥×−…")
SUB_SCALE, SUB_SHIFT = 0.72, 0.3


def _char_em(ch: str, mono: bool) -> float:
    if mono:
        return MONO_EM
    if ch in WIDE_CHARS:
        return WIDE_EM
    if ch.isupper():
        return UPPER_EM
    if ch.isdigit() or ch == "µ":
        return DIGIT_EM
    return SANS_EM


def _chunks(text: str) -> list[tuple[str, bool]]:
    """Split text into (chunk, is_subscript) for the parameter names of SUBSCRIPT (T_max, t_buf, ...)."""
    parts: list[tuple[str, bool]] = []
    pos = 0
    for match in SUBSCRIPT.finditer(text):
        parts += [(text[pos : match.start()] + match.group(1), False), (match.group(2), True)]
        pos = match.end()
    parts.append((text[pos:], False))
    return [(chunk, sub) for chunk, sub in parts if chunk]


def _run_width(chunk: str, size: float, bold: bool, mono: bool) -> float:
    return size * sum(_char_em(ch, mono) for ch in chunk) * (BOLD_FACTOR if bold else 1.0)


def _width(text: str, size: float, bold: bool = False, mono: bool = False) -> float:
    """Conservative estimate of the rendered width of text in px."""
    return sum(_run_width(chunk, size * (SUB_SCALE if sub else 1), bold, mono) for chunk, sub in _chunks(text))


UNIT_AFTER_NUMBER = re.compile(r"^(?:(?:ms|µs|s|T_max)\b|%)")
OPERATORS = frozenset("=≈≤≥<>+−×/→")


def _tokens(text: str) -> list[str]:
    """Words that must not be split: a number and its unit, an operator and its operand."""
    tokens: list[str] = []
    carry = ""
    for word in text.split(" "):
        if carry:
            word, carry = f"{carry} {word}", ""
        if word in OPERATORS:
            carry = word
        elif tokens and UNIT_AFTER_NUMBER.match(word) and tokens[-1][-1:].isdigit():
            tokens[-1] += f" {word}"
        else:
            tokens.append(word)
    return tokens + ([carry] if carry else [])


def _wrap(text: str, width: float, size: float, bold: bool = False, mono: bool = False) -> list[str]:
    lines: list[str] = []
    line = ""
    for word in _tokens(text):
        candidate = f"{line} {word}" if line else word
        if line and _width(candidate, size, bold, mono) > width:
            lines.append(line)
            line = word
        else:
            line = candidate
    return lines + [line]


def _f(v: float) -> str:
    s = f"{v:.2f}".rstrip("0").rstrip(".")
    return "0" if s in ("", "-0") else s


def _rich(text: str, size: float) -> str:
    """Escape text and lower the subscripts of SUBSCRIPT (T_max, t_buf, ...) with dy (and raise back)."""
    shift = _f(size * SUB_SHIFT)
    out, low = [], False
    for chunk, sub in _chunks(text):
        if sub:
            out.append(f'<tspan dy="{shift}" font-size="{_f(size * SUB_SCALE)}">{escape(chunk)}</tspan>')
            low = True
        elif low:
            out.append(f'<tspan dy="-{shift}">{escape(chunk)}</tspan>')
            low = False
        else:
            out.append(escape(chunk))
    return "".join(out)


def _text(
    x: float,
    y: float,
    text: str,
    colour: str = INK,
    size: float = 12,
    bold: bool = False,
    anchor: str = "middle",
    mono: bool = False,
    rotate: bool = False,
) -> str:
    if anchor != "start" and SUBSCRIPT.search(text):
        # Renderers that anchor each tspan separately would misplace the subscripts.
        shift = _width(text, size, bold, mono) * (1.0 if anchor == "end" else 0.5)
        if rotate:
            y += shift
        else:
            x -= shift
        anchor = "start"
    extra = ' font-weight="bold"' if bold else ""
    extra += f' font-family="{MONO}"' if mono else ""
    if rotate:
        extra += f' transform="rotate(-90 {_f(x)} {_f(y)})"'
    return (
        f'<text x="{_f(x)}" y="{_f(y)}" font-size="{_f(size)}" fill="{colour}" '
        f'text-anchor="{anchor}"{extra}>{_rich(text, size)}</text>'
    )


def _boxed_text(x: float, y: float, text: str, size: float, colour: str = INK, bold: bool = False,
                rotate: bool = False) -> list[str]:
    """Centred text on a white box (instead of a stroke halo, which not every renderer supports)."""
    w = _width(text, size, bold) + 6
    h = size * 1.25
    if rotate:
        box = _rect(x - size * 0.95, y - w / 2, h, w, "#ffffff")
    else:
        box = _rect(x - w / 2, y - size * 0.95, w, h, "#ffffff")
    return [box, _text(x, y, text, colour, size, bold, rotate=rotate)]


def _cls(cls: str) -> str:
    return f' class="{cls}"' if cls else ""


def _line(x0: float, y0: float, x1: float, y1: float, colour: str, width: float = 1, dash: str = "",
          cls: str = "") -> str:
    dashed = f' stroke-dasharray="{dash}"' if dash else ""
    return (
        f'<line x1="{_f(x0)}" y1="{_f(y0)}" x2="{_f(x1)}" y2="{_f(y1)}" stroke="{colour}" '
        f'stroke-width="{_f(width)}"{dashed}{_cls(cls)}/>'
    )


def _rect(x: float, y: float, w: float, h: float, fill: str, stroke: str = "", cls: str = "") -> str:
    outline = f' stroke="{stroke}"' if stroke else ""
    return f'<rect x="{_f(x)}" y="{_f(y)}" width="{_f(w)}" height="{_f(h)}" fill="{fill}"{outline}{_cls(cls)}/>'


def _polyline(points: list[tuple[float, float]], colour: str, width: float = 2, dash: str = "",
              cls: str = "") -> str:
    dashed = f' stroke-dasharray="{dash}"' if dash else ""
    pts = " ".join(f"{_f(x)},{_f(y)}" for x, y in points)
    return (
        f'<polyline points="{pts}" fill="none" stroke="{colour}" stroke-width="{_f(width)}" '
        f'stroke-linejoin="round" stroke-linecap="round"{dashed}{_cls(cls)}/>'
    )


def _cross(x: float, y: float, r: float = 3.5, cls: str = "") -> str:
    return (
        f'<path d="M{_f(x - r)},{_f(y - r)}L{_f(x + r)},{_f(y + r)}M{_f(x - r)},{_f(y + r)}'
        f'L{_f(x + r)},{_f(y - r)}" stroke="{INK}" stroke-width="1.6" stroke-linecap="round"{_cls(cls)}/>'
    )


def _marker(x: float, y: float, colour: str, shape: str, cls: str = "data") -> str:
    if shape == "square":
        return _rect(x - 3, y - 3, 6, 6, colour, cls=cls)
    fill = colour if shape == "dot" else "#ffffff"
    return f'<circle cx="{_f(x)}" cy="{_f(y)}" r="3" fill="{fill}" stroke="{colour}" stroke-width="1.5"{_cls(cls)}/>'


def _plot_area(x: float, y: float, w: float, h: float) -> str:
    """Invisible rectangle that marks a plot area; --check keeps all data marks inside it."""
    return _rect(x, y, w, h, "none", cls="plot-area")


def _stripes(pattern_id: str, colour: str, background: str) -> str:
    return (
        f'<pattern id="{pattern_id}" width="6" height="6" patternUnits="userSpaceOnUse" '
        f'patternTransform="rotate(45)"><rect width="6" height="6" fill="{background}"/>'
        f'<line x1="0" y1="0" x2="0" y2="6" stroke="{colour}" stroke-width="2.4"/></pattern>'
    )


def _svg_doc(width: int, height: float, title: str, defs: list[str], body: list[str]) -> str:
    h = math.ceil(height)
    head = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{h}" '
            f'viewBox="0 0 {width} {h}" font-family="{FONT}">'
        ),
        f"<title>{escape(title)}</title>",
        "<defs>",
        *defs,
        "</defs>",
        f'<rect width="{width}" height="{h}" fill="#ffffff"/>',
    ]
    return "\n".join(head + body + ["</svg>"]) + "\n"


def _header(body: list[str], width: int, title: str, lines: list[str]) -> float:
    """Title and monospace subtitle lines; returns the y of the last baseline."""
    body.append(_text(16, 28, title, INK, 18, True, "start"))
    y = 28.0
    for line in lines:
        for part in _wrap(line, width - 16 - 2 * EDGE, 12, mono=True):
            y += 18
            body.append(_text(16, y, part, "#3c4043", 12, anchor="start", mono=True))
    return y


Symbol = Callable[[float, float], list[str]]


def _legend(body: list[str], y: float, width: int, items: list[tuple[Symbol, str]], x0: float = 16) -> float:
    """Lay out (symbol, label) pairs left to right, wrapping at the right edge; returns the last baseline."""
    x = x0
    for symbol, label in items:
        w = 30 + _width(label, 12)
        if x > x0 and x + w > width - EDGE:
            x, y = x0, y + 20
        body += symbol(x, y)
        body.append(_text(x + 30, y, label, INK, 12, anchor="start"))
        x += w + 22
    return y


def _sym_rect(fill: str, stroke: str = "") -> Symbol:
    return lambda x, y: [_rect(x, y - 10, 24, 12, fill, stroke)]


def _sym_line(colour: str, width: float = 2, dash: str = "", shape: str = "") -> Symbol:
    def draw(x: float, y: float) -> list[str]:
        out = [_line(x, y - 4, x + 24, y - 4, colour, width, dash)]
        if shape:
            out.append(_marker(x + 12, y - 4, colour, shape, cls=""))
        return out

    return draw


def _notes(body: list[str], y: float, width: int, notes: list[str]) -> float:
    """Wrapped grey notes starting at baseline y; returns the last baseline."""
    y -= 17
    for note in notes:
        for line in _wrap(note, width - 16 - 2 * EDGE, 11.5):
            y += 17
            body.append(_text(16, y, line, GREY, 11.5, anchor="start"))
        y += 3
    return y - 3


# --------------------------------------------------------------------------- figure: timelines

TL_W, TL_X0, TL_PLOT = 1220, 120, 1000
TL_X1 = TL_X0 + TL_PLOT
TL_VERDICT_X = 16 + max(_width(_policy_label(q), 14, True) for q in POLICIES) + 40
TIMELINE_TITLES = {
    "S1": "Three saturated controllers, maximum-length packets",
    "S2": "Three saturated controllers, short packets",
    "S3": "Three controllers under Poisson load, short packets",
}


def _owner_label(x0: float, x1: float, y: float, c: Controller) -> list[str]:
    """Target address on a bus-owner bar, with the small controller letter if there is room."""
    w = x1 - x0
    for text in (f"→0x{c.address:02x}", f"0x{c.address:02x}"):
        for size in (12, 11, 10):
            tw = _width(text, size, True)
            if tw <= w - 6:
                out = [_text((x0 + x1) / 2, y, text, "#ffffff", size, True)]
                if (w - tw) / 2 >= _width(c.name, 9) + 10:
                    out.append(_text(x0 + 4, y, c.name, "#ffffff", 9, anchor="start"))
                return out
    return []


def _hold_label(run: Run) -> list[str]:
    if run.policy == FIXED:
        return ["holding off T_max + margin", "T_max + margin"]
    if run.policy == IDLE:
        return ["waits for t_idle of idle bus", "waits for idle bus"]
    if run.policy == PROBE:
        return ["r + random backoff", "backoff"]
    return []


def _count(x: float) -> str:
    """Rounded count with a non-breaking thousands separator: 13 200."""
    return f"{round(x, -2 if x >= 1000 else 0):,.0f}".replace(",", "\u00a0")


def _probe_rate(run: Run) -> float:
    """Policy 4: mean probes per second per controller."""
    return sum(run.probe_rate(i) for i in range(len(run.controllers))) / len(run.controllers)


def _probe_timeline_note(sc: Scenario, base: Run, idle: Run, probe: Run) -> str:
    names = [c.name for c in probe.controllers]
    n = len(names)
    worst = max(probe.worst_delay(i) for i in range(n))
    worst_idle = max(idle.worst_delay(i) for i in range(n))
    rate = f"each controller probes about {_count(_probe_rate(probe))} times per second"
    if sc.saturated:
        gap = probe.horizon * (1 - probe.utilisation()) / len(probe.turns)
        turns = ", ".join(f"{c} {probe.turn_count(i)}" for i, c in enumerate(names))
        return (
            "Policy 4: after every STOP the waiting controllers probe at random times and the first probe takes "
            "the bus; the last sender first waits r plus a backoff, so it rarely wins the next transfer. Turns "
            f"{turns} ({_duration(probe.horizon)} run): the address no longer decides who sends, but the order is "
            "random, so a controller may lose several times in a row: worst access delay "
            f"{_ms(worst)} ms = {_tmax(worst)} (policy 3: {_ms(worst_idle)} ms). The bus stays idle until the first "
            f"probe, on average {_duration(gap)} per transfer: utilisation {_pct(probe.utilisation())}; {rate}."
        )
    means = ", ".join(f"{c} {_ms(probe.mean_delay(i))}" for i, c in enumerate(names))
    heads = ", ".join(f"{c} {_ms(probe.worst_head_delay(i))}" for i, c in enumerate(names))
    return (
        f"Policy 4: worst access delay {_ms(worst)} ms, mean {means} ms: the address no longer decides who waits "
        f"(policy 1: {names[-1]} {_ms(base.mean_delay(n - 1))} ms). The bus stays idle after every STOP until the "
        "first probe, so the mean over all packets rises to "
        f"{_ms(mean_delay_all(probe))} ms (policies 1 and 3: {_ms(mean_delay_all(base))} and "
        f"{_ms(mean_delay_all(idle))} ms). The wait from the head of the own queue has no bound, only a "
        f"probability: here {heads} ms (policy 3: ≤ (N − 1 + k)·m). {rate[0].upper() + rate[1:]}."
    )


def _timeline_panel(
    body: list[str], y0: float, run: Run, long: Run, ticks: list[tuple[int, bool]], t0: int, t1: int
) -> float:
    """One policy in the window [t0, t1): header, bus-owner row and one lane per controller; returns the bottom y."""
    n = len(run.controllers)
    scale = TL_PLOT / (t1 - t0)
    shown = [t for t in run.turns if t.end > t0 and t.start < t1]

    def tx(t: int) -> float:
        return TL_X0 + (t - t0) * scale

    body.append(_text(16, y0 + 16, _policy_label(run.policy), INK, 14, True, "start"))
    verdict, colour = _verdict(long)
    body.append(_text(TL_VERDICT_X, y0 + 16, verdict, colour, 13, True, "start"))
    y = y0 + 16
    for line in _wrap(POLICY_RULES[run.policy], TL_W - 32, 12):
        y += 17
        body.append(_text(16, y, line, GREY, 12, anchor="start"))
    bus_y, bus_h = y + 28, 28
    row_y, row_h, pitch = bus_y + bus_h + 9, 12, 18
    bottom = row_y + (n - 1) * pitch + row_h
    body.append(_plot_area(TL_X0 - 6, bus_y - 18, TL_PLOT + 12, bottom - bus_y + 20))
    for t, strong in ticks:
        body.append(_line(tx(t), bus_y - 2, tx(t), bottom + 2, GRID_STRONG if strong else GRID))

    body.append(_text(TL_X0 - 12, bus_y + 18, "bus owner", INK, 12, True, "end"))
    body.append(_rect(TL_X0, bus_y, TL_PLOT, bus_h, IDLE_FILL))
    prev_end = None
    for t in shown:
        c = run.controllers[t.owner]
        x0, x1 = tx(max(t.start, t0)), tx(min(t.end, t1, run.horizon))
        body.append(_rect(x0, bus_y, x1 - x0, bus_h, COLOURS[c.name], cls="data"))
        body += _owner_label(x0, x1, bus_y + 18.5, c)
        if run.saturated and prev_end is not None and tx(t.start) - tx(prev_end) >= 40:
            gx0, gx1 = tx(prev_end), tx(t.start)
            body.append(_polyline([(gx0, bus_y - 2), (gx0, bus_y - 6), (gx1, bus_y - 6), (gx1, bus_y - 2)],
                                  INK, 1, cls="data"))
            body.append(_text((gx0 + gx1) / 2, bus_y - 9, f"bus idle {_duration(t.start - prev_end)}", INK, 10))
        prev_end = t.end
    body.append(_text(TL_X1 + 12, bus_y + 11, "utilisation", INK, 11, anchor="start"))
    body.append(_text(TL_X1 + 12, bus_y + 25, _pct(long.utilisation()), INK, 11, True, "start"))

    labels = _hold_label(run)
    labelled = False
    for i, c in enumerate(run.controllers):
        y = row_y + i * pitch
        body.append(_text(TL_X0 - 12, y + 10, f"{c.name} → 0x{c.address:02x}", COLOURS[c.name], 12, True, "end"))
        body.append(_rect(TL_X0, y, TL_PLOT, row_h, LANE_FILL))
        for a, b, state in run.segments(i, t0, t1):
            fill = {TX: COLOURS[c.name], WAIT: TINTS[c.name], HOLD: "url(#hold)"}[state]
            body.append(_rect(tx(a), y, tx(b) - tx(a), row_h, fill, cls="data"))
            if state == HOLD and not labelled:
                for label in labels:
                    if _width(label, 10) + 16 <= tx(b) - tx(a):
                        body += _boxed_text((tx(a) + tx(b)) / 2, y + 10, label, 10)
                        labelled = True
                        break
        if run.arrivals is not None:
            for a in run.arrivals[i]:
                if t0 <= a < t1:
                    body.append(_line(tx(a), y - 5, tx(a), y, COLOURS[c.name], 1.5, cls="data"))
        for t in shown:
            if i in t.losers and t.start >= t0:
                body.append(_cross(tx(t.start), y + row_h / 2, cls="data"))
        if run.starved(i):
            body.append(_text(TL_X1 + 12, y + 10, "starved", ALARM, 12, True, "start"))
        else:
            count = sum(1 for t in shown if t.owner == i and t.start >= t0)
            body.append(_text(TL_X1 + 12, y + 10, f"{count} {'turn' if count == 1 else 'turns'}", INK, 12,
                              anchor="start"))
    return bottom


def timeline_svg(key: str) -> str:
    sc = SCENARIO[key]
    p = Params()
    longs = [scenario_run(key, policy) for policy in POLICIES]
    runs = [scenario_run(key, policy, sc.span) if sc.saturated else long for policy, long in zip(POLICIES, longs)]
    t0, t1 = timeline_window(key)
    scale = TL_PLOT / (t1 - t0)
    step = T_MAX if t1 - t0 >= 5 * T_MAX else 5 * MS
    ticks = [(t, t % T_MAX == 0) for t in range(t0, t1 + 1, step)]
    body: list[str] = []
    m = sc.length
    source_line = f"{sc.key}: {sc.describe()}"
    if not sc.saturated:
        source_line += "; seeded arrivals, FIFO queues, empty at t = 0"
    where = "" if t0 == 0 else f" (t = {t0 / 1e9:g} s to {t1 / 1e9:g} s)"
    shown = (f"{_duration(t1 - t0)} = {(t1 - t0) / T_MAX:g} T_max shown{where}; verdicts and utilisation from the "
             f"full {_duration(sc.horizon)} run")
    y = _header(body, TL_W, TIMELINE_TITLES[key], [source_line, shown, _params_line(p), _derivation_line()])

    first, last = CONTROLLERS[0], CONTROLLERS[-1]
    controller_items: list[tuple[Symbol, str]] = []
    for c in CONTROLLERS:
        rank = {first.name: " (lowest address, wins ties)", last.name: " (highest address)"}.get(c.name, "")
        controller_items.append((_sym_rect(COLOURS[c.name]), f"Controller {c.name} → target 0x{c.address:02x}{rank}"))
    y = _legend(body, y + 26, TL_W, controller_items)
    a = first.name
    state_items: list[tuple[Symbol, str]] = [
        (_sym_rect(COLOURS[a]), "transmitting (colour = sending controller, label = target address)"),
        (_sym_rect(TINTS[a]), "packet queued, competing"),
        (_sym_rect("url(#hold)", MUTED), "packet queued, holding off (re-arm r or policy deferral)"),
        (lambda x, yy: [_cross(x + 12, yy - 4)], "started together, lost arbitration"),
    ]
    if not sc.saturated:
        state_items.append((lambda x, yy: [_line(x + 12, yy - 10, x + 12, yy + 1, COLOURS[a], 1.5)],
                            "packet arrival"))
    y = _legend(body, y + 20, TL_W, state_items)
    y += 20
    body.append(_text(16, y, "Arbitration is decided by the target address: of the controllers that START "
                             "together, the one sending to the lowest address wins.", INK, 12, anchor="start"))

    y += 12
    for k, (run, long) in enumerate(zip(runs, longs)):
        if k:
            body.append(_line(16, y + 2, TL_W - 16, y + 2, GRID))
        y = _timeline_panel(body, y + 8, run, long, ticks, t0, t1) + 14

    axis_y = y + 2
    body.append(_line(TL_X0, axis_y, TL_X1, axis_y, INK))
    for t, strong in ticks:
        x = TL_X0 + (t - t0) * scale
        body.append(_line(x, axis_y, x, axis_y + (7 if strong else 4), INK))
        body.append(_text(x, axis_y + 19, f"{t / MS:g}", INK, 12, strong))
    grid_note = "gridlines every T_max" if step == T_MAX else "darker gridlines every T_max"
    body.append(_text(TL_X0 + TL_PLOT / 2, axis_y + 40, f"time [ms]; {grid_note} = {_param(T_MAX)}", INK, 12))

    px_us = (t1 - t0) / TL_PLOT / US
    base, fixed, idle, probe = longs
    starved = [c.name for i, c in enumerate(CONTROLLERS) if base.starved(i)]
    owners = [c.name for i, c in enumerate(CONTROLLERS) if base.turn_count(i)]
    names = [c.name for c in CONTROLLERS]
    notes = []
    if sc.saturated:
        notes.append(
            f"Every controller always has a packet of length {_m_label(m)} queued. Controllers waiting when the "
            "bus becomes free start together t_buf after STOP; the lowest target address wins arbitration."
        )
    else:
        notes.append(
            f"Each controller receives packets of length {_m_label(m)} as an independent Poisson stream with "
            f"{_load(sc.load)} offered load ({_load(len(CONTROLLERS) * sc.load)} in total) and sends them in FIFO order. "
            "Ticks above a lane mark arrivals; access delay = START of the winning transfer − packet arrival."
        )
    notes.append(
        f"At this scale 1 px = {px_us:.3g} µs: t_buf = {_param(p.t_buf)}, t_idle = {_param(p.t_idle)}, "
        f"r = {_param(p.rearm)}, margin = {_param(p.margin)} and W = {_param(p.backoff)} are far too small to "
        "draw, so adjacent bars that seem to touch are in fact separated by such gaps."
    )
    if starved:
        base_note = (
            f"Policy 1: waiting controllers start t_buf = {_param(p.t_buf)} after STOP, the previous winner only "
            f"after r = {_param(p.rearm)}, so it cannot take the next START; but r < m + 2·t_buf, so it is "
            f"re-armed before the STOP after that and beats {_names(starved)} again: {_names(owners)} "
            f"alternate, {_names(starved)} {_verb(starved, 'starves', 'starve')}. Only r < t_buf would let "
            f"{names[0]} keep the bus; {names[-1]} would get turns only for r > m + 2·t_buf = "
            f"{_duration(m + 2 * p.t_buf)}."
        )
    else:
        worst = max(range(len(names)), key=base.worst_delay)
        means = [base.mean_delay(i) for i in range(len(names))]
        ordered = all(a <= b for a, b in zip(means, means[1:])) and worst == len(names) - 1
        base_note = (
            "Policy 1: the bus is free often enough that every controller is served"
            f"{', but the higher the address, the longer the wait' if ordered else ''}: "
            f"worst access delay {names[worst]} {_ms(base.worst_delay(worst))} ms, "
            f"mean {', '.join(f'{c} {_ms(base.mean_delay(i))}' for i, c in enumerate(names))} ms "
            f"({_duration(base.horizon)} run)."
        )
    notes.append(base_note)
    cap = _capacity(m, p)
    if sc.saturated and m == T_MAX:
        notes.append(
            f"Policy 2 needs margin > 2·t_buf = {_param(2 * p.t_buf)} (bus-free gaps before and after the other "
            f"packet), otherwise {names[0]} is back in time to beat {names[-1]}; its deferral covers only one "
            "other maximum-length packet, so it is fair for N ≤ 3."
        )
    elif sc.saturated:
        notes.append(
            f"Policy 2 holds a controller off for T_max + margin = {_param(T_MAX + p.margin)} after its own STOP; "
            f"with {_param(m)} packets the bus then stays idle: utilisation {_pct(fixed.utilisation())} = "
            f"N·m / (m + T_max + margin)."
        )
    else:
        over = [i for i in range(len(names)) if fixed.overloaded(i)]
        if over:
            notes.append(
                f"Policy 2 lets a controller send at most one packet per m + T_max + margin = "
                f"{_param(m + T_MAX + p.margin)}, i.e. {_pct(cap)} of bus time < {_load(sc.load)} offered: the queues "
                f"grow without bound ({', '.join(str(fixed.backlog(i)) for i in over)} packets waiting after "
                f"{_duration(fixed.horizon)}) while the bus is idle {_pct(1 - fixed.utilisation())} of the time."
            )
        else:
            notes.append(
                f"Policy 2: capacity {_pct(cap)} per controller ≥ {_load(sc.load)} offered, the queues stay bounded."
            )
    worst_idle = max(idle.worst_delay(i) for i in range(len(names)))
    window_note = ""
    if sc.saturated:
        k = len(names) - 1
        formula = " = (N − 1)·m + (N − 1)·t_buf + t_idle" if worst_idle == k * (m + p.t_buf) + p.t_idle else ""
        notes.append(
            f"Policy 3: after every round the bus is idle for t_idle, which re-arms all controllers; worst access "
            f"delay {_ms(worst_idle)} ms{formula}; utilisation {_pct(idle.utilisation())}."
        )
    else:
        w = max(range(len(names)), key=idle.worst_delay)
        worst_turn = max((t for t in idle.turns if t.owner == w), key=lambda t: t.start - t.arrival)
        ahead = sum(1 for t in idle.turns if t.owner == w and t.arrival < worst_turn.arrival < t.start)
        heads = ", ".join(
            f"{c.name} {_ms(idle.worst_head_delay(i))} ms ≤ "
            f"{len(names) - 1 + sum(o.address < c.address for o in idle.controllers)} m"
            for i, c in enumerate(idle.controllers)
        )
        notes.append(
            f"Policy 3: worst access delay {_ms(worst_idle)} ms, mean "
            f"{', '.join(f'{c} {_ms(idle.mean_delay(i))}' for i, c in enumerate(names))} ms; utilisation "
            f"{_pct(idle.utilisation())} ≈ offered load ({_duration(idle.horizon)} run). The delay counts from "
            f"arrival: after a burst, {names[w]}'s worst packet had {ahead} of {names[w]}'s own packets waiting "
            "ahead of it. From the head of its own queue a controller waits at most (N − 1 + k)·m plus gaps, with "
            "k = number of lower target addresses: if its next packet arrives while it defers, every other "
            "controller may send once before the idle window and the lower addresses once more after it. "
            f"Here: {heads}. The round of (N − 1)·m holds only for saturated controllers."
        )
        first, share = first_owner_difference(base, idle)
        if first is not None and t0 > 0:
            window_note = (
                f"Window: policies 1 and 3 give every transfer to the same controller until t = {_ms(first)} ms "
                f"(they only shift STARTs by microseconds) and to different controllers in {_pct(share)} of all "
                f"transfers of the run. Shown is the stretch in which policy 1 delays {names[-1]} longest: as long "
                f"as {_names(names[:-1])} both have packets queued they alternate and {names[-1]} waits, while "
                f"policy 3 serves {_names(names)} in turn. Both serve the whole offered load, so their utilisation "
                "is the same; they only change who sends when."
            )
    notes.append(_probe_timeline_note(sc, base, idle, probe))
    if window_note:
        notes.append(window_note)
    y = _notes(body, axis_y + 68, TL_W, notes)
    defs = [_stripes("hold", MUTED, "#ffffff")]
    return _svg_doc(TL_W, y + 14, f"I2C multi-controller bus access policies: timeline {key}", defs, body)


# --------------------------------------------------------------------------- figure: throughput

TP_W, TP_PLOT_H, TP_PAD = 1220, 330, 8
TP_PANELS = ((80, 600, 0.0, "full range"), (730, 1190, 0.9, "zoom: 90 % to 100 %"))
ACTIVE_STYLES = {3: ("", "dot"), 2: ("7 4", "square"), 1: ("2 3", "ring")}


def _clip_low(points: list[tuple[float, float]], lo: float) -> list[list[tuple[float, float]]]:
    """Split a (m, u) polyline into the pieces with u >= lo."""
    pieces: list[list[tuple[float, float]]] = []
    current: list[tuple[float, float]] = []
    for k, (m, u) in enumerate(points):
        if k:
            pm, pu = points[k - 1]
            if (pu < lo) != (u < lo):
                cross = (pm + (m - pm) * (lo - pu) / (u - pu), lo)
                current.append(cross)
                if u < lo:
                    pieces.append(current)
                    current = []
        if u >= lo:
            current.append((m, u))
    if current:
        pieces.append(current)
    return [piece for piece in pieces if len(piece) > 1]


def throughput_svg() -> str:
    data = throughput_data()
    p = Params()
    n = len(CONTROLLERS)

    def u(policy: str, active: int, m: float) -> float:
        return data[policy, active][LENGTHS.index(m)]

    body: list[str] = []
    y = _header(body, TP_W, "Bus utilisation vs packet length", [
        f"1, 2 or 3 saturated controllers, every packet of length m; {THROUGHPUT_HORIZON // T_MAX} T_max = "
        f"{_duration(THROUGHPUT_HORIZON)} simulated per point",
        _params_line(p),
        _derivation_line(),
    ])
    items: list[tuple[Symbol, str]] = [(_sym_line(POLICY_COLOURS[q], 3), _policy_label(q)) for q in POLICIES]
    y = _legend(body, y + 26, TP_W, items)
    names = [c.name for c in CONTROLLERS]
    items = [
        (_sym_line(GREY, 2, ACTIVE_STYLES[k][0], ACTIVE_STYLES[k][1]),
         f"{k} {'controller' if k == 1 else 'controllers'} ({', '.join(names[:k])})")
        for k in range(n, 0, -1)
    ]
    items.append((lambda x, yy: [_line(x + 12, yy - 11, x + 12, yy + 1, GREY, 1.2, "4 3")],
                  "m = 0.1 T_max (10 % of max length)"))
    y = _legend(body, y + 20, TP_W, items)

    top = y + 40
    bot = top + TP_PLOT_H
    for x0, x1, lo, title in TP_PANELS:
        def px(m: float, x0: float = x0, x1: float = x1) -> float:
            return x0 + TP_PAD + m * (x1 - x0 - 2 * TP_PAD)

        def py(v: float, lo: float = lo) -> float:
            return bot - TP_PAD - (v - lo) / (1 - lo) * (TP_PLOT_H - 2 * TP_PAD)

        body.append(_text(x0, top - 10, title, INK, 12, True, "start"))
        steps = 10
        for k in range(steps + 1):
            v = lo + (1 - lo) * k / steps
            body.append(_line(x0, py(v), x1, py(v), GRID))
            body.append(_text(x0 - 8, py(v) + 4, f"{v * 100:.0f} %", INK, 11, anchor="end"))
            body.append(_line(px(k / 10), top, px(k / 10), bot, GRID))
            body.append(_text(px(k / 10), bot + 18, f"{k / 10:g}", INK, 11))
        body.append(_rect(x0, top, x1 - x0, bot - top, "none", INK))
        body.append(_plot_area(x0, top, x1 - x0, bot - top))
        body.append(_text((x0 + x1) / 2, bot + 40, f"packet length m / T_max (T_max = {_param(T_MAX)})", INK, 12))
        body.append(_text(x0 - 52, (top + bot) / 2, "bus utilisation", INK, 13, rotate=True))
        body.append(_line(px(SHORT), top + 1, px(SHORT), bot - 1, GREY, 1.2, "4 3", cls="data"))
        if lo == 0:
            body += _boxed_text(px(SHORT) + 4 + _width("10 % of max length", 11) / 2 + 3, py(0.85),
                                "10 % of max length", 11, GREY)
        for policy in (FIXED, BASELINE, IDLE, PROBE):
            for active in range(n, 0, -1):
                dash, _ = ACTIVE_STYLES[active]
                for piece in _clip_low(list(zip(LENGTHS, data[policy, active])), lo):
                    body.append(_polyline([(px(m), py(v)) for m, v in piece], POLICY_COLOURS[policy], 2, dash,
                                          cls="data"))
        for policy in (FIXED, BASELINE, IDLE, PROBE):
            for active in range(n, 0, -1):
                shape = ACTIVE_STYLES[active][1]
                for m, v in zip(LENGTHS, data[policy, active]):
                    if v >= lo:
                        body.append(_marker(px(m), py(v), POLICY_COLOURS[policy], shape))

    base = simulate(BASELINE, CONTROLLERS, T_MAX, THROUGHPUT_HORIZON)
    starved = [c.name for i, c in enumerate(CONTROLLERS) if base.starved(i)]
    owners = [c.name for i, c in enumerate(CONTROLLERS) if base.turn_count(i)]
    short = LENGTHS[0]
    idle_low = {k: min(data[IDLE, k]) for k in (2, 3)}

    def same(a: tuple[str, int], b: tuple[str, int]) -> bool:
        return all(abs(x - y) < 1e-9 for x, y in zip(data[a], data[b]))

    coincide = " (the 2- and 3-controller curves coincide)" if same((BASELINE, 2), (BASELINE, n)) else ""
    if same((IDLE, 1), (BASELINE, 1)) and p.rearm > p.t_idle:
        single_idle = " One controller waits only for r (> t_idle), so its curve coincides with policy 1."
    else:
        single_idle = f" One controller: {_pct(u(IDLE, 1, short))} at m = {short:g} T_max."
    notes = [
        (
            f"Policy 1: {_pct(min(data[BASELINE, n]))} to {_pct(max(data[BASELINE, n]))} with {n} controllers, "
            f"but {_names(owners)} alternate and {_names(starved)} never {_verb(starved, 'gets', 'get')} the bus "
            f"{coincide}. One controller is limited by its re-arm time: m / (m + r) "
            f"= {_pct(u(BASELINE, 1, short))} at m = {short:g} T_max."
        ),
        (
            f"Policy 2: one controller {_pct(u(FIXED, 1, 1.0))} at m = T_max and {_pct(u(FIXED, 1, SHORT))} at "
            f"m = {SHORT:g} T_max (= m / (m + T_max + margin)); {n} controllers {_pct(u(FIXED, n, 1.0))} at "
            f"m = T_max but only {_pct(u(FIXED, n, SHORT))} at m = {SHORT:g} T_max: one round of N packets per "
            "T_max + margin, so it reaches 100 % only for m ≥ about (T_max + margin) / (N − 1)."
        ),
        (
            f"Policy 3: the idle window costs t_idle + (N − 1)·t_buf per round: ≥ {_pct(idle_low[3])} with {n} and "
            f"≥ {_pct(idle_low[2])} with 2 controllers for every m ≥ {short:g} T_max, {_pct(u(IDLE, n, SHORT))} at "
            f"m = {SHORT:g} T_max.{single_idle}"
        ),
        (
            f"Policy 4: after every STOP the bus stays idle until the first waiting controller probes: "
            f"≥ {_pct(min(data[PROBE, n]))} with {n} and ≥ {_pct(min(data[PROBE, 2]))} with 2 controllers for "
            f"every m ≥ {short:g} T_max, {_pct(u(PROBE, n, SHORT))} at m = {SHORT:g} T_max. One controller waits r "
            f"plus a backoff after each STOP: m / (m + r + W/2) = {_pct(u(PROBE, 1, short))} at m = {short:g} T_max."
        ),
        "Utilisation = fraction of time a packet is on the bus.",
    ]
    y = _notes(body, bot + 72, TP_W, notes)
    return _svg_doc(TP_W, y + 14, "I2C multi-controller bus access policies: utilisation", [], body)


# --------------------------------------------------------------------------- figure: access delay

AD_W, AD_PLOT_H, AD_BAR, AD_GAP = 1220, 210, 28, 5
AD_COLUMNS = ((96, 566), (706, 1186))


def _nice_top(v: float) -> tuple[float, float]:
    """Axis maximum and tick step (in ms) with headroom above v."""
    target = max(v, 1e-9) * 1.15
    raw = target / 5
    exp = 10 ** math.floor(math.log10(raw))
    step = next(s * exp for s in (1, 2, 2.5, 5, 10) if s * exp >= raw)
    return step * math.ceil(target / step), step


def _delay_panel(body: list[str], sc: Scenario, x0: float, x1: float, top: float) -> None:
    runs = [scenario_run(sc.key, policy) for policy in POLICIES]
    n = len(CONTROLLERS)
    bot = top + AD_PLOT_H
    body.append(_text(x0 - 80, top - 40, f"{sc.key} · {sc.describe()}", INK, 13, True, "start"))
    if sc.saturated:
        sub = f"{sc.horizon // T_MAX} T_max = {_duration(sc.horizon)} simulated"
    else:
        packets = sum(len(s) for s in poisson_arrivals(sc.key)) // n
        sub = f"{sc.horizon // T_MAX} T_max = {_duration(sc.horizon)} simulated, ≈ {packets} packets per controller"
    body.append(_text(x0 - 80, top - 24, sub, GREY, 11, anchor="start"))
    finite = [r.worst_delay(i) for r in runs for i in range(n) if not r.unbounded(i)]
    ymax, step = _nice_top(max(finite) / MS if finite else 1.0)

    def py(v_ms: float) -> float:
        return bot - min(v_ms, ymax) / ymax * AD_PLOT_H

    k = 0
    while k * step <= ymax + 1e-9:
        v = k * step
        body.append(_line(x0, py(v), x1, py(v), GRID))
        body.append(_text(x0 - 6, py(v) + 4, f"{v:g}", INK, 11, anchor="end"))
        k += 1
    body.append(_line(x0, bot, x1, bot, INK))
    body.append(_line(x0, top, x0, bot, INK))
    body.append(_plot_area(x0, top, x1 - x0, bot - top))
    body.append(_text(x0 - 46, (top + bot) / 2, "access delay [ms]", INK, 12, rotate=True))

    group_w = (x1 - x0) / len(runs)
    span = n * AD_BAR + (n - 1) * AD_GAP
    for g, run in enumerate(runs):
        cx = x0 + (g + 0.5) * group_w
        body.append(_text(cx, bot + 18, f"Policy {g + 1}", INK, 12, True))
        body.append(_text(cx, bot + 33, f"util. {_pct(run.utilisation())}", GREY, 11))
        for i, c in enumerate(run.controllers):
            bx = cx - span / 2 + i * (AD_BAR + AD_GAP)
            mid = bx + AD_BAR / 2
            if run.unbounded(i):
                body.append(_rect(bx, top + 0.5, AD_BAR, AD_PLOT_H - 1, f"url(#unbounded-{c.name})",
                                  COLOURS[c.name], cls="data"))
                body.append(_text(mid, top - 4, "∞", ALARM, 15, True))
                label = "starved" if run.starved(i) else "overloaded"
                body += _boxed_text(mid + 4, (top + bot) / 2, label, 11, ALARM, True, rotate=True)
                continue
            worst, mean = run.worst_delay(i) / MS, run.mean_delay(i) / MS
            body.append(_rect(bx, py(worst), AD_BAR, bot - py(worst), COLOURS[c.name], cls="data"))
            body.append(_line(bx - 3, py(mean), bx + AD_BAR + 3, py(mean), INK, 2.5, cls="data"))
            body.append(_text(mid, py(worst) - 5, _ms(worst * MS), INK, 10))


def access_delay_svg() -> str:
    p = Params()
    n = len(CONTROLLERS)
    names = [c.name for c in CONTROLLERS]
    body: list[str] = []
    y = _header(body, AD_W, "Access delay per controller: worst case and mean", [
        "S1, S2: 3 saturated controllers; S3, S4: 3 Poisson sources with 25 % offered load each (75 % in total)",
        _params_line(p),
        _derivation_line(),
    ])
    items: list[tuple[Symbol, str]] = [
        (_sym_rect(COLOURS[c.name]), f"Controller {c.name} → target 0x{c.address:02x}") for c in CONTROLLERS
    ]
    items += [
        (lambda x, yy: [_rect(x + 4, yy - 12, 16, 14, TINTS[names[0]]),
                        _line(x + 1, yy - 6, x + 23, yy - 6, INK, 2.5)],
         "bar = worst delay, dark tick = mean; util. = bus utilisation"),
        (_sym_rect(f"url(#unbounded-{names[-1]})", COLOURS[names[-1]]),
         "∞: starved (never gets the bus) or overloaded (queue grows without bound)"),
    ]
    y = _legend(body, y + 26, AD_W, items)
    y += 20
    body.append(_text(16, y, " · ".join(f"Policy {k + 1} = {POLICY_TITLES[q]}" for k, q in enumerate(POLICIES)),
                      INK, 12, anchor="start"))
    y += 12
    for row in range(2):
        top = y + 62
        for col, (x0, x1) in enumerate(AD_COLUMNS):
            _delay_panel(body, SCENARIOS[2 * row + col], x0, x1, top)
        y = top + AD_PLOT_H + 40

    runs = {(sc.key, q): scenario_run(sc.key, q) for sc in SCENARIOS for q in POLICIES}

    def worst(key: str, policy: str) -> float:
        return max(runs[key, policy].worst_delay(i) for i in range(n))

    def means(key: str, policy: str) -> str:
        return ", ".join(_ms(runs[key, policy].mean_delay(i)) for i in range(n))

    def exact(value: float, formula: str, expected: int) -> str:
        return f" = {formula}" if value == expected else ""

    base_starved = [sc.key for sc in SCENARIOS if runs[sc.key, BASELINE].starved(n - 1)]
    base_owners = [names[i] for i in range(n) if runs["S1", BASELINE].turn_count(i)]
    poisson = [sc.key for sc in SCENARIOS if not sc.saturated]
    longest = {k: max(range(n), key=runs[k, BASELINE].worst_delay) for k in poisson}
    if all(i == n - 1 for i in longest.values()):
        base_poisson = f"under Poisson load the highest address {names[-1]} waits longest: worst "
    else:
        base_poisson = "under Poisson load the longest worst-case waits are "
    base_poisson += ", ".join(f"{names[i]} {_ms(runs[k, BASELINE].worst_delay(i))} ms in {k}"
                              for k, i in longest.items())
    s3_fixed, s4_fixed, s4_idle = runs["S3", FIXED], runs["S4", FIXED], runs["S4", IDLE]
    over = [i for i in range(n) if s3_fixed.overloaded(i)]
    if all(s4_fixed.mean_delay(i) > s4_idle.mean_delay(i) for i in range(n)):
        s4_note = "but the deferral raises the mean delay to"
    else:
        s4_note = "mean delay"
    poisson_idle = [k for k in poisson if worst(k, IDLE) > 2 * T_MAX + p.t_idle + n * p.t_buf]
    notes = [
        (
            "Access delay: S1, S2 from queueing a packet (end of the controller's previous turn, or t = 0) to the "
            "START of the transfer that wins the bus; S3, S4 from the packet's arrival to that START, including "
            "the wait behind the controller's own earlier packets. Packets still waiting at the end of the run "
            "count with their wait so far."
        ),
        (
            f"Policy 1: {names[-1]} never gets the bus in {_names(base_starved)} ({_names(base_owners)} "
            f"alternate); {base_poisson}."
        ),
        (
            f"Policy 2: worst {_ms(worst('S1', FIXED))} ms = {_tmax(worst('S1', FIXED))} in S1"
            f"{exact(worst('S1', FIXED), '2 T_max + 3·t_buf', 2 * T_MAX + 3 * p.t_buf)}. In S3 a controller can "
            f"use at most m / (m + T_max + margin) = {_pct(_capacity(SCENARIO['S3'].length, p))} of bus time "
            f"< {_load(SCENARIO['S3'].load or 0)} offered: {len(over)} of {n} queues grow without bound ("
            + ", ".join(str(s3_fixed.backlog(i)) for i in over)
            + f" packets waiting after {_duration(s3_fixed.horizon)}). "
            f"S4 stays stable (capacity {_pct(_capacity(SCENARIO['S4'].length, p))}), {s4_note} "
            f"{means('S4', FIXED)} ms (A, B, C; policy 3: {means('S4', IDLE)} ms)."
        ),
        (
            f"Policy 3: worst {_ms(worst('S1', IDLE))} ms = {_tmax(worst('S1', IDLE))} in S1"
            f"{exact(worst('S1', IDLE), '2 T_max + 2·t_buf + t_idle', 2 * T_MAX + 2 * p.t_buf + p.t_idle)}, "
            f"{_ms(worst('S2', IDLE))} ms in S2, {_ms(worst('S3', IDLE))} ms in S3 and {_ms(worst('S4', IDLE))} ms "
            "in S4."
            + (f" Under Poisson load ({_names(poisson_idle)}) 2 T_max + t_idle does not bound these delays: a packet "
               "may wait behind its controller's own earlier packets, and from the head of its own queue a "
               "controller with k lower target addresses waits up to (N − 1 + k) transfers, because if its next "
               "packet arrives while it defers, the lower addresses may send before and after the idle window "
               f"({names[-1]}: {_ms(runs['S4', IDLE].worst_head_delay(n - 1))} ms = "
               f"{_tmax(runs['S4', IDLE].worst_head_delay(n - 1))} in S4)." if poisson_idle else "")
        ),
        (
            f"Policy 4: nobody starves and the address no longer decides who waits longest, but every STOP is a "
            f"lottery among the waiting controllers: worst {_ms(worst('S1', PROBE))} ms = "
            f"{_tmax(worst('S1', PROBE))} in S1 and {_ms(worst('S2', PROBE))} ms in S2 (policy 3: "
            f"{_ms(worst('S1', IDLE))} and {_ms(worst('S2', IDLE))} ms), {_ms(worst('S3', PROBE))} ms in S3 and "
            f"{_ms(worst('S4', PROBE))} ms in S4. The bus stays idle after every STOP until the first probe, so the "
            "mean over all packets is higher than with policies 1 and 3: "
            + "; ".join(f"{_ms(mean_delay_all(runs[k, PROBE]))} ms in {k} (policies 1 and 3: "
                        f"{_ms(mean_delay_all(runs[k, BASELINE]))} and {_ms(mean_delay_all(runs[k, IDLE]))} ms)"
                        for k in poisson)
            + "."
        ),
    ]
    y = _notes(body, y + 22, AD_W, notes)
    defs = [_stripes(f"unbounded-{c.name}", COLOURS[c.name], TINTS[c.name]) for c in CONTROLLERS]
    return _svg_doc(AD_W, y + 14, "I2C multi-controller bus access policies: access delay", defs, body)


FIGURES: tuple[tuple[str, Callable[[], str]], ...] = (
    ("policy_timeline.svg", functools.partial(timeline_svg, "S1")),
    ("policy_timeline_short.svg", functools.partial(timeline_svg, "S2")),
    ("policy_timeline_load.svg", functools.partial(timeline_svg, "S3")),
    ("policy_throughput.svg", throughput_svg),
    ("policy_access_delay.svg", access_delay_svg),
)


# --------------------------------------------------------------------------- SVG self-checks

SVG_NS = "{http://www.w3.org/2000/svg}"
Box = tuple[float, float, float, float]  # x0, y0, x1, y1


def _num(el: ET.Element, name: str, default: float = 0.0) -> float:
    value = el.get(name)
    return default if value is None else float(value)


def _text_box(el: ET.Element) -> Box:
    size = _num(el, "font-size")
    bold = el.get("font-weight") == "bold"
    mono = "monospace" in el.get("font-family", "")
    runs = [(el.text or "", size, 0.0)]
    offset = 0.0
    for child in el:
        offset += _num(child, "dy")
        runs.append((child.text or "", _num(child, "font-size", size), offset))
        if child.tail:
            runs.append((child.tail, size, offset))
    width = sum(_run_width(text, s, bold, mono) for text, s, _ in runs if text)
    up = max(0.8 * s - off for text, s, off in runs if text)
    down = max(0.25 * s + off for text, s, off in runs if text)
    x, y = _num(el, "x"), _num(el, "y")
    lead = {"start": 0.0, "middle": 0.5, "end": 1.0}[el.get("text-anchor", "start")] * width
    if el.get("transform", "").startswith("rotate(-90"):
        return x - up, y - width + lead, x + down, y + lead
    return x - lead, y - up, x - lead + width, y + down


def _shape_box(el: ET.Element, tag: str) -> Box | None:
    half = _num(el, "stroke-width", 1.0) / 2 if el.get("stroke") else 0.0
    if tag == "rect":
        x, y = _num(el, "x"), _num(el, "y")
        return x - half, y - half, x + _num(el, "width") + half, y + _num(el, "height") + half
    if tag == "line":
        xs, ys = (_num(el, "x1"), _num(el, "x2")), (_num(el, "y1"), _num(el, "y2"))
    elif tag == "circle":
        r = _num(el, "r")
        xs, ys = (_num(el, "cx") - r, _num(el, "cx") + r), (_num(el, "cy") - r, _num(el, "cy") + r)
    elif tag in ("polyline", "path"):
        values = [float(v) for v in re.findall(r"-?\d+(?:\.\d+)?", el.get("points") or el.get("d") or "")]
        xs, ys = tuple(values[0::2]), tuple(values[1::2])
    else:
        return None
    return min(xs) - half, min(ys) - half, max(xs) + half, max(ys) + half


def _inside(inner: Box, outer: Box, tol: float = 0.01) -> bool:
    return (inner[0] >= outer[0] - tol and inner[1] >= outer[1] - tol
            and inner[2] <= outer[2] + tol and inner[3] <= outer[3] + tol)


def svg_problems(svg: str) -> list[str]:
    """Layout problems: text or shapes too close to the edges, overlapping texts, data outside its plot area."""
    root = ET.fromstring(svg)
    width, height = _num(root, "width"), _num(root, "height")
    defs = root.find(f"{SVG_NS}defs")
    skip = set(defs.iter()) if defs is not None else set()
    page = (EDGE, 0.0, width - EDGE, height)
    problems: list[str] = []
    texts: list[tuple[Box, str]] = []
    areas: list[Box] = []
    data: list[tuple[Box, str]] = []
    for el in root.iter():
        tag = el.tag.removeprefix(SVG_NS)
        if el is root or el in skip or tag in ("title", "defs", "tspan"):
            continue
        if any(name in el.attrib for name in ("paint-order", "baseline-shift", "dominant-baseline")):
            problems.append(f"<{tag}> uses an attribute that not every renderer supports")
        if tag == "text":
            content = "".join(el.itertext())
            box = _text_box(el)
            texts.append((box, content))
            if len(el) and el.get("text-anchor", "start") != "start":
                problems.append(f"rich text not start-anchored: {content!r}")
            if "--check" in content:
                problems.append(f"text refers to --check: {content!r}")
            if not _inside(box, page):
                problems.append(f"text outside [{EDGE}, {width - EDGE:g}] x [0, {height:g}]: {content!r} "
                                f"({box[0]:.1f}, {box[1]:.1f}, {box[2]:.1f}, {box[3]:.1f})")
            continue
        box = _shape_box(el, tag)
        if box is None:
            continue
        if tag == "rect" and "x" not in el.attrib and _num(el, "width") == width:
            continue  # background
        classes = el.get("class", "").split()
        if "plot-area" in classes:
            areas.append(box)
        elif "data" in classes:
            data.append((box, f"<{tag}> at ({box[0]:.1f}, {box[1]:.1f})"))
        if not _inside(box, page):
            problems.append(f"<{tag}> outside [{EDGE}, {width - EDGE:g}]: ({box[0]:.1f}, {box[2]:.1f})")
    for box, what in data:
        if not any(_inside(box, area) for area in areas):
            problems.append(f"data mark outside every plot area: {what}")
    for k, (a, ta) in enumerate(texts):
        for b, tb in texts[k + 1 :]:
            if a[0] < b[2] - 0.5 and b[0] < a[2] - 0.5 and a[1] < b[3] - 0.5 and b[1] < a[3] - 0.5:
                problems.append(f"texts overlap: {ta!r} and {tb!r}")
    return problems


# --------------------------------------------------------------------------- checks and summary


def run_checks() -> int:
    p = Params()
    n = len(CONTROLLERS)
    four = CONTROLLERS + (EXTRA,)
    results: list[tuple[bool, str]] = []

    def check(ok: bool, text: str) -> None:
        results.append((bool(ok), text))

    def sim(policy: str, controllers: tuple[Controller, ...] = CONTROLLERS, m: int = T_MAX, **kw: int) -> Run:
        return simulate(policy, controllers, m, CHECK_HORIZON, replace(p, **kw))

    def worst(run: Run) -> list[float]:
        return [run.worst_delay(i) for i in range(len(run.controllers))]

    def ms_list(values: list[float]) -> str:
        return ", ".join("∞" if math.isinf(v) else _ms(v) for v in values) + " ms"

    check(
        WIRE_TIME + CLOCK_STRETCH <= T_MAX,
        f"T_max: {_param(WIRE_TIME)} + {_param(CLOCK_STRETCH)} = {_param(WIRE_TIME + CLOCK_STRETCH)} ≤ {_param(T_MAX)}",
    )
    check(p.rearm > p.t_buf and p.margin > 2 * p.t_buf, "defaults: r > t_buf and margin > 2·t_buf")

    for key in ("S1", "S2"):
        run = scenario_run(key, BASELINE)
        check(
            run.starved(2) and abs(run.share(0) - 0.5) < 0.01 and abs(run.turn_count(0) - run.turn_count(1)) <= 1,
            f"policy 1, {key}, r = {_param(p.rearm)}: C gets {run.turn_count(2)} turns, A/B "
            f"{run.turn_count(0)}/{run.turn_count(1)} turns, share {_pct(run.share(0))}/{_pct(run.share(1))}",
        )
    run = sim(BASELINE, rearm=p.t_buf // 2)
    check(
        run.share(0) == 1.0 and run.starved(1) and run.starved(2),
        f"policy 1, r = t_buf/2 < t_buf: A owns {_pct(run.share(0))} of the busy time, B and C starve",
    )
    for m in (T_MAX, T_MAX // 10):
        run = sim(BASELINE, m=m, rearm=2 * (m + p.t_buf))
        turns = [run.turn_count(i) for i in range(n)]
        check(min(turns) > 0, f"policy 1, m = {_param(m)}, r = 2·(m + t_buf): all get turns {turns}")
        edge = m + 2 * p.t_buf
        below, above = sim(BASELINE, m=m, rearm=edge), sim(BASELINE, m=m, rearm=edge + 1)
        check(
            below.starved(2) and above.turn_count(2) > 0,
            f"policy 1, m = {_param(m)}: C starves for r = m + 2·t_buf, gets {above.turn_count(2)} turns for "
            "r = m + 2·t_buf + 1 ns",
        )

    for m in (T_MAX, T_MAX // 10):
        u = sim(FIXED, CONTROLLERS[:1], m).utilisation()
        expected = m / (m + T_MAX + p.margin)
        check(abs(u - expected) < 0.002,
              f"policy 2, 1 controller, m = {_param(m)}: utilisation {_pct(u)} ≈ m/(m + T_max + margin) = "
              f"{_pct(expected)}")
    run = scenario_run("S1", FIXED)
    check(max(worst(run)) <= 2 * T_MAX + n * p.t_buf and run.utilisation() > 0.999,
          f"policy 2, S1: worst access delay {ms_list(worst(run))} ≤ 2 T_max + N·t_buf; "
          f"utilisation {_pct(run.utilisation())}")
    run = scenario_run("S2", FIXED)
    expected = n * _capacity(T_MAX // 10, p)
    check(abs(run.utilisation() - expected) < 0.005,
          f"policy 2, S2: utilisation {_pct(run.utilisation())} ≈ N·m/(m + T_max + margin) = {_pct(expected)}")
    run = sim(FIXED, four)
    check(run.starved(3), f"policy 2, 4 saturated, m = T_max: D gets {run.turn_count(3)} turns "
                          "(the deferral covers one other packet)")
    run = sim(FIXED, margin=p.t_buf * 3 // 2)
    check(run.starved(2), f"policy 2, margin = 1.5·t_buf < 2·t_buf: C gets {run.turn_count(2)} turns")

    run = scenario_run("S1", IDLE)
    bound = (n - 1) * T_MAX + n * p.t_buf + p.t_idle + p.rearm
    check(max(worst(run)) <= bound,
          f"policy 3, S1: worst access delay {ms_list(worst(run))} ≤ (N − 1)·T_max + N·t_buf + t_idle + r "
          f"= {_ms(bound)} ms")
    check(max(worst(run)) <= 2 * T_MAX + n * p.t_buf + p.t_idle,
          "policy 3, S1: also within 2 T_max + t_idle + N·t_buf (no r: the last sender is never the next one)")
    for key, floor in (("S1", 0.99), ("S2", 0.98)):
        u = scenario_run(key, IDLE).utilisation()
        check(u >= floor, f"policy 3, {key}: utilisation {_pct(u)} ≥ {floor:.0%}")
    run = sim(IDLE, four)
    bound = 3 * T_MAX + 4 * p.t_buf + p.t_idle + p.rearm
    check(max(worst(run)) <= bound, f"policy 3, 4 saturated: worst access delay {_ms(max(worst(run)))} ms "
                                    f"≤ 3 T_max + 4·t_buf + t_idle + r")

    for m in (T_MAX, T_MAX // 10, T_MAX // 100):
        u = sim(PROBE, CONTROLLERS[:1], m).utilisation()
        expected = m / (m + p.rearm + (p.backoff + 1) / 2)
        check(abs(u - expected) < 0.002,
              f"policy 4, 1 controller, m = {_param(m)}: utilisation {_pct(u)} ≈ m/(m + r + W/2) = {_pct(expected)}")
    for controllers in (CONTROLLERS, four):
        k = len(controllers) - 1
        for m in (T_MAX, T_MAX // 10):
            run = sim(PROBE, controllers, m)
            gap = run.horizon * (1 - run.utilisation()) / len(run.turns)
            shares = [run.turn_count(i) / len(run.turns) for i in range(len(controllers))]
            # k controllers wait; the time from the STOP to the next probe of each exceeds x with (1 − x/W)².
            check(
                abs(gap / (p.backoff / (2 * k + 1)) - 1) < 0.05 and all(abs(s * (k + 1) - 1) < 0.1 for s in shares),
                f"policy 4, {k + 1} saturated, m = {_param(m)}: idle per transfer {_duration(gap)} ≈ W/(2(N − 1) + 1) "
                f"= {_duration(p.backoff / (2 * k + 1))}; turn shares {', '.join(_pct(s) for s in shares)} ≈ 1/N",
            )
    for key in ("S1", "S2"):
        sc = SCENARIO[key]
        long, short = scenario_run(key, PROBE), scenario_run(key, PROBE, sc.span)
        check(short.turns == tuple(t for t in long.turns if t.start < sc.span),
              f"policy 4, {key}: the timeline run ({_duration(sc.span)}) is the start of the full run")
    for key in ("S3", "S4"):
        base, probe = scenario_run(key, BASELINE), scenario_run(key, PROBE)
        means = (mean_delay_all(base), mean_delay_all(probe))
        check(
            means[1] > means[0] and probe.worst_delay(n - 1) < base.worst_delay(n - 1),
            f"{key}, policy 4: not work-conserving, mean access delay over all packets {_ms(means[1])} ms > "
            f"{_ms(means[0])} ms (policy 1); {CONTROLLERS[-1].name}'s worst access delay "
            f"{_ms(probe.worst_delay(n - 1))} ms < {_ms(base.worst_delay(n - 1))} ms (policy 1)",
        )

    for key in ("S3", "S4"):
        sc = SCENARIO[key]
        for policy in POLICIES:
            run = scenario_run(key, policy)
            offered = [run.offered(i) or 0.0 for i in range(n)]
            served = [run.served(i) for i in range(n)]
            cap = _capacity(sc.length, p) if policy == FIXED else 1.0
            if sc.load is not None and sc.load > cap:
                ok = all(run.overloaded(i) and abs(s - cap) < 0.002 for i, s in enumerate(served))
                text = (f"{key}, policy {POLICIES.index(policy) + 1}: overloaded as expected, served "
                        f"{', '.join(_pct(s) for s in served)} = capacity {_pct(cap)} < offered "
                        f"{', '.join(_pct(o) for o in offered)}; backlog "
                        f"{', '.join(str(run.backlog(i)) for i in range(n))}")
            else:
                ok = all(abs(s - o) < 0.005 and not run.overloaded(i)
                         for i, (s, o) in enumerate(zip(served, offered)))
                text = (f"{key}, policy {POLICIES.index(policy) + 1}: stable, served "
                        f"{', '.join(_pct(s) for s in served)} ≈ offered {', '.join(_pct(o) for o in offered)}")
            check(ok, text)

    t0, t1 = timeline_window("S3")
    base, idle = scenario_run("S3", BASELINE), scenario_run("S3", IDLE)
    in_window = [(a.owner, b.owner) for a, b in zip(base.turns, idle.turns) if t0 <= a.start < t1]
    check(
        t1 - t0 == SCENARIO["S3"].span and any(x != y for x, y in in_window),
        f"S3 timeline window {t0 / MS:g} to {t1 / MS:g} ms: policies 1 and 3 give "
        f"{sum(x != y for x, y in in_window)} of {len(in_window)} transfers to different controllers",
    )

    for key in ("S3", "S4"):
        base, idle = scenario_run(key, BASELINE), scenario_run(key, IDLE)
        means = (mean_delay_all(base), mean_delay_all(idle))
        last = n - 1
        check(
            abs(means[1] / means[0] - 1) < 0.01 and abs(idle.mean_delay(last) / base.mean_delay(last) - 1) > 0.1,
            f"{key}: conservation law: mean access delay over all packets {_ms(means[0])} ms (policy 1) ≈ "
            f"{_ms(means[1])} ms (policy 3), while {CONTROLLERS[last].name}'s mean changes from "
            f"{_ms(base.mean_delay(last))} to {_ms(idle.mean_delay(last))} ms",
        )
        ratios = md1_ratios(key)
        average = sum(ratios) / len(ratios)
        check(
            abs(average - 1) < 0.05,
            f"{key}: policy 1 mean access delay / M/D/1 mean wait over {REPLICATIONS} seeds: mean {average:.3f} "
            f"(range {min(ratios):.3f} to {max(ratios):.3f}; this run {means[0] / md1_wait(base):.3f})",
        )

    for sc in SCENARIOS:
        run = scenario_run(sc.key, IDLE)
        heads = [run.worst_head_delay(i) for i in range(n)]
        bounds = [idle_head_bound(run, i) for i in range(n)]
        reached = sc.saturated or heads[-1] >= (2 * n - 2) * sc.length
        check(
            all(h <= b for h, b in zip(heads, bounds)) and reached,
            f"{sc.key}, policy 3: wait from the head of the own queue {ms_list(heads)} ≤ "
            f"(N − 1 + k)·(m + t_buf) + t_idle + r = {ms_list(bounds)}"
            + ("" if sc.saturated else f"; {CONTROLLERS[-1].name} reaches (2N − 2)·m"),
        )

    first = {name: build() for name, build in FIGURES}
    _clear_caches()
    for name, build in FIGURES:
        svg = build()
        try:
            ET.fromstring(svg)
            problems = svg_problems(svg)
        except ET.ParseError as error:
            problems = [f"not well-formed: {error}"]
        check(svg == first[name], f"{name}: deterministic (generated twice, byte-identical)")
        check(not problems, f"{name}: well-formed, text and marks within bounds, no text overlaps"
              + "".join(f"\n      {problem}" for problem in problems[:12]))

    for ok, text in results:
        print(f"{'PASS' if ok else 'FAIL'}  {text}")
    failed = sum(not ok for ok, _ in results)
    print(f"{len(results) - failed}/{len(results)} checks passed")
    return 1 if failed else 0


def print_summary() -> None:
    n = len(CONTROLLERS)
    for sc in SCENARIOS:
        print(f"{sc.key}: {sc.describe()}; {_duration(sc.horizon)} simulated")
        if not sc.saturated:
            assert sc.load is not None
            short, below, peak = arrival_stats(sc.key)
            print(f"  arrivals: {sc.load * SECOND / sc.length:g} packets/s per controller, mean gap "
                  f"{_ms(sc.length / sc.load)} ms; gaps < m {_pct(short)} (Poisson {_pct(1 - math.exp(-sc.load))}), "
                  f"gaps < mean gap {_pct(below)} (Poisson {_pct(1 - math.exp(-1))}); highest offered load in "
                  f"{_duration(BURST_WINDOW)}: {_pct(peak)}")
        for policy in POLICIES:
            run = scenario_run(sc.key, policy)
            mean_all = ""
            if not run.saturated and not any(run.unbounded(i) for i in range(n)):
                mean_all = f"; mean access delay over all packets {_ms(mean_delay_all(run))} ms"
            if policy == PROBE:
                mean_all += f"; {_count(_probe_rate(run))} probes/s per controller"
            print(f"  {_policy_label(policy)}: utilisation {_pct(run.utilisation())}{mean_all}")
            for i, c in enumerate(run.controllers):
                load = "" if run.saturated else f", served {_pct(run.served(i))} of {_pct(run.offered(i) or 0)}"
                if run.starved(i):
                    state = "starved"
                elif run.overloaded(i):
                    state = f"overloaded, {run.backlog(i)} packets waiting at the end"
                else:
                    state = (f"access delay worst {_ms(run.worst_delay(i))} ms, "
                             f"mean {_ms(run.mean_delay(i))} ms")
                    if not run.saturated:
                        state += f", from the head of the own queue worst {_ms(run.worst_head_delay(i))} ms"
                print(f"    {c.name} → 0x{c.address:02x}: {run.turn_count(i)} turns{load}; {state}")
        if not sc.saturated:
            base = scenario_run(sc.key, BASELINE)
            ratios = md1_ratios(sc.key)
            rho = sum(base.offered(i) or 0.0 for i in range(n))
            print(f"  M/D/1 mean wait at the realised load {_pct(rho)}: {_ms(md1_wait(base))} ms; policy 1 / M/D/1 "
                  f"{mean_delay_all(base) / md1_wait(base):.3f} here, over {REPLICATIONS} other seeds "
                  f"{min(ratios):.3f} to {max(ratios):.3f}, mean {sum(ratios) / len(ratios):.3f}")
    data = throughput_data()
    for policy in POLICIES:
        cells = [f"{k}: " + ", ".join(f"{_pct(data[policy, k][LENGTHS.index(m)])} (m = {m:g})"
                                      for m in (LENGTHS[0], SHORT, 1.0)) for k in range(1, n + 1)]
        print(f"{_policy_label(policy)} utilisation by active controllers: {'; '.join(cells)}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="run self-checks and exit non-zero on failure")
    if parser.parse_args(argv).check:
        return run_checks()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name, build in FIGURES:
        path = OUT_DIR / name
        path.write_text(build(), encoding="utf-8", newline="\n")
        print(f"wrote {path}")
    print_summary()
    return 0


if __name__ == "__main__":
    sys.exit(main())
