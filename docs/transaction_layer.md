# I2C Transaction Layer (draft)

Status: draft for agreement. Items marked **Open** still need a decision. Once agreed, the STM bridge implements this layer first; the other platforms follow.

This document defines how a bus participant handles an I2C transfer on a shared multi-controller bus: from submission to its final outcome, including retries, time budgets, bus access, hold limits and bus recovery. It is independent of the host interface. [protocol.md](protocol.md) defines how a host talks to a bridge; its raw `W`/`R` commands stay single attempts without retries, so the conformance tests keep checking the unmodified bus behaviour.

The layer is chip-agnostic. Where a requirement depends on what an I2C peripheral can do, it links to [platform_capabilities.md](platform_capabilities.md), which records for each platform what the vendor documents, what the bridge implements and what has been verified in hardware.

The key words MUST, SHOULD and MAY are used as in RFC 2119.

## 1. Terms

- **Participant**: a device on the bus that can act as controller and as target. While it is a controller, its target role MUST stay active, because it may lose arbitration to a transfer that addresses it.
- **Transfer**: one unit of work submitted to the layer: one START to STOP sequence, possibly with repeated STARTs (for example a register write followed by a read).
- **Attempt**: one try of a transfer on the bus, starting with a START.
- **Turn**: an attempt that won arbitration, until its STOP.
- **Attempt outcome**: the result of one attempt. **Final outcome**: the result reported to the caller.

## 2. Parameters

| Name | Meaning | Default | Status |
| --- | --- | --- | --- |
| `T_max` | Longest legal transfer including clock stretching: 1024 B × 9 bit / 800 kbit/s + 2.7 ms = 14.22 ms, rounded up | 20 ms | measured basis |
| `F` | Flush time: how long the layer keeps trying to start a transfer (queue head, bus wait, backoff, retries). It never cuts an attempt that is on the bus (section 4.3) | 1000 ms | **Open** |
| `t_backoff` | Wait after `AddrNak` before the next attempt | `F / 10` | **Open** |
| `B_hold` | Longest continuous bus ownership of a controller, START to STOP | `T_max` | **Open** |
| `S_max` | Longest clock stretch a target may apply (section 6) | 5 ms | **Open** |
| `t_stuck` | SCL or SDA held low this long means the bus is stuck | 25 ms | **Open** |
| `t_idle` | Idle window of the idle-window access policy (section 5) | 10 µs | **Open** |
| `t_buf` | Bus-free time before waiting controllers start (observed) | 0.78 µs | measured |
| `r` | Re-arm time of a controller after its own STOP (STM bridge) | 60 µs | measured |

The defaults must satisfy `S_max < B_hold ≤ T_max < t_stuck` and `t_backoff < F`.

## 3. Outcomes

| Attempt outcome | Meaning | Final? |
| --- | --- | --- |
| `Success` | Every byte acknowledged as required; STOP sent (or the transaction is held open on request) | yes |
| `ArbitrationLost` | Another controller won the address phase; nothing was transferred | no, retry |
| `AddrNak` | The address was not acknowledged; nothing reached the target | no, back off and retry |
| `DataNak` | The address was acknowledged, but a data byte of a write was not | yes |
| `Aborted` | Bus error or local abort | yes |

When the budget `F` runs out before a final attempt outcome, no further attempt is started and the final outcome is `Timeout`; an attempt that is on the bus is completed first (section 4.3). The report MUST also contain the last attempt outcome (or `Busy` if the bus never became free) and the number of attempts, so the caller can tell a missing target from a congested bus.

The tokens are the canonical status tokens of [protocol.md](protocol.md).

**Open:** `DataNak` SHOULD include the number of acknowledged data bytes. Platform support: [C3](platform_capabilities.md#c3-nack-classes-and-acknowledged-bytes).

## 4. Transfer lifecycle

### 4.1 Queue

- Transfers MUST be served in FIFO order from a bounded queue.
- A submission to a full queue MUST fail immediately with `Busy`, without touching the bus.
- The budget `F` starts when the transfer becomes the head of the queue. The result time of a submission is then bounded by `(q + 1) × F` plus processing time, where `q` is the number of transfers ahead of it.
- **Open:** the alternative is to start `F` at submission. That bounds the result time by `F`, but transfers can then expire while they are still queued.

### 4.2 Attempt rules

1. **Bus busy.** The controller MUST arm its START in the peripheral and let the hardware wait for the free bus. It MUST NOT poll for STOP in software and then issue START, because that adds interrupt latency to every contended access (see section 5). The waiting time counts against `F`.
2. **`ArbitrationLost`.** The controller MUST serve an incoming transfer that addresses it, and then retry at the next opportunity. A lost arbitration does not count as a turn for the access policy.
3. **`AddrNak`.** The controller MUST wait `t_backoff` and then retry the same address. Addressing costs bus time for all participants, so a NAKing target must not be polled continuously.
   - **Open (later):** a list of alternate addresses tried in order, with the backoff applied after the last one.
4. **`DataNak`.** Final. The controller MUST NOT resend automatically: the target has already received and possibly acted on the acknowledged bytes.
   - **Open:** the caller may mark a transfer idempotent to allow resending.
5. **`Aborted`.** Final, after local cleanup (section 4.4).

### 4.3 Giving up

`F` limits how long the controller keeps *trying*. It never cuts an attempt that is on the bus.

- **Not owning the bus** (at the queue head, backing off, or waiting with its START armed): when `F` runs out, the controller MUST NOT start another attempt. It withdraws an armed START and reports `Timeout` as in section 3.
  - Withdrawing an armed START MUST NOT abandon a START that the peripheral has already generated. If the peripheral cannot withdraw it atomically, the controller lets the attempt run and ends it as in the next point.
  - Platform support: [C2](platform_capabilities.md#c2-withdraw-an-armed-start).
- **Owning the bus** (from its START to its STOP): the controller MUST complete the attempt and end it with a STOP. `F` running out has no effect on the attempt. If the attempt outcome is final, it is the final outcome; otherwise the final outcome is `Timeout`. The duration of the attempt is bounded by the hold limits of section 6, not by `F`.
- The controller MUST NOT resubmit the transfer by itself. Resubmission is the caller's decision.

Measured from the queue head, the final outcome is therefore reported within `F + B_hold + S_max` plus one byte time.

**Why the owner must not leave the bus.** After a START, every participant considers the bus busy until it sees a STOP (UM10204 §3.1.4); the armed STARTs of waiting controllers fire only after that STOP. Only the controller that owns the transaction can generate the STOP, and only while it drives SCL. A controller that stops in the middle, for example by disabling its peripheral, leaves the bus busy for everyone else ("missing STOP", section 7), and a target in the middle of a byte may keep SDA low. Giving up on the bus would turn a slow target into a bus failure.

A local `Timeout` does not mean the target did nothing. An unconfirmed command can still have been executed.

### 4.4 Cleanup

Before a buffer is reused or the next transfer starts, all controller activity of the old attempt (peripheral state, DMA, interrupts) MUST be retired. A late completion of an old attempt MUST NOT be attributed to the next transfer.

## 5. Bus access policy

### 5.1 Baseline: re-arm and retry

The baseline submits every attempt as soon as the transfer is at the head of the queue and the previous turn has completed. The simulator ([bus_policy_sim.py](../tools/bus_policy_sim.py), measured `r` = 60 µs) shows its limitation. With three saturated controllers, the two lowest target addresses alternate and the highest address never gets the bus. Under moderate random load every controller is served, but the highest address waits longest.

A fixed delay after the own STOP, without observing the bus, is the same policy with a larger `r`. It does not prevent this starvation. The current `r` = 60 µs is already a delay of about 48 clock cycles.

### 5.2 Optional: idle-window deferral

This policy follows DMTF DSP0237 section 6.13.

- After its own turn, a controller MUST NOT start a new attempt until the bus has been **continuously idle for `t_idle` after its own STOP**. Any START by another controller restarts the window.
- Controllers that have not had a turn since the last idle window start as soon as the bus is free.
- With 2–3 controllers it keeps the utilisation at or above 97 %.
- From the head of its own queue, a controller with `k` lower target addresses waits at most about `(N − 1 + k) × T_max + t_idle`: if its next transfer arrives while it defers, every other controller may send once before the idle window and the `k` lower addresses once more after it. The highest address therefore waits up to `2 (N − 1) × T_max`. Saturated controllers stay in a fixed round and wait at most about `(N − 1) × T_max + t_idle`. A transfer queued behind the controller's own earlier transfers waits for those in addition. The simulator checks this bound.

Implementation requirements:

- The idle observation MUST come from hardware or from latched flags, so interrupt latency can only lengthen the wait, never shorten it. Examples: a hardware bus-idle timer, or a latched bus-monitor flag plus a timer ([C10](platform_capabilities.md#c10-bus-idle-observation)).
- All participants MUST arm their START in hardware while the bus is busy (rule 4.2.1). Otherwise a waiting controller with a slow interrupt may start later than `t_idle`, and the controller that just had its turn retakes the bus.

**Open:** whether the product needs this policy, or the baseline is acceptable at the expected load. That depends on the load measurements.

## 6. Bus hold limits

No participant may block the bus indefinitely. The limits are layered: a controller can end its own transaction, but it cannot end a clock stretch, because no STOP can be generated while SCL is held low. The strict limit that keeps the bus closable therefore lies with the target.

SCL is a wired-AND line. Every participant sees only that SCL is low, not who holds it; a peripheral can only know whether it holds SCL low itself. The limits below are therefore defined on a participant's own hold, never on the time SCL is low. The controller does not need to know who holds SCL: it waits in either case (section 4.3).

1. **Controller.** A controller MUST complete each transfer, including all repeated STARTs and gaps, within `B_hold` of its first START. If `B_hold` runs out, it ends the transaction at the next byte boundary ("forced STOP") and reports `Timeout`; part of the data may already have been transferred.
   - Write: send no further data byte; STOP after the current byte.
   - Read: NACK the next received byte, then STOP, so that the target releases SDA.
   - Transaction held open between segments (`np`): STOP.

   The controller can always do this while it drives SCL. A clock stretch by the target can delay the next byte boundary by at most `S_max`. Platform support: [C5](platform_capabilities.md#c5-forced-stop-at-a-byte-boundary).

   A controller also holds SCL low itself between bytes while it waits for its own firmware or DMA. That hold is part of its turn and therefore bounded by `B_hold`; because `B_hold` < `t_stuck`, it is never taken for a stuck bus. Devices that reset after SCL has been low for 25–35 ms, whoever holds it (SMBus `tTIMEOUT`), additionally need `B_hold` < 25 ms. Platform support: [C6](platform_capabilities.md#c6-limit-on-the-controllers-own-clock-hold).

   An error indication that does not end the transfer in the peripheral, for example a spurious bus error, MUST NOT make the controller abandon the attempt. The controller continues the attempt or ends it with a forced STOP as above. Platform support: [C7](platform_capabilities.md#c7-controller-reaction-to-bus-errors).
2. **Target.** A target MUST NOT hold SCL low itself for longer than `S_max` at a time. The limit applies to the target's own hold only: the controller may hold SCL low at the same time (rule 1), and a target that counts that time as its own stretch abandons transfers for no reason. A target MUST NOT use the time SCL is low as its stretch time. A limit of at most `S_max` on the target's cumulative stretch per transfer also satisfies this rule.

   If the target is not ready within `S_max`, it MUST release SCL and SDA by itself, abandon its part of the transfer and ignore the bus until the next STOP or START. The controller then completes the current byte and ends the transaction:
   - Write: the target no longer acknowledges, so the controller sees a NACK, sends STOP and reports `DataNak`.
   - Read: the controller reads `0xFF` for the remaining bytes, NACKs the last one and sends STOP. It cannot tell these bytes from valid data.

   A target SHOULD enforce the limit with a mechanism that releases the lines without firmware, so that the limit also holds when its firmware hangs. If the limit relies on firmware, a hung firmware falls through to rule 3. Platform support: [C8](platform_capabilities.md#c8-limit-on-the-targets-own-stretch).

   **Open:** a target that releases during a read is invisible to the controller. Reads whose content matters need an integrity check in the payload (length and CRC, or SMBus PEC), or targets must not stretch during reads.
3. **Bus.** SCL or SDA held low for `t_stuck` means the bus is stuck and section 7 applies. A controller that owns the bus at that moment reports a bus failure, not `Timeout`, and keeps its obligation to end the transaction: as soon as SCL is released again (for example after the participant holding it has been reset), it generates a STOP before any other access, by GPIO if its peripheral had to be reset.

## 7. Bus recovery

### 7.1 Failure classes and detection

| Failure | Typical cause | Detection |
| --- | --- | --- |
| SDA stuck low | A target lost bit synchronisation (e.g. controller reset mid-byte) and keeps driving a 0 | SDA low while SCL high for `t_stuck` |
| SCL stuck low | A target stretches forever (hung firmware), or a short | SCL low for `t_stuck` |
| Missing STOP | A controller reset during its transaction; peripherals see the bus as busy | Bus busy without edges for `T_max` |
| Repeated `Timeout` | Target missing, permanently NAKing, or congested bus | `K` consecutive `Timeout` outcomes (**Open:** `K`) |

A detector that only sees SCL low cannot tell who holds SCL. A participant MUST NOT end its part of a transfer because of such a detector before `t_stuck`, so that a legal controller hold is never cut (section 6). Combined with a check of the participant's own hold, the same timer can serve the stretch limit of section 6, rule 2. Platform support: [C9](platform_capabilities.md#c9-stuck-bus-detection).

### 7.2 Reaction

- **Product baseline.** A detected bus failure MUST be reported as such, not as an ordinary `Timeout`. The system then enters a failure mode that the user leaves by restarting the device. **Open:** a distinct status token, e.g. `BusError`.
- **Automatic recovery (optional, evaluated in the thesis):**
  - **SDA stuck:** the controller clocks SCL up to nine times until SDA is released, then sends STOP (UM10204 §3.1.16).
  - **Missing STOP:** the controller that owns the transaction sends the STOP as soon as SCL is free (section 6, rule 3). If that controller was reset, another participant sends a STOP by GPIO after the bus has been idle for `T_max`.
  - **SCL held by the own target:** reset the own peripheral.
  - **SCL held by another participant:** clocking cannot clear it. It needs an out-of-band reset by a participant that has one (e.g. a housekeeping controller with reset or power control).
- **After any reset or recovery:**
  - Discard partial transfers.
  - Do not continue old open transactions.
  - Wait until the bus has been idle for `T_max` before the first access. A participant that joins during a transfer cannot otherwise know that the bus is in use.

## 8. Observability

Each transfer SHOULD record:

- submission time, first START and completion time
- the number of attempts per outcome
- the final outcome

The participant SHOULD keep cumulative counters per outcome and per fault class. A host command to read them (`STATS`) and a command to generate load inside the firmware (`LOAD`) will be added to [protocol.md](protocol.md) as optional commands when they are implemented.

## 9. Platform capabilities and implementation status

[platform_capabilities.md](platform_capabilities.md) records, for each requirement that depends on the I2C peripheral, what each platform documents, what its bridge implements and what has been verified in hardware, together with the hardware tests that are still needed. It also lists which parts of this layer the bridges implement.

**Open:** the signal-processing participant has not been checked against these capabilities yet.

## 10. Open decisions

1. `F` and when it starts (section 4.1).
2. `t_backoff`: fixed `F / 10`, or exponential.
3. `S_max`, `B_hold` and `t_stuck` values.
4. Whether to report the acknowledged byte count with `DataNak`, and whether to allow resending idempotent transfers.
5. Alternate-address retry (later).
6. Whether to adopt idle-window deferral, and the value of `t_idle`.
7. A distinct status for bus failures, and the threshold `K` for repeated timeouts.
8. Whether automatic recovery is used in the product, or only the failure mode with manual restart.
9. Integrity check for reads, since a target that releases during a read returns `0xFF` that the controller cannot detect (section 6).
10. How a controller withdraws an armed START when `F` runs out without abandoning a START that was just generated (section 4.3). No current platform documents an atomic withdrawal ([C2](platform_capabilities.md#c2-withdraw-an-armed-start)), so for now the attempt runs to its STOP.
