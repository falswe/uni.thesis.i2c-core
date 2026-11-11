# Platform Capabilities (informative)

The [transaction layer](transaction_layer.md) is chip-agnostic. This document records what each platform can do for the requirements that depend on the I2C peripheral, so that the layer is written with the platforms' limits in mind. It is not normative.

Each capability is recorded at three levels of evidence:

| Level | Meaning | Source |
| --- | --- | --- |
| Documented | What the vendor documentation says | Reference manual, errata, register descriptions |
| Implemented | What the bridge firmware does today | Bridge source at the revision in section 1 |
| Verified | What a hardware test has shown | Test ID from section 4 and its result |

"Not documented" means that the documentation is silent, not that the feature is missing. A capability counts as available only when it is verified. Test results go into the Verified rows with the date, the firmware revisions and a link to the captures in `host/output/`.

## 1. Sources

| Platform | Documentation | Code |
| --- | --- | --- |
| STM32U375 | RM0487 Rev 3 (26 Feb 2026), chapter 48 (I2C); ES0626 Rev 2 (Jan 2026), section 2.12 | `../stm32u375_bridge` at `476019f` |
| Signal-processing participant | Not checked | Not checked |

## 2. Summary

| ID | Capability | Transaction layer | STM32U375 |
| --- | --- | --- | --- |
| [C1](#c1-arm-start-while-the-bus-is-busy) | Arm START while the bus is busy | 4.2 | implemented, verified |
| [C2](#c2-withdraw-an-armed-start) | Withdraw an armed START atomically | 4.3 | not possible |
| [C3](#c3-nack-classes-and-acknowledged-bytes) | Tell `AddrNak` from `DataNak`; count acknowledged bytes | 3 | implemented; `DataNak` not tested |
| [C4](#c4-target-role-while-acting-as-controller) | Serve as target while acting as controller | 1, 4.2 | implemented, verified |
| [C5](#c5-forced-stop-at-a-byte-boundary) | Forced STOP at a byte boundary | 6 | documented for writes; reads unclear |
| [C6](#c6-limit-on-the-controllers-own-clock-hold) | Limit on the controller's own clock hold | 6 | hardware, not enabled |
| [C7](#c7-controller-reaction-to-bus-errors) | Keep the attempt after a bus error | 6 | hardware continues; firmware abandons |
| [C8](#c8-limit-on-the-targets-own-stretch) | Limit on the target's own stretch | 6 | hardware, not enabled |
| [C9](#c9-stuck-bus-detection) | Stuck-bus detection | 6, 7.1 | SCL-low timer; not used on the main bus |
| [C10](#c10-bus-idle-observation) | Bus-idle observation | 5.2 | hardware timer, not used |
| [C11](#c11-recovery) | Peripheral reset, clock pulses, STOP by GPIO | 7.2 | partly implemented, unused |

Nothing in C5 to C11 has been verified in hardware.

## 3. Capabilities

### C1 Arm START while the bus is busy

Required by transaction layer section 4.2, rule 1.

| | STM32U375 |
| --- | --- |
| Documented | `START` may be set while `BUSY` = 1; the START is sent tBUF after `BUSY` falls (RM0487 §48.4.9, `I2C_CR2.START` §48.9.2) |
| Implemented | Yes: `i2c_wait_controller_turn` arms while the bus is busy |
| Verified | `test_arbitration.py` (the tester holds the bus, both DUTs arm, the tester releases) in a 3000-suite campaign. The read variant failed a few times in early campaign segments |

### C2 Withdraw an armed START

Required by transaction layer section 4.3.

| | STM32U375 |
| --- | --- |
| Documented | Not possible in software. `START` is cleared by hardware after the address is sent, on arbitration loss, on an address match as target, on a timeout, or with `PE` = 0; "Writing 0 to this bit has no effect" (§48.9.2). `PE` = 0 also resets the target role and, if the START has already been sent, abandons the transaction (§48.4.6) |
| Implemented | Never withdraws; a submitted attempt runs |
| Verified | Not tested (HW-09) |

Consequence: on the STM32 the rule of section 4.3 applies: the controller lets the attempt run and ends it with STOP. `F` can therefore be exceeded by one attempt.

### C3 NACK classes and acknowledged bytes

Required by transaction layer section 3.

| | STM32U375 |
| --- | --- |
| Documented | `NACKF` only; address and data NACK are not separate flags, so firmware derives the class from the transfer progress. With `AUTOEND` = 1 a STOP follows a NACK automatically (§48.4.9) |
| Implemented | `AddrNak` and `DataNak` (commit `0cd0678`). The acknowledged byte count can be derived from the TX DMA counter; not reported |
| Verified | `AddrNak`: `test_runtime_address.py`. `DataNak`: no hardware test |

### C4 Target role while acting as controller

Required by transaction layer sections 1 and 4.2, rule 2.

| | STM32U375 |
| --- | --- |
| Documented | On arbitration loss "the controller automatically switches back to target mode and can acknowledge its own address" (§48.4.9) |
| Implemented | Yes; `ADDRIE` stays enabled while acting as controller |
| Verified | `test_arbitration.py`, write and read, in the campaign of C1 |

### C5 Forced STOP at a byte boundary

Required by transaction layer section 6, rule 1 (write: STOP after the current byte; read: NACK the next byte, then STOP).

| | STM32U375 |
| --- | --- |
| Documented | `STOP`: "STOP generation after current byte transfer" (§48.9.2). For a read the RM does not say whether that byte is NACKed. The documented clean end of a read is the end of `NBYTES` (NACK, then STOP; §48.4.9), which the firmware can bring forward only at a `TCR` stop, i.e. at most every 255 bytes |
| Implemented | `ARM_I2C_ABORT_TRANSFER` calls `HAL_I2C_Master_Abort_IT`; no forced-STOP policy uses it |
| Verified | Not tested (HW-07) |

Consequence: unless HW-07 shows that `STOP` NACKs the current byte, a forced STOP in an STM32 read takes effect only at the next `NBYTES` boundary, up to 255 bytes (about 2.9 ms at 800 kbit/s) later.

### C6 Limit on the controller's own clock hold

Required by transaction layer section 6, rule 1 (the controller's own hold is part of its turn).

| | STM32U375 |
| --- | --- |
| Documented | The controller holds SCL while `TXIS` (from the ninth pulse), `RXNE` (between the eighth and ninth pulse), `TCR` or `TC` waits for firmware (§48.4.7, §48.4.9). `TIMEOUTB` with `TEXTEN` = 1 counts this own hold cumulatively (SMBus tLOW:MEXT) and sends STOP on expiry (§48.4.12, §48.4.17). One `TIMEOUTB` value serves both roles and is writable only with `TEXTEN` = 0. The RM does not say how this STOP ends a read |
| Implemented | Not enabled |
| Verified | Not tested (HW-02) |

Consequence: on the STM32 the hardware limit is the same value as the target limit (C8): enabling it for the target role also limits the controller's own hold, so the bridge must keep its own holds well below it (DMA, prompt `TCR` handling).

### C7 Controller reaction to bus errors

Required by transaction layer section 6, rule 1 (an error that does not end the transfer in hardware must not make the controller abandon the attempt).

| | STM32U375 |
| --- | --- |
| Documented | `BERR` on a misplaced START or STOP (§48.4.17). ES0626 §2.12.2: `BERR` can be set spuriously in controller mode; "Detection of bus error has no effect on the I2C-bus transfer in controller mode and any such transfer continues normally". Workaround: clear `BERR` and continue |
| Implemented | Any `BERR` goes from `HAL_I2C_ER_IRQHandler` to `I2C_ITError` (transfer interrupts off, `TXDR` flushed, driver idle); `HAL_I2C_ErrorCallback` reports `Aborted` and the bridge returns to listen mode. From the code, the hardware then continues and holds SCL at the next `TXIS` or `RXNE` with nobody serving it |
| Verified | Not tested (HW-08) |

Consequence: the STM bridge does not yet meet the rule.

### C8 Limit on the target's own stretch

Required by transaction layer section 6, rule 2 (the limit applies to the time the target itself holds SCL).

| | STM32U375 |
| --- | --- |
| Documented | `TIMEOUTB` with `TEXTEN` = 1 in target mode counts only the target's own stretch: "a cumulative SCL stretch … done by the I2C interface" (§48.9.6), summed from START to STOP (SMBus tLOW:SEXT, §48.4.11). On expiry "the SDA and SCL lines are automatically released" (§48.4.17), without firmware. Range: 42.7 µs steps up to 174.8 ms at the 48 MHz kernel clock. The RM does not say how the target behaves for the rest of the transfer. `TIMEOUTA` with `TIDLE` = 0 also releases the lines in target mode, but counts SCL low whoever holds it |
| Implemented | Not enabled. `TIMEOUTA` = 500 ms on I2C2 (auxiliary bus); off on I2C1 (main bus) |
| Verified | Not tested (HW-03, HW-04, HW-05) |

Consequence: the STM32 measures its own stretch in hardware, as rule 2 requires. An SCL-low timer must not end a target's stretch (HW-05).

### C9 Stuck-bus detection

Required by transaction layer sections 6, rule 3, and 7.1.

| | STM32U375 |
| --- | --- |
| Documented | `TIMEOUTA` with `TIDLE` = 0: SCL low continuously for (`TIMEOUTA` + 1) × 2048 × tI2CCLK, whoever holds it, up to 174.8 ms. On expiry: STOP in controller mode, lines released in target mode (§48.4.12, §48.4.17). Shares `TIMEOUTA` with idle detection (C10). SDA low with SCL high, and a busy bus without edges, have no detector; the GPIO input register reads the pin level also in alternate-function mode (§12.3.11), so firmware can sample them |
| Implemented | `TIMEOUTA` = 500 ms on I2C2; none on I2C1 |
| Verified | Not tested (HW-10) |

Consequence: an SCL-low timer in a participant that is also a target must be set to at least `t_stuck`, because it cannot tell a controller hold from its own stretch. A legal controller hold is shorter (`B_hold` < `t_stuck`).

### C10 Bus-idle observation

Required by transaction layer section 5.2.

| | STM32U375 |
| --- | --- |
| Documented | `TIMEOUTA` with `TIDLE` = 1: SCL and SDA high for (`TIMEOUTA` + 1) × 4 × tI2CCLK, 83 ns steps up to 341 µs at 48 MHz; latched `TIMEOUT` flag (§48.4.12, §48.9.6). Cannot run together with the SCL-low timeout (C9); `TIDLE` is writable only with `TIMOUTEN` = 0 |
| Implemented | Not used |
| Verified | Not tested (HW-11) |

Consequence: on the STM32, idle-window deferral and SCL-low detection compete for `TIMEOUTA`; one of them has to be switched over at run time or done in firmware.

### C11 Recovery

Required by transaction layer section 7.2.

| | STM32U375 |
| --- | --- |
| Documented | `PE` = 0 releases SCL and SDA and resets the state machines; the configuration is kept (§48.4.6). The pins can be switched to GPIO open-drain for clock pulses and a STOP |
| Implemented | `ARM_I2C_BUS_CLEAR` (`i2c_clear_bus`) exists; the layer does not use it |
| Verified | Not tested (HW-12) |

### Other platform constraints

- ES0626 §2.12.1: the STM32 samples SDA wrongly when tSU;DAT is shorter than one kernel clock period; Fast-mode Plus needs I2CCLK ≥ 20 MHz. The bridge runs at 48 MHz with `SCLDEL` = 5, verified in [scldel_ab_report.md](scldel_ab_report.md).

## 4. Hardware tests

### 4.1 Instruments and hooks

- **Pico tester** (address `0x55`): holds SCL and SDA by GPIO (`hold_bus`, `release_bus`, or `gpio-write` with `delay` for timed holds); writes without STOP (`w ns`), whose gap emulates a controller hold if the RP2350 keeps SCL low during it (HW-00); acts as a target with preloaded data.
- **Logic analyser** (Saleae Logic 2 with `tools/saleae_i2c_hla.py`): timing, and which participant releases which line.
- **Bridge test hooks**, not implemented yet:
  1. Set the platform timeouts at run time (STM32 `TIMEOUTA`, `TIDLE`, `TIMEOUTB`, `TEXTEN`).
  2. Delay serving a target byte by a given time (target stretch).
  3. Delay serving a controller byte by a given time (controller hold).
  4. Report timeout and bus-error events to the host.

  **Open:** how the host sets them, for example an optional test command in [protocol.md](protocol.md).

### 4.2 Tests

The hold and stretch questions of transaction layer section 6 come first: HW-00, HW-01, HW-03, HW-04, HW-05, HW-07 and HW-08.

| ID | Question | Setup and procedure | Pass criterion | Platforms | Needs |
| --- | --- | --- | --- | --- | --- |
| HW-00 | Does the Pico hold SCL low between `w ns` and the next `w`? | `w ns <a7> 0x01; delay 10; w <a7> 0x02` to a DUT target; logic analyser | SCL low for 10 ms between the bytes, no STOP | Pico | – |
| HW-01 | Does the controller's own-hold limit ignore another participant's hold? | STM32 controller writes 64 bytes; the Pico pulls SCL low for 10 ms during the transfer; `TIMEOUTB` = 5 ms, `TIMOUTEN` = 0. Control run: `TIMEOUTA` = 5 ms, `TIDLE` = 0 | No `TIMEOUT`, `Success`. Control run: `TIMEOUT` | STM32 | Hook 1 |
| HW-02 | Does the hardware bound the controller's own hold, and how does it end the transfer? | STM32 controller holds 10 ms before a byte of a write and of a read; `TIMEOUTB` = 5 ms | `TIMEOUT` after 5 ms, followed by STOP; record whether a read byte is NACKed before the STOP | STM32 | Hooks 1, 3 |
| HW-03 | Does the target's stretch limit ignore a controller hold? | Pico controller hold as in HW-00, 10 ms, against the DUT target; limit 5 ms (STM32 `TIMEOUTB`) | No abort; `P` shows both bytes | STM32 | HW-00, hook 1 |
| HW-04 | Does the target release after its own stretch? | DUT target stretches 10 ms in a write and in a read; limit 5 ms; controller is the Pico or the other DUT | SCL and SDA released after 5 ms (± one timer step). Write: NACK, STOP, `DataNak`. Read: `0xFF` for the remaining bytes, NACK, STOP. The next transfer succeeds. | STM32 | Hooks 1, 2 |
| HW-05 | Negative control: does an SCL-low timer in a target abort on a controller hold? | As HW-03, with STM32 `TIMEOUTA` = 5 ms, `TIDLE` = 0 | The target releases; this is why rule 2 forbids it | STM32 | Hook 1 |
| HW-07 | Does a forced STOP end the transfer cleanly? | The controller aborts a 256-byte write and read at random byte positions | Write: STOP after the current byte. Read: last byte NACKed, then STOP. SDA free afterwards; the next transfer succeeds | STM32 | Abort command |
| HW-08 | Does the controller keep its attempt after a bus error? | Provoke `BERR` with `SCLDEL` = 2, which caused controller bus errors in the [SCLDEL A/B campaign](scldel_ab_report.md); trigger the analyser on the error | The transaction ends with a STOP from the controller and the bus stays usable. Expected to fail on the STM bridge today (C7) | STM32 | Hook 4 |
| HW-09 | Can an armed START be withdrawn without an orphan transaction? | The Pico holds the bus, the DUT arms START, the host aborts while the Pico releases; offsets swept over ±100 µs | Every START on the bus is followed by a STOP from the same controller | STM32 | Abort command |
| HW-10 | Is a stuck bus detected? | The Pico holds SCL low for 30 ms; then SDA low with SCL high for 30 ms | Detection within `t_stuck` plus one timer step, reported as a bus failure | STM32 | Hooks 1, 4 |
| HW-11 | Is idle observed correctly? | Pico controller transfers with gaps from 1 µs to 1 ms; DUT idle observation with `t_idle` = 10 µs | Idle only after gaps ≥ `t_idle`; a START inside the window restarts it | STM32 | Hook 1 |
| HW-12 | Does recovery work? | (a) Reset the Pico controller during a read from a DUT target, so that the target keeps SDA low. (b) The Pico holds SCL during a DUT transfer, then releases | (a) At most nine clocks and a STOP free SDA. (b) The owner sends STOP before any other access | STM32 | Recovery implementation |

## 5. Transaction layer implementation

| Feature | STM bridge |
| --- | --- |
| Queue, budget `F`, backoff, give-up | no |
| Forced STOP when `B_hold` runs out | no |
| Target stretch limit `S_max` | no |
| Idle-window deferral | no |
| Bus-failure detection and recovery | no |
