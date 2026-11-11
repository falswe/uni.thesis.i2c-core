# Shared I2C Session Protocol

This file is the canonical protocol definition for host and firmware integration across the bridge firmware repos:

- STM bridge: `../stm32u375_bridge`

This contract is transport-agnostic and must work over UART and RTT.

## Scope

- Line-oriented ASCII command/response session protocol.
- Electrical I2C details remain implementation-specific.

## Command grammar (normative)

Commands are space-delimited. Command verbs and segmentation tokens are case-insensitive; implementations must accept lowercase and mixed-case forms and should normalize internally.

Examples: `info`, `w np ...`, and `R S-P ...` are valid.

- `INFO`
  - Returns protocol banner, command set, and status map.
- `W [<seg>] <a7> <b0> [b1] ...`
  - Controller write to 7-bit I2C address.
  - `<a7>` accepts decimal or `0x` hex.
  - `<bN>` accepts decimal or `0x` hex bytes.
- `R [<seg>] <a7> <len>`
  - Controller read from 7-bit I2C address.
  - `<len>` is requested byte count.
- `P`
  - Peek latest target RX capture.
- `ADDR [<a7>]`
  - Without `<a7>`: report the bridge's own 7-bit target address.
  - With `<a7>`: change the own target address at runtime (see [Own target address](#own-target-address-normative)).
- `END`
  - End/ack command for scripted sessions.
- `BURST <count> <a7> <b0> [b1] ...` (optional, measurement)
  - `<count>` writes with STOP back-to-back from a tight firmware loop, without host round trips, to expose the controller's re-arm time on the bus.
  - Response: `BURST status=Success` if every write succeeded, otherwise `status=Aborted`, followed by
    `burst count=<n> ok=<n> arlo=<n> addrnak=<n> datanak=<n> timeout=<n> other=<n>`,
    `burst submit_us n=<n> min=<us> mean=<us> max=<us>` (time to submit one write) and
    `burst period_us n=<n> min=<us> mean=<us> max=<us>` (start-to-start period).
  - Only bridges that list `BURST` in `INFO commands=` support it (currently the STM bridge).
- `Q W <a7> <b0> [b1] ...`, `Q R <a7> <len>`, `Q WR <a7> <len> <b0> [b1] ...` (optional, transaction layer)
  - One transfer through the [transaction layer](transaction_layer.md): queued, retried after `ArbitrationLost`, backed off after `AddrNak` and bounded by the budget `F`. `W` writes with STOP, `R` reads with STOP, `WR` writes `<b0> ...`, sends a repeated START and reads `<len>` bytes (a memory read as one transfer).
  - Response: `Q status=<Token> last=<Token> attempts=<n> arlo=<n> addrnak=<n> queue_us=<n> head_us=<n>`, then `data: ...` on `Success` (the written bytes for `W`, the read bytes for `R` and `WR`).
    - `status` is the final outcome. `last` is the outcome of the last attempt, or `Busy` if no attempt got onto the bus. `arlo` and `addrnak` count the attempts with these outcomes.
    - `queue_us` is the time from submission to the head of the queue, `head_us` the time from the head of the queue to the final outcome.
  - While a raw transaction is held open (`np`), the command reports `status=Busy` and changes nothing.
- `TXN [F=<ms>] [backoff=<ms>] [stuck=<ms>]` (optional, transaction layer)
  - Without arguments: report the parameters. With arguments: change them until the next reset. `F` and `backoff` are `F` and `t_backoff` of the transaction layer; `stuck` is how long after `F` an armed START that the platform cannot withdraw may wait for the bus before the transfer ends with `Timeout`, last `Busy`.
  - Response: `TXN status=<Token>`, then `txn F_ms=<n> backoff_ms=<n> stuck_ms=<n> queue=<n>` (`queue` is the queue depth).
- Only bridges that list `Q` and `TXN` in `INFO commands=` support them (currently the STM bridge).

Optional segmentation token `<seg>`. Tokens are case-insensitive. Two forms are accepted:

- Shorthand: `np`
- Explicit segment: `s-p`, `s-np`, `ns-p`, `ns-np`

The token has two orthogonal axes:

- START: `s` asserts a start (a repeated start when the bus is already held); `ns` emits no start and continues the current addressed transfer.
- STOP: `p` emits a final STOP; `np` keeps the transaction open for the next command.

When the token is omitted, firmware must:

- Emit a final STOP (the `-p` axis).
- Infer START from the current transaction state:

  | last segment seen | `R` command         | `W` command         |
  | ----------------- | ------------------- | ------------------- |
  | write, bus held   | repeated start      | no start (continue) |
  | read, bus held    | no start (continue) | repeated start      |
  | none / bus idle   | start               | start               |

`np` is shorthand for "inferred START, no STOP": it is equivalent to whichever of `s-np` / `ns-np` the inference selects.

The explicit `s-*` / `ns-*` tokens are a legacy-compatibility form and an escape hatch. They override the inferred START so hosts can express sequences the inference does not choose (for example a repeated start to a different address between two writes: `S-W-Sr-W`). Firmware must accept them and normalize to the same transaction state machine as the inferred path.

Validation: `ns-p` / `ns-np` require an open transaction. If the bus is idle (no segment is currently held open), the command must report `status=InvalidParameter`.

This represents a write without STOP followed by a read that completes the transaction. The read infers a repeated start because the previous segment was a held write.

## Canonical status model (normative)

All command completions that report status must include:

- `status=<Token>`

Canonical token set:

- `Success`
- `Busy`
- `InvalidParameter`
- `ArbitrationLost`
- `AddrNak`
- `DataNak`
- `Timeout`
- `Aborted`
- `Unknown`

Rationale: These tokens can be produced or inferred on all considered systems. `Busy` is canonical because it can be reported by firmware and is advertised by `INFO status_map`; hosts should treat it as retryable policy, not as success.

The two NAK tokens tell where the target refused the transfer:

- `AddrNak`: the address byte was not acknowledged; no data byte reached the target. On a read every target NAK is an address NAK, because the controller acknowledges the data bytes itself.
- `DataNak`: the address was acknowledged, but a data byte of a write was not. The target has already received the preceding bytes, so retrying the write repeats data the target has partially seen.

A write segment without START (`ns-*`) has no address phase; any NAK it receives is a `DataNak`.

## Own target address (normative)

`ADDR` moves a bridge to another target address without re-flashing.

- The response is `ADDR status=<Token>` followed by `addr: 0x<hh>`, the address in effect after the command. On failure this is the unchanged previous address.
- Valid addresses are `0x08`..`0x77`. The reserved ranges `0x00`..`0x07` and `0x78`..`0x7F`, and values above `0x7F`, report `status=InvalidParameter`.
- The new address is active when the response is emitted: from then on the bridge acknowledges only the new address and the old one is NAKed. Setting the current address again succeeds without effect.
- While the bridge holds a controller transaction open (`np`) or is being addressed as a target, the command reports `status=Busy` and changes nothing.
- The address is volatile: a reset restores the address built into the firmware.
- Hosts must not change the address while traffic to that bridge is in flight; a transfer that addresses it during the switch may be NAKed. After `Success`, hosts update their address map before addressing the bridge again. Hosts are responsible for not assigning an address that another participant, including the tester, already uses.

## Data output format (normative)

Read, write, and peek data should use the same block shape:

- `data: <hex bytes...>`

Example:

`data: 0x11 0x22 0x33`

Rules:

- Empty payloads or captures should be emitted as `data:` with no trailing bytes.
- For write, `data:` echoes bytes accepted for transfer.

## INFO minimum content

Firmware should emit these `INFO` lines:

- `INFO protocol=session_v1`
- `INFO commands=INFO,W,R,P,ADDR,END`
- `INFO status_map=Success,Busy,InvalidParameter,ArbitrationLost,AddrNak,DataNak,Timeout,Aborted,Unknown`
