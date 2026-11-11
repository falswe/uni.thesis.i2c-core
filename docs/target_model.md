# Target Device Model (register file)

This is the normative behavioral contract for the **target** side of a bridge:
the small byte-addressable register file each bridge exposes so that reads are
deterministic and participant-agnostic across bridges.

It is a companion to the wire protocol in [protocol.md](protocol.md). The
protocol defines the `session_v1` command/response grammar and formats; this
document defines what a conformant target *does* with those commands. Both are
normative for bridge participants.

## Register file (normative)

- The register file is at least 128 bytes and is initialised to `0xFF` (an
  erased memory).
- **Memory write** — a write that ends with STOP (`W <a7> <b0> [b1] ...`) resets
  the register file to `0xFF`, then stores the written bytes starting at
  offset 0.
- **Memory read** — a no-stop write followed by a read
  (`W np <a7> <mem_addr>` then `R <a7> <len>`) returns `len` bytes starting at
  `mem_addr`. `mem_addr` is the big-endian value of the no-stop write payload
  (one or more bytes), so `0x02` and `0x00 0x02` both select offset 2. Bytes
  past what was last written read back as `0xFF`.
- A plain read with no preceding no-stop write (`R <a7> <len>`) starts at
  offset 0. The selected offset applies to the immediately following read only;
  it returns to 0 afterwards.
- `P` (peek) reports the most recent bytes written to the target (the last write
  payload), independent of the register-file contents.

## Examples

With the register file loaded by `W 0x12 0x01 0x02 0x03 0x04 0x05`:

- `W np 0x12 0x02` then `R 0x12 2` returns `data: 0x03 0x04`.
- `W 0x12 0x0b 0x0c` then `R 0x12 3` returns `data: 0x0b 0x0c 0xff`
  (each write resets the file to `0xFF` first).
