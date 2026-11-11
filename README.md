# i2c-core

Protocol specification, host tooling, and shared conformance tests for I2C bridge firmware.

## Repository contents

- `docs/protocol.md` — normative `session_v1` command, response, segmentation, and status contract
- `docs/target_model.md` — normative target-side register-file behavior
- `docs/transaction_layer.md` — draft: how a participant handles a transfer on the bus (retries, budgets, access policy, hold limits, recovery)
- `docs/platform_capabilities.md` — informative: what each platform documents, implements and has verified in hardware for the transaction layer, and the hardware tests still needed
- `docs/diagrams/` — PlantUML sequence diagram of the transaction-layer behaviour (`sd_i2c_behavior.puml`), simulator figures (`policy_*.svg`), plus SCL/SDA timing diagrams (`td_*.svg`) for write, read and mem_read, coloured by the driving device
- `firmware/txn/` — portable C transaction layer (queue, budget `F`, retries, report; baseline access policy) compiled into the bridge firmware; `tests/test_txn.c` runs it on the host against a fake platform
- `host/i2c_test_host/` — TCP line backends, protocol adapters, peer loading, and DUT bring-up
- `host/tests/` — host-only unit tests and hardware conformance tests
- `host/devices/peers_2_stms.yaml` — default rig manifest: two STM bridges, addresses `0x65` and `0x66`
- `tools/win_tester_bridge.py` — Windows USB-serial-to-TCP bridge for the Pico tester
- `tools/saleae_i2c_hla.py` — Saleae Logic 2 High Level Analyzer for I2C traffic
- `tools/watch_tcp` — raw TCP traffic monitor
- `tools/i2c_timing_diagrams.py` — regenerates the `docs/diagrams/td_*.svg` timing diagrams
- `tools/bus_policy_sim.py` — simulates four multi-controller bus access policies and writes `docs/diagrams/policy_*.svg`; `--check` runs its self-checks

Firmware is built in sibling repositories:

- `../stm32u375_bridge` — STM bridge over RTT (`session_v1`)

The STM bridge compiles `firmware/txn` from this repository, found as the sibling `../i2c-core`.

## Supported environment

Run firmware builds, host tooling, and tests from WSL.

Physical debug probes and the optional Pico tester are accessed through services running on Windows:

- SEGGER J-Link Remote Server for debug probes
- `tools/win_tester_bridge.py` for the Pico tester

## Hardware setup

The rig is a single I2C bus shared by two STM32U375 boards (one per STM bridge) and a Raspberry Pi Pico 2 (tester). Jumper wires are sufficient:

| Signal | Each STM32U375 board | Pico 2 tester |
| --- | --- | --- |
| SCL | `PG14` | `GP6` |
| SDA | `PG13` | `GP7` |
| GND | `GND` | `GND` |

- Add one pull-up resistor from SCL and one from SDA to 3.3 V, about 1 kΩ. The bridges run the bus at 1 MHz and neither participant enables internal pull-ups. Keep the wires short.
- Use 3.3 V I/O on every participant. On the STM32U375, `PG13` and `PG14` are supplied from `VDDIO2`.
- Power each STM32U375 board and connect a J-Link to its SWD port. The host reaches the bridge over RTT through this probe.
- Connect the Pico 2 by USB. The hardware tests hold and release the bus by driving `GP6` and `GP7` as open-drain outputs.
- Optionally connect a Saleae logic analyzer to SCL, SDA and GND.

The bridge firmware was developed on a custom STM32U375 board. Apart from the I2C pins, it expects a 32.768 kHz clock on `OSC32_IN` (LSE bypass) and drives `PA11` high at start-up.

### Pico tester

Flash the Raspberry Pi Pico 2 by holding BOOTSEL while connecting it by USB, then copy `tools/pico_tester/firmware/pico_serial_to_i2c.uf2` to the mounted RPI-RP2 drive.

### Addresses and network endpoints

| Participant | I2C address | Endpoint | Purpose |
| --- | ---: | --- | --- |
| STM | `0x65` | `wsl-host:19021` | J-Link RTT/TCP endpoint used by the host |
| STM 2 | `0x66` | `wsl-host:19122` | J-Link RTT/TCP endpoint of the second STM bridge |
| Pico tester | `0x55` | `wsl-host:7000` | Bus hold/release and arbitration control |

The default manifest `peers_2_stms.yaml` uses two STM bridges, `0x65` and `0x66`.

In the arbitration tests, each controller targets the other DUT. The controller transmitting the lower target address wins.

## Windows rig services

WSL reaches the debug probes and optional serial tester through Windows-side services.

For standard WSL NAT networking, set Windows-side host fields (`bringup.host_ip` and the tester's `tcp.host`) to `wsl-host`. The host resolves this value from WSL's lowest-metric default route each time it loads the manifest, avoiding a machine-specific IP address. With WSL mirrored networking, use `localhost` instead. Literal hostnames and IP addresses are also accepted.

### J-Link Remote Servers

The J-Link installation inside WSL provides `JLinkGDBServerCLExe` and its target device database. STM32U3 support requires J-Link V8.16 or newer; V8.26 is known to work with `STM32U375RG`. Older releases such as V8.10 fail during device lookup with an error similar to `Failed to get index for device STM32U375RG`.

Updating the Windows J-Link Remote Server alone does not update the WSL device database. Download the Linux x86_64 DEB from the [SEGGER J-Link downloads](https://www.segger.com/downloads/jlink/) page, then install and verify it inside WSL:

```sh
sudo apt install ./JLink_Linux_V*_x86_64.deb
hash -r
JLinkGDBServerCLExe -version
```

Start one SEGGER J-Link Remote Server for each probe before running the hardware suite. Select each probe by its USB serial number and expose it on the corresponding listening port:

| DUT | Example J-Link USB serial | Remote Server port |
| --- | ---: | ---: |
| STM | `50101802` | `19020` |
| STM 2 | — | `19025` |

The serial numbers identify the current rig and may differ on another setup. Keep the manifest ports synchronized with the ports configured in the J-Link Remote Servers:

- STM: `bringup.host_ip=172.24.48.1`, `bringup.jlink_remote_port=19020`
- STM 2: `bringup.host_ip=172.24.48.1`, `bringup.jlink_remote_port=19025`

The Python bring-up code connects to these remote servers and starts the per-DUT J-Link GDB servers. Those GDB servers expose RTT to the host on `localhost:19021` for the first and `localhost:19122` for the second STM.

### Pico tester bridge

Run the tester bridge on Windows, replacing the COM port and baud rate as required:

```powershell
uv run --with pyserial python tools/win_tester_bridge.py COM21 115200 --port 7000
```

The bridge listens on TCP port `7000` and forwards bytes without interpreting the tester protocol.

Before running the suite, verify that all three Windows-side connections are available:

- STM J-Link Remote Server on `wsl-host:19020`
- second STM J-Link Remote Server on `wsl-host:19025`
- Pico tester bridge on `wsl-host:7000`

## Prerequisites

### WSL

Install:

- CMake
- Ninja
- Arm GNU Toolchain, including `arm-none-eabi-gdb`
- SEGGER J-Link tools, including `JLinkGDBServerCLExe`
- Python 3.10 or newer
- GCC for the host, to build the `firmware/txn` unit tests
- `uv`
- Wireshark/TShark and `xxd` for optional transport monitoring

Install the host package from the repository root:

```sh
uv venv
uv pip install -e host
```

### Windows

Install:

- SEGGER J-Link Remote Server
- `uv`
- `pyserial` for the Pico tester bridge

## Build the DUT firmware

```bash
(
  cd ../stm32u375_bridge
  cmake -S . -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
  cmake --build build
)

```

The ELF paths in `host/devices/*.yaml` are resolved relative to the manifest directory and must match the build outputs. The ELF parent directory is used as the process working directory.

## Run the tests

### Host-only tests

These tests use fake backends and require no hardware. `unit_tests/test_txn.py` builds and runs the C unit tests of `firmware/txn` with the host GCC:

```sh
cd host
python -m pytest -q unit_tests
```

### Full hardware suite

Before running the suite:

1. Connect both J-Link probes and the Pico tester.
2. Start both Windows J-Link Remote Servers.
3. Start the Windows Pico tester bridge.
4. Build both DUT firmware images.

Run:

```sh
cd host
python -m pytest -v --tb=short tests
```

`tests/conftest.py` manages DUT bring-up. It:

1. Starts each J-Link GDB server.
2. Loads the configured ELF.
3. Waits for the readiness banner.
4. Runs the tests.
5. Tears down the managed processes.

All hardware tests use the default manifest, which is the first item in `MANIFESTS` within `conftest.py`.

`test_arbitration.py` is marked `both_manifests` and runs against every manifest in `MANIFESTS`.

Select one manifest for an isolated hardware campaign with
`I2C_TEST_MANIFEST`, for example:

```sh
I2C_TEST_MANIFEST=devices/peers_2_stms.yaml python -m pytest -v --tb=short tests
```

`test_runtime_address.py` moves bridges to other addresses with `ADDR` (see `docs/protocol.md`) and restores the manifest address before each test ends. It skips a bridge whose firmware does not advertise `ADDR` in `INFO commands=`.

## Observe traffic

### I2C traffic

`tools/saleae_i2c_hla.py` is a Saleae Logic 2 High Level Analyzer that provides terminal-friendly I2C transaction output.

### TCP traffic

`tools/watch_tcp` displays raw TCP payload exchanged with a DUT or tester.

Run it in separate WSL terminals using ports from the local manifest:

```sh
tools/watch_tcp lo <dut-rtt-port>
tools/watch_tcp any <tester-port>
```

Pass `--keep-ansi` as the third argument to preserve terminal escape sequences:

```sh
tools/watch_tcp lo <dut-rtt-port> --keep-ansi
```
