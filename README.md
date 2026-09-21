# can

CAN 2.0B controller.

![maturity](https://img.shields.io/badge/maturity-simulated-yellow) ![license](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0%20OR%20MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`xirang`](https://github.com/Tape-Out/xirang). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Simulated. The IP is one Classical CAN node that follows the Bosch CAN Specification 2.0 Part B. It drives `can_tx` and reads `can_rx`; the transceiver (a TJA1050 or similar) is off chip.

It sends and receives standard and extended data and remote frames. It covers bit stuffing, CRC-15, acknowledgement, and the five error types with active and passive error flags. All twelve fault-confinement rules are implemented, together with overload conditions 2 and 3, intermission, suspend transmission, and bus-off with recovery after 128 × 11 recessive bits. Bit timing has a prescaler, hard synchronization and resynchronization with a programmable jump width. A lost arbitration or a failed frame is retransmitted automatically.

`CanFrame.bs` holds the frame layout in Bluespec Haskell. The field positions are written twice, once to lay a frame out for sending and once to take a received frame apart, and at compile time a few sample frames are sent through both; if they disagree, the build stops. `CanFault.bs` holds the fault-confinement rules, one constructor and one equation per rule, and derives error-active, error-passive and bus-off from the two counters. `Can.bsv` is the bit-timing and MAC engine. CRC-15 comes from `Gf2` in `hwcore`.

The testbench puts two controllers, a scripted node and a fault injector on a wired-AND bus. It computes every frame's bit stream in Python and self-checks its CRC-15 against the RevEng catalogue first. It checks:

- reception and acknowledgement of standard, extended and remote frames;
- a corrupt CRC, a dominant CRC delimiter and six equal bits, each raising its error flag on the right bit with the right error code and receive error count;
- the acceptance filter;
- following a node that runs 0.5% fast;
- the transmitted bit stream, bit by bit;
- arbitration with automatic retransmission;
- a forced stuff bit;
- unacknowledged retries, which must stop at a transmit error count of 128 with the right recessive gaps before each retry;
- bus-off and its recovery time.

## Registers

| Offset | Register | Contents |
| :--: | :--: | :-- |
| 0x00 | `ctrl` | `en`, `ien` |
| 0x04 | `btr` | `brp`, `ts1` (PROP_SEG + PHASE_SEG1), `ts2`, `sjw`, each minus one |
| 0x08 | `status` | fault-confinement state, pending transmission, bus idle, last error code |
| 0x0C | `errcnt` | transmit and receive error counts |
| 0x10–0x1C | `txid`, `txdlc`, `txd0`, `txd1` | frame to send |
| 0x20 | `cmd` | `tx`, `abort` |
| 0x24–0x30 | `rxid`, `rxdlc`, `rxd0`, `rxd1` | last frame received |
| 0x34, 0x38 | `fid`, `fmask` | acceptance filter (feature `filter`) |
| 0x3C | `events` | `txok`, `rxok`, `err`, `passive`, `busoff`, `arblost`, `rxover`; write one to clear |

A standard identifier sits in bits 28:18 of the identifier registers.

## Parameters

| Feature | Default | Meaning |
| :--: | :--: | :-- |
| `filter` | on | one identifier and mask pair; off accepts every frame |

CAN FD, overload condition 1, sleep and wake-up, listen-only and loopback modes, several buffers and FIFOs, time stamps, single-shot transmission and triple sampling are not implemented. Bosch's CAN protocol licence page lists CAN FD, CAN FD Light, TTCAN and CAN XL; check the licensing for your product before tape-out.

## Specification sources

The specifications this IP is implemented against, with their links, digests and the clause-by-clause comparison, are kept on the [`spec` branch](https://github.com/Tape-Out/can/tree/spec).

## License

任选其一：

- [MIT](LICENSE-MIT)
- [Apache 2.0](LICENSE-APACHE)
- [木兰宽松许可证 第2版](LICENSE-MULAN)

`SPDX-License-Identifier: MIT OR Apache-2.0 OR MulanPSL-2.0`

除非另行说明，你提交的贡献按上述三者同时授权，不附加其他条件。
