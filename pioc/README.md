# pioc/ - the PIOC blobs + their toolchain

The PIOC (WCH "eMCU", a RISC8B coprocessor) runs a tiny program we write in its
own assembly. It has 2 bidirectional IO pins (IO0=PC18, IO1=PC19), a 33-byte
data-register file shared with the CPU (`SFR_DATA_REG0..31`, host-readable while
it runs), edge-wait (`WAITB`) and bit instructions. This folder isolates that
flow so the rest of the firmware just `#include`s a generated C blob.

Three PIOC blobs live here: two passive capture engines and one active MDIO master:

- **`clocked_sniffer`**: follows an external clock (MDC/SCLK) and samples the data
  line at each edge (MDIO/SPI). Drained by `ClockedSniffer` in `../src/`.
- **`rle_sniffer`**: no clock line: run-length-encodes the data line's transitions
  for unclocked / asynchronous NRZ buses (CAN/DMX). Drained by `RleSniffer`; the host recovers timing.
- **`mdio_master`**: actively drives MDC/MDIO for Clause-22 register access. Drained
  by `MdioMaster`; see `../docs/mdio-master.md`.

## Assembling a blob

`assemble.py` is the assembler. It reads a PIOC `.ASM` file and outputs the C header
that the firmware includes. The `_inc.h` file is not magic: it is just the assembled
program as 16-bit instruction words, stored as little-endian byte pairs in a C array.

The WCH assembler is Windows-only, so this repo keeps a tiny native assembler instead:

```
python3 assemble.py                  # assemble clocked_sniffer.ASM and check the committed header
python3 assemble.py foo.ASM --write  # (re)generate foo_inc.h from foo.ASM
```

By default it does **not** rewrite files: it assembles the `.ASM` in memory and compares
the result with the committed `_inc.h`. `MATCH` simply means "the checked-in generated
header matches the source". Use `--write` when you actually want to regenerate the header.

Scope: the opcode table only contains the instructions we use here and whose encodings
are known. If you use a new mnemonic, `assemble.py` fails instead of guessing.

**New instruction?** Assemble it once with the vendor tool (`build.bat` →
`WASM53B.EXE`), read the real `C=xxxx` encoding from the generated `.LST`, add it to
`assemble.py`'s `ENC` table, and from then on the native assembler can handle it.

Tools come from `…/ch32x035/EVT/EXAM/PIOC/Tool_Manual/Tool/`; the manuals
(`PIOC-EN.pdf`, `CHRISC8B-EN.pdf`) and worked examples (UART/IIC/NEC/1-Wire) are
in the same EVT tree; the indirect-addressing ops came from `PIOC_IIC`.

## The capture engine - RING (DMA-style byte FIFO)

The eMCU never stops. It mid-bit-samples the clocked bus, packs 8 bits into a byte,
and stores it into a ring inside the data regs via the auto-incrementing indirect
port, publishing a monotonic HEAD as the LAST write. The CPU drains strictly behind
HEAD, so it never reads the slot being written: **torn-read-free with no lock and
without ever parking the eMCU.**

```
                 ┌─────────────────────────────────────────────────────┐
clock ──────────▶│ PIOC / eMCU capture loop                            │
data  ──────────▶│                                                     │
                 │ per bit:                                            │
                 │   WAITB rising → WAITB falling → BCTC IN1 → RCL acc │
                 │                    ▲                                │
                 │                    └─ sample near the bit centre    │
                 │                                                     │
                 │ per byte:                                           │
                 │   store acc through SFR_INDIR_PORT2                 │
                 │   increment/publish HEAD last                       │
                 │   wrap the indirect pointer at the FIFO size        │
                 └──────────────────────┬──────────────────────────────┘
                                        │
                                        ▼
                 ┌─────────────────────────────────────────────────────┐
                 │ DATA_REG FIFO                                       │
                 │                                                     │
                 │ slots live in the PIOC DATA_REG file                │
                 │ slot count is a power of two and divides 256        │
                 │ HEAD is monotonic and published after the byte write│
                 └──────────────────────┬──────────────────────────────┘
                                        │
                                        ▼
                 ┌─────────────────────────────────────────────────────┐
                 │ CH32 CPU drain                                      │
                 │                                                     │
                 │ avail = (HEAD - tail) & 0xFF                        │
                 │ read DATA_REG[tail & mask]                          │
                 │ advance tail behind HEAD                            │
                 │                                                     │
                 │ The CPU never reads the live slot being written.    │
                 └──────────────────────┬──────────────────────────────┘
                                        │
                                        ▼
                 ┌─────────────────────────────────────────────────────┐
                 │ ClockedSniffer                                      │
                 │                                                     │
                 │ bytes → RAM ring → USB-CDC binary records           │
                 └──────────────────────┬──────────────────────────────┘
                                        │
                                        ▼
                 ┌─────────────────────────────────────────────────────┐
                 │ Host decoder                                        │
                 │                                                     │
                 │ discard preamble → hunt ST → decode Clause-22 / MMD │
                 └─────────────────────────────────────────────────────┘
```

(The `rle_sniffer` blob is the same idea without a clock line: it counts ticks at a
level instead of sampling per edge, and uses a **30-slot** FIFO: `CNT` in `DATA_REG0`,
`HEAD` in `DATA_REG1`, the FIFO in `DATA_REG2..31` (otherwise-idle SHM bytes reclaimed).
The CPU cursor is deliberately double: `sniffTail_` is the 8-bit logical count PIOC
publishes, `sniffSlot_` the physical 0…29 position — never derive a slot from `HEAD % 30`
or `HEAD & 29`, since HEAD's 256-wrap isn't aligned with 30.)

**Why torn-read-free without a lock** (per the WCH manual + IIC example): each
single data-reg write is atomic w.r.t. the host, and publishing HEAD *after* the
byte is committed means a host that sees `HEAD=N` knows slots `0..N-1` are stable.

**Passive-tap safety:** the sniffer blobs' first instruction is `CLR SFR_PORT_DIR`,
forcing both PIOC IO pins to INPUT (`DIR=1`=output, `0`=input) — the eMCU can never
drive the bus in passive capture builds. The SPI loopback generator (`SpiGen`,
bench-only) is the *only* thing that ever drives PA5/PA7; the passive build never
configures it. The `mdio_master` blob is the explicit exception: it is an active
driver and intentionally drives PC18/PC19.

**Indirect-addressing facts:** `MOVIA k` sets `SFR_INDIR_ADDR2`; `MOVA
SFR_INDIR_PORT2` stores A then **auto-increments the pointer by 1** (group 2 only);
there's **no hardware ring-wrap** (done in software via `BTSC`); the host **cannot**
read the pointer, so the eMCU must publish HEAD into a data reg.

### Validated

- **Loopback (SPI generator):** 220-byte stream, **0 loss / 0 corruption / 0
  overflow at 0.75, 1.5, 3.0 and 6.0 MHz** — the 2-instr/bit loop ceiling is far
  above MDIO's 2.5 MHz.
- **Real bus (Linux ↔ 10BASE-T1L PHY):** lossless capture, decoded end-to-end;
  the **mid-bit sample point is correct on real silicon** (write `TA=10` / read
  `TA=00` are clean and constant — a 1-bit misalignment would scramble them).
  Measured **MDC clock ≈ 1.4 MHz**, ~1 Hz `phylib` polling.
- **RLE blob (30-slot ring @ TIM3 50 kHz):** lossless at the worst-case CAN bus, on
  silicon — **~6000 honest 8-byte frames/s** (`0x555` payload ≈ 85 % of a saturated
  1 Mbit bus), `ovf=0` / `drop=0` / no host `0xFD`, `maxGap ~19–20/30` (ceiling 24/30),
  **~500 KB/s** USB with full 64-byte packets. A real bus runs 10–40 % load, so this
  worst-case operating point is pure margin in practice.

## PIOC timing facts (reference)

Measured on silicon while de-risking the NRZ (RLE) sampling — they apply to any PIOC
timing work:

- **PIOC clock = 48 MHz** (the datasheet max is the actual free-running clock). The
  "~12 MHz" you'd deduce from MDIO throughput is misleading: there the `WAITB`s
  synchronize on MDC, so that measures synchronized throughput, not the clock. A
  non-resyncing NRZ sweep confirmed 48 MHz (`32 cycles/bit × 1.5 MHz SPI`).
- **`BCTC` takes 2 cycles** (it samples IO into the carry flag - the "almost" in the
  andelf/pioc "single cycle (almost)"); **`MOV`/`RCL`/`INC`/`JMP` = 1 cycle.** Proven
  by the sweep: the optimum sample pad lands at `2+1+pad=32`, not `1+1+pad`.
- **Sample at mid-bit, not on a clock edge**: sampling right at the edge is a fixed
  delay that drifts into the data transition at low clock rates (a deterministic 1-bit
  error). Sampling after the falling edge (~mid-bit) is robust across rate and phase.
- **Calibrate by sweep, not by a freq-meter blob**: a `BCTC` right after `WAITB` reads
  a stale state; building a few blobs with the bit-pad ±2 and testing each is reliable.

For NRZ (no edge per bit) the eMCU samples at a fixed delay after the start edge, then
at a fixed interval — which is what `rle_sniffer` does; everything above bit timing
(framing, CRC, stuffing) stays CPU/host-side, like MMD folding on the clocked path.

Future clocked-mode variants could expose the sampling policy instead of baking it
into one blob: rising edge, falling edge, DDR/both edges, and possibly a small
programmable delay before `BCTC`. Today the blob is intentionally fixed to the
validated MDIO-safe point.

## How we got here (explorations)

Each scheme below was built, tested on the SPI-loopback bench, and dropped; the
lesson is what survived into the ring. Wiring was always SPI1 as the known
generator (PA5→PC18, PA7→PC19; `GPIO_Remap_SWJ_Disable` → flash via USB bootloader).

- **Edge-count, then rolling 8/32-bit capture**: proved the basics and that a *rolling*
  shift (newest → bit0) is alignment-proof. Throughput ceiling well above 2.5 MHz.
- **Per-32-bit snapshot + IRQ**: snapshotting every 32 bits does ~12 instructions of
  work mid-stream and corrupted a bit at every boundary. **Lesson: no periodic mid-stream
  work in the capture loop.**
- **Timer0 idle-gap framing**: both timeout→reset and timeout→fall-through were dead.
  **Lesson: no usable PIOC-side idle primitive → idle/frame detection moves to the CPU.**
- **Sample point: rising vs falling**: see the mid-bit fact above. Confirmed on the real bus.
- **64-bit window, read by peek/flush**: works, but `peek` can tear and `flush` must park
  the eMCU (losing preamble). **Replaced by the ring**, which has neither problem.
