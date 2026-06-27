# USB wire API and capture pipeline

- [Capture pipeline](#capture-pipeline)
- [USB wire protocol](#usb-wire-protocol)
- [Device → host](#device--host)
- [RLE mode: raw run-byte stream](#rle-mode-raw-run-byte-stream)
- [`clocked` mode: COBS-`0xFF` records](#clocked-mode-cobs-0xff-records)
- [Host → device](#host--device)
- [Diagnostics](#diagnostics)

## Capture pipeline

The PIOC eMCU is the only truly deterministic sampler in the system. Everything
after it is buffering and formatting: keep the tiny PIOC FIFO from overflowing,
absorb USB endpoint jitter, and report loss explicitly if any stage falls behind.

```text
bus pins
  -> PIOC capture blob
  -> PIOC DATA_REG FIFO
  -> RAM staging ring
  -> UsbCdc TX ring
  -> 64-byte USB endpoint buffer
  -> host decoder
```

There are two datapaths because the timing constraints are different.

### RLE / timed datapath

Used for unclocked or self-clocked NRZ buses such as CAN and DMX.

```text
data pin
  -> PIOC RLE blob: measure how long the line stays at each level
  -> 30-slot PIOC FIFO
  -> TIM3 drain ISR @ 50 kHz: dumb copy into the 2 KB RAM ring
  -> main-loop service(): idle squelch + loss markers + copy into USB TX ring
  -> USB endpoint buffer
  -> host: reconstruct timing, recover bits, decode CAN/DMX/etc.
```

Important design points:

- The hot TIM3 ISR does not touch USB and does not do protocol work. It only keeps
  the PIOC FIFO drained.
- Long idle runs are allowed into the RAM ring; the main loop performs the idle
  squelch and emits `0xFF` boundaries.
- The RAM ring absorbs USB jitter. If it fills, the firmware emits a `0xFD` loss
  marker later so the host never silently decodes across missing bytes.
- The RLE RAM ring is intentionally hand-rolled in the hot path. A generic `Ring<>`
  version was benchmarked with `objdump`; the clean API added ISR prologue/branches
  unless it exposed low-level producer cursors, which defeated the clarity win.

### Clocked datapath

Used for externally clocked buses such as MDIO or SPI-style captures.

```text
clock + data pins
  -> PIOC clocked blob: sample data at the configured clock phase
  -> PIOC DATA_REG FIFO
  -> main-loop service(): group bytes into timestamped bursts
  -> RAM staging ring containing COBS-framed records
  -> UsbCdc TX ring
  -> USB endpoint buffer
  -> host: discard preamble, align transactions, decode MDIO/SPI/etc.
```

Important design points:

- There is no 50 kHz drain ISR in the clocked datapath; the main loop drains the
  PIOC FIFO and records burst timing.
- Completed records are framed before they enter the RAM staging ring, so they can
  be split across USB packets without ambiguity.
- The clocked RAM staging ring uses the generic `Ring<>` abstraction. Benchmarking
  showed negligible size impact and a cleaner contiguous read path.

### Why not let USB read directly from the capture ring?

The CH32 USB device endpoint wants a concrete 64-byte endpoint buffer. The
firmware therefore keeps a software USB TX ring in front of that endpoint buffer.
That extra ring is deliberate:

- USB/host service timing is jittery compared with the PIOC capture timing.
- The RLE ISR must stay tiny; adding cross-stage ownership logic there would move
  complexity into the preemptive hot path.
- Copies from the RAM ring to the USB TX ring happen in normal main-loop context,
  where they are cheaper than missed PIOC drains.
- If the host cannot keep up, the firmware can apply backpressure or mark loss at
  a clean boundary instead of corrupting an in-flight capture.

In other words: the extra copy is not elegant for its own sake, but it keeps the
hard-real-time path boring. Current CAN validation has plenty of headroom, so this
is a deliberate YAGNI trade-off rather than an accidental abstraction leak.

## USB wire protocol

The device speaks a small binary envelope over USB-CDC. One byte, `0xFF`, is the
universal segment boundary. In raw `rle`, bytes `0x81..0xFF` are free for sentinels
because run data only uses `0x00..0x80`; in `clocked`, `0xFD` / `0xFE` are control
bytes only when they start a `0xFF`-delimited segment.

### Device → host

The active mode is self-describing: the device periodically emits a `[MODE ...]`
metadata block. The `wire=2` field is a format-version tag so hosts can reject or
adapt to future envelope changes.

#### Common sentinel bytes

| byte / block | meaning |
|---|---|
| `0xFF` | segment boundary / hard cut |
| `0xFD` | capture loss marker: bytes were dropped here; do not decode across it |
| `0xFE '[' ascii... ']' 0xFE` | metadata block, e.g. `[MODE ...]` or `[T ...]` diagnostics |

The metadata segment itself is:

```text
FE "[MODE clocked wire=2]" FE FF
FE "[MODE rle wire=2 thi=9 tlo=8 fcpu=48000000]" FE FF
```

Timing:

- At mode start, both datapaths emit an immediate `[MODE ...]` marker.
- In `clocked`, the marker is queued as a standalone `0xFF`-terminated segment,
  between records, then repeated about once per second.
- In `rle`, the periodic marker is only emitted after an idle boundary, when no
  run-byte frame is in flight. On the wire this commonly looks like:

```text
... FF  FE "[MODE rle ...]" FE FF  ...
    │   └─ metadata segment ─┘
    └─ idle boundary that closed the previous RLE frame
```

So the two stream shapes are:

```text
clocked:
  FF  FE "[MODE clocked...]" FE FF <COBS-record> FF <COBS-record> FF  ...
      └──── mode metadata ────┘   └── record ───┘  └── record ───┘

rle:
  FF  FE "[MODE rle ...]" FE FF <run-bytes> FF  <run-bytes> FF  ...
      └── mode metadata ───┘    └─ frame ─┘     └─ frame ─┘
```

With `-D DIAG`, `[T ...]` diagnostics use the same `0xFE ... 0xFE` metadata shape.
In `clocked` they are standalone `0xFF`-terminated segments. In `rle` they may be
inserted inline in the run-byte stream, without adding a `0xFF`; hosts strip the
paired `0xFE ... 0xFE` block before reconstructing runs.

#### RLE mode: raw run-byte stream

RLE mode is not record-framed and does not use COBS. It is a raw stream of
alternating run durations plus sentinels.

How to read run-bytes:

- Bytes `0x00..0x7F` end the current run. Their value is the last chunk of the run.
- Byte `0x80` means: “this same run is still going; add 128 ticks and keep the same level”.
- So a run is: zero or more `0x80` bytes, followed by one byte `< 0x80`.
- Total run length = `128 × number_of_0x80_bytes + final_byte`.

Examples:

| bytes | decoded run length | what happens next |
|---|---:|---|
| `06` | 6 ticks | run ends, level flips |
| `80 2C` | 128 + 44 = 172 ticks | run ends, level flips |
| `80 80 2C` | 128 + 128 + 44 = 300 ticks | run ends, level flips |

- The special case is long idle: after enough consecutive `0x80` bytes, the firmware emits
  `0xFF` as an idle boundary and drops the rest of that idle run. In that case the idle run
  does **not** need to end with a byte `< 0x80` on the wire.
- Run-bytes do **not** carry HIGH/LOW. The protocol decoder anchors the first
  level: CAN starts at SOF dominant/LOW; DMX idles HIGH then BREAK is LOW.
- `0xFF` is a hard boundary inserted by the firmware on long idle.
- `0xFD` is a hard boundary caused by capture loss.

Example, shortened for readability:

```text
06 0C 06 80 80 ... 80 FF
│  │  │  └───────────┘ └─ idle boundary
│  │  │       long idle run, emitted as many 0x80 continuation bytes
│  │  └─ run C = 6 ticks, then flip level
│  └──── run B = 12 ticks, then flip level
└─────── run A = 6 ticks, then flip level
```

With the default `RLE_MIN_KBPS=5`, the idle squelch threshold is
`CAP_IDLE_K=150`: the firmware forwards the first 149 idle `0x80` bytes, emits
one `0xFF` instead of the 150th, then drops the rest of that idle run until the
line moves again. So `0xFF` means “idle boundary”, not “the previous run ended
exactly here”.

#### `clocked` mode: COBS-`0xFF` records

`clocked` mode is record-based. Every data record is:

```text
COBS-FF(raw_record) FF
```

The raw record layout is little-endian:

| offset | field | bytes in the example | meaning |
|---:|---|---|---|
| 0 | `type` | `01` | sampled-data record |
| 1..4 | `t_us` | `e8 03 00 00` | start timestamp = 1000 µs |
| 5..6 | `dur_us` | `c8 00` | burst duration = 200 µs |
| 7..8 | `onset_us` | `26 00` | time to the early clock-estimation point = 38 µs |
| 9 | `flags` | `00` | bit0 PIOC overflow, bit1 RAM overflow, bit2 continued |
| 10..11 | `n` | `02 00` | payload length = 2 bytes |
| 12.. | `payload` | `a5 5a` | clock-sampled data bytes |

That example record before COBS is:

```text
01 e8 03 00 00 c8 00 26 00 00 02 00 a5 5a
```

Because this particular raw record contains no `0xFF`, the COBS-`0xFF` encoding is
just one prefix byte plus the payload, then the boundary:

```text
0f 01 e8 03 00 00 c8 00 26 00 00 02 00 a5 5a FF
```

If the sampled payload contains `0xFF`, COBS-`0xFF` escapes it so `0xFF` remains
only a segment boundary. If a burst exceeds the firmware record cap, `flags&0x04`
marks the record as `continued`; the host joins continued records before protocol
decode.

### Host → device

| line | meaning |
|---|---|
| `!mode rle\n` / `!mode clocked\n` | switch the capture mode (strict parse: a malformed line is ignored, so line noise can't trigger a reconfig) |

The host owns the protocol↔mode mapping (CAN/DMX → RLE, MDIO/SPI → clocked) and all
decoding; the device only knows the two modes. (The browser UI labels the RLE mode
**"timed"** for humans; on the wire it's `rle`.)

### Diagnostics

Built with `-D DIAG`, the device adds a `[T …]` telemetry block
(~1 Hz) inside the same `0xFE`-framed envelope - **off by default** so the capture stream
stays clean. The fields differ by mode:

- **RLE** reports the full drain path including the
  TIM3 ISR cost (`drnCpu`) and PIOC-ring margin (`maxGap`)
- **clocked** reports loop/USB
  throughput and overflow (no `drnCpu`/`maxGap` - it drains from the main loop, not an ISR).

Watch it live with:

```sh
python3 scripts/diag_monitor.py /dev/cu.usbmodemXXXX
```
(it also prints the `[MODE …]` heartbeat, which is always on). The `rle_sniffer` / `clocked_sniffer` build envs enable `DIAG` by default for exactly this bus-revalidation use.
