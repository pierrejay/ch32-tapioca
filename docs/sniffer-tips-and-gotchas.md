# Tips & gotchas

Notes from the CH32X035/PIOC bring-up. Real-time on this chip is mostly about
*not* paying hidden latencies - and proving which latency you are actually paying.

## Firmware timing

- **Keep exactly one hard interrupt.** On CH32V, the WCH `interrupt-fast` path did not
  give us useful nested USB-vs-TIM3 preemption in practice. The stable shape is:
  TIM3 is the only time-critical ISR, at 50 kHz for RLE, and USB is polled from the
  main loop. That keeps the PIOC drain deterministic instead of occasionally being
  blocked by a CDC transaction.
- **Run the hot path from RAM, and keep it boring.** The TIM3 drain ISR, the RLE
  `service()` drain and the USB bulk-IN `pump()` live in `.highcode` (RAM). Leaving
  the hot drain in flash was enough to cripple throughput by itself: during bring-up,
  moving the drain path to RAM took a drain loop from ~1.5 kHz to ~14 kHz before the
  later USB/drain rewrites. The ISR itself only copies the PIOC DATA-register FIFO into
  a RAM ring and gets out; idle squelch, `0xFF` boundaries, metadata and USB backpressure
  all live in the main-loop `service()` path. On the validated 30-slot RLE ring,
  worst-case CAN sits around `maxGap` 19-20/30, below the overflow ceiling.
- **Skip expensive SDK helpers in the ISR.** Clearing TIM3 directly (`INTFR = ~flag`)
  avoids `TIM_GetITStatus` / `TIM_ClearITPendingBit` calls that cost a few microseconds
  each when not inlined. At a 50 kHz drain rate, those helper calls alone can eat most
  of the CPU budget; the bad version measured ~95% CPU in the drain path and starved the
  main loop to ~800 Hz.

## USB throughput

- **USB throughput was an `INT_BUSY` cadence problem, not a host-driver problem.** The
  USBFS engine auto-NAKs while `UIF_TRANSFER` is pending. If you only clear/re-arm it
  once per main-loop pass, you get the "one packet per loop" ceiling. Interleaving
  `usb_.pump()` every 16 drained bytes shrinks that freeze window from a whole loop
  (~180 us during the CAN bring-up) to roughly the drain chunk time (~10 us), taking the
  stream near the ~500 KB/s device-side ceiling. Beyond that, CPU is the bottleneck.

## RLE capture

- **For RLE, the cliff is the publish blind window, not the counting loop.** The PIOC
  can count a level quickly, but every transition has a short store/publish sequence
  where it is not looking at the pin. At CAN 1 Mbps, one bit is 48 CPU cycles and this
  is still inside the green zone; at higher transition densities it is the thing that
  makes runs merge. The host's per-level clock recovery then absorbs the fixed bias that
  remains.
- **RLE timing is per-level.** The HIGH and LOW counting loops are not the same length
  (currently 9 and 8 CPU cycles/tick, measured by `test_rle_tick`). The device advertises
  that in `[MODE rle ... thi=9 tlo=8 fcpu=48000000]`; hosts should fit LOW/HIGH runs
  separately and convert to absolute baud from those slopes, not from a single average.

## PIOC specifics

The PIOC-specific gotchas live in [../pioc/README.md](../pioc/README.md), including:

- mid-bit sampling;
- the real 48 MHz PIOC clock;
- `BCTC` taking 2 cycles;
- 30-slot non-power-of-two rings;
- torn-read-free HEAD publishing.

## Performance envelope

Roughly:

- CAN 1 Mbps: >8000 fps typical;
- DMX: 44 Hz / 250 kbps is trivial;
- clocked capture: clean to ~3 MHz.

## Scope and blind spots

This is not a full logic analyzer. It is an experiment in pushing a tiny MCU as
far as it can go on useful, mainstream 1- and 2-wire buses - and seeing where that
becomes a practical embedded building block.

The interesting niche: when a product already needs a small USB/UART/I2C/debug-side
bridge, a CH32X035-class part can add protocol-aware capture/control for peanuts, in a
3x3 mm package, instead of reaching for a larger RP2040/RP2350 MCU or an FPGA.

Known limits:

- `clocked` has one fixed sampling phase today; doesn't yet cover rising,
  falling, DDR/both-edge, and small sampling delays.
- `clocked` captures only raw clock+data (no CS/direction pin)
- `rle` is for clean NRZ timing; Manchester/PWM/biphase-style signals may need dedicated
  recovery and might not fit the current idle model.
- Long-idle squelch is tuned for the validated buses; very slow protocols may need
  retuning or raw mode.
