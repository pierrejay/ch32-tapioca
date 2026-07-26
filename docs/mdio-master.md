# MDIO master driver

Active-drive MDIO master for the CH32X035 (Tapioca). Where the sniffer is a passive,
protocol-agnostic capture engine, the driver actively drives the bus to communicate
with an Ethernet PHY.

Status: **functional Clause-22 master with MMD / Clause-45-style access through
the standard REGCR/ADDAR indirect method.** Native Clause-45 frames can be added
later behind the same user-facing syntax.

## Why protocol-specific firmware (not the sniffer's agnostic model)

The sniffer ships raw transitions and decodes protocol **host-side**. That works because
*capturing is passive*: the PIOC records, it never decides.

Driving an MDIO **read** is the opposite, and it's a structural wall, not an optimisation:
a Clause-22 frame is `PRE(32) · ST · OP · PHYAD(5) · REGAD(5) · TA(2) · DATA(16)`. At the
**turnaround (TA)** the remote PHY takes the data line, so we must (1) stop driving MDIO,
(2) keep generating MDC (the clock never stops, SPI-style), and (3) sample the 16 returned
bits on the right clock edges. A generic "replay these samples" engine *cannot* do this — it
has no idea *when* to flip direction or read, because those instants are defined by the protocol.

So emulating protocols **requires** protocol awareness in the firmware. (Bandwidth — e.g. shipping 512
structured DMX bytes vs a full waveform — reinforces it, but the turnaround is what decides.)

"Protocol-specific" does **not** mean a different codebase: we keep the shared plumbing (UsbCdc,
printf-over-CDC, LED, PIOC blob loader, the `assemble.py` toolchain) and specialise only (a) the
blob and (b) the protocol layer. One PlatformIO env per driver, exactly like the sniffer's
`rle_sniffer` / `clocked_sniffer`.

The firmware split is deliberately two-layered: `Mdio::Master` owns/arbitrates the
physical MDIO bus, while `Mdio::UsbBridge` is only one client that exposes a USB
pass-through command protocol. That keeps the path open for a later supervision
build where the same chip can both pass host commands through to one or more PHYs
and run firmware-side configuration sequences at boot or during operation. Those
internal sequences can submit the same read/write requests as USB; the master
serializes access and answers busy instead of letting two clients race the PIOC
mailbox.

## Build and use

Pin assignment is the same as the sniffer:
  - MDIO pin (data) → `PC19` (PIOC IO1)
  - MDC pin (clock) → `PC18` (PIOC IO0)
  - common ground

```sh
pio run -e mdio_master -t upload      # Plug board in USB bootloader mode & flash
cd app/mdioctl
export MDIO_PORT=/dev/ttyACM0         # USB device
./mdioctl read 1/2
./mdioctl write 1/4 0x01e1
./mdioctl read 1:31/0x0300
./mdioctl write --verify 0:31/0x0302 0x3e80
./mdioctl print --mmd 31 --start 0x0300 --count 16 1
./mdioctl print 1
```

`mdioctl` follows `phytool`: successful writes are silent, including
`write --verify`. The firmware line protocol still returns `write ... ok`; the
CLI consumes that response and exits `0` if the operation succeeded. With
`--verify`, success means the write was accepted, a follow-up read answered, and
the exact 16-bit readback matched the requested value. Failures print to stderr
and exit non-zero.

The `mdio_master_stub` environment builds the same USB ASCII bridge with a canned
`Mdio::Master` backend and no PIOC access, which keeps the parser/formatter path
testable without hardware.

## Wire API — ASCII line protocol, both directions

No throughput is needed (one tiny serialized transaction at a time), so we drop the
binary COBS/0xFF envelope entirely: the only things that matter are useability and
debuggability, and ASCII wins both. You can drive the bus by hand from any serial
terminal; boot text is just lines the host ignores; USB-CDC already gives CRC +
retransmit, so no application checksum. Grammar is modelled on `phytool`: C22 uses
`<phy>/<reg>`, and MMD / Clause-45-style access uses `<phy>:<mmd>/<reg>`.

```
TX  (host -> device)
  !read  <phy>/<reg>            e.g.  !read 1/4
  !write <phy>/<reg> <val16>    e.g.  !write 1/4 0x1A2B
  !read  <phy>:<mmd>/<reg16>    e.g.  !read 1:31/0x0300
  !write <phy>:<mmd>/<reg16> <val16>
  !print <phy>                  bulk-read regs 0..31

RX  (device -> host)            request-echoed -> self-correlating
  read  <phy>/<reg> 0xVAL       e.g.  read 1/4 0x1A2B
  write <phy>/<reg> ok
  read  <phy>:<mmd>/0xREG 0xVAL e.g.  read 1:31/0x0300 0x1234
  write <phy>:<mmd>/0xREG ok
  <verb> <phy>/<reg> err <why>  e.g.  read 1/4 err noresp
  err invalid                     malformed or over-long command
```

- **Numbers are base-0**: `4` or `0x4`, `6699` or `0x1A2B` (the CPU parses both).
  C22 `phy`/`reg` and MMD `mmd` are 0..31; MMD `reg` and `val` are 16-bit.
- **Invalid input never reaches the bus**: malformed commands return `err invalid`.
  If a line exceeds the 48-byte receive buffer, the bridge rejects the entire line
  and ignores every remaining byte through the next CR/LF; a valid-looking suffix
  can therefore never become a second command.
- **MMD access is currently indirect over Clause 22**: the master serializes the
  REGCR (`0x0D`) / ADDAR (`0x0E`) sequence internally, so no USB or firmware
  client can interleave a command in the middle and corrupt the MMD address latch.
- **Writes and `noresp`**: a Clause-22 write has no PHY-driven turnaround phase,
  so a raw write can only report that the master emitted the frame. `mdioctl
  write --verify` adds a readback, which can detect `noresp` and value mismatch.
- **`!print` is device-side bulk read but emits RAW values** — the same `read phy/reg 0xVAL`
  lines as a single read, so the host has one parser and we save 31 USB round-trips. Register
  **naming** (BMCR, BMSR, …) stays host-side: the device serves primitives, the host interprets
  (same split as the sniffer).
- **No-PHY detection via the TA bit**: on a read, the PHY must pull TA low. If it stays high
  (pull-up) nobody answered → `err noresp`, instead of silently returning `0xFFFF`.
- **Device-side timeout**: if the blob ever stalls, the firmware falls back to `err timeout`
  so the host is never left hanging. The host layers its own timeout on top as it likes.

## CPU <-> PIOC division

Staged, not finely interleaved (fine interleaving would re-introduce the CPU jitter we use the
PIOC to avoid):

- **USB bridge (`Mdio::UsbBridge`)**: parse ASCII commands, submit one request at a
  time, format the response, and run `!print` as a sequence of 32 reads.
- **MDIO master (`Mdio::Master`)**: owns the bus and exposes one asynchronous slot
  (`read`/`write`/`readMmd`/`writeMmd` accept or return busy; `tick` publishes
  `Done`, `NoResp`, or `Timeout` into the caller-owned response).
- **CPU/PIOC mailbox path**: assemble the Clause-22 bit frame, write the TX mailbox,
  kick the blob, poll for done from the main loop, and read the RX mailbox.
- **PIOC blob** (`mdio_master.ASM`, SPI-master-like): generate MDC, drive the command bits,
  handle the turnaround (read: release + sample 16 on edges; write: drive 16), raise the
  TA-no-PHY flag. Executes a whole frame atomically (~64 clocks) then signals done.

No PIOC ring buffer here (unlike the sniffer): the master serializes access to one
transaction in flight, so two small fixed mailboxes suffice — TX descriptor (frame
bits + op + bit-count) and RX descriptor (data16 + status).
