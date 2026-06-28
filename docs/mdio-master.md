# MDIO master driver

Active-drive MDIO master for the CH32X035 (Tapioca). Where the sniffer is a passive,
protocol-agnostic capture engine, the driver actively drives the bus to communicate
with an Ethernet PHY.

Status: **functional Clause-22 master.** Clause 45 will be added next.

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
./mdioctl print 1
```

The `mdio_master_stub` environment builds the same command layer with a canned backend 
and no PIOC access, which keeps the ASCII parser/formatter testable without hardware.

## Wire API — ASCII line protocol, both directions

No throughput is needed (one tiny synchronous transaction at a time), so we drop the binary
COBS/0xFF envelope entirely: the only things that matter are useability and
debuggability, and ASCII wins both. You can drive the bus by hand from any serial terminal;
boot text is just lines the host ignores; USB-CDC already gives CRC + retransmit, so no
application checksum. Grammar modelled on `phytool` (instant familiarity for PHY folks);
Clause 45 would add a `<phy>:<dev>/<reg>` path form.

```
TX  (host -> device)
  !read  <phy>/<reg>            e.g.  !read 1/4
  !write <phy>/<reg> <val16>    e.g.  !write 1/4 0x1A2B
  !print <phy>                  bulk-read regs 0..31

RX  (device -> host)            request-echoed -> self-correlating
  read  <phy>/<reg> 0xVAL       e.g.  read 1/4 0x1A2B
  write <phy>/<reg> ok
  <verb> <phy>/<reg> err <why>  e.g.  read 1/4 err noresp
```

- **Numbers are base-0**: `4` or `0x4`, `6699` or `0x1A2B` (the CPU parses both). `phy`/`reg`
  are 0..31 (5-bit), `val` is 0..0xFFFF (16-bit).
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

- **CPU**: parse the ASCII command, assemble the Clause-22 bit frame, write the TX mailbox,
  kick the blob, poll for done, read the RX mailbox, format the response.
- **PIOC blob** (`mdio_master.ASM`, SPI-master-like): generate MDC, drive the command bits,
  handle the turnaround (read: release + sample 16 on edges; write: drive 16), raise the
  TA-no-PHY flag. Executes a whole frame atomically (~64 clocks) then signals done.

No PIOC ring buffer here (unlike the sniffer): strictly one transaction in flight, so two small
fixed mailboxes suffice — TX descriptor (frame bits + op + bit-count) and RX descriptor
(data16 + status).
