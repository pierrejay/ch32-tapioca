#!/usr/bin/env python3
"""diag_monitor.py - tail the firmware's 0xFE-framed ASCII blocks from the BINARY stream.

The device wraps every ASCII metadata block as `0xFE <ascii, starts with '['> 0xFE`
(see the README "USB wire protocol" section). Run-byte data (rle), COBS-0xFF records (clocked) and the
0xFF boundary are all skipped - this tool pulls out only the bracketed blocks:

  [MODE rle wire=2] / [MODE clocked wire=2]   - the active mode, ~1 Hz (always on)
  [T ...]                                     - telemetry, only with -D DIAG

What [T] carries depends on the active mode (both build with -D DIAG):
  rle      : [T t loopHz pktHz shortHz usbB svcUs drnCpu drnMax ramHi maxGap/SLOTS ovf txFree drop]
             - full drain telemetry incl. the TIM3 ISR cost (drnCpu) + PIOC ring margin (maxGap).
  clocked  : [T t loopHz pktHz shortHz usbB ovfPioc ovfRam ramHi txFree]
             - loop/USB throughput + overflow. No drnCpu/maxGap: the clocked path drains from
               the main loop, not a TIM3 ISR, so there's no separate drain-ISR cost to report.

A block is anchored on `0xFE[` (a real block always opens with the '[' of "[T.." or "[MODE"):
so even if we start reading INSIDE a block (the first 0xFE seen is a close), we resync on the
next real open instead of mis-pairing delimiters and printing data as garbage.

Usage:
    diag_monitor.py /dev/cu.usbmodemXXXX      # live device
    diag_monitor.py -                          # stdin
"""
import sys
import os

try:
    import tty as ttymod
except Exception:
    ttymod = None

OPEN_NEXT = 0x5B        # '[' : the byte that must follow 0xFE for a real block open
DELIM     = 0xFE


def extract(buf, emit):
    """Pull every complete 0xFE[...0xFE block out of `buf`, call emit(text), and return
       the leftover (unconsumed tail) as a new bytearray. Anchors opens on 0xFE'['."""
    i = 0
    n = len(buf)
    while True:
        # find a block open: 0xFE immediately followed by '['
        start = -1
        j = buf.find(DELIM, i)
        while j != -1:
            if j + 1 >= n:
                return bytearray(buf[j:])           # 0xFE at the very end -> keep, need next byte
            if buf[j + 1] == OPEN_NEXT:
                start = j; break
            j = buf.find(DELIM, j + 1)              # stray/close 0xFE -> skip, keep scanning
        if start == -1:
            return bytearray()                      # no open in hand; drop scanned data
        end = buf.find(DELIM, start + 1)            # closing 0xFE
        if end == -1:
            return bytearray(buf[start:])           # open seen, close not yet -> keep tail
        emit(buf[start + 1:end].decode("ascii", "replace"))
        i = end + 1


def main():
    if any(a in ("-h", "--help") for a in sys.argv[1:]):
        print(__doc__)
        sys.exit(0)
    path = sys.argv[1] if len(sys.argv) > 1 else "-"
    fd = 0 if path == "-" else os.open(path, os.O_RDONLY | os.O_NOCTTY)
    try:
        if ttymod and os.isatty(fd):
            ttymod.setraw(fd)                       # raw: don't let the tty mangle binary bytes
    except Exception:
        pass

    stats = {"blocks": 0}

    def emit(text):
        stats["blocks"] += 1
        sys.stdout.write(text + "\n"); sys.stdout.flush()

    buf = bytearray()
    seen = 0
    hinted = False
    try:
        while True:
            chunk = os.read(fd, 4096)
            if not chunk:
                break
            buf += chunk
            seen += len(chunk)
            buf = extract(buf, emit)
            if len(buf) > 65536:                    # runaway guard (no blocks): keep a small tail
                del buf[:-256]
            if not hinted and seen > 200000 and stats["blocks"] == 0:
                sys.stderr.write("[diag_monitor] %d bytes, no block yet - the device always emits "
                                 "[MODE]; if you see nothing, check the port/baud. For [T] "
                                 "telemetry the firmware must be built with -D DIAG.\n" % seen)
                sys.stderr.flush()
                hinted = True
    except KeyboardInterrupt:
        pass
    finally:
        if fd != 0:
            os.close(fd)


if __name__ == "__main__":
    main()
