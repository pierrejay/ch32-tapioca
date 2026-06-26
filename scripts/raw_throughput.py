#!/usr/bin/env python3
"""raw_throughput.py - measure RAW host read throughput from the CDC device.

A tight os.read() loop with NO parsing/processing, printing received KB/s each
second. Answers ONE question: is the host READ PATTERN the cap, or the device?

A decoder that does real work per chunk (scan for 0xFE, decode) can fall behind, fill
the kernel CDC buffer, stall the driver's IN polling, and throttle throughput host-side
- NOT a device limit. This reader does the minimum (read + count + discard), so it's the
fastest a Python host reader can pull: compare its KB/s against the device's own usbB
telemetry; if they match, the host reader isn't the bottleneck.

    raw_throughput.py /dev/cu.usbmodemXXXX
Ctrl-C to stop; prints the overall average.
"""
import sys
import os
import time

try:
    import tty as ttymod
except Exception:
    ttymod = None


def main():
    if any(a in ("-h", "--help") for a in sys.argv[1:]):
        print(__doc__)
        sys.exit(0)
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    fd = os.open(sys.argv[1], os.O_RDONLY | os.O_NOCTTY)
    try:
        if ttymod and os.isatty(fd):
            ttymod.setraw(fd)            # raw: don't let the tty mangle/limit binary bytes
    except Exception:
        pass

    total = 0          # bytes since start
    window = 0         # bytes since last 1s print
    t0 = time.time()
    last = t0
    try:
        while True:
            chunk = os.read(fd, 65536)   # one big read, minimal syscall overhead
            if not chunk:
                break
            n = len(chunk)
            total += n
            window += n
            now = time.time()
            if now - last >= 1.0:
                kbps = window / (now - last) / 1000.0
                sys.stderr.write("%8.1f KB/s   (%.1f MB total)\n" % (kbps, total / 1e6))
                sys.stderr.flush()
                window = 0
                last = now
    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)
        dt = time.time() - t0
        if dt > 0:
            sys.stderr.write("\n--- avg %.1f KB/s over %.1fs (%.1f MB) ---\n"
                             % (total / dt / 1000.0, dt, total / 1e6))


if __name__ == "__main__":
    main()
