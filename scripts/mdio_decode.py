#!/usr/bin/env python3
"""
mdio_decode.py - host-side decoder for the device's wire=2 MDIO capture.

Reads the binary envelope (COBS-0xFF records, 0xFF boundary, 0xFD loss, 0xFE metadata),
joins cap-split bursts, decodes Clause-22 at the bit level (preamble hunt -> ST=01 ->
fields), and folds the MMD-indirect mechanism (regs 13/14) into Clause-45 MMD accesses.
Also prints throughput / polling cadence / effective MDC-rate stats. (The browser app
is the live decoder; this is for offline files / CLI / CI.)

Usage:
    mdio_decode.py capture.bin             # decode a binary capture
    cat /dev/tty.usbmodemXXXX | mdio_decode.py
    mdio_decode.py --raw capture.bin       # every raw C22 frame, no MMD folding
"""
import sys
import argparse

import mdio_lib as M


def run_binary(data, raw):
    folder = M.MmdFolder()
    stream = M.BitStream()                          # carries frames across burst cuts
    nframes = 0
    bursts = []                                     # (t_start, dur_us, nbytes)
    clk = []                                         # onset-based MDC clock estimates
    tot_bytes = 0
    ovf_pioc = ovf_ram = 0
    # join cap-split records first so frames spanning the BURST_MAX boundary survive
    for lb in M.logical_bursts(M.records_from_binary(data)):
        bursts.append((lb.t_us, lb.dur_us, len(lb.payload)))
        r = M.onset_rate_kbits(lb.onset_us)
        if r is not None:
            clk.append(r)
        tot_bytes += len(lb.payload)
        if lb.flags & M.FLAG_OVF_PIOC:
            ovf_pioc += 1
        if lb.flags & M.FLAG_OVF_RAM:
            ovf_ram += 1
        if lb.flags & (M.FLAG_OVF_PIOC | M.FLAG_OVF_RAM):
            print("    *** OVF (flags=0x%02X) ***" % lb.flags)
            folder.reset()
            stream.reset()                          # a loss seam breaks bit-contiguity
        for fr in stream.feed(M.burst_to_bits(lb.payload, lb.valid_bits)):
            nframes += 1
            emit_frame(fr, folder, raw)
        if lb.valid_bits:
            stream.reset()                          # firmware flushed a partial + restarted the
                                                    # eMCU -> next burst isn't bit-contiguous (keep
                                                    # the folder: MMD reg13/14 spans transactions)
        print("    --- burst @%.3fs  %d B  %.0fus ---"
              % (lb.t_us / 1e6, len(lb.payload), lb.dur_us))
    print_stats(bursts, tot_bytes, nframes, ovf_pioc, ovf_ram, clk)
    return nframes


def run_bits(data):
    """Dump the raw sampled bit stream per burst - for debugging capture/timing when nothing
       decodes as MDIO. A good Clause-22 frame shows a run of 1s (>=32-bit preamble) then 01
       (ST). Garbled / phase-shifted samples show no clean preamble. The bit order is the
       sniffer's sample order (MSB-first per byte), same as the normal decoder."""
    n = 0
    for lb in M.logical_bursts(M.records_from_binary(data)):
        bits = M.burst_to_bits(lb.payload, lb.valid_bits)
        s = "".join(str(b) for b in bits)
        print("burst @%.3fs  %dB  %d bits (vbits=%d):\n  %s"
              % (lb.t_us / 1e6, len(lb.payload), len(bits), lb.valid_bits, s))
        n += 1
    if not n:
        print("no burst records (sniffer captured nothing decodable as records)")
    return n


def emit_frame(fr, folder, raw):
    if raw:
        print(M.raw_line(fr))
    else:
        text, _ = folder.feed(fr) or (None, None)
        if text is not None:
            print(text)


def print_stats(bursts, tot_bytes, nframes, ovf_pioc, ovf_ram, clk=None):
    print("\n===== timing / throughput =====")
    if not bursts:
        print("no burst records")
        return
    span_us = bursts[-1][0] - bursts[0][0]
    span_s = span_us / 1e6 if span_us > 0 else 0
    # polling cadence: inter-burst start deltas
    starts = [b[0] for b in bursts]
    gaps = [(starts[i + 1] - starts[i]) / 1e6 for i in range(len(starts) - 1)]
    # effective MDC rate per burst = nbytes*8 / dur
    rates = [(n * 8) / (dur / 1e6) for (_, dur, n) in bursts if dur > 0 and n > 4]
    print("bursts            : %d over %.3f s" % (len(bursts), span_s))
    print("captured bytes    : %d  (%d C22 frames)" % (tot_bytes, nframes))
    if span_s > 0:
        print("avg throughput    : %.0f B/s (%.0f bit/s incl. idle)"
              % (tot_bytes / span_s, tot_bytes * 8 / span_s))
    if gaps:
        period = sum(gaps) / len(gaps)
        print("polling cadence   : %.1f ms avg between bursts (%.2f Hz)"
              % (period * 1e3, 1.0 / period if period else 0))
    if rates:
        print("MDC effective     : %.0f .. %.0f kbit/s (median %.0f) - diluted by idle"
              % (min(rates) / 1e3, max(rates) / 1e3,
                 sorted(rates)[len(rates) // 2] / 1e3))
    if clk:
        sc = sorted(clk)
        print("MDC clock (onset) : ~%.0f kbit/s (best of %d onset samples; ~the real clock)"
              % (sc[int(len(sc) * 0.9)], len(clk)))
    print("overflows         : pioc=%d ram=%d" % (ovf_pioc, ovf_ram))


def main():
    ap = argparse.ArgumentParser(description="Decode the CH32X035 MDIO sniffer (wire=2).")
    ap.add_argument("file", nargs="?", help="capture file (default: stdin)")
    ap.add_argument("--raw", action="store_true",
                    help="print every raw Clause-22 frame (no MMD folding)")
    ap.add_argument("--bits", action="store_true",
                    help="dump the raw sampled bit stream per burst (debug: no MDIO decode)")
    args = ap.parse_args()

    data = open(args.file, "rb").read() if args.file else sys.stdin.buffer.read()
    if args.bits:
        run_bits(data)
        return
    n = run_binary(data, args.raw)
    print("\n[decoded %d Clause-22 frames]" % n, file=sys.stderr)


if __name__ == "__main__":
    main()
