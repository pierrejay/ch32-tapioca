#!/usr/bin/env python3
"""
dmx_rle_decode.py - DMX512 decoder on top of the generic RLE capture engine.

The PIOC blob (rle_sniffer) is protocol-agnostic: it streams transition-timing
runs. CAN framing lives in can_rle_decode; DMX framing lives HERE, reusing the
same front-end (can_rle_decode.capture_to_runs + clock recovery).

DMX512 (host's job to frame):
  * 250 kbit/s, RS-485, UART 8N2: each byte = START(0) + 8 data LSB-first + 2 STOP(1).
  * Bus idle/rest = MARK = HIGH.
  * BREAK = line LOW for >= 88 us (~22 bit-times) -> start of a packet. This is the
    long in-frame run that a FIXED tick-cap would have mangled; cap-continuation
    captures it verbatim (the whole reason DMX is the generic-encoding proof).
  * MAB (Mark After Break) = HIGH >= 8 us, then the SLOTS: slot0 = START CODE
    (0x00 = dimmer data), slot1.. = up to 512 channel levels.

Decoding is done by a TICK-ACCURATE mid-bit UART sampler: runs are laid on a
fractional-bit timeline (cumulative run/UI, NO per-run rounding) and each UART bit
is sampled at its centre, re-syncing on every START falling edge - exactly like a
real UART RX. This replaced an earlier expand-runs-to-bits path whose per-run
round(run/UI) flipped runs near a half-bit boundary.

STATUS: validated on the synthetic round-trip self-test (3 UIs + back-to-back) and
decodes real captures byte-perfectly. DMX @ 250 kbit/s sits comfortably inside the
validated rle drain margin.
Oracle to validate against: ch1=counter, ch2-8 = DE AD BE EF CA FE BA, ch9-16 = chase.

Usage:
    dmx_rle_decode.py --selftest
    dmx_rle_decode.py capture.bin          # decode a binary capture
"""
import sys
import os
import bisect

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can_rle_decode as C

# A byte = START + 8 data + 2 STOP = 11 bit-times. Max consecutive LOWs in normal
# data = START + 8 zero data bits = 9 (a 0x00 slot); BREAK is >= ~22. So a LOW run
# of >= BREAK_BITS is unambiguously a BREAK, never a data byte.
BREAK_BITS = 16
IDLE_SEP_BITS = 4          # nominal HIGH gap to represent a BOUNDARY_RUN on the timeline


def assign_levels(runs, idle_high=True):
    """Absolute level per run: re-anchor on every BOUNDARY_RUN (idle MARK = HIGH, the
       run after it is the BREAK), and alternate in between. MARK/idle = HIGH;
       idle_high=False is the inverted-wiring fallback.

       NOTE: an earlier version also hard-anchored on absolute run LENGTH (a long run =
       BREAK -> force LOW). That was tempting, but UI-fragile: a long HIGH run at a high
       UI can trip the same threshold. The firmware now provides explicit idle/boundary
       markers, so level anchoring belongs there, not in a second host-side heuristic."""
    idle = 1 if idle_high else 0
    levels, lv = [], idle
    for r in runs:
        if r >= C.BOUNDARY_RUN:
            levels.append(idle)
            lv = idle ^ 1                    # next real run is the BREAK (opposite of idle)
        else:
            levels.append(lv)
            lv ^= 1
    return levels


class _Timeline:
    """Runs laid end-to-end on a fractional-bit axis: each run contributes
       run/UI_level bits (NO rounding), so absolute bit positions are exact and a
       run that is 9.4 bits long stays 9.4 - it never snaps to 9 or 10. A
       BOUNDARY_RUN becomes a short HIGH idle separator. Sampling/edge queries
       are O(log n) via bisect; the UART sampler reads bit centres off this axis."""

    def __init__(self, runs, levels, ui_lo, ui_hi):
        self.segs = []                           # (start_bit, end_bit, level)
        self.bounds = []                         # bit positions of BOUNDARY_RUN idles
        pos = 0.0
        for r, lv in zip(runs, levels):
            if r >= C.BOUNDARY_RUN:
                self.bounds.append(pos)          # one firmware boundary (0xFF) per DMX frame
                dur, l = float(IDLE_SEP_BITS), 1  # idle MARK separator
            else:
                l = lv
                dur = r / (ui_hi if l else ui_lo)
            if dur <= 0:
                continue
            self.segs.append((pos, pos + dur, l))
            pos += dur
        self.starts = [s for s, _, _ in self.segs]
        self.end = pos

    def level_at(self, p):
        i = bisect.bisect_right(self.starts, p) - 1
        if i < 0 or i >= len(self.segs):
            return 1                             # before/after the stream = idle HIGH
        s, e, lv = self.segs[i]
        return lv if p < e else 1

    def next_falling(self, p):
        """Smallest segment-start > p whose level is LOW = the next START bit edge.
           Segments alternate level, so every LOW-segment start is a 1->0 edge."""
        i = bisect.bisect_right(self.starts, p)
        segs = self.segs
        while i < len(segs):
            if segs[i][2] == 0:
                return segs[i][0]
            i += 1
        return None


def _uart_sample(tl, p_start, p_end):
    """Resyncing UART 8N2 sampler over [p_start, p_end) of a _Timeline. Finds each
       START falling edge, samples the 8 data bits at their centres (START+1.5+k),
       checks the first STOP at +9.5, then RE-SYNCS on the next falling edge so
       jitter never accumulates across bytes. Returns (bytes, framing_errors)."""
    out, errs = [], 0
    p = tl.next_falling(p_start - 1e-9)
    while p is not None and p + 9.5 <= p_end:
        if tl.level_at(p + 0.5) != 0:            # START centre must be LOW
            p = tl.next_falling(p + 0.5)
            continue
        val = 0
        for k in range(8):                        # data LSB-first, sampled mid-bit
            if tl.level_at(p + 1.5 + k):
                val |= (1 << k)
        if tl.level_at(p + 9.5) != 1:             # STOP must be HIGH -> framing error
            errs += 1
            nf = tl.next_falling(p + 0.5)
            if nf is None or nf <= p:
                break
            p = nf
            continue
        out.append(val)
        nf = tl.next_falling(p + 9.5)             # resync on the real next START edge
        if nf is None:
            break
        p = nf
    return out, errs


def _first_break_end(tl, p_start, p_end):
    """Bit position right after the first LOW segment >= BREAK_BITS in [p_start,p_end)
       = where the MAB/slots begin. None if no BREAK there (so we skip that region)."""
    i = bisect.bisect_right(tl.starts, p_start)
    if i > 0:
        i -= 1                                    # include a segment straddling p_start
    while i < len(tl.segs):
        s, e, lv = tl.segs[i]
        if s >= p_end:
            break
        if lv == 0 and (e - s) >= BREAK_BITS:
            return e
        i += 1
    return None


def _decode_timeline(tl):
    """One DMX frame per firmware boundary (0xFF; the squelched idle MARK before each
       BREAK): split on boundaries, skip the BREAK+MAB, UART-sample the slots. The
       firmware's own framing is far more robust than re-detecting BREAKs (a parity-
       flipped idle can masquerade as a giant LOW 'break'). Falls back to BREAK-
       splitting when there are no boundaries (e.g. the synthetic self-test)."""
    if tl.bounds:
        edges = list(tl.bounds) + [tl.end]
        regions = [(edges[k], edges[k + 1]) for k in range(len(edges) - 1)]
    else:
        brk = [tl.segs[i][0] for i, (s, e, lv) in enumerate(tl.segs)
               if lv == 0 and (e - s) >= BREAK_BITS]
        regions = [(brk[k], brk[k + 1] if k + 1 < len(brk) else tl.end)
                   for k in range(len(brk))]
    packets, perrs = [], []                       # perrs[i] = framing errors of packets[i]
    for p0, p1 in regions:
        mab = _first_break_end(tl, p0, p1)        # past the BREAK -> slots start
        if mab is None:
            continue
        slots, e = _uart_sample(tl, mab, p1)
        if slots:
            packets.append(slots)
            perrs.append(e)
    return packets, perrs


def _dmx_score(packets, errs):
    """How DMX-plausible is a decode? Reward slots that belong to a 0x00-start-code
       (dimmer) packet, penalise UART framing errors. The comb fit alone can't pick
       between UI and its 9/10 harmonic (a 9-bit 0x00 run = 10*21.5 = 9*23.9 exactly);
       the start-code/structure is the tie-breaker the raw runs don't give."""
    good = sum(len(p) for p in packets if p and p[0] == 0x00)
    return good - 5 * errs


def calibrate(runs):
    """Scan UI (the comb fit lands on the wrong harmonic for 0x00-heavy universes) and
       both idle polarities, scored against DMX structure on a short subset. Returns
       (ui, idle_high). EXPENSIVE - 74 trial decodes; do it ONCE, the bit-rate is fixed.
       (Re-calibrating per batch is what made the live TUI fall seconds behind.)"""
    bidx = [i for i, r in enumerate(runs) if r >= C.BOUNDARY_RUN]
    sub = runs[:bidx[4]] if len(bidx) > 4 else runs        # ~4 packets to calibrate on
    best = None
    for idle_high in (True, False):
        levels = assign_levels(sub, idle_high)
        for ui_t in range(150, 331, 5):                    # UI 15.0 .. 33.0, step 0.5
            ui = ui_t / 10.0
            packets, perrs = _decode_timeline(_Timeline(sub, levels, ui, ui))
            score = _dmx_score(packets, sum(perrs))
            if best is None or score > best[0]:
                best = (score, ui, idle_high)
    return best[1], best[2]


def decode_with(runs, ui, idle_high):
    """Decode with a KNOWN (ui, idle_high) - no calibration scan. Returns (packets, errs)."""
    levels = assign_levels(runs, idle_high)
    return _decode_timeline(_Timeline(runs, levels, ui, ui))


def capture_to_runs(data):
    """Binary capture -> flat run list (for callers that cache calibration, e.g. the TUI)."""
    return C.capture_to_runs(data)


def decode_packets(runs):
    """runs (from capture_to_runs) -> (packets, ui, errs). A packet = [start_code, slot1...].
       Calibrates UI/polarity then decodes the whole stream."""
    ui, idle_high = calibrate(runs)
    packets, errs = decode_with(runs, ui, idle_high)
    return packets, ui, errs


def decode_capture(data):
    packets, ui, _ = decode_packets(C.capture_to_runs(data))
    return packets, ui


def decode_capture_full(data):
    """Like decode_capture but also returns total UART framing-error count (for report())."""
    return decode_packets(C.capture_to_runs(data))   # (packets, ui, errs)


# ---- synthetic encoder (for the self-test / a future generator reference) ------
def encode_packet(slots, ui=22, break_bits=24, mab_bits=4, idle_bits=8, c=0,
                  lead_idle=True):
    """Build the run list (ticks) for one DMX packet: idle, BREAK, MAB, then UART
       8N2 bytes. c = constant push-bias per run (host fit absorbs it). lead_idle
       False omits the leading MARK so packets concatenate without two adjacent
       same-level runs (real captures always alternate level)."""
    bits = ([1] * idle_bits if lead_idle else []) + [0] * break_bits + [1] * mab_bits
    for b in slots:
        bits.append(0)                           # START
        bits += [(b >> k) & 1 for k in range(8)]  # data LSB-first
        bits += [1, 1]                           # 2 STOP
    bits += [1] * idle_bits
    runs, i = [], 0
    while i < len(bits):
        j = i
        while j < len(bits) and bits[j] == bits[i]:
            j += 1
        runs.append((j - i) * ui + c)
        i = j
    return runs


def selftest():
    start_code = 0x00
    chans = [0, 255, 1, 128, 64, 200, 7, 0, 0, 254]
    slots = [start_code] + chans

    # round-trip at a couple of UIs + with the level polarity inverted
    ok = True
    for ui in (22, 18, 30):
        runs = encode_packet(slots, ui=ui, c=-1)
        packets, fui, _ = decode_packets(runs)
        good = len(packets) == 1 and packets[0] == slots
        ok = ok and good
        print("[%s] round-trip UI=%d -> %d packet(s), UI~%.1f, slots %s"
              % ("ok" if good else "FAIL", ui, len(packets), fui,
                 "match" if (packets and packets[0] == slots) else
                 (packets[0] if packets else "none")))

    # two back-to-back packets in one run stream (2nd without a leading MARK, so the
    # 1st packet's trailing idle HIGH is followed by the 2nd's BREAK LOW - alternating)
    slots2 = [0x00] + chans[::-1]
    runs = encode_packet(slots, ui=22, c=-1) + encode_packet(slots2, ui=22, c=-1, lead_idle=False)
    packets, _, _ = decode_packets(runs)
    multi = len(packets) == 2 and packets[0] == slots and packets[1] == slots2
    print("[%s] two back-to-back packets (got %d)" % ("ok" if multi else "FAIL", len(packets)))
    ok = ok and multi

    if not ok:
        sys.exit(1)
    print("SELF-TEST PASSED")


def report(packets, ui, perrs=None):
    """Health + activity map. Frames repeat ~27 Hz and are near-identical, so a frame
       LIST is useless; what a tech sniffing for a lighting fault wants is (1) is the
       link healthy, (2) which channels are active and what are they doing. We collapse
       all frames into a link-health line + a table of only the ACTIVE channels (ever
       non-zero) with last/min/max and static-vs-dynamic behaviour. perrs[i] = framing
       errors of packets[i] (trimmed in parallel so edge-frame errors aren't reported)."""
    from collections import Counter
    if not packets:
        print("DMX: no packets decoded."); return
    if perrs is None or len(perrs) != len(packets):
        perrs = [0] * len(packets)
    modal = Counter(len(p) for p in packets).most_common(1)[0][0]   # full frame length = slots

    # Capture-edge trim: a stream grabbed on the fly starts/stops mid-frame, so the first
    # & last decoded frames are partial (and over-read into the boundary -> spurious framing
    # errors). Drop leading/trailing non-modal frames, then the literal first & last. Trim
    # the per-frame error list IN PARALLEL so we only report errors of analysed frames.
    pairs = list(zip(packets, perrs))
    while len(pairs) > 2 and len(pairs[0][0])  != modal: pairs.pop(0)
    while len(pairs) > 2 and len(pairs[-1][0]) != modal: pairs.pop()
    if len(pairs) > 2:
        pairs = pairs[1:-1]
    dropped = len(packets) - len(pairs)
    packets = [p for p, _ in pairs]
    errs    = sum(e for _, e in pairs)              # framing errors over ANALYSED frames only

    n      = len(packets)
    sclen  = Counter(len(p) for p in packets)
    scodes = Counter(p[0] for p in packets if p)
    good   = [p for p in packets if len(p) == modal and p[0] == 0x00]
    ok, nchan = len(good), modal - 1                    # channels = slots - 1 (start code is slot 0)

    # ---- (1) link health
    scode  = scodes.most_common(1)[0][0]
    scname = "0x%02X (dimmer)" % scode if scode == 0x00 else "0x%02X" % scode
    edge   = "  (%d capture-edge frames dropped)" % dropped if dropped else ""
    print("DMX512  |  %d frames%s  |  start-code %s" % (n, edge, scname))
    integ = "%d/%d frames = %d slots" % (ok, n, modal)
    if n - ok:
        odd = ", ".join("%dx%d-slot" % (v, k) for k, v in sclen.most_common() if k != modal)
        integ += "  (%d anomaly: %s)" % (n - ok, odd)
    print("  integrity:  %s   framing-err %d" % (integ, errs))
    print("  timing:     UI ~%.1f ticks/bit  (250 kbaud, ~%.1f ms/frame nominal)"
          % (ui, modal * 11 * 4.0 / 1000.0))
    print("  refresh:    live only (squelched gaps can't be timed offline)")

    # ---- (2) active-channel map
    if not good:
        print("  no clean dimmer frames to map."); return
    cmin = [255] * nchan; cmax = [0] * nchan; clast = good[-1][1:1 + nchan]
    for p in good:
        for i, v in enumerate(p[1:1 + nchan]):
            if v < cmin[i]: cmin[i] = v
            if v > cmax[i]: cmax[i] = v
    active = [i for i in range(nchan) if cmax[i] > 0]
    if active:
        lo, hi = active[0] + 1, active[-1] + 1
        rng = "ch%d" % lo if lo == hi else "ch%d-%d" % (lo, hi)
    else:
        rng = "none"
    print("  active:     %d channels (%s), %d idle@0\n" % (len(active), rng, nchan - len(active)))
    print("  ch   last   min   max   behaviour")
    for shown, i in enumerate(active):
        if shown >= 64:
            print("  ... (+%d more active channels)" % (len(active) - shown)); break
        if cmin[i] == cmax[i]:
            beh = "static"
        elif cmax[i] - cmin[i] >= 200:
            beh = "DYNAMIC  (ramp/counter)"
        else:
            beh = "dynamic"
        print("  %3d  %4d  %4d  %4d   %s" % (i + 1, clast[i], cmin[i], cmax[i], beh))


def main():
    args = sys.argv[1:]
    if any(a in ("-h", "--help") for a in args):
        print(__doc__)
        return
    if "--selftest" in args:
        selftest(); return
    if not args:
        print(__doc__); return
    data = sys.stdin.buffer.read() if args[0] == "-" else open(args[0], "rb").read()
    packets, ui, errs = decode_capture_full(data)
    report(packets, ui, errs)


if __name__ == "__main__":
    main()
