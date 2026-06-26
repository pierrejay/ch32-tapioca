#!/usr/bin/env python3
"""
can_rle_decode.py - decode the PIOC run-length (transition-timing) capture.

The firmware (pioc/rle_sniffer.ASM) streams alternating run lengths in PIOC
count-units. A short run is one terminal byte (<128); a longer run is one or more
0x80 continuation bytes (+128 ticks, same level) followed by a terminal byte.
This tool recovers the unit interval (UI) from the data itself (no knowledge of
the bus bitrate or the PIOC clock), rebuilds the NRZ bitstream, removes CAN
bit-stuffing, and parses the frame.

UI is measured as the spacing between run-length clusters, per level -> drift-
and oscillator-tolerance-immune, and it auto-cancels the constant per-run push
bias of the capture loop. This is generic clock recovery for any non-clocked
NRZ bus; only the framing/parse layer below is CAN-specific.

    ./can_rle_decode.py --selftest        # validate decode logic, no hardware
    ./can_rle_decode.py capture.bin       # decode a capture FILE (batch)
    ./can_rle_decode.py --live /dev/cu.usbmodemXXXX   # REALTIME: live device, OK frames -> stdout
                                                      #   ("0x<id> <dlc> <hex>"); health -> stderr
"""
import sys

RECESSIVE = 1   # idle / high
DOMINANT  = 0   # low / dominant (SOF)


# ---- CAN CRC-15 (poly 0x4599), over the unstuffed SOF..data+CRCseed bits -----
def can_crc15(bits):
    crc = 0
    for b in bits:
        inb = b ^ ((crc >> 14) & 1)
        crc = (crc << 1) & 0x7FFF
        if inb:
            crc ^= 0x4599
    return crc


# ---- CAN bit stuffing (build side, for the self-test) ------------------------
def stuff(bits):
    out, run, last = [], 0, None
    for b in bits:
        out.append(b)
        run = run + 1 if b == last else 1
        last = b
        if run == 5:
            s = 1 - b
            out.append(s)
            run, last = 1, s
    return out


# ---- de-stuffing: drop the complementary bit after 5 identical received bits -
def unstuff(bits):
    if not bits:
        return []
    out = [bits[0]]
    run, last = 1, bits[0]
    i = 1
    while i < len(bits):
        b = bits[i]; i += 1
        if run == 5:                 # b is a stuff bit: remove it, it resets run
            run, last = 1, b
            continue
        out.append(b)
        run = run + 1 if b == last else 1
        last = b
    return out


# ---- run-length <-> bits -----------------------------------------------------
def bits_to_runs(bits):
    """[(value, length), ...] in time order."""
    runs, i = [], 0
    while i < len(bits):
        j = i
        while j < len(bits) and bits[j] == bits[i]:
            j += 1
        runs.append((bits[i], j - i))
        i = j
    return runs


def _comb_residual(runs, ui, iters=4):
    """For a candidate UI, snap each run to its nearest integer bit-count k>=1,
       fit C = median(run - k*UI) (robust to a stray merged/split run), and return
       (sum-of-squared rounding error, C, ks). A valid frame's runs are all near
       k*UI+C, so the true UI minimises this residual."""
    c = 0.0
    for _ in range(iters):
        ks = [max(1, round((x - c) / ui)) for x in runs]
        res = sorted(x - k * ui for x, k in zip(runs, ks))
        c = res[len(res) // 2]                        # median C (push bias)
    ks = [max(1, round((x - c) / ui)) for x in runs]
    sse = sum((x - k * ui - c) ** 2 for x, k in zip(runs, ks))
    return sse, c, ks


def fit_ui(counts):
    """Per-level clock recovery. counts ~= k_i*UI + C for integer k_i>=1, with C
       the constant push-bias of the capture loop. The old per-burst least-squares
       on cluster centres was fragile: a ~9% UI error rounds a borderline 5-bit run
       (~29 ticks) to 6 bits and corrupts the whole frame. This COMB FIT instead
       sweeps UI over a window anchored on the 1-bit run (c1) and picks the UI that
       makes ALL runs land closest to integer multiples (k*UI + C) - exactly the
       constraint a real frame must satisfy. In-frame runs only (k<=5, bit-stuffing
       max) so the giant idle/EOF recessive runs can't drag the fit."""
    if not counts:
        return 1.0, 0.0
    c1 = max(1, min(counts))                          # the 1-UI cluster (k=1); >=1 so a
    #                                                   stray 0-tick run can't zero the UI
    inframe = [x for x in counts if x < 6 * c1] or counts   # drop idle/EOF outliers
    if len(set(inframe)) < 2:
        return float(c1), 0.0                         # only one run length seen
    # UI is ticks/bit; c1 ~= 1*UI + C with C a small negative push bias, so UI is
    # bounded to [~0.9*c1, ~1.6*c1]. This window excludes the comb-fit degeneracy
    # (a tiny UI makes every run an "integer multiple" with ~0 residual).
    lo, hi = 0.90 * c1, 1.65 * c1
    best = None
    n = max(1, int(round((hi - lo) / 0.02)))
    for i in range(n + 1):
        ui = lo + (hi - lo) * i / n
        if ui <= 0:
            continue
        sse, c, ks = _comb_residual(inframe, ui)
        if len(set(ks)) < 2:                          # UI must resolve >1 bit-count
            continue
        # tie-break toward larger UI (fewer bits): a too-small UI can also fit by
        # splitting runs, but inflates k; prefer the coarser comb at equal residual.
        key = (round(sse, 6), -ui)
        if best is None or key < best[0]:
            best = (key, ui, c)
    if best is None:
        return float(c1), 0.0
    return best[1], best[2]


# CAN framing - THIS is where protocol knowledge lives (the blob is agnostic).
# Bit-stuffing caps any in-frame run at 5 bits (~35-40 ticks); the EOF / inter-
# frame gap is a longer recessive run (>= ~11 bits). So a run past this threshold
# is a frame boundary. Tunable for bitrate / sample rate (it is in ticks; the loop
# runs ~7 ticks/bit @ 1 Mbit).
BOUNDARY = 56   # floor: between 5-bit max in-frame run and ~11-bit gap @ UI~6


def split_frames(allbytes, bound):
    """Cut the reconstructed run list into per-frame lists: a run >= `bound` is a long
       recessive gap (or a BOUNDARY_RUN sentinel from an idle boundary / loss marker)."""
    frames, cur = [], []
    for b in allbytes:
        if b >= bound:                   # frame boundary: end the current frame
            if len(cur) >= 8:
                frames.append(cur)
            cur = []
        else:
            cur.append(b)
    if len(cur) >= 8:
        frames.append(cur)
    return frames


# ---- cap-continuation wire format: rle_sniffer blob --------------------------
IDLE_BOUNDARY = -1       # firmware idle boundary (0xFF on the wire): a hard frame cut
BOUNDARY_RUN = 1 << 30   # sentinel run length: always >= any frame_boundary threshold


def reconstruct_continuation(items):
    """rle_sniffer wire protocol: a run is a sequence of >=128 CONTINUATION bytes
       (each = +128 ticks, same level) ended by a TERMINAL byte <128 (run ends,
       level flips). Sum them into true run lengths. This is what makes the encoding
       bitrate-agnostic: a run of any duration survives as caps+remainder, so no
       fixed tick-cap floor. `items` may contain IDLE_BOUNDARY sentinels (the firmware's 0xFF idle
       boundary): each flushes the in-progress run and forces a frame boundary.
       Returns the reconstructed run-lengths (may exceed 255). A trailing
       unterminated continuation (capture cut mid-idle) is dropped."""
    runs, acc = [], 0
    for b in items:
        if b == IDLE_BOUNDARY:              # firmware idle boundary -> hard frame cut
            if acc:
                runs.append(acc); acc = 0   # flush whatever idle ticks we summed
            runs.append(BOUNDARY_RUN)       # a run guaranteed above any threshold
        elif b >= 128:
            acc += 128
        else:
            runs.append(acc + b)
            acc = 0
    return runs


def frame_boundary_runs(runs):
    """Boundary threshold for reconstructed runs (continuation mode): like
       frame_boundary but WITHOUT the 128 ceiling - a reconstructed in-frame run can
       legitimately exceed 128 ticks at low bitrate; only the idle gap is the cut."""
    small = [r for r in runs if r > 0] or [BOUNDARY]
    c1 = min(small)
    # The min run is a 1-bit pulse, but jitter makes it read a touch SHORT, so 8*c1
    # compensates that bias to land between the 5-bit max in-frame run and the EOF/IFS gap.
    # COLD-START proxy only: bound_from_ui takes over once a real per-level UI is locked.
    return max(BOUNDARY, 8 * c1)


def bound_from_ui(cal):
    """Frame boundary from the LOCKED per-level UI (cal = (ui_l, _, ui_h, _)). The cut must sit
       between the legal 5-bit max in-frame run and the >=10-bit (EOF 7 + IFS 3) inter-frame gap,
       so 7*UI is the centre. Replaces frame_boundary_runs' 8*min proxy once a real UI is known:
       the fit averages out the jitter that biased min, so it tracks the true bit length at any
       rate with no empirical factor. (8*min ~= 8*0.87*UI ~= 7*UI - same value, cleaner origin.)"""
    return max(BOUNDARY, round(3.5 * (cal[0] + cal[2])))     # 3.5*(ui_l+ui_h) = 7*UI_mean


def fit_burst_ui(counts):
    """Per-level clock recovery for ONE frame -> calibration (ui_l, c_l, ui_h, c_h).
       EXPENSIVE (two comb sweeps). At a FIXED bus rate the UI is the same for every
       frame, so fit it ONCE and reuse via decode_burst_cal - re-fitting per frame is
       ~400x slower and is what made the live view fall behind the USB (-> device-side
       RAM overflow). Never returns a 0 UI (would divide-by-zero on glitchy runs)."""
    ui_l, c_l = fit_ui(counts[0::2])      # DOMINANT runs (SOF first)
    ui_h, c_h = fit_ui(counts[1::2])      # RECESSIVE runs
    return (ui_l if ui_l > 0 else 1.0, c_l, ui_h if ui_h > 0 else 1.0, c_h)


def decode_burst_cal(runs, cal):
    """Expand one frame's runs to NRZ bits with a KNOWN calibration (no fit). Cheap -
       this is the per-frame hot path for the live decoder."""
    ui_l, c_l, ui_h, c_h = cal
    bits, level = [], DOMINANT
    for cnt in runs:
        if level == DOMINANT:
            n = max(1, round((cnt - c_l) / ui_l))
        else:
            n = max(1, round((cnt - c_h) / ui_h))
        bits.extend([level] * n)
        level ^= 1
    return bits


def decode_burst(runs):
    """runs = the run-length counts of ONE frame (between idle markers). They
       alternate DOMINANT, RECESSIVE, DOMINANT, ... starting at the SOF - the
       blob slept through the idle, so the first run here is the SOF itself."""
    if len(runs) < 4:
        return [], 0.0
    cal = fit_burst_ui(runs)
    return decode_burst_cal(runs, cal), (cal[0] + cal[2]) / 2.0


# ---- CAN standard-frame parse (stops at the CRC field) -----------------------
def parse_frame(bits):
    if len(bits) < 1 + 11 + 3 + 4 + 15 or bits[0] != DOMINANT:
        return None
    pos = [0]
    def take(n):
        v = 0
        for _ in range(n):
            v = (v << 1) | bits[pos[0]]
            pos[0] += 1
        return v
    take(1)                          # SOF (dominant)
    can_id = take(11)
    rtr = take(1)
    ide = take(1)
    if ide == 1:                     # extended frame: not parsed in v1
        return {"extended": True, "id": can_id}
    take(1)                          # r0
    dlc = take(4)
    ndata = min(dlc, 8)
    # Trailing recessive CRC bits can fall in the post-frame idle run (pushed to
    # the NEXT burst, discarded). They are unambiguously recessive -> pad them.
    need = pos[0] + ndata * 8 + 15
    if len(bits) < need:
        bits = bits + [RECESSIVE] * (need - len(bits))
    data = [take(8) for _ in range(ndata)]
    crc_input = bits[0:pos[0]]       # SOF..data, unstuffed
    crc = take(15)
    return {
        "extended": False, "id": can_id, "rtr": rtr, "dlc": dlc,
        "data": data, "crc": crc, "crc_calc": can_crc15(crc_input),
        "crc_ok": crc == can_crc15(crc_input),
    }


def _parse_runs(counts):
    bits, ui = decode_burst(counts)
    if not bits:
        return None, ui
    return parse_frame(unstuff(bits)), ui


def decode_frame_runs(counts):
    """Decode one frame's runs into (frame_dict, ui, dropped) or None.

    The first run may be the idle TAIL: the recessive leftover the firmware emits
    between the firmware idle boundary and the SOF. It leaks into the frame when it's below
    the boundary threshold, flipping the level parity (-> extended-ID / bad CRC) or,
    when ~0 ticks, wrecking the per-level UI fit. Try the runs as-is AND with the
    leading run dropped; keep the parse that yields a valid frame (prefer no drop)."""
    best = None
    for drop in (0, 1):
        if drop >= len(counts):
            break
        f, ui = _parse_runs(counts[drop:])
        if f is None:
            continue
        if (not f.get("extended")) and f.get("crc_ok"):
            return (f, ui, drop)           # valid frame -> done (fewest drops)
        if best is None:
            best = (f, ui, drop)           # fall back to first parseable (even if BAD)
    return best


def decode_and_report(counts, idx):
    best = decode_frame_runs(counts)
    if best is None:
        print("burst %d: %d runs -> no valid CAN frame" % (idx, len(counts)))
        return
    f, ui, drop = best
    tail = " (idle-tail dropped)" if drop else ""
    if f.get("extended"):
        print("burst %d: extended-ID frame (id partial 0x%X) - v1 parses 11-bit only%s"
              % (idx, f["id"], tail))
        return
    data = " ".join("%02X" % b for b in f["data"])
    print("burst %d  UI~%.1f cnt  ID=0x%03X DLC=%d  data=[%s]  CRC=%s%s" %
          (idx, ui, f["id"], f["dlc"], data,
           "OK" if f["crc_ok"] else "BAD(0x%04X!=0x%04X)" % (f["crc"], f["crc_calc"]),
           tail))


# ---- self-test: synthesise a frame -> runs -> decode -> verify ---------------
def build_frame_bits(can_id, data):
    core = [DOMINANT]                                  # SOF
    core += [(can_id >> (10 - k)) & 1 for k in range(11)]
    core += [0, 0, 0]                                  # RTR, IDE(std), r0
    core += [(len(data) >> (3 - k)) & 1 for k in range(4)]  # DLC
    for byte in data:
        core += [(byte >> (7 - k)) & 1 for k in range(8)]
    crc = can_crc15(core)
    core += [(crc >> (14 - k)) & 1 for k in range(15)]
    stuffed = stuff(core)                              # stuffing: SOF..CRC
    tail = [1, 1, 1] + [1] * 7 + [1] * 3               # CRCdelim, ACK, ACKdelim, EOF, IFS
    return stuffed + tail


def frame_to_bytes_continuation(frame_bits, ui_dom=30, ui_rec=31, c_dom=1, c_rec=2):
    """Emulate the rle_sniffer cap-continuation byte stream: a run of t ticks
       is emitted as floor(t/128) continuation bytes (=128) then a terminal byte (t mod 128).
       Default UI~30 (low bitrate) so in-frame runs EXCEED 128 ticks - the exact
       case the fixed cap broke and continuation must recover."""
    out = []
    for (val, blen) in bits_to_runs(frame_bits):
        t = blen * ui_rec + c_rec if val == RECESSIVE else blen * ui_dom + c_dom
        while t >= 128:
            out.append(128)                # continuation byte (bit7 set)
            t -= 128
        out.append(t)                      # terminal byte (<128)
    return out


def selftest():
    can_id = 0x123
    dataA = [0x00, 0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA]
    dataB = [0x01, 0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA]   # differs only in counter

    def good(f, data):
        return (f and not f["extended"] and f["id"] == can_id and
                f["dlc"] == len(data) and f["data"] == data and f["crc_ok"])

    # Two BACK-TO-BACK frames in the rle_sniffer cap-continuation wire format: low
    # bitrate (UI~30) so in-frame runs exceed 128 ticks -> the blob emits continuation
    # sequences that reconstruct must sum. Each frame ends in its long EOF recessive
    # run = the boundary the host derives (frame_boundary_runs). The leading idle is a
    # continuation seq. This is the exact split-on-EOF-run case the live TUI relies on.
    sc = frame_to_bytes_continuation(build_frame_bits(can_id, dataA))
    sd = frame_to_bytes_continuation(build_frame_bits(can_id, dataB))
    cstream = [128, 128, 128, 30] + sc + sd
    cruns = reconstruct_continuation(cstream)
    cframes = split_frames(cruns, frame_boundary_runs(cruns))
    cres = [parse_frame(unstuff(decode_burst(r)[0])) for r in cframes]
    okN = len(cframes) == 2
    print("[%s] split 2 back-to-back frames on the EOF run (want 2)" %
          ("ok" if okN else "FAIL"))
    okA = okN and good(cres[0], dataA)
    okB = okN and good(cres[1], dataB)
    print("[%s] frame A (counter 0x00)   [%s] frame B (counter 0x01)" %
          ("ok" if okA else "FAIL", "ok" if okB else "FAIL"))

    # robustness: +-1 tick of jitter on every (non-cap) run must still decode
    import random
    rng = random.Random(1)
    jit = [b if b >= 128 else max(1, b + rng.choice((-1, 0, 1))) for b in cstream]
    jruns = reconstruct_continuation(jit)
    jres = [parse_frame(unstuff(decode_burst(r)[0])) for r in
            split_frames(jruns, frame_boundary_runs(jruns))]
    okj = len(jres) == 2 and good(jres[0], dataA) and good(jres[1], dataB)
    print("[%s] with +-1 tick jitter on every run" % ("ok" if okj else "FAIL"))

    # the firmware idle boundary (0xFF on the wire) must force a cut WITHOUT summing
    # the runs across it - even when there is no long run to separate the two groups.
    pitems = [5, 6] * 4 + [IDLE_BOUNDARY] + [7, 8] * 4
    pruns = reconstruct_continuation(pitems)
    pframes = split_frames(pruns, frame_boundary_runs(pruns))
    okp = (pframes == [[5, 6, 5, 6, 5, 6, 5, 6], [7, 8, 7, 8, 7, 8, 7, 8]])
    print("[%s] 0xFF idle boundary forces a clean cut, no cross-sum"
          % ("ok" if okp else "FAIL"))

    # a stray 0-tick run (publish-blind-window glitch on real silicon) must not
    # crash the per-level UI fit (was a ZeroDivisionError); worst case = CRC=BAD.
    try:
        decode_burst([6, 0, 6, 7] * 4)
        okz = True
    except ZeroDivisionError:
        okz = False
    print("[%s] 0-tick run does not crash the decoder" % ("ok" if okz else "FAIL"))

    # device time-base parsing: pull thi/tlo/fcpu out of the [MODE rle ...] heartbeat,
    # and fall back to the blob constants when the block is absent.
    tb_blk = b"\x00\xfe[MODE rle wire=2 thi=9 tlo=8 fcpu=48000000]\xfe\xff\x06\x06"
    okt = (parse_device_timebase(tb_blk) == (9, 8, 48000000)
           and parse_device_timebase(b"\x06\x07\x06") == (DEFAULT_THI, DEFAULT_TLO, DEFAULT_FCPU))
    print("[%s] [MODE rle] time-base parse (thi/tlo/fcpu) + fallback" % ("ok" if okt else "FAIL"))

    # baud math: a real 1 Mbit bus at 48 MHz has UI_dom=48/8=6.0, UI_rec=48/9=5.333 (the
    # two levels' UIs differ so that UI*cyc is equal) -> both must recover ~1 Mbit and agree.
    mean, bd, br = baud_from_ui(6.0, 48.0 / 9.0, thi=9, tlo=8, fcpu=48_000_000)
    okb = abs(mean - 1_000_000) < 1000 and abs(bd - br) < 1000
    print("[%s] baud from per-level UI: %.0f (dom %.0f / rec %.0f) for a 1 Mbit bus"
          % ("ok" if okb else "FAIL", mean, bd, br))

    # boundary from the locked UI: 7*UI must sit between the 5-bit in-frame run and the 10-bit
    # EOF/IFS gap. 500 kbit/s (UI_dom 12.0 / UI_rec 10.67) -> ~79 ticks; 1 Mbit hits the 56 floor.
    ui500 = (12.0, 0.0, 48.0 / 9.0 * 2, 0.0)
    b500 = bound_from_ui(ui500)
    okbd = (5 * ui500[0] < b500 < 10 * ui500[2]) and bound_from_ui((6.0, 0.0, 48.0 / 9.0, 0.0)) == BOUNDARY
    print("[%s] frame boundary from locked UI: %d ticks @500k (between 5-bit run and EOF gap)"
          % ("ok" if okbd else "FAIL", b500))

    if not (okA and okB and okj and okp and okz and okt and okb and okbd):
        sys.exit(1)
    print("SELF-TEST PASSED")


# Binary wire sentinels (native firmware format). The cap-continuation only ever emits
# run-bytes 0x00..0x80, so 0x81..0xFF are free sentinels - no escaping needed.
BIN_BOUNDARY = 0xFF     # firmware idle boundary / hard frame cut
BIN_DIAG     = 0xFE     # diag block delimiter: 0xFE <ascii> 0xFE
BIN_LOSS     = 0xFD     # capture-loss marker: device dropped data (RAM ring overflow) here


def count_loss_markers(data):
    """Number of 0xFD capture-loss markers in a BINARY stream (device dropped data).
       0 on real traffic that fits; >0 means the capture has gaps - never silent."""
    return data.count(0xFD) if isinstance(data, (bytes, bytearray)) else 0


def capture_to_runs(data):
    """BINARY capture (native firmware) -> FLAT run list.
       run-bytes 0x00..0x80 ; 0xFF = boundary ; 0xFD = capture-loss (forced boundary,
       data was dropped - don't merge across it) ; 0xFE <ascii> 0xFE = diag block
       (skipped) ; 0x81..0xFC = reserved (ignored). Cap-continuations are summed and a
       the boundary between segments becomes a BOUNDARY_RUN. No protocol framing here - callers
       segment the returned runs themselves (split_frames)."""
    segs, cur = [], []
    i, n = 0, len(data)
    while i < n:
        b = data[i]
        if b == BIN_BOUNDARY or b == BIN_LOSS:   # frame boundary / data lost here
            segs.append(cur); cur = []
        elif b == BIN_DIAG and i + 1 < n and data[i + 1] == 0x5B:  # 0xFE '[' = REAL diag block
            i += 2                          # (firmware contract). Skip past 0xFE '[' ...
            while i < n and data[i] != BIN_DIAG:                   # ... to the closing 0xFE.
                i += 1
        elif b <= 0x80:
            cur.append(b)
        # else 0x81..0xFC, or a STRAY 0xFE not followed by '[' (mid-stream connect landed on
        # a closing sentinel): reserved/noise -> ignore. NOT skip-to-next-0xFE, which would
        # swallow a whole block of real run-bytes and desync the capture.
        i += 1
    segs.append(cur)
    items = []
    for si, sb in enumerate(segs):
        if si:
            items.append(IDLE_BOUNDARY)     # boundary between segments = hard cut
        items += sb
    return reconstruct_continuation(items)


def parse_capture(data):
    """Binary capture -> list of per-CAN-frame run lists (split on the EOF recessive run).
       Bootstrap the boundary from 8*min, then - once a frame resolves a per-level UI - refine
       it to 7*UI (the unbiased bit length) and re-split, mirroring the browser decoder."""
    runs = capture_to_runs(data)
    frames = split_frames(runs, frame_boundary_runs(runs))      # bootstrap split (8*min)
    for fr in frames:                                           # first well-resolved frame -> lock UI
        if len(fr) < 8:
            continue
        cal = fit_burst_ui(fr)
        if cal[0] > 0 and cal[2] > 0:
            return split_frames(runs, bound_from_ui(cal))       # refine + re-split at 7*UI
    return frames


# ---- absolute bitrate: the device tells us its tick time-base -----------------------
# A run-byte is a count of blob-loop iterations (ticks). One tick is a FIXED integer
# number of CPU cycles - but DIFFERENT per level (the HIGH loop carries one extra branch):
# HIGH=9, LOW=8, measured exactly by the constant-level cap bench (firmware env
# test_rle_tick). The firmware ships them in [MODE rle ... thi tlo fcpu], so the host can
# turn UI (ticks/bit) into an absolute bitrate:  baud = fcpu / (UI_level * cyc_per_tick).
# The per-run edge overhead k cancels here (it's the SLOPE that matters) and fit_ui already
# absorbs it as its intercept C - so we use the integer cycles, never a decimal average.
DEFAULT_THI, DEFAULT_TLO, DEFAULT_FCPU = 9, 8, 48_000_000     # fallback = the measured blob


def parse_device_timebase(data):
    """Scan a BINARY capture for the [MODE rle ... thi=.. tlo=.. fcpu=..] block and return
       (thi, tlo, fcpu). Falls back to the known blob constants if the field is absent
       (older firmware / a capture that missed the ~1 Hz heartbeat)."""
    thi, tlo, fcpu = DEFAULT_THI, DEFAULT_TLO, DEFAULT_FCPU
    if isinstance(data, (bytes, bytearray)):
        text = bytes(data).decode("latin1", "ignore")
        m = re.search(r"\[MODE\s+rle\b[^\]]*\]", text)
        if m:
            blk = m.group(0)
            mt, ml, mf = (re.search(r"thi=(\d+)", blk), re.search(r"tlo=(\d+)", blk),
                          re.search(r"fcpu=(\d+)", blk))
            if mt and ml and mf:
                thi, tlo, fcpu = int(mt.group(1)), int(ml.group(1)), int(mf.group(1))
    return thi, tlo, fcpu


def baud_from_ui(ui_dom, ui_rec, thi=DEFAULT_THI, tlo=DEFAULT_TLO, fcpu=DEFAULT_FCPU):
    """Absolute bitrate from the per-level UI fit. DOMINANT runs are the LOW level (tlo
       cyc/tick), RECESSIVE are HIGH (thi). T_bit = UI*cyc/fcpu, so baud = fcpu/(UI*cyc);
       the two levels must agree (cross-check). Returns (baud_mean, baud_dom, baud_rec)."""
    bd = fcpu / (ui_dom * tlo) if ui_dom > 0 else 0.0
    br = fcpu / (ui_rec * thi) if ui_rec > 0 else 0.0
    mean = (bd + br) / 2.0 if (bd and br) else (bd or br)
    return mean, bd, br


def estimate_baud(data):
    """One-shot bus bitrate: device time-base + the first well-resolved frame's per-level
       UI. Returns (baud_mean, baud_dom, baud_rec, (thi, tlo, fcpu)) or None if nothing fit."""
    tb = parse_device_timebase(data)
    for runs in parse_capture(data):
        if len(runs) < 8:                              # need both levels resolved
            continue
        ui_l, _cl, ui_h, _ch = fit_burst_ui(runs)
        if ui_l > 0 and ui_h > 0:
            mean, bd, br = baud_from_ui(ui_l, ui_h, *tb)
            if mean > 0:
                return mean, bd, br, tb
    return None


def decode_capture(data):
    """Decode a binary capture (or a live chunk) and RETURN the frame dicts (no
       printing). Each dict is parse_frame's output + 'ui'. For programmatic use."""
    out = []
    for runs in parse_capture(data):
        best = decode_frame_runs(runs)
        if best:
            f, ui, _ = best
            f = dict(f); f["ui"] = ui
            out.append(f)
    return out


def decode_stream(data):
    est = estimate_baud(data)
    if est:
        mean, bd, br, (thi, tlo, fcpu) = est
        print("bus bitrate ~%.0f baud  (dominant %.0f / recessive %.0f; thi=%d tlo=%d fcpu=%d)"
              % (mean, bd, br, thi, tlo, fcpu))
    frames = parse_capture(data)
    for n, runs in enumerate(frames):
        decode_and_report(runs, n)
    if not frames:
        print("no frames found (need a binary RLE byte stream)")



# ============================================================================
#  --live : headless realtime reader (frames -> stdout). The reader() below is
#  Realtime reader: O_NONBLOCK+select CDC input, incremental run reconstruction,
#  3-tier decode with calibration cache and load-shed. The sink prints OK frames
#  to stdout (one per line:
#  "0x<id> <dlc> <hex bytes>"), health (loss/ovf/shed) to stderr - so stdout
#  stays clean and scriptable. Bench-validated path; re-test on the bench.
# ============================================================================
import os, re, time, select
import tty as ttymod
D = sys.modules[__name__]
_OVF = re.compile(r"ovf=(\d+)")
_DROP = re.compile(r"drop=(\d+)")


class _NullLock:
    def __enter__(self): return self
    def __exit__(self, *a): return False


class _LiveSink:
    def __init__(self):
        self.lock = _NullLock(); self.log = self; self.eof = False
        self.ovf_pioc = 0; self.ovf_ram = 0
        self.ok = 0; self.bad = 0; self.loss = 0; self.dropped = 0

    def append(self, msg):                      # st.log.append (open errors etc.) -> stderr
        sys.stderr.write(str(msg) + "\n")

    def add_frame(self, f):                     # clean policy: only OK std frames hit stdout
        if f.get("extended") or not f.get("crc_ok"):
            self.bad += 1
            return
        self.ok += 1
        data = " ".join("%02X" % b for b in f["data"])
        sys.stdout.write("0x%X %d %s\n" % (f["id"], f["dlc"], data))
        sys.stdout.flush()                      # flush so a pipe sees frames in real time

    def set_ovf(self, p, r): self.ovf_pioc, self.ovf_ram = p, r
    def add_loss(self, n):
        self.loss += n; sys.stderr.write("# loss x%d (device dropped data)\n" % n)
    def add_dropped(self, n): self.dropped += n


def reader(path, st):
    # O_NONBLOCK + select(), NOT a blocking open + blocking os.read: on macOS a CDC
    # cu.* device intermittently leaves a blocking read WEDGED at connect (~2-in-8 =
    # the "1-in-3 nothing" freeze, even though bytes are pumping). Proven: select-based
    # read connects 10/10; blocking read stalls ~1/3. The decode was never the cause.
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as e:
        with st.lock:
            st.log.append("open failed: %s" % e)
            if getattr(e, "errno", None) == 16:
                st.log.append("port BUSY - close screen/cat/pio monitor; use /dev/cu.*")
        st.eof = True
        return
    try:
        if os.isatty(fd):
            ttymod.setraw(fd)
    except Exception:
        pass

    # The device streams the BINARY wire format: run-bytes 0x00..0x80, 0xFF = idle-gap
    # boundary, 0xFD = capture-loss marker, 0xFE <ascii> 0xFE = diag block. 0xFF is NOT
    # per-frame (under dense traffic there are NONE - it only fires on a long bus idle),
    # so frames are delimited by their long EOF/IFS recessive RUN. We reconstruct runs
    # incrementally (cap-continuations summed across reads) and split them on the
    # boundary run, EXACTLY the proven offline path (frame_boundary_runs + split_frames).
    #
    # DECODE TIERS (the bus rate is FIXED, so the per-level UI calibration is reused):
    #   1. cached cal     decode_burst_cal      ~0.06 ms   hot path, ~100% at steady state
    #   2. narrow re-fit  ~10 candidates / cal   ~1 ms     recovers UI jitter, REFRESHES cal
    #   3. full comb-fit  decode_frame_runs     ~11 ms     cold start only; BUDGETED per flush
    # At UI~4.5 ticks/bit a single frozen cal only hits ~46% (per-frame quantisation), so
    # tier 2 MUST refresh the cal (never just "mark BAD") or half the frames go false-BAD.
    # Tier 3 is the only expensive path; if every frame hit it (~86 f/s << 2000 f/s source)
    # the read loop falls behind -> USB back-pressure -> DEVICE RAM ring overflow -> 0xFD
    # -> corrupt frames -> cal never locks = the freeze. So tier 3 is capped per flush AND
    # the backlog is load-shed: the host always drains the CDC faster than the device fills.
    BOUNDARY_RUN = D.BOUNDARY_RUN
    FLUSH_AT    = 64           # split once this many runs buffer (>~1 frame of runs)
    MAX_BACKLOG = 8000         # load-shed ceiling (~100 frames): drop OLDEST past this
    SLOW_BUDGET = 8            # tier-3 full-fits allowed per flush (bounds flush time)
    WARMUP_S    = 0.20         # drain & DISCARD the connect burst for this long: at open the
    #                            USB controller buffer holds stale / backed-up bytes (and a
    #                            possible mid-frame start) that are not trustworthy. We flush
    #                            them, then lock onto the first fresh CRC-OK frame. Invisible
    #                            to the user (the frame counter just starts ~200 ms in).
    diag = None                # ASCII payload while inside a 0xFE [ ... 0xFE block
    fe = False                 # previous byte was 0xFE (awaiting '[' to confirm a real block)
    cont = 0                   # in-progress cap-continuation accumulator (may straddle reads)
    runbuf = []                # reconstructed runs awaiting frame splitting
    cal = [None]               # cached per-level UI calibration (ui_l,c_l,ui_h,c_h); rate fixed
    bnd = [0]                  # cached boundary threshold (derived once; rate fixed)
    synced = [False]           # seen the first boundary yet? (drop the mid-stream partial frame)

    def add(f, ui):
        # Startup sync: a mid-stream connect (mid-frame, or inside a diag block whose
        # ASCII gets misread as run-bytes) produces a burst of BAD frames with phantom
        # IDs before the framing locks. Drop EVERY BAD frame until the first CRC-OK one;
        # after that, show BAD frames (they're real bus errors). The first good frame
        # also locks the cal, so "synced" and "cal locked" happen together.
        if f.get("crc_ok"):
            synced[0] = True
        elif not synced[0]:
            return
        f = dict(f); f["ui"] = ui
        st.add_frame(f)

    def emit(runs, slow):
        if len(runs) < 8:
            return
        c = cal[0]
        if c is not None:
            # tier 1: cached cal (cheap expand+parse)
            for drop in (0, 1):                  # 0/1 = with/without a leading idle tail
                if drop >= len(runs):
                    break
                try:
                    f = D.parse_frame(D.unstuff(D.decode_burst_cal(runs[drop:], c)))
                except Exception:
                    f = None
                if f and not f.get("extended") and f.get("crc_ok"):
                    add(f, (c[0] + c[2]) / 2.0)
                    return
            # tier 2: narrow re-fit around the cached UI (~10 candidates). Recovers the
            # per-frame quantisation jitter AND refreshes the cal so it tracks the bus.
            ui0 = (c[0] + c[2]) / 2.0
            lo, hi = ui0 * 0.85, ui0 * 1.15
            for drop in (0, 1):
                if drop >= len(runs):
                    break
                r = runs[drop:]
                for k in range(11):
                    ui = lo + (hi - lo) * k / 10.0
                    nc = (ui, c[1], ui, c[3])
                    try:
                        f = D.parse_frame(D.unstuff(D.decode_burst_cal(r, nc)))
                    except Exception:
                        f = None
                    if f and not f.get("extended") and f.get("crc_ok"):
                        cal[0] = nc
                        add(f, ui)
                        return
        # tier 3: full comb-fit - the ONLY expensive path, so it is BUDGETED. If the
        # budget is spent (cold-start / corruption burst), SHED this frame rather than
        # block the read loop (which would starve the USB -> device overflow cascade).
        if slow[0] <= 0:
            st.add_dropped(1)
            return
        slow[0] -= 1
        try:
            best = D.decode_frame_runs(runs)
        except Exception:
            best = None
        if not best:
            return
        f, ui, drop = best
        if f.get("crc_ok"):
            cal[0] = D.fit_burst_ui(runs[drop:])  # lock / refresh the cal from a clean frame
            bnd[0] = D.bound_from_ui(cal[0])      # re-derive the boundary from the locked UI
        add(f, ui)

    def flush(force=False):
        if not runbuf or (len(runbuf) < FLUSH_AT and not force):
            return
        # LOAD-SHED: a slow decode must never let the backlog grow without bound - that
        # stalls os.read -> CDC back-pressure -> device RAM ring overflow -> 0xFD loss.
        # Drop the OLDEST runs so the view stays on the FRESHEST data; the next EOF run
        # re-syncs the framing. Far better to drop a frame here than to corrupt the wire.
        if len(runbuf) > MAX_BACKLOG:
            n = len(runbuf) - MAX_BACKLOG
            del runbuf[:n]
            st.add_dropped(n)
        # Boundary threshold: bootstrap from 8*min (no UI yet), then emit() refreshes it from
        # the locked UI (bound_from_ui = 7*UI) so it tracks the true bit length, re-deriving on
        # a live rate change instead of staying pinned to the first rate seen.
        if not bnd[0]:
            bnd[0] = D.bound_from_ui(cal[0]) if cal[0] else D.frame_boundary_runs(runbuf)
        bound = bnd[0]
        slow = [SLOW_BUDGET]
        cur, last = [], 0
        for i, r in enumerate(runbuf):
            if r >= bound:                       # EOF gap / 0xFF / 0xFD = frame end
                if len(cur) >= 8:
                    emit(cur, slow)              # add() drops pre-sync garbage frames
                cur = []; last = i + 1
            else:
                cur.append(r)
        if force and len(cur) >= 8:              # EOF: emit the final partial frame too
            emit(cur, slow); last = len(runbuf)
        if last == 0 and len(runbuf) > MAX_BACKLOG:   # no boundary in a huge backlog
            del runbuf[:4096]                         # (pure noise) -> drop stale
        else:
            del runbuf[:last]                    # keep only the unterminated tail

    # warm-up only for a live device; a replay FILE is not a tty and would be fully
    # consumed (and discarded) inside the 200 ms window -> 0 frames.
    warmup_end = (time.time() + WARMUP_S) if os.isatty(fd) else 0
    try:
        while True:
            try:
                ready, _, _ = select.select([fd], [], [], 0.5)
            except OSError:
                break
            if not ready:                              # no data this tick (live idle)
                continue
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:                    # select raced; nothing to read yet
                continue
            except OSError:
                break
            if not chunk:                              # EOF (replay file) / disconnect
                break
            if warmup_end:                             # warm-up: drain & discard the stale
                if time.time() < warmup_end:           # connect burst, don't decode any of it
                    continue
                warmup_end = 0                         # done -> process THIS chunk onward
            for b in chunk:
                if diag is not None:                   # inside a 0xFE [ ... 0xFE block
                    if b == D.BIN_DIAG:                # closing 0xFE
                        text = bytes(diag).decode("ascii", "ignore")
                        mo, md = _OVF.search(text), _DROP.search(text)
                        if mo or md:
                            st.set_ovf(int(mo.group(1)) if mo else st.ovf_pioc,
                                       int(md.group(1)) if md else st.ovf_ram)
                        diag = None
                    else:
                        diag.append(b)
                    continue
                if fe:                                 # previous byte was 0xFE
                    fe = False
                    if b == 0x5B:                      # 0xFE '[' = REAL diag block (firmware
                        diag = bytearray(b"[")         # contract); open it, keep the '['.
                        continue
                    # stray 0xFE (mid-stream connect on a closing sentinel): drop the 0xFE,
                    # fall through and handle b as a normal byte below.
                if b == D.BIN_DIAG:                    # potential block open -> confirm next byte
                    fe = True
                    continue
                if b == D.BIN_BOUNDARY or b == D.BIN_LOSS:     # 0xFF gap / 0xFD loss = hard boundary
                    if b == D.BIN_LOSS:
                        st.add_loss(1)
                        synced[0] = False     # data was dropped here -> the next frame may be
                        #                       a mid-loss partial; re-sync exactly like at
                        #                       connect (drop until the next CRC-OK frame).
                    if cont:
                        runbuf.append(cont); cont = 0
                    runbuf.append(BOUNDARY_RUN)
                elif b == 0x80:                        # continuation byte (+128 ticks, same level)
                    cont += 128
                elif b < 0x80:                         # terminal run-byte -> a run ends here
                    runbuf.append(cont + b); cont = 0
                # else 0x81..0xFC: reserved sentinels, ignore
            flush()                                    # split out whatever full frames we have
    finally:
        flush(force=True)
        os.close(fd)
        st.eof = True


def live(path):
    """Headless realtime CAN: decode the live device and print OK frames to stdout."""
    st = _LiveSink()
    try:
        reader(path, st)
    except KeyboardInterrupt:
        pass
    sys.stderr.write("# %d ok, %d bad, %d loss, %d host-shed\n"
                     % (st.ok, st.bad, st.loss, st.dropped))


def main():
    args = sys.argv[1:]
    if any(a in ("-h", "--help") for a in args):
        print(__doc__)
        return
    if "--selftest" in args:
        selftest(); return
    if "--live" in args:
        devs = [a for a in args if not a.startswith("--")]
        if not devs:
            sys.stderr.write("usage: can_rle_decode.py --live /dev/cu.usbmodemXXXX\n"); return
        live(devs[0]); return
    args = [a for a in args if not a.startswith("--")]
    if not args:
        print(__doc__); return
    src = args[0]
    data = sys.stdin.buffer.read() if src == "-" else open(src, "rb").read()
    loss = count_loss_markers(data)
    if loss:
        sys.stderr.write("⚠ %d capture-loss marker(s) (0xFD): the device dropped data "
                         "(RAM ring overflow) - this capture has gaps, frames near them are "
                         "split, not silently merged.\n" % loss)
    decode_stream(data)


if __name__ == "__main__":
    main()
