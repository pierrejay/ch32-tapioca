"""
mdio_lib.py - shared core for the MDIO host tools, and the parity oracle for the JS
codec test (app/test/mdio_codec.js was ported from this).

Decodes the device's wire=2 envelope: COBS-0xFF records framed by 0xFF, with 0xFD
loss markers and 0xFE…0xFE metadata blocks skipped. Each record's payload is captured
bytes (bit-packed, MSB-first) which we decode at the BIT level - hunt a preamble
(>=32 ones), then ST=01, then the Clause-22 fields. The MMD-indirect mechanism
(regs 13/14) is folded into readable Clause-45 MMD accesses.
"""
import struct
from collections import namedtuple

# ---- Clause-22 decode ------------------------------------------------------
OP_R, OP_W = 0b10, 0b01
OP_NAME = {0b10: "READ", 0b01: "WRITE", 0b00: "ADDR", 0b11: "WR-INC"}

DEVAD_NAME = {
    1: "PMA/PMD", 3: "PCS", 4: "PHY-XS", 5: "DTE-XS",
    7: "AN", 29: "C22ext", 30: "vendor2", 31: "vendor1",
}

REG_MMD_CTRL = 13      # MACR : [15:14]=function, [4:0]=devad
REG_MMD_DATA = 14      # MAADR: address or data, per last function

Frame = namedtuple("Frame", "op phyad regad ta data preamble")
Record = namedtuple("Record", "type t_us dur_us onset_us flags n payload")

ONSET_N = 8                 # firmware stamps onset at the 8th byte (clocked_sniffer.hpp)
ONSET_BITS = (ONSET_N - 1) * 8   # bit-times spanned by onset_us (1st -> 8th byte)


def bytes_to_bits(bs):
    bits = []
    for v in bs:
        bits += [(v >> k) & 1 for k in (7, 6, 5, 4, 3, 2, 1, 0)]
    return bits


def _bval(bits):
    v = 0
    for b in bits:
        v = (v << 1) | b
    return v


def decode_bits(bits):
    """Yield Clause-22 frames found in a bit list (preamble resync)."""
    n = len(bits)
    i = 0
    ones = 0
    while i < n:
        if bits[i] == 1:
            ones += 1
            i += 1
            continue
        if ones >= 32 and i + 32 <= n:
            f = bits[i:i + 32]
            if _bval(f[0:2]) == 0b01:                       # ST = 01
                yield Frame(_bval(f[2:4]), _bval(f[4:9]), _bval(f[9:14]),
                            _bval(f[14:16]), _bval(f[16:32]), ones)
                i += 32
                ones = 0
                continue
        ones = 0
        i += 1


# ---- MMD-indirect (Clause-45 over Clause-22) folding -----------------------
class MmdFolder:
    """Track REG13/REG14 sequences -> readable MMD<dev>[addr] accesses."""
    def __init__(self):
        self.func = {}
        self.devad = {}
        self.addr = {}

    def reset(self):
        self.func.clear(); self.devad.clear(); self.addr.clear()

    def feed(self, fr):
        """Return (text, event_dict|None). event carries a resolved MMD access."""
        op, phy, reg, data = fr.op, fr.phyad, fr.regad, fr.data

        if reg == REG_MMD_CTRL and op == OP_W:
            self.func[phy] = (data >> 14) & 0b11
            self.devad[phy] = data & 0x1F
            return None, None

        if reg == REG_MMD_DATA:
            devad = self.devad.get(phy)
            func = self.func.get(phy)
            if devad is None or func is None:
                return self._raw(fr), None
            if func == 0b00:
                if op == OP_W:
                    self.addr[(phy, devad)] = data
                    return None, None
                return self._raw(fr), None
            addr = self.addr.get((phy, devad))
            dn = DEVAD_NAME.get(devad, "dev%d" % devad)
            where = ("MMD%d(%s)[0x%04X]" % (devad, dn, addr)) if addr is not None \
                else ("MMD%d(%s)[?]" % (devad, dn))
            arrow = "=>" if op == OP_R else "<="
            kind = "read " if op == OP_R else "write"
            text = "PHY%-2d %-22s %s %s 0x%04X" % (phy, where, kind, arrow, data)
            ev = {"phy": phy, "devad": devad, "addr": addr, "op": op, "data": data}
            inc = (func == 0b10) or (func == 0b11 and op == OP_W)
            if inc and addr is not None:
                self.addr[(phy, devad)] = (addr + 1) & 0xFFFF
            return text, ev

        return self._raw(fr), {"phy": phy, "devad": None, "addr": reg,
                               "op": op, "data": data}

    @staticmethod
    def _raw(fr):
        arrow = "=>" if fr.op == OP_R else "<="
        return "PHY%-2d C22 REG%-2d            %-5s %s 0x%04X" % (
            fr.phyad, fr.regad, OP_NAME.get(fr.op, "?"), arrow, fr.data)


def raw_line(fr):
    arrow = "=>" if fr.op == OP_R else "<="
    return ("PHY%-2d C22 REG%-2d %-6s %s 0x%04X  | TA=%s pre=%d"
            % (fr.phyad, fr.regad, OP_NAME.get(fr.op, "?"), arrow,
               fr.data, format(fr.ta, "02b"), fr.preamble))


# ---- wire=2 transport envelope (COBS-0xFF records, 0xFF boundary) -----------
BIN_BOUNDARY = 0xFF        # record/frame boundary
BIN_LOSS     = 0xFD        # capture-loss marker (a lone segment)
BIN_DIAG     = 0xFE        # metadata block delimiter: 0xFE <ascii> 0xFE


def cobs_ff_decode(buf):
    """Decode one COBS-0xFF segment (the 0xFF boundary already split off) -> bytes,
       or None on a malformed segment. Mirrors Cobs::ffDecode (src/util/cobs.hpp):
       code bytes are in [0x01,0xFE]; a code != 0xFE implies a 0xFF after its group."""
    out = bytearray()
    i, n = 0, len(buf)
    while i < n:
        code = buf[i]
        if code == 0xFF or code == 0x00:            # 0xFF=boundary, 0x00=invalid code
            return None
        i += 1
        if i + (code - 1) > n:                       # truncated
            return None
        out.extend(buf[i:i + code - 1])
        i += code - 1
        if code != 0xFE and i < n:
            out.append(0xFF)
    return bytes(out)


def _is_meta(seg):
    """A 0xFE '[' … 0xFE ASCII metadata block (mirrors mdio_codec.js metaText)."""
    return (len(seg) >= 3 and seg[0] == BIN_DIAG and seg[-1] == BIN_DIAG
            and seg[1] == 0x5B and all(0x20 <= c <= 0x7E for c in seg[1:-1]))


def parse_record(rec):
    """Parse a decoded record; return Record or None if malformed."""
    if len(rec) < 12:
        return None
    typ = rec[0]
    t_us, dur_us, onset_us, flags, n = struct.unpack("<IHHBH", rec[1:12])
    payload = rec[12:12 + n]
    if len(payload) != n:
        return None
    return Record(typ, t_us, dur_us, onset_us, flags, n, bytes(payload))


def records_from_binary(data):
    """wire=2 capture -> valid type=1 Records. Splits the envelope on the 0xFF boundary;
       per segment a lone 0xFD = loss (skipped), a 0xFE…0xFE block = metadata (skipped),
       anything else = a COBS-0xFF record. Garbage / boot text before the first boundary
       (or a torn segment) just fails to decode and is skipped."""
    for seg in bytes(data).split(bytes([BIN_BOUNDARY])):
        if not seg or (len(seg) == 1 and seg[0] == BIN_LOSS) or _is_meta(seg):
            continue
        dec = cobs_ff_decode(seg)
        if dec is None:
            continue
        r = parse_record(dec)
        if r is not None and r.type == 0x01:
            yield r


FLAG_OVF_PIOC = 1
FLAG_OVF_RAM = 2
FLAG_CONTINUED = 4          # record cut at BURST_MAX; bit-contiguous with the next

LBurst = namedtuple("LBurst", "t_us dur_us onset_us flags payload")


class BurstJoiner:
    """Stitch cap-split (FLAG_CONTINUED) records back into one logical burst, so a
    frame straddling the BURST_MAX boundary isn't lost. Streaming: feed() returns
    a LBurst once a terminal (non-continued) record arrives, else None. The onset
    (clock) measurement is taken from the FIRST record of a chain - the one that
    opened right after a real idle gap (empty ring -> unbatched timing)."""
    def __init__(self):
        self._pay = bytearray()
        self._t0 = None
        self._onset = 0
        self._dur = 0
        self._flags = 0

    def feed(self, rec):
        if self._t0 is None:
            self._t0 = rec.t_us
            self._onset = rec.onset_us
        self._pay += rec.payload
        self._dur += rec.dur_us
        self._flags |= rec.flags
        if rec.flags & FLAG_CONTINUED:
            return None
        out = LBurst(self._t0, self._dur, self._onset,
                     self._flags & ~FLAG_CONTINUED, bytes(self._pay))
        self._pay = bytearray()
        self._t0 = None
        self._onset = 0
        self._dur = 0
        self._flags = 0
        return out


def onset_rate_kbits(onset_us):
    """MDC clock estimate (kbit/s) from an onset_us measurement, or None if the
    sample is missing or implausible (0 = n/a; tiny = batched -> absurd rate)."""
    if not onset_us:
        return None
    r = ONSET_BITS / (onset_us / 1e6) / 1e3        # kbit/s
    if r < 50 or r > 20000:                        # outside plausible MDC range
        return None
    return r


def logical_bursts(records):
    """Generator: Records -> LBursts (continued chains joined)."""
    j = BurstJoiner()
    for r in records:
        lb = j.feed(r)
        if lb is not None:
            yield lb
