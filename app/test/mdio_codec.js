/*
 * mdio_codec.js — host-side MDIO codec, faithful port of scripts/mdio_lib.py.
 *
 * Binary firmware format: COBS-framed timestamped burst records. A record =
 *   u8 type=1 | u32 t_us | u16 dur_us | u16 onset_us | u8 flags | u16 n | u8 payload[n]
 * payload = captured bytes, bit-packed MSB-first. Decode at the BIT level: hunt a
 * preamble (>=32 ones), then ST=01, then the Clause-22 fields; fold the REG13/14
 * MMD-indirect mechanism into readable Clause-45 MMD accesses.
 *
 * WIRE VERSIONS (the format-version tag in [MODE … wire=N]):
 *   wire=1 : COBS keyed on 0x00, records split on 0x00. The original framing,
 *            bit-for-bit validated against mdio_lib.py.
 *   wire=2 : COBS keyed on 0xFF inside the shared transport envelope —
 *                      0xFF = the single record boundary, 0xFD = loss marker,
 *                      0xFE[ascii]0xFE = metadata ([MODE], or [T] under -D DIAG). 0xFD/0xFE
 *                      may occur INSIDE a COBS-0xFF payload, so they are only
 *                      interpreted in segment-leading position (just after a 0xFF
 *                      boundary): a lone 0xFD segment = loss; a 0xFE…0xFE ASCII
 *                      segment = metadata; anything else = a COBS-0xFF record.
 *                      (record field layout & bit-packing are identical to wire=1.)
 *
 * Loads in a browser (attaches to window.MDIO, uses window.COBS) and in Node
 * (module.exports; requires ./cobs.js).
 */
(function (root) {
  "use strict";

  const COBS = (typeof module !== "undefined" && module.exports)
    ? require("./cobs.js")
    : root.COBS;

  const OP_R = 0b10, OP_W = 0b01;
  const OP_NAME = { 0b10: "READ", 0b01: "WRITE", 0b00: "ADDR", 0b11: "WR-INC" };
  const DEVAD_NAME = { 1: "PMA/PMD", 3: "PCS", 4: "PHY-XS", 5: "DTE-XS", 7: "AN", 29: "C22ext", 30: "vendor2", 31: "vendor1" };
  const REG_MMD_CTRL = 13, REG_MMD_DATA = 14;
  const ONSET_N = 8, ONSET_BITS = (ONSET_N - 1) * 8;
  const FLAG_OVF_PIOC = 1, FLAG_OVF_RAM = 2, FLAG_CONTINUED = 4;
  const VALIDBITS_SHIFT = 3, VALIDBITS_MASK = 0x7;   // flags bits 3..5 = valid bits in last byte

  const hex4 = v => v.toString(16).toUpperCase().padStart(4, "0");

  // ---- bit-level Clause-22 decode ------------------------------------------
  function bytesToBits(bs) {
    const bits = [];
    for (const v of bs) for (const k of [7, 6, 5, 4, 3, 2, 1, 0]) bits.push((v >> k) & 1);
    return bits;
  }
  // Like bytesToBits, but the LAST byte contributes only `validBits` MSBs when the firmware
  // flushed a mid-byte partial at idle (1..7; 0 = all bytes complete). A non-byte-aligned stop
  // then decodes exactly, with no 1..7 phantom bits shifting the frame.
  function burstToBits(payload, validBits) {
    if (!payload.length || !validBits) return bytesToBits(payload);
    const bits = bytesToBits(payload.slice(0, -1));
    const last = payload[payload.length - 1];
    for (let i = 0; i < validBits; i++) bits.push((last >> (7 - i)) & 1);
    return bits;
  }
  function bval(bits, lo, hi) { let v = 0; for (let i = lo; i < hi; i++) v = (v << 1) | bits[i]; return v; }

  // Yield Clause-22 frames found in a bit list (preamble resync), mirrors decode_bits.
  function decodeBits(bits) {
    const out = [];
    const n = bits.length;
    let i = 0, ones = 0;
    while (i < n) {
      if (bits[i] === 1) { ones++; i++; continue; }
      if (ones >= 32 && i + 32 <= n) {
        const f = i;                                  // frame starts at i (32 bits)
        if (bval(bits, f, f + 2) === 0b01) {           // ST = 01
          const op = bval(bits, f + 2, f + 4);
          const phyad = bval(bits, f + 4, f + 9);
          const regad = bval(bits, f + 9, f + 14);
          const ta = bval(bits, f + 14, f + 16);
          const data = bval(bits, f + 16, f + 32);
          // Conservative "no slave drove the read" detector: on a pulled-up MDIO
          // line, a missing read response appears as TA=11 + DATA=0xffff.
          const noResponse = op === OP_R && ta === 0b11 && data === 0xFFFF;
          out.push({ op, phyad, regad, ta, data, preamble: ones, noResponse });
          i += 32; ones = 0; continue;
        }
      }
      ones = 0; i++;
    }
    return out;
  }

  // Cross-burst frame hunter (mirrors mdio_lib.BitStream). feed() the bits of each
  // successive transport burst; it carries the unconsumed tail forward so a frame
  // straddling the BURST_MAX cut decodes whole. MDIO self-frames on its >=32-bit
  // preamble, so transport-burst alignment must not gate decoding. The carry is
  // bounded (one preamble+frame) so pure-idle bursts can't grow it; reset() drops it.
  const BITSTREAM_CARRY_MAX = 80;
  class BitStream {
    constructor() { this._carry = []; }
    reset() { this._carry = []; }
    feed(bits) {
      const buf = this._carry.concat(bits);
      const n = buf.length;
      const out = [];
      let i = 0, ones = 0, end = 0;
      while (i < n) {
        if (buf[i] === 1) { ones++; i++; continue; }
        if (ones >= 32 && i + 32 <= n) {
          const f = i;
          if (bval(buf, f, f + 2) === 0b01) {            // ST = 01
            const op = bval(buf, f + 2, f + 4);
            const ta = bval(buf, f + 14, f + 16);
            const data = bval(buf, f + 16, f + 32);
            // mirror decodeBits: a missing read response = TA=11 + DATA=0xffff
            const noResponse = op === OP_R && ta === 0b11 && data === 0xFFFF;
            out.push({
              op, phyad: bval(buf, f + 4, f + 9),
              regad: bval(buf, f + 9, f + 14), ta, data, preamble: ones, noResponse,
            });
            i += 32; ones = 0; end = i; continue;
          }
        }
        ones = 0; i++;
      }
      this._carry = buf.slice(Math.max(end, n - BITSTREAM_CARRY_MAX));
      return out;
    }
  }

  // ---- MMD-indirect (Clause-45 over Clause-22) folding ---------------------
  class MmdFolder {
    constructor() { this.func = {}; this.devad = {}; this.addr = {}; }
    reset() { this.func = {}; this.devad = {}; this.addr = {}; }
    feed(fr) {                                         // -> [text|null, event|null]
      const phy = fr.phyad, reg = fr.regad, op = fr.op, data = fr.data;
      if (reg === REG_MMD_CTRL && op === OP_W) {
        this.func[phy] = (data >> 14) & 0b11;
        this.devad[phy] = data & 0x1F;
        return [null, null];
      }
      if (reg === REG_MMD_DATA) {
        const devad = this.devad[phy], func = this.func[phy];
        if (devad === undefined || func === undefined) return [this._raw(fr), fr.noResponse ? this._event(fr) : null];
        if (func === 0b00) {
          if (op === OP_W) { this.addr[phy + "_" + devad] = data; return [null, null]; }
          return [this._raw(fr), fr.noResponse ? this._event(fr) : null];
        }
        const key = phy + "_" + devad;
        const addr = this.addr[key];
        const dn = DEVAD_NAME[devad] || ("dev" + devad);
        const where = addr !== undefined ? `MMD${devad}(${dn})[0x${hex4(addr)}]` : `MMD${devad}(${dn})[?]`;
        if (fr.noResponse) {
          return [`PHY${phy} ${where} read  !! NO RESP (TA=${fr.ta.toString(2).padStart(2, "0")})`,
                  { phy, devad, addr: addr === undefined ? null : addr, op, data: null, noResponse: true }];
        }
        const arrow = op === OP_R ? "=>" : "<=";
        const kind = op === OP_R ? "read " : "write";
        const text = `PHY${phy} ${where} ${kind} ${arrow} 0x${hex4(data)}`;
        const ev = { phy, devad, addr: addr === undefined ? null : addr, op, data };
        const inc = (func === 0b10) || (func === 0b11 && op === OP_W);
        if (inc && addr !== undefined) this.addr[key] = (addr + 1) & 0xFFFF;
        return [text, ev];
      }
      return [this._raw(fr), this._event(fr)];
    }
    _event(fr) {
      const ev = { phy: fr.phyad, devad: null, addr: fr.regad, op: fr.op, data: fr.noResponse ? null : fr.data };
      if (fr.noResponse) ev.noResponse = true;
      return ev;
    }
    _raw(fr) {
      if (fr.noResponse) return `PHY${fr.phyad} C22 REG${fr.regad} READ !! NO RESP (TA=${fr.ta.toString(2).padStart(2, "0")})`;
      const arrow = fr.op === OP_R ? "=>" : "<=";
      return `PHY${fr.phyad} C22 REG${fr.regad} ${OP_NAME[fr.op] || "?"} ${arrow} 0x${hex4(fr.data)}`;
    }
  }

  // ---- COBS framing (binary mode, delimiter 0x00) --------------------------
  function cobsDecode(buf) {
    const out = [];
    let i = 0; const n = buf.length;
    while (i < n) {
      const code = buf[i]; i++;
      if (code === 0) break;
      for (let k = 0; k < code - 1; k++) if (i + k < n) out.push(buf[i + k]);
      i += code - 1;
      if (code < 0xFF && i < n) out.push(0);
    }
    return out;
  }
  // wire=1: split a COBS stream on 0x00, decode each non-empty segment
  function deframe(data) {
    const recs = [];
    let seg = [];
    for (let i = 0; i < data.length; i++) {
      if (data[i] === 0) { if (seg.length) recs.push(cobsDecode(seg)); seg = []; }
      else seg.push(data[i]);
    }
    if (seg.length) recs.push(cobsDecode(seg));
    return recs;
  }

  // is a 0xFF-delimited segment an ASCII metadata block (0xFE[…]0xFE)?
  function metaText(seg) {
    if (seg.length < 3 || seg[0] !== 0xFE || seg[seg.length - 1] !== 0xFE) return null;
    if (seg[1] !== 0x5B /* '[' */) return null;
    for (let i = 1; i < seg.length - 1; i++) if (seg[i] < 0x20 || seg[i] > 0x7E) return null;
    return String.fromCharCode.apply(null, Array.prototype.slice.call(seg, 1, seg.length - 1));
  }

  // wire=2: demultiplex the shared transport envelope. 0xFF = boundary; per
  // segment, dispatch on leading byte (see header note). Returns decoded records
  // plus any losses (0xFD) and metadata strings encountered.
  function demuxV2(data) {
    const recs = [], metas = []; let losses = 0;
    let seg = [];
    const flush = () => {
      if (!seg.length) return;
      if (seg.length === 1 && seg[0] === 0xFD) { losses++; }
      else {
        const m = metaText(seg);
        if (m !== null) metas.push(m);
        else { const d = COBS.cobsFfDecode(Uint8Array.from(seg)); if (d) recs.push(Array.from(d)); }
      }
      seg = [];
    };
    for (let i = 0; i < data.length; i++) {
      if (data[i] === 0xFF) flush();
      else seg.push(data[i]);
    }
    flush();
    return { recs, metas, losses };
  }

  function parseRecord(rec) {
    if (rec.length < 12) return null;
    const u = Uint8Array.from(rec);
    const dv = new DataView(u.buffer);
    const type = u[0];
    const t_us = dv.getUint32(1, true);
    const dur_us = dv.getUint16(5, true);
    const onset_us = dv.getUint16(7, true);
    const flags = u[9];
    const n = dv.getUint16(10, true);
    const payload = rec.slice(12, 12 + n);
    if (payload.length !== n) return null;
    return { type, t_us, dur_us, onset_us, flags, n, payload };
  }

  // wire defaults to 1. Returns { records, metas, losses }.
  function recordsFromBinary(data, wire) {
    const records = [];
    let metas = [], losses = 0, recs;
    if (wire === 2) { const d = demuxV2(data); recs = d.recs; metas = d.metas; losses = d.losses; }
    else recs = deframe(data);
    for (const rec of recs) {
      const r = parseRecord(rec);
      if (r && r.type === 0x01) records.push(r);
    }
    return { records, metas, losses };
  }

  // parse a 0xFE[…]0xFE ASCII block into a tagged object. The device emits only
  // "[MODE … wire=N]" (active mode + format tag) and, under -D DIAG, "[T …]" telemetry;
  // anything else falls through to { tag, raw }. (The device advertises MODE only;
  // protocol/codec is a host concern, never on the wire.)
  function parseMeta(text) {
    const body = text.replace(/^\[|\]$/g, "").trim();
    const tok = body.split(/\s+/);
    const tag = tok[0];
    if (tag === "MODE") {
      const o = { tag, mode: tok[1] || null };
      for (const t of tok.slice(2)) { const [k, v] = t.split("="); if (k) o[k] = v; }
      if (o.wire !== undefined) o.wire = parseInt(o.wire, 10);
      return o;
    }
    return { tag, raw: text };
  }

  // ---- stitch cap-split (FLAG_CONTINUED) records into logical bursts -------
  class BurstJoiner {
    constructor() { this._reset(); }
    _reset() { this._pay = []; this._t0 = null; this._onset = 0; this._dur = 0; this._flags = 0; }
    feed(rec) {
      if (this._t0 === null) { this._t0 = rec.t_us; this._onset = rec.onset_us; }
      this._pay = this._pay.concat(rec.payload);
      this._dur += rec.dur_us;
      this._flags |= rec.flags;
      if (rec.flags & FLAG_CONTINUED) return null;
      // valid_bits lives in the TERMINAL record's flags (continued records are whole bytes).
      const validBits = (rec.flags >> VALIDBITS_SHIFT) & VALIDBITS_MASK;
      const out = { t_us: this._t0, dur_us: this._dur, onset_us: this._onset,
                    flags: this._flags & ~FLAG_CONTINUED, payload: this._pay, valid_bits: validBits };
      this._reset();
      return out;
    }
  }
  function logicalBursts(records) {
    const out = []; let j = new BurstJoiner();
    for (const r of records) {
      if (r.flags & (FLAG_OVF_PIOC | FLAG_OVF_RAM)) j = new BurstJoiner();
      const lb = j.feed(r); if (lb) out.push(lb);
    }
    return out;
  }

  // MDC CLOCK frequency in kHz — NOT a throughput. MDIO clocks exactly 1 bit per
  // MDC cycle, so cycles/s == bits/s numerically; we report the physical quantity
  // of interest (the clock), since burst/average bit-rate is biased by inter-frame
  // idle. onset_us spans ONSET_BITS MDC cycles measured on an empty post-idle ring.
  function onsetMdcKHz(onset_us) {
    if (!onset_us) return null;
    const f = ONSET_BITS / (onset_us / 1e6) / 1e3;     // kHz (== kbit/s value, 1 bit/cycle)
    if (f < 50 || f > 20000) return null;              // plausible MDC range: 50 kHz .. 20 MHz
    return f;
  }

  // ---- top-level: binary capture -> folded MDIO events ---------------------
  // wire defaults to 1. Returns
  //   { frames, events:[{phy,devad,addr,op,data,text}], bursts, metas:[…], losses }.
  // metas are parsed transport blocks (parseMeta); the MDC clock comes from each
  // record's onset_us (onsetMdcKHz), not a meta block.
  function decodeCapture(data, wire) {
    const { records, metas, losses } = recordsFromBinary(data, wire);
    const bursts = logicalBursts(records);
    const folder = new MmdFolder();
    const stream = new BitStream();                 // carries frames across burst cuts
    const frames = [], events = [];
    for (const lb of bursts) {
      if (lb.flags & (FLAG_OVF_PIOC | FLAG_OVF_RAM)) {
        stream.reset();
        folder.reset();
      }
      for (const fr of stream.feed(burstToBits(lb.payload, lb.valid_bits))) {
        frames.push(fr);
        const [text, ev] = folder.feed(fr);
        if (ev) events.push(Object.assign({ text }, ev));
      }
      if (lb.valid_bits) stream.reset();            // partial flushed + eMCU restarted -> next burst
    }                                               // isn't bit-contiguous (keep the folder for MMD)
    return { frames, events, bursts, metas: metas.map(parseMeta), losses };
  }

  root.MDIO = {
    OP_R, OP_W, OP_NAME, DEVAD_NAME, FLAG_OVF_PIOC, FLAG_OVF_RAM, FLAG_CONTINUED,
    hex4, bytesToBits, burstToBits, decodeBits, BitStream, MmdFolder, cobsDecode, deframe, demuxV2, metaText,
    parseRecord, recordsFromBinary, parseMeta, BurstJoiner, logicalBursts,
    onsetMdcKHz, decodeCapture,
  };
})(typeof module !== "undefined" && module.exports ? module.exports : (typeof window !== "undefined" ? window : globalThis));
