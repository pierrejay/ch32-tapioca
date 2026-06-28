// mdio_codec_test.js — wire=1 <-> wire=2 envelope parity + metadata/loss parsing.
//
//   node mdio_codec_test.js
//
// Builds real Clause-22 bursts, frames them BOTH as wire=1 (COBS-0x00, split on
// 0x00) and wire=2 (COBS-0xFF in the shared envelope, split on 0xFF), and asserts
// decodeCapture() yields identical MDIO events from both — i.e. the new wire=2
// path is correct relative to the Python-validated wire=1 path. Plus a known
// decode vector and the [MODE]/0xFD transport-block parsing.
'use strict';
const M = require('./mdio_codec.js').MDIO;
const { cobsEncode, cobsFfEncode } = require('./cobs.js');

let pass = 0, fail = 0;
const ok = (c, m) => { if (c) pass++; else { fail++; console.log('  ✗ ' + m); } };

// ---- builders ---------------------------------------------------------------
const pushN = (bits, v, n) => { for (let i = n - 1; i >= 0; i--) bits.push((v >> i) & 1); };
function mdioFrameBits(op, phyad, regad, data, ta) {
  const bits = [];
  if (ta === undefined) ta = (op === M.OP_R) ? 0b00 : 0b10;
  for (let i = 0; i < 32; i++) bits.push(1);   // preamble (>=32 ones)
  bits.push(0, 1);                              // ST = 01
  pushN(bits, op, 2); pushN(bits, phyad, 5); pushN(bits, regad, 5);
  pushN(bits, ta, 2);
  pushN(bits, data, 16);
  return bits;                                  // 64 bits = 8 bytes, byte-aligned
}
function bitsToBytes(bits) {
  const out = [];
  for (let i = 0; i < bits.length; i += 8) { let b = 0; for (let k = 0; k < 8; k++) b = (b << 1) | (bits[i + k] || 0); out.push(b); }
  return out;
}
const p16 = (a, v) => { a.push(v & 0xff, (v >> 8) & 0xff); };
const p32 = (a, v) => { a.push(v & 0xff, (v >> 8) & 0xff, (v >> 16) & 0xff, (v >> 24) & 0xff); };
function record(payload, o = {}) {
  const buf = [0x01]; p32(buf, o.t_us || 0); p16(buf, o.dur_us || 0); p16(buf, o.onset_us || 0);
  buf.push(o.flags || 0); p16(buf, payload.length);
  for (const b of payload) buf.push(b);
  return buf;
}
const frameSet = [
  [M.OP_R, 1, 0, 0x1234], [M.OP_W, 1, 4, 0xABCD], [M.OP_R, 7, 1, 0x5A5A],
  [M.OP_W, 2, 2, 0x0001], [M.OP_R, 5, 0, 0xFFFF], [M.OP_W, 3, 3, 0x00FF],
];
// each burst = a couple of frames; build a few records
const records = [
  record(bitsToBytes([].concat(mdioFrameBits(...frameSet[0]), mdioFrameBits(...frameSet[1]))), { t_us: 100, onset_us: 56 }),
  record(bitsToBytes([].concat(mdioFrameBits(...frameSet[2]), mdioFrameBits(...frameSet[3]))), { t_us: 200 }),
  record(bitsToBytes([].concat(mdioFrameBits(...frameSet[4]), mdioFrameBits(...frameSet[5]))), { t_us: 300 }),
];

// ---- envelope assembly ------------------------------------------------------
function wire1(recs) { const s = []; for (const r of recs) { s.push(...cobsEncode(Uint8Array.from(r))); s.push(0x00); } return Uint8Array.from(s); }
function wire2(recs, extras) {
  const s = [];
  for (const r of recs) { s.push(...cobsFfEncode(Uint8Array.from(r))); s.push(0xFF); }
  if (extras) for (const e of extras) { s.push(...e); s.push(0xFF); }
  return Uint8Array.from(s);
}
const asciiSeg = (txt) => [0xFE, ...Array.from(txt, c => c.charCodeAt(0)), 0xFE];

// ---- 1. wire1 <-> wire2 event parity ---------------------------------------
const d1 = M.decodeCapture(wire1(records), 1);
const d2 = M.decodeCapture(wire2(records), 2);
ok(d1.events.length === 6 && d2.events.length === 6, `event count (w1=${d1.events.length} w2=${d2.events.length}, want 6)`);
const key = e => `${e.op}/${e.phy}/${e.addr}/${e.data}`;
ok(JSON.stringify(d1.events.map(key)) === JSON.stringify(d2.events.map(key)), 'wire1 vs wire2 events differ');
// spot-check first event matches the source frame
ok(d1.events[0].op === M.OP_R && d1.events[0].phy === 1 && d1.events[0].addr === 0 && d1.events[0].data === 0x1234,
   'first decoded event != source C22 frame');

// ---- 2. transport metadata + loss in the wire=2 envelope -------------------
const d3 = M.decodeCapture(wire2(records, [
  asciiSeg('[MODE clocked wire=2]'),
  [0xFD],                                   // lone loss marker
]), 2);
ok(d3.events.length === 6, 'meta/loss injection corrupted record decode');
ok(d3.losses === 1, `loss count = ${d3.losses}, want 1`);
const mode = d3.metas.find(m => m.tag === 'MODE');
ok(mode && mode.mode === 'clocked' && mode.proto === undefined && mode.wire === 2, 'MODE meta mis-parsed: ' + JSON.stringify(mode));

// ---- 3. onset -> MDC kHz ----------------------------------------------------
ok(Math.abs(M.onsetMdcKHz(56) - 1000) < 1, `onsetMdcKHz(56us) = ${M.onsetMdcKHz(56)}, want ~1000 kHz`);
ok(M.onsetMdcKHz(0) === null, 'onsetMdcKHz(0) should be null');

// ---- 4. a 0xFE data byte inside a COBS-0xFF payload must NOT split ----------
// force a record whose COBS-0xFF encoding contains a 0xFE/0xFD data byte
const trick = record([0xFE, 0xFD, 0x00, 0xFE, 0xAB], { t_us: 9 });
const enc = Array.from(cobsFfEncode(Uint8Array.from(trick)));
ok(enc.includes(0xFE) || enc.includes(0xFD), 'expected a 0xFE/0xFD byte inside the test payload');
const d4 = M.recordsFromBinary(Uint8Array.from([...enc, 0xFF]), 2);
ok(d4.records.length === 1 && d4.losses === 0 && d4.metas.length === 0,
   `mid-payload 0xFD/0xFE wrongly treated as envelope marker: ${JSON.stringify({ r: d4.records.length, l: d4.losses, m: d4.metas.length })}`);

// ---- 5. missing MDIO read response: TA=11 + DATA=0xffff --------------------
// In MMD post-increment mode, a failed read must NOT advance the folded address.
const noRespPayload = bitsToBytes([].concat(
  mdioFrameBits(M.OP_W, 1, 13, (0b00 << 14) | 31),
  mdioFrameBits(M.OP_W, 1, 14, 0x060C),
  mdioFrameBits(M.OP_W, 1, 13, (0b10 << 14) | 31),
  mdioFrameBits(M.OP_R, 1, 14, 0xFFFF, 0b11),       // pulled-up/no slave response
  mdioFrameBits(M.OP_R, 1, 14, 0xBEEF, 0b00)        // should still be MMD31[0x060C]
));
const d5 = M.decodeCapture(wire2([record(noRespPayload)]), 2);
const mmd31 = d5.events.filter(e => e.devad === 31);
ok(mmd31.length === 2, `MMD no-response event count = ${mmd31.length}, want 2`);
ok(mmd31[0].noResponse === true && mmd31[0].data === null && mmd31[0].addr === 0x060C,
   `no-response event malformed: ${JSON.stringify(mmd31[0])}`);
ok(mmd31[1].noResponse !== true && mmd31[1].data === 0xBEEF && mmd31[1].addr === 0x060C,
   `MMD address advanced after failed read: ${JSON.stringify(mmd31[1])}`);

// ---- 6. continued records must decode only after an explicit terminal seam ----
const oneWrite = bitsToBytes(mdioFrameBits(M.OP_W, 1, 4, 0x3333));
const danglingContinued = M.decodeCapture(wire2([
  record(oneWrite, { flags: M.FLAG_CONTINUED }),
]), 2);
ok(danglingContinued.events.length === 0,
   `dangling continued record decoded without terminal: ${danglingContinued.events.length}`);
const closedContinued = M.decodeCapture(wire2([
  record(oneWrite, { flags: M.FLAG_CONTINUED }),
  record([]),
]), 2);
ok(closedContinued.events.length === 1 && closedContinued.events[0].data === 0x3333,
   `empty terminal did not close continued chain: ${JSON.stringify(closedContinued.events)}`);

// ---- 7. overflow is a loss seam before the flagged record, not after joining ----
const afterOvf = bitsToBytes(mdioFrameBits(M.OP_W, 1, 5, 0x4444));
const ovfSeam = M.decodeCapture(wire2([
  record(oneWrite, { flags: M.FLAG_CONTINUED }),
  record(afterOvf, { flags: M.FLAG_OVF_PIOC }),
]), 2);
ok(ovfSeam.events.length === 1 && ovfSeam.events[0].data === 0x4444,
   `overflow did not break continued chain: ${JSON.stringify(ovfSeam.events)}`);

// ---- 8. valid_bits keeps only the top k bits of the last byte (mid-byte flush) ----
// burstToBits is what reconstructs a non-byte-aligned stop exactly (no phantom bits).
ok(JSON.stringify(M.burstToBits([0xFF, 0xA0], 3)) === JSON.stringify([1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1]),
   'burstToBits(...,3) must keep the top 3 bits (1,0,1) of 0xA0');
ok(M.burstToBits([0xFF, 0xA0], 0).length === 16, 'burstToBits(...,0) keeps all 16 bits');
ok(M.burstToBits([0xAA], 5).length === 5, 'burstToBits yields exactly valid_bits bits');

// ---- 9. a mid-byte partial flush (valid_bits in flags) decodes the exact frame ----
// Firmware idle flush of a 65-clock frame: 33-one preamble + 32-bit frame = 65 bits =
// 8 full bytes + 1 partial bit (the DATA LSB) in a 9th byte (MSB-justified, low 7 zero
// padded); the terminal record carries valid_bits=1 in flags bits 3..5.
const f65 = [1].concat(mdioFrameBits(M.OP_W, 1, 4, 0x3333));   // 1 + 64 = 33 preamble + 32 frame
ok(f65.length === 65, `test setup: want 65 bits, got ${f65.length}`);
const p65 = bitsToBytes(f65);                                  // 9 bytes, last = 1 real bit + 7 pad
ok(p65.length === 9, `test setup: want 9 bytes, got ${p65.length}`);
const d9 = M.decodeCapture(wire2([record(p65, { flags: 1 << 3 })]), 2);   // valid_bits = 1
ok(d9.frames.length === 1 && d9.frames[0].data === 0x3333,
   `valid_bits=1 partial flush mis-decoded: ${JSON.stringify(d9.frames)}`);
ok(d9.bursts.length === 1 && d9.bursts[0].valid_bits === 1,
   `valid_bits not propagated onto the logical burst: ${JSON.stringify(d9.bursts[0] && d9.bursts[0].valid_bits)}`);

// ---- 10. a valid_bits seam must reset the carry (firmware restarted the eMCU) -------
// Burst A is an INCOMPLETE frame (last 2 data bits missing) flushed+restarted at idle
// (valid_bits>0). Burst B happens to start with 2 bits then a separate frame. If the
// carry bled across the seam, A's 30 frame bits + B's 2 bits would STITCH into a phantom
// REG4=0x3333. The reset-on-seam must drop A's tail -> only B's real REG1 frame survives.
const fb = mdioFrameBits(M.OP_W, 1, 4, 0x3333);       // 32 preamble + 32 frame
const aBits = fb.slice(0, 62);                         // preamble + 30/32 frame bits (incomplete)
const bBits = fb.slice(62).concat(mdioFrameBits(M.OP_W, 7, 1, 0x1234));  // the 2 missing bits + a clean frame
const d10 = M.decodeCapture(wire2([
  record(bitsToBytes(aBits), { flags: (62 % 8) << 3 }),  // valid_bits = 6 -> restart seam
  record(bitsToBytes(bBits)),
]), 2);
ok(d10.frames.length === 1 && d10.frames[0].regad === 1 && d10.frames[0].data === 0x1234,
   `valid_bits seam not reset -> phantom stitch: ${JSON.stringify(d10.frames.map(f => ({ r: f.regad, d: f.data })))}`);

console.log(`${pass} passed, ${fail} failed`);
console.log(fail === 0 ? '\n✅ ALL MDIO CODEC TESTS PASSED' : `\n❌ ${fail} FAILURE(S)`);
process.exit(fail === 0 ? 0 : 1);
