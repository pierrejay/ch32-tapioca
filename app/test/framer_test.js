// framer_test.js — device wire=2 emitter (src/record_framer.h) <-> host decoder
// (mdio_codec.js demuxV2) end-to-end. Compiles framer_harness.cpp, decodes its
// output, asserts records / metadata / loss / CONTINUED stitching all agree.
//
//   node framer_test.js     (needs c++/clang)
'use strict';
const M = require('./mdio_codec.js').MDIO;
const { execFileSync } = require('child_process');
const fs = require('fs'), path = require('path');

let pass = 0, fail = 0;
const ok = (c, m) => { if (c) pass++; else { fail++; console.log('  ✗ ' + m); } };
const eqArr = (a, b) => a.length === b.length && a.every((v, i) => v === b[i]);

const src = path.join(__dirname, 'framer_harness.cpp');
const bin = path.join(__dirname, '.framer_harness');
let stream;
try {
  execFileSync('c++', [src, '-O2', '-o', bin], { stdio: 'pipe' });
  const hex = execFileSync(bin, { encoding: 'utf8' }).trim();
  stream = Uint8Array.from(hex.match(/../g).map(h => parseInt(h, 16)));
} catch (e) {
  if (e.code === 'ENOENT') { console.log('⚠ c++ not found, skipping framer round-trip'); process.exit(0); }
  console.log('  ✗ framer harness compile/run FAILED:\n' + (e.stderr || e.message)); process.exit(1);
}
try { fs.unlinkSync(bin); } catch (_) {}

// the emitted stream must never contain 0xFF except at boundaries; here we just
// confirm it decodes cleanly (no malformed COBS segment dropped).
const dec = M.recordsFromBinary(stream, 2);

// ---- records (pre-BurstJoiner) ---------------------------------------------
ok(dec.records.length === 3, `record count = ${dec.records.length}, want 3`);
const A = dec.records[0];
ok(A && A.t_us === 0x11223344 && A.dur_us === 0x0102 && A.onset_us === 56 && A.flags === 0,
   'record A header mismatch: ' + JSON.stringify(A && { t: A.t_us, d: A.dur_us, o: A.onset_us, f: A.flags }));
ok(A && eqArr(A.payload, [0x00, 0xFF, 0xFD, 0xFE, 0x01, 0xFF, 0xFF, 0x80, 0x7F]),
   'record A payload (envelope-sensitive bytes) corrupted: ' + JSON.stringify(A && A.payload));
ok(dec.records[1] && (dec.records[1].flags & M.FLAG_CONTINUED) !== 0, 'record B1 missing CONTINUED flag');
ok(dec.records[2] && (dec.records[2].flags & M.FLAG_CONTINUED) === 0, 'record B2 should not be CONTINUED');

// ---- metadata + loss --------------------------------------------------------
ok(dec.losses === 1, `loss count = ${dec.losses}, want 1`);
const metas = dec.metas.map(M.parseMeta);
const mode = metas.find(m => m.tag === 'MODE');
ok(mode && mode.mode === 'clocked' && mode.proto === undefined && mode.wire === 2, 'MODE meta: ' + JSON.stringify(mode));

// ---- CONTINUED stitching ----------------------------------------------------
const bursts = M.logicalBursts(dec.records);
ok(bursts.length === 2, `logical burst count = ${bursts.length}, want 2 (A, B1+B2)`);
ok(bursts[1] && eqArr(bursts[1].payload, [0xAA, 0xBB, 0xCC]),
   'CONTINUED stitch wrong: ' + JSON.stringify(bursts[1] && bursts[1].payload));

console.log(`${pass} passed, ${fail} failed`);
console.log(fail === 0 ? '\n✅ ALL FRAMER ROUND-TRIP TESTS PASSED' : `\n❌ ${fail} FAILURE(S)`);
process.exit(fail === 0 ? 0 : 1);
