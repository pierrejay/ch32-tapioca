// cobs_test.js — round-trip + edge cases + C<->JS parity for COBS-0xFF / COBS-0x00.
//
//   node cobs_test.js
//
// Compiles cobs_harness.cpp (needs c++/clang) and cross-checks every vector's
// encoded bytes against the JS implementation. If cc is unavailable the parity
// stage is skipped with a warning; JS round-trip still runs.
'use strict';
const { cobsEncode, cobsDecode, cobsFfEncode, cobsFfDecode } = require('./cobs.js');
const { execFileSync, spawnSync } = require('child_process');
const fs = require('fs');
const path = require('path');

const hex = (a) => Array.from(a, (b) => b.toString(16).padStart(2, '0')).join('');
const eq = (a, b) => a.length === b.length && a.every((v, i) => v === b[i]);

let pass = 0, fail = 0;
const ok = (cond, msg) => { if (cond) { pass++; } else { fail++; console.log('  ✗ ' + msg); } };

// ---- vectors: edge cases that the spec calls out + structural boundaries ----
function rep(byte, n) { return Uint8Array.from({ length: n }, () => byte); }
function seq(n) { return Uint8Array.from({ length: n }, (_, i) => i & 0xff); }

const vectors = [
  new Uint8Array([]),                         // empty
  new Uint8Array([0x00]),                      // single 0x00
  new Uint8Array([0xff]),                      // single 0xff (the ff-delimiter)
  new Uint8Array([0x42]),                      // single ordinary byte
  new Uint8Array([0x00, 0x00, 0x00]),
  new Uint8Array([0xff, 0xff, 0xff]),
  new Uint8Array([0x01, 0xff, 0x02, 0x00, 0x03]),
  new Uint8Array([0xff, 0x00, 0xff, 0x00]),
  rep(0xff, 300),                              // run of delimiters (ff-key stress)
  rep(0x00, 300),                              // run of delimiters (std-key stress)
  rep(0x42, 253),                              // exactly one ff full group - 1
  rep(0x42, 254),                              // ff continuation boundary
  rep(0x42, 255),
  rep(0xab, 254),                              // std continuation boundary
  rep(0xab, 255),
  seq(512),                                    // every byte value, twice
  seq(1000),
];

// deterministic pseudo-random fuzz (no Math.random dependency for repeatability)
let s = 0x12345678;
const rnd = () => ((s = (s * 1103515245 + 12345) & 0x7fffffff) >>> 8) & 0xff;
for (let t = 0; t < 200; t++) {
  const n = rnd() % 400;
  vectors.push(Uint8Array.from({ length: n }, () => rnd()));
}

// ---- JS round-trip + invariants --------------------------------------------
for (const v of vectors) {
  const ef = cobsFfEncode(v);
  ok(!ef.includes(0xff), `ff-encoded contains 0xff for ${hex(v).slice(0, 32)}…`);
  const df = cobsFfDecode(ef);
  ok(df && eq(Array.from(df), Array.from(v)), `ff round-trip len=${v.length}`);

  const es = cobsEncode(v);
  ok(!es.includes(0x00), `std-encoded contains 0x00 for len=${v.length}`);
  const ds = cobsDecode(es);
  ok(ds && eq(Array.from(ds), Array.from(v)), `std round-trip len=${v.length}`);
}

// malformed streams must be rejected (return null)
ok(cobsFfDecode(new Uint8Array([0x02, 0xaa, 0xff])) === null, 'ff-decode rejects embedded 0xff');
ok(cobsFfDecode(new Uint8Array([0x05, 0x01])) === null, 'ff-decode rejects truncated group');
ok(cobsFfDecode(new Uint8Array([0x00, 0x01])) === null, 'ff-decode rejects 0x00 code (codes are [0x01,0xFE])');
ok(cobsFfDecode(new Uint8Array([0x02, 0xaa, 0x00, 0x01])) === null, 'ff-decode rejects a 0x00 code mid-stream');
ok(cobsDecode(new Uint8Array([0x02, 0xaa, 0x00])) === null, 'std-decode rejects embedded 0x00');
ok(cobsDecode(new Uint8Array([0x05, 0x01])) === null, 'std-decode rejects truncated group');

console.log(`JS round-trip + invariants: ${pass} passed, ${fail} failed`);

// ---- C <-> JS parity --------------------------------------------------------
const harnessSrc = path.join(__dirname, 'cobs_harness.cpp');
const harnessBin = path.join(__dirname, '.cobs_harness');
let haveC = false;
try {
  execFileSync('c++', [harnessSrc, '-O2', '-o', harnessBin], { stdio: 'pipe' });
  haveC = true;
} catch (e) {
  if (e.code !== 'ENOENT') { console.log('  ✗ cobs harness compile FAILED:\n' + (e.stderr || e.message)); process.exit(1); }
  console.log('⚠ parity: c++ not found, skipping C<->JS cross-check');
}

if (haveC) {
  let pp = 0, pf = 0;
  for (const key of ['ff', 'std']) {
    const enc = key === 'ff' ? cobsFfEncode : cobsEncode;
    for (const v of vectors) {
      const r = spawnSync(harnessBin, [key, hex(v)], { encoding: 'utf8' });
      const [cHex, cOk] = r.stdout.trim().split(' ');
      const jHex = hex(enc(v));
      if (cHex === jHex && cOk === '1') pp++;
      else { pf++; console.log(`  ✗ parity ${key} len=${v.length}: C=${cHex} (ok=${cOk}) JS=${jHex}`); }
    }
  }
  console.log(`C<->JS parity: ${pp} passed, ${pf} failed`);
  fail += pf;
  try { fs.unlinkSync(harnessBin); } catch (_) {}
}

console.log(fail === 0 ? '\n✅ ALL COBS TESTS PASSED' : `\n❌ ${fail} FAILURE(S)`);
process.exit(fail === 0 ? 0 : 1);
