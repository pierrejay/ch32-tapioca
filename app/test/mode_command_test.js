// mode_command_test.js — strict parsing of "!mode …" + [MODE] marker parsing through
// the host meta parser. Compiles mode_cmd_harness.cpp (needs c++).
//
// The device knows MODES only (rle/clocked). Protocol/codec is a host concern and
// never reaches the firmware, so `!mode` carries NO proto token and the [MODE] block
// the firmware emits is just "[MODE <mode> wire=2]".
'use strict';
const M = require('./mdio_codec.js').MDIO;
const { execFileSync } = require('child_process');
const fs = require('fs'), path = require('path');

let pass = 0, fail = 0;
const ok = (c, m) => { if (c) pass++; else { fail++; console.log('  ✗ ' + m); } };

const src = path.join(__dirname, 'mode_cmd_harness.cpp');
const bin = path.join(__dirname, '.mode_cmd_harness');
try { execFileSync('c++', [src, '-O2', '-o', bin], { stdio: 'pipe' }); }
catch (e) {
  if (e.code === 'ENOENT') { console.log('⚠ c++ not found, skipping'); process.exit(0); }
  console.log('  ✗ mode_cmd harness compile FAILED:\n' + (e.stderr || e.message)); process.exit(1);
}
const run = (...a) => execFileSync(bin, a, { encoding: 'utf8' }).trim();

// ---- strict parsing: exactly `!mode <rle|clocked>`, nothing else ----------
const P = [
  ['!mode rle',                 '1 rle'],
  ['!mode clocked',               '1 clocked'],
  ['  !mode   clocked  \r\n',     '1 clocked'],          // surrounding whitespace tolerated
  ['!mode rle  ',               '1 rle'],            // trailing whitespace tolerated
  ['!mode clocked mdio',          '0 -'],                // STRICT: NO proto token (host-only concern)
  ['!mode clocked spi extra',     '0 -'],                // STRICT: trailing tokens -> reject
  ['!mode bogus',                 '0 -'],                // unknown mode
  ['!mod rle',                  '0 -'],                // wrong prefix
  ['mode rle',                  '0 -'],                // missing !
  ['!modeX rle',                '0 -'],                // no separator after !mode
  ['!mode rleX',                '0 -'],                // mode must be a whole token
  ['!mode',                       '0 -'],                // no mode
  ['random noise 0xFF',           '0 -'],                // line noise never triggers
];
for (const [line, want] of P) ok(run('parse', line) === want, `parse "${line}" -> "${run('parse', line)}" want "${want}"`);

// ---- the fixed [MODE] blocks the firmware emits parse cleanly, with NO proto ----
const p1 = M.parseMeta('[MODE clocked wire=2]');
ok(p1.tag === 'MODE' && p1.mode === 'clocked' && p1.wire === 2 && p1.proto === undefined,
   'parseMeta(clocked) = ' + JSON.stringify(p1));
const p2 = M.parseMeta('[MODE rle wire=2]');
ok(p2.tag === 'MODE' && p2.mode === 'rle' && p2.wire === 2 && p2.proto === undefined,
   'parseMeta(rle) = ' + JSON.stringify(p2));

try { fs.unlinkSync(bin); } catch (_) {}
console.log(`${pass} passed, ${fail} failed`);
console.log(fail === 0 ? '\n✅ ALL MODE COMMAND TESTS PASSED' : `\n❌ ${fail} FAILURE(S)`);
process.exit(fail === 0 ? 0 : 1);
