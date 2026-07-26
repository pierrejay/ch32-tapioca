// mdio_command_test.js — strict parsing of the MDIO master command line
// (`!read`/`!write`/`!print`, phytool-style path, base-0 numbers). Compiles
// mdio_cmd_harness.cpp (needs c++). The device only ever drives the bus on a well-formed
// line, so anything malformed — bad range, missing '/', trailing junk — must reject.
'use strict';
const { execFileSync } = require('child_process');
const fs = require('fs'), path = require('path');

let pass = 0, fail = 0;
const ok = (c, m) => { if (c) pass++; else { fail++; console.log('  ✗ ' + m); } };

const src = path.join(__dirname, 'mdio_cmd_harness.cpp');
const bin = path.join(__dirname, '.mdio_cmd_harness');
try { execFileSync('c++', [src, '-O2', '-o', bin], { stdio: 'pipe' }); }
catch (e) {
  if (e.code === 'ENOENT') { console.log('⚠ c++ not found, skipping'); process.exit(0); }
  console.log('  ✗ mdio_cmd harness compile FAILED:\n' + (e.stderr || e.message)); process.exit(1);
}
const run = (...a) => execFileSync(bin, a, { encoding: 'utf8' }).trim();

const P = [
  // ---- valid forms ----
  ['!read 1/4',               '1 read 1 4 -'],
  ['!read 0x1/0x4',           '1 read 1 4 -'],            // base-0: hex phy/reg
  ['  !read   1/4  \r\n',     '1 read 1 4 -'],            // surrounding whitespace tolerated
  ['!read 31/31',             '1 read 31 31 -'],          // max 5-bit addresses
  ['!write 1/4 0x1A2B',       '1 write 1 4 6699'],        // hex value -> decimal echo
  ['!write 1/4 6699',         '1 write 1 4 6699'],        // decimal value
  ['!write 0/0 0',            '1 write 0 0 0'],
  ['!write 2/3 0xFFFF',       '1 write 2 3 65535'],       // full 16-bit value
  ['!read 1:31/0x0300',       '1 read 1:31 768 -'],       // MMD / Clause-45-style path
  ['!read 0x1:0x1f/0x18f6',   '1 read 1:31 6390 -'],      // base-0 everywhere
  ['!write 1:7/0x020f 0xabcd','1 write 1:7 527 43981'],   // MMD write
  ['!print 7',                '1 print 7 - -'],
  ['!print 0x1F',             '1 print 31 - -'],
  // ---- strict rejects ----
  ['!read 32/4',              '0 -'],                     // phy out of range (>31)
  ['!read 1/32',              '0 -'],                     // reg out of range (>31)
  ['!read 1:32/0',            '0 -'],                     // MMD/devad out of range (>31)
  ['!read 1:31/0x10000',      '0 -'],                     // MMD reg out of range (>16-bit)
  ['!write 1/4 0x10000',      '0 -'],                     // value out of range (>0xFFFF)
  ['!read 1',                 '0 -'],                     // missing /reg
  ['!read 1/',                '0 -'],                     // missing reg after '/'
  ['!read 1:/0',              '0 -'],                     // missing MMD after ':'
  ['!print 1:31',             '0 -'],                     // print stays Clause-22 PHY-only
  ['!print 1/4',              '0 -'],                     // print takes phy only, no /reg
  ['!write 1/4',              '0 -'],                     // write needs a value
  ['!read 1/4 5',             '0 -'],                     // trailing junk
  ['!write 1/4 5 6',          '0 -'],                     // trailing junk after value
  ['!reads 1/4',              '0 -'],                     // unknown verb (whole-token)
  ['read 1/4',                '0 -'],                     // missing '!'
  ['!read1/4',                '0 -'],                     // no space after verb
  ['!read 1-4',               '0 -'],                     // wrong separator
  ['random noise 0xFF',       '0 -'],                     // line noise never drives the bus
  ['!read',                   '0 -'],                     // no args
];
for (const [line, want] of P)
  ok(run('parse', line) === want, `parse "${line}" -> "${run('parse', line)}" want "${want}"`);

// ---- line framing / overflow safety -----------------------------------------
// Once the 48-byte buffer overflows, the entire physical line is rejected. A valid
// !write suffix on that same line must never be surfaced as a second command.
const longPrefix = 'x'.repeat(48);
ok(run('frame', longPrefix + '!write 0/0 1\n') === 'invalid',
   'over-long line rejects a valid write suffix');
ok(run('frame', longPrefix + 'x\r\n') === 'invalid',
   'CRLF emits exactly one invalid event');
ok(run('frame', longPrefix + '!write 0/0 1\n!read 1/2\n') ===
   'invalid\nready:!read 1/2',
   'receiver resumes only with the line after the delimiter');
ok(run('frame', '!read 1/\x1e!read 1/2\n') === 'ready:!read 1/2',
   'new DTR session clears an unterminated line');
ok(run('frame', longPrefix + 'x\x1e!read 1/2\n') === 'ready:!read 1/2',
   'new DTR session clears overflow discard state');

try { fs.unlinkSync(bin); } catch (_) {}
console.log(`${pass} passed, ${fail} failed`);
console.log(fail === 0 ? '\n✅ ALL MDIO COMMAND TESTS PASSED' : `\n❌ ${fail} FAILURE(S)`);
process.exit(fail === 0 ? 0 : 1);
