// mdio_parity_test.js — lock the JS MDIO codec to the Python source of truth.
//
//   node mdio_parity_test.js        (needs python3 + scripts/mdio_lib.py)
//
// Builds a wire=2 (COBS-0xFF) capture exercising C22 reads, MMD-indirect folding
// with post-increment, and a CONTINUED burst split across two records, then
// decodes it with BOTH app/test/mdio_codec.js (JS) and scripts/mdio_lib.py (Python)
// and asserts identical folded event streams. Guards the codec against drift.
'use strict';
const M = require('./mdio_codec.js').MDIO;
const { cobsFfEncode } = require('./cobs.js');
const { execFileSync } = require('child_process');
const path = require('path');

let pass = 0, fail = 0;
const ok = (c, m) => { if (c) pass++; else { fail++; console.log('  ✗ ' + m); } };

// ---- builders (wire=2: COBS-0xFF record + 0xFF boundary) -------------------
const pushN = (b, v, n) => { for (let i = n - 1; i >= 0; i--) b.push((v >> i) & 1); };
function frameBits(op, phy, reg, ta, data) {
  const b = []; for (let i = 0; i < 32; i++) b.push(1);
  b.push(0, 1); pushN(b, op, 2); pushN(b, phy, 5); pushN(b, reg, 5); pushN(b, ta, 2); pushN(b, data, 16);
  return b;
}
function packBits(bits) { while (bits.length % 8) bits.push(1); const o = []; for (let i = 0; i < bits.length; i += 8) { let v = 0; for (let k = 0; k < 8; k++) v = (v << 1) | bits[i + k]; o.push(v); } return o; }
const p16 = (a, v) => a.push(v & 255, (v >> 8) & 255), p32 = (a, v) => a.push(v & 255, (v >> 8) & 255, (v >> 16) & 255, (v >> 24) & 255);
function record(payload, flags, t) {
  const buf = [0x01]; p32(buf, t); p16(buf, 200); p16(buf, 56); buf.push(flags); p16(buf, payload.length);
  for (const b of payload) buf.push(b);
  return [...cobsFfEncode(Uint8Array.from(buf)), 0xFF];
}
const R = M.OP_R, W = M.OP_W;
function mmdRead(phy, dev, addr, data, postInc) {
  return [].concat(frameBits(W, phy, 13, 0x10, (0b00 << 14) | dev), frameBits(W, phy, 14, 0x10, addr),
    frameBits(W, phy, 13, 0x10, ((postInc ? 0b10 : 0b01) << 14) | dev), frameBits(R, phy, 14, 0x00, data));
}
// burst 1: C22 + an MMD post-increment sequence, then SPLIT across two records
let b1 = [].concat(frameBits(R, 1, 0, 0x796D), frameBits(R, 1, 16, 0x00C1),
                   mmdRead(1, 7, 0x0202, 0x8000, false), mmdRead(1, 31, 0x060C, 0x1234, true));
const pay1 = packBits(b1);
const cut = Math.floor(pay1.length / 2);              // tear mid-burst -> CONTINUED
// burst 2: relies on PHY1/MMD31 post-incremented address from burst 1 (folder state)
let b2 = [].concat(frameBits(W, 1, 14, 0x10, 0x060D), frameBits(R, 1, 14, 0x00, 0xBEEF),  // MMD31 data @ post-inc'd addr
                   frameBits(R, 2, 1, 0x5A5A));
const pay2 = packBits(b2);

const capture = [
  ...record(pay1.slice(0, cut), M.FLAG_CONTINUED, 100),
  ...record(pay1.slice(cut), 0, 110),
  ...record(pay2, 0, 200),
];
const data = Uint8Array.from(capture);

// ---- JS decode --------------------------------------------------------------
const jsEvents = M.decodeCapture(data, 2).events.map(e => [e.phy, e.devad, e.addr, e.op, e.data]);

// ---- Python decode (source of truth) ---------------------------------------
const py = `
import sys, json
sys.path.insert(0, ${JSON.stringify(path.resolve(__dirname, '..', '..', 'scripts'))})
import mdio_lib as m
data = bytes.fromhex(sys.argv[1])
folder = m.MmdFolder(); out = []
for lb in m.logical_bursts(m.records_from_binary(data)):
    for fr in m.decode_bits(m.bytes_to_bits(lb.payload)):
        text, ev = folder.feed(fr)
        if ev: out.append([ev["phy"], ev["devad"], ev["addr"], ev["op"], ev["data"]])
print(json.dumps(out))
`;
const hex = Array.from(data, b => b.toString(16).padStart(2, '0')).join('');
let pyEvents;
try {
  pyEvents = JSON.parse(execFileSync('python3', ['-c', py, hex], { encoding: 'utf8' }).trim());
} catch (e) {
  console.log('⚠ python3/mdio_lib unavailable, skipping parity (' + (e.message || e).split('\n')[0] + ')');
  process.exit(0);
}

// ---- compare ----------------------------------------------------------------
ok(jsEvents.length > 0, `JS produced events (${jsEvents.length})`);
ok(jsEvents.length === pyEvents.length, `event count JS=${jsEvents.length} PY=${pyEvents.length}`);
ok(JSON.stringify(jsEvents) === JSON.stringify(pyEvents), 'JS vs Python folded events differ');
// the post-increment + cross-burst folder state must have resolved MMD31[0x060D]
const hasPostInc = jsEvents.some(e => e[1] === 31 && e[2] === 0x060D);
ok(hasPostInc, 'post-incremented MMD31[0x060D] resolved across the CONTINUED split + burst boundary');

if (fail) { console.log('JS:', JSON.stringify(jsEvents)); console.log('PY:', JSON.stringify(pyEvents)); }
console.log(`${pass} passed, ${fail} failed`);
console.log(fail === 0 ? '\n✅ MDIO JS↔PYTHON PARITY PASSED' : `\n❌ ${fail} FAILURE(S)`);
process.exit(fail === 0 ? 0 : 1);
