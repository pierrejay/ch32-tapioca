// dashboard_smoke.mjs — headless smoke test for app/web-sniffer/index.html.
//
//   node dashboard_smoke.mjs
//
// Drives the standalone dashboard in headless Chrome via CDP (Node 22 global
// WebSocket) and asserts the CAN, MDIO and device-mode-reconciliation paths.
// Skips cleanly (exit 0) if Chrome isn't found, so the suite stays portable.
import { spawn } from 'node:child_process';
import { existsSync, rmSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const HERE = dirname(fileURLToPath(import.meta.url));
const INDEX = resolve(HERE, '..', 'web-sniffer', 'index.html');
const CHROME = [
  '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',
  '/Applications/Chromium.app/Contents/MacOS/Chromium',
  '/usr/bin/google-chrome', '/usr/bin/chromium', '/usr/bin/chromium-browser',
].find(existsSync);
if (!CHROME) { console.log('⚠ Chrome not found, skipping dashboard smoke test'); process.exit(0); }

// unique port + profile per process so back-to-back runs never collide on a lock
const PORT = 9300 + (process.pid % 600), URL = 'file://' + INDEX;
const PROFILE = `/tmp/cdp_dash_smoke_${process.pid}`;
try { rmSync(PROFILE, { recursive: true, force: true }); } catch {}
const sleep = ms => new Promise(r => setTimeout(r, ms));
const chrome = spawn(CHROME, ['--headless=new', '--disable-gpu', '--no-sandbox',
  `--remote-debugging-port=${PORT}`, `--user-data-dir=${PROFILE}`,
  '--no-first-run', '--allow-file-access-from-files', 'about:blank'], { stdio: 'ignore' });

let id = 0; const pending = new Map(), errors = [];
const send = (ws, method, params = {}) => new Promise(res => { const i = ++id; pending.set(i, res); ws.send(JSON.stringify({ id: i, method, params })); });
const j = async (p, m = 'GET') => (await fetch(`http://localhost:${PORT}${p}`, { method: m })).json();
let pass = 0, fail = 0;
const ok = (c, m) => { if (c) pass++; else { fail++; console.log('  ✗ ' + m); } };

try {
  for (let i = 0; i < 60; i++) { try { await j('/json/version'); break; } catch { await sleep(200); } }
  const tab = await j(`/json/new?${encodeURIComponent(URL)}`, 'PUT');
  const ws = new WebSocket(tab.webSocketDebuggerUrl);
  await new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
  ws.onmessage = e => {
    const m = JSON.parse(e.data);
    if (m.id && pending.has(m.id)) { pending.get(m.id)(m.result); pending.delete(m.id); }
    if (m.method === 'Runtime.exceptionThrown') errors.push(m.params.exceptionDetails.exception?.description || m.params.exceptionDetails.text);
    if (m.method === 'Runtime.consoleAPICalled' && m.params.type === 'error') errors.push('console.error: ' + m.params.args.map(a => a.value || a.description).join(' '));
  };
  await send(ws, 'Runtime.enable');
  await send(ws, 'Emulation.setFocusEmulationEnabled', { enabled: true });   // no bg-tab throttling
  const evalv = async expr => (await send(ws, 'Runtime.evaluate', { expression: expr, returnByValue: true })).result.value;
  await sleep(700);

  // ---- CAN demo ----
  await evalv(`document.querySelector('[data-proto=can]').click(); startDemo();`);
  await sleep(900); await evalv('tick()');
  const can = JSON.parse(await evalv(`JSON.stringify({ids:state.ids.size, frames:state.frames, rows:document.getElementById('grid').children.length})`));
  ok(can.ids > 0 && can.frames > 0 && can.rows > 0, `CAN demo produced traffic: ${JSON.stringify(can)}`);

  // ---- MDIO demo: codec + view + MMD fold + MDC clock + auto-detect mode ----
  await evalv(`if(state.demoTimer)stopDemo(); document.querySelector('[data-proto=mdio]').click(); startDemo();`);
  await sleep(1300); await evalv('tick()');
  const m = JSON.parse(await evalv(`JSON.stringify({frames:mdio.frames, regs:mdio.regs.size, clk:mdio.clk, rows:document.getElementById('mdio-grid').children.length, modeUi:modeS.ui, modeDev:modeS.device, pending:modeS.pending, proto:state.protocol})`));
  ok(m.frames > 0 && m.regs === 6, `MDIO demo decoded 6 registers: ${JSON.stringify(m)}`);
  ok(m.rows === 6, `MDIO table rendered 6 rows (got ${m.rows})`);
  ok(m.clk && Math.abs(m.clk - 1473.68) < 1, `MDC clock ~1474 kHz (got ${m.clk})`);
  ok(m.modeUi === 'clocked' && m.modeDev === 'clocked' && !m.pending, `mode auto-detected + reconciled to clocked: ${JSON.stringify(m)}`);
  // MMD folding present (resolved DEVAD names)
  const tgts = await evalv(`[...mdio.regs.values()].map(e=>e.target).join(' | ')`);
  ok(/MMD7\(AN\)/.test(tgts) && /MMD31\(vendor1\)/.test(tgts) && /0x060D/.test(tgts), `MMD fold + post-increment in targets: ${tgts}`);

  // ---- streaming BurstJoiner: a frame cut at BURST_MAX (FLAG_CONTINUED) arrives as two
  //      records across two mdioFeed() chunks. The persistent joiner must stitch them so the
  //      decode matches the whole frame (no phantom MMD reads at the split), and a continued
  //      record alone must decode NOTHING yet. Build records by hand (demo helper is flags=0). ----
  await evalv(`if(state.demoTimer)stopDemo();`);
  const splitRes = JSON.parse(await evalv(`(function(){
    const mkRec = (bytes, flags) => { const buf=[0x01,0xE8,0x03,0,0,0xC8,0,38,0,flags,bytes.length&255,(bytes.length>>8)&255,...bytes];
      return Uint8Array.from([...COBS.cobsFfEncode(Uint8Array.from(buf)), 0xFF]); };
    const pay = mdioPackBits(mdioFrameBits(MDIO.OP_R, 3, 1, 0x00, 0x1234));   // one C22 read frame
    resetMdio(); mdio._synced = true; mdioFeed(mkRec(pay, 0));                // baseline: whole record
    const whole = { frames: mdio.frames, regs: mdio.regs.size, loss: mdio.loss };
    resetMdio(); mdio._synced = true; const cut = pay.length >> 1;
    mdioFeed(mkRec(pay.slice(0, cut), MDIO.FLAG_CONTINUED));                  // chunk 1: continued half
    const mid = { frames: mdio.frames, regs: mdio.regs.size };
    mdioFeed(mkRec(pay.slice(cut), 0));                                       // chunk 2: terminal half
    const split = { frames: mdio.frames, regs: mdio.regs.size, loss: mdio.loss };
    return JSON.stringify({ whole, mid, split });
  })()`));
  ok(splitRes.mid.frames === 0, `continued record alone decodes nothing yet (got ${JSON.stringify(splitRes.mid)})`);
  ok(splitRes.split.frames === splitRes.whole.frames && splitRes.split.regs === splitRes.whole.regs && splitRes.split.loss === 0,
     `split frame stitched == whole, no spurious loss (${JSON.stringify(splitRes)})`);

  // ---- reconciliation timeout -> revert (flip with no confirming device) ----
  await evalv(`if(state.demoTimer)stopDemo();`);
  await evalv(`modeS.pending=false; modeS.ui='clocked'; modeS.device='clocked'; renderMode();
               state.port=null; modeFlip('rle');`);                // optimistic, no device to confirm
  const mid = JSON.parse(await evalv(`JSON.stringify({pending:modeS.pending, ui:modeS.ui})`));
  ok(mid.pending && mid.ui === 'rle', `flip is optimistic + locked (got ${JSON.stringify(mid)})`);
  await sleep(2600);                                                    // > 2.2s timeout
  const after = JSON.parse(await evalv(`JSON.stringify({pending:modeS.pending, ui:modeS.ui, dev:modeS.device})`));
  ok(!after.pending && after.ui === 'clocked', `timeout reverted to device mode + unlocked (got ${JSON.stringify(after)})`);

  // ---- boundary-aware [MODE]: a mid-payload 0xFE…[MODE rle]…0xFE must NOT spoof the mode ----
  await evalv(`metaSniff.atStart=true; metaSniff.inMeta=false; metaSniff.buf=[]; modeS.pending=false; modeS.ui='clocked'; modeS.device='clocked'; renderMode();`);
  const spoofed = await evalv(`(function(){ const t='[MODE rle wire=2]'; const s=[0x03,0x41,0x42,0xFE]; for(const c of t)s.push(c.charCodeAt(0)); s.push(0xFE,0xFF); sniffMeta(new Uint8Array(s)); return modeS.device; })()`);
  ok(spoofed === 'clocked', `mid-payload [MODE] spoof must not flip device mode (got ${spoofed})`);
  const real = await evalv(`(function(){ const t='[MODE rle wire=2]'; const s=[0xFE]; for(const c of t)s.push(c.charCodeAt(0)); s.push(0xFE,0xFF); sniffMeta(new Uint8Array(s)); return modeS.device; })()`);
  ok(real === 'rle', `leading-0xFE [MODE] segment must update device mode (got ${real})`);

  // ---- "traffic but no recognised frames" banner: appears on sustained throughput + no decode,
  //      clears the instant a frame returns. Drive lastDecodeT directly (no real 2 s wait). ----
  await evalv(`if(state.demoTimer)stopDemo();`);
  const warnOn = await evalv(`(function(){ state.port={}; state.protocol='can'; state._prevCount=state.frames;
     state.lastDecodeT=performance.now()-4000; state.usbWin=8000; tick(); return $('nosig').classList.contains('on'); })()`);
  ok(warnOn === true, `banner shows on traffic + no frames for >2 s (got ${warnOn})`);
  const warnOff = await evalv(`(function(){ state.frames++; state.usbWin=8000; tick(); return $('nosig').classList.contains('on'); })()`);
  ok(warnOff === false, `banner clears the instant a frame returns (got ${warnOff})`);
  await evalv(`state.port=null; state.usbWin=0; tick();`);

  ok(errors.length === 0, `no console errors/exceptions (got: ${errors.join(' ; ')})`);
  ws.close();
} catch (e) { fail++; console.log('  ✗ DRIVER ERROR: ' + e.message); }
finally { chrome.kill(); try { rmSync(PROFILE, { recursive: true, force: true }); } catch {} }

console.log(`${pass} passed, ${fail} failed`);
console.log(fail === 0 ? '\n✅ DASHBOARD SMOKE PASSED' : `\n❌ ${fail} FAILURE(S)`);
process.exit(fail === 0 ? 0 : 1);
