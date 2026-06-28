#!/usr/bin/env node
// inline.mjs — inline the codec source into index.html so the dashboard is a single
// portable file. The .js files stay the source of truth (the Node test suite imports
// them); this just copies them into the <script> blocks marked in index.html.
//
//   node app/web-sniffer/inline.mjs           regenerate index.html's inlined blocks
//   node app/web-sniffer/inline.mjs --check   exit 1 if index.html is stale (used by run_all.sh)
import { readFileSync, writeFileSync } from 'node:fs';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = dirname(fileURLToPath(import.meta.url));        // app/web-sniffer/
const INDEX = resolve(HERE, 'index.html');
const FILES = ['test/cobs.js', 'test/mdio_codec.js'];       // app/ paths, inlined in this order
const check = process.argv.includes('--check');
const esc = (s) => s.replace(/[.*+?^${}()|[\]\\/]/g, '\\$&');

const orig = readFileSync(INDEX, 'utf8');
let out = orig;
for (const f of FILES) {
  const code = readFileSync(resolve(HERE, '..', f), 'utf8').replace(/\s+$/, '');
  if (code.includes('</script')) { console.error(`✗ ${f} contains "</script" — cannot inline`); process.exit(2); }
  const open = `<!-- inline:${f} -->`, close = `<!-- /inline:${f} -->`;
  const re = new RegExp(esc(open) + '[\\s\\S]*?' + esc(close));
  if (!re.test(out)) { console.error(`✗ markers ${open} … ${close} not found in index.html`); process.exit(2); }
  out = out.replace(re, () => `${open}\n<script>\n${code}\n</script>\n${close}`);
}

if (check) {
  if (out !== orig) { console.error('✗ index.html inlined codec is OUT OF SYNC with test/*.js — run `node app/web-sniffer/inline.mjs`'); process.exit(1); }
  console.log('✓ index.html inlined codec in sync');
  process.exit(0);
}
if (out === orig) { console.log('✓ index.html already up to date'); process.exit(0); }
writeFileSync(INDEX, out);
console.log(`✓ inlined ${FILES.join(' + ')} into index.html`);
