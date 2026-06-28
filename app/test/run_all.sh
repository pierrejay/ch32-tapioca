#!/usr/bin/env bash
# run_all.sh — host-side codec/framing test suite (no hardware required).
# Needs node + cc (clang/gcc) for the C<->JS parity & framer round-trip stages.
set -uo pipefail
cd "$(dirname "$0")"
fail=0
# the dashboard inlines test/cobs.js + test/mdio_codec.js — fail if it drifted from source
echo "=== inline sync (index.html <- test/*.js) ==="
node ../web-sniffer/inline.mjs --check || fail=1
echo
for t in cobs_test.js mdio_codec_test.js framer_test.js mode_command_test.js mdio_command_test.js mdio_parity_test.js; do
  echo "=== $t ==="
  node "$t" || fail=1
  echo
done
# headless dashboard smoke (skips itself if Chrome is absent)
echo "=== dashboard_smoke.mjs ==="
node dashboard_smoke.mjs || fail=1
echo
if [ "$fail" -eq 0 ]; then echo "✅ SUITE GREEN"; else echo "❌ SUITE HAS FAILURES"; fi
exit "$fail"
