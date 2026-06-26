// cobs.js — COBS framing, two keyings. 1:1 mirror of src/cobs.h.
//
//   cobsEncode / cobsDecode     : standard COBS, delimiter 0x00.
//   cobsFfEncode / cobsFfDecode : COBS keyed on 0xFF (what the records use) — encoded payload
//                                 never contains 0xFF, so 0xFF stays the single
//                                 frame boundary in both capture modes.
//
// decode() returns null on a malformed stream (matching the C `return 0`).
// Inputs/outputs are Uint8Array (or plain arrays of bytes).
(function (root) {
  'use strict';

  function cobsEncode(input) {
    const inp = input, len = inp.length;
    const out = new Uint8Array(len + ((len / 254) | 0) + 1);
    let read = 0, write = 1, codeIdx = 0, code = 1;
    while (read < len) {
      if (inp[read] === 0x00) {
        out[codeIdx] = code; code = 1; codeIdx = write++; read++;
      } else {
        out[write++] = inp[read++]; code++;
        if (code === 0xFF) { out[codeIdx] = code; code = 1; codeIdx = write++; }
      }
    }
    out[codeIdx] = code;
    return out.subarray(0, write);
  }

  function cobsDecode(input) {
    const inp = input, len = inp.length;
    const out = new Uint8Array(len);
    let read = 0, write = 0;
    while (read < len) {
      const code = inp[read];
      if (code === 0x00) return null;
      read++;
      if (read + (code - 1) > len) return null;
      for (let i = 1; i < code; i++) out[write++] = inp[read++];
      if (code !== 0xFF && read < len) out[write++] = 0x00;
    }
    return out.subarray(0, write);
  }

  function cobsFfEncode(input) {
    const inp = input, len = inp.length;
    const out = new Uint8Array(len + ((len / 253) | 0) + 1);
    let read = 0, write = 1, codeIdx = 0, code = 1;
    while (read < len) {
      if (inp[read] === 0xFF) {
        out[codeIdx] = code; code = 1; codeIdx = write++; read++;
      } else {
        out[write++] = inp[read++]; code++;
        if (code === 0xFE) { out[codeIdx] = code; code = 1; codeIdx = write++; }
      }
    }
    out[codeIdx] = code;
    return out.subarray(0, write);
  }

  function cobsFfDecode(input) {
    const inp = input, len = inp.length;
    const out = new Uint8Array(len);
    let read = 0, write = 0;
    while (read < len) {
      const code = inp[read];
      if (code === 0xFF || code === 0x00) return null;  // 0xFF=boundary, 0x00=invalid code ([0x01,0xFE])
      read++;
      if (read + (code - 1) > len) return null;
      for (let i = 1; i < code; i++) out[write++] = inp[read++];
      if (code !== 0xFE && read < len) out[write++] = 0xFF;
    }
    return out.subarray(0, write);
  }

  const api = { cobsEncode, cobsDecode, cobsFfEncode, cobsFfDecode };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.COBS = api;
})(typeof window !== 'undefined' ? window : globalThis);
