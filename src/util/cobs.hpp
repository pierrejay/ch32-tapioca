// cobs.hpp - COBS framing, two keyings. Stateless namespace of inline helpers.
//
// Shared by the firmware (clocked record framing) and mirrored 1:1 in
// app/test/cobs.js. The caller always owns the output buffer.
//
//   Cobs::encode / Cobs::decode     : standard COBS, delimiter = 0x00.
//   Cobs::ffEncode / Cobs::ffDecode : COBS keyed on 0xFF (what the records use) - identical
//                                     algorithm with the roles swapped:
//                                       delimiter byte    0x00 -> 0xFF
//                                       continuation code 0xFF -> 0xFE
//                                     => code bytes in [0x01,0xFE], data bytes in
//                                        [0x00,0xFE] => the encoded payload NEVER
//                                        contains 0xFF, so 0xFF stays THE single
//                                        frame boundary in both capture modes
//                                        (RLE run-bytes <= 0x80; SAMPLED = COBS-
//                                        0xFF) - zero sentinel collision.
//
// Worst-case encoded size is maxEncoded()/ffMaxEncoded(). decode()/ffDecode()
// return 0 on a malformed stream.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace Cobs {

inline constexpr size_t maxEncoded(size_t n)   { return n + n / 254 + 1; }
inline constexpr size_t ffMaxEncoded(size_t n) { return n + n / 253 + 1; }

// ---- standard COBS (delimiter 0x00) ----------------------------------------
inline size_t encode(const uint8_t* in, size_t length, uint8_t* out)
{
    size_t read_index = 0, write_index = 1, code_index = 0;
    uint8_t code = 1;
    while (read_index < length) {
        if (in[read_index] == 0x00) {
            out[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            out[write_index++] = in[read_index++];
            code++;
            if (code == 0xFF) {               // full group, force a code byte
                out[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    out[code_index] = code;
    return write_index;
}

// Returns decoded length, or 0 on a malformed stream.
inline size_t decode(const uint8_t* in, size_t length, uint8_t* out)
{
    size_t read_index = 0, write_index = 0;
    while (read_index < length) {
        uint8_t code = in[read_index];
        if (code == 0x00) return 0;                 // delimiter inside payload
        read_index++;
        if (read_index + (size_t)(code - 1) > length) return 0;  // truncated
        for (uint8_t i = 1; i < code; i++) out[write_index++] = in[read_index++];
        if (code != 0xFF && read_index < length) out[write_index++] = 0x00;
    }
    return write_index;
}

// ---- COBS keyed on 0xFF (the record framing) -------------------------------
inline size_t ffEncode(const uint8_t* in, size_t length, uint8_t* out)
{
    size_t read_index = 0, write_index = 1, code_index = 0;
    uint8_t code = 1;
    while (read_index < length) {
        if (in[read_index] == 0xFF) {
            out[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            out[write_index++] = in[read_index++];
            code++;
            if (code == 0xFE) {               // full group (max code != 0xFF)
                out[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    out[code_index] = code;
    return write_index;
}

// Returns decoded length, or 0 on a malformed stream.
inline size_t ffDecode(const uint8_t* in, size_t length, uint8_t* out)
{
    size_t read_index = 0, write_index = 0;
    while (read_index < length) {
        uint8_t code = in[read_index];
        if (code == 0xFF || code == 0x00) return 0;  // 0xFF=boundary, 0x00=invalid code ([0x01,0xFE])
        read_index++;
        if (read_index + (size_t)(code - 1) > length) return 0;  // truncated
        for (uint8_t i = 1; i < code; i++) out[write_index++] = in[read_index++];
        if (code != 0xFE && read_index < length) out[write_index++] = 0xFF;
    }
    return write_index;
}

} // namespace Cobs
