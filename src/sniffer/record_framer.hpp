// record_framer.hpp - device-side record framing. Stateless namespace of
// inline helpers; the caller owns every output buffer.
//
// Emits the shared transport envelope (README §"USB wire protocol") decoded by the host
// in mdio_codec.js demuxV2():
//
//   record   : COBS-0xFF( type|t_us|dur_us|onset_us|flags|n|payload )  then 0xFF
//   metadata : 0xFE <ascii, starts with '['> 0xFE                      then 0xFF
//   loss     : 0xFD                                                    then 0xFF
//
// 0xFF is the single boundary in both modes; Cobs::ffEncode guarantees the encoded
// record never contains 0xFF. 0xFD/0xFE are only meaningful in segment-leading
// position, which is exactly how the host interprets them.
//
// frameRecord() needs the raw record laid out contiguously (header then payload):
//
//   uint8_t raw[RecordFramer::HEADER_LEN + BURST_MAX];
//   RecordFramer::putHeader(raw, RecordFramer::TYPE_MDIO, t_us, dur_us, onset_us, flags, n);
//   memcpy(raw + RecordFramer::HEADER_LEN, payload, n);
//   size_t w = RecordFramer::frameRecord(raw, RecordFramer::HEADER_LEN + n, out);
//   // out must hold Cobs::ffMaxEncoded(len) + 1 bytes
//
// Sized for the host parseRecord layout (little-endian).
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "../util/cobs.hpp"   // explicit path: resolves for BOTH the -I firmware build and
// the standalone C++ test harness (compiled without -I dirs)

namespace RecordFramer {

static constexpr size_t  HEADER_LEN = 12;
static constexpr uint8_t TYPE_MDIO  = 0x01;

// flags bits - mirror mdio_codec.js FLAG_*
static constexpr uint8_t FLAG_OVF_PIOC  = 0x01;
static constexpr uint8_t FLAG_OVF_RAM   = 0x02;
static constexpr uint8_t FLAG_CONTINUED = 0x04;

inline void putU16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
inline void putU32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

// Serialize the 12-byte record header in place. Returns HEADER_LEN.
inline size_t putHeader(uint8_t* raw, uint8_t type, uint32_t t_us,
                        uint16_t dur_us, uint16_t onset_us, uint8_t flags, uint16_t n)
{
    raw[0] = type;
    putU32(raw + 1, t_us);
    putU16(raw + 5, dur_us);
    putU16(raw + 7, onset_us);
    raw[9] = flags;
    putU16(raw + 10, n);
    return HEADER_LEN;
}

// COBS-0xFF encode raw[0..len) into out, append the 0xFF boundary.
// out must hold Cobs::ffMaxEncoded(len) + 1 bytes. Returns bytes written. 
inline size_t frameRecord(const uint8_t* raw, size_t len, uint8_t* out)
{
    size_t w = Cobs::ffEncode(raw, len, out);
    out[w++] = 0xFF;
    return w;
}

// Emit a metadata block 0xFE <ascii> 0xFE 0xFF. `ascii` is NUL-terminated and
// must start with '[' and stay printable (caller's contract). out must hold
// strlen(ascii) + 3 bytes. Returns bytes written (0 if ascii is malformed). 
inline size_t frameMeta(const char* ascii, uint8_t* out)
{
    if (!ascii || ascii[0] != '[') return 0;
    size_t w = 0;
    out[w++] = 0xFE;
    for (const char* s = ascii; *s; ++s) {
        uint8_t c = (uint8_t)*s;
        if (c < 0x20 || c > 0x7E) return 0;        // non-printable would corrupt the block
        out[w++] = c;
    }
    out[w++] = 0xFE;
    out[w++] = 0xFF;
    return w;
}

// Emit a hard loss boundary 0xFD 0xFF. out must hold 2 bytes.
inline size_t frameLoss(uint8_t* out)
{
    out[0] = 0xFD;
    out[1] = 0xFF;
    return 2;
}

} // namespace RecordFramer
