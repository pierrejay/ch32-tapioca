/* framer_harness.cpp - emit a wire=2 envelope with src/record_framer.hpp and print
 * it as hex, so framer_test.js can decode it with the host codec and confirm the
 * device emitter and the host demux agree. Scenario mirrored in framer_test.js.
 */
#include <stdio.h>
#include <string.h>
#include "../../src/sniffer/record_framer.hpp"

static uint8_t out[4096];
static size_t W;
static void put(size_t n) { W += n; }

static void rec(uint32_t t, uint16_t dur, uint16_t onset, uint8_t flags,
                const uint8_t *pl, uint16_t n) {
    uint8_t raw[RecordFramer::HEADER_LEN + 512];
    RecordFramer::putHeader(raw, RecordFramer::TYPE_MDIO, t, dur, onset, flags, n);
    memcpy(raw + RecordFramer::HEADER_LEN, pl, n);
    put(RecordFramer::frameRecord(raw, RecordFramer::HEADER_LEN + n, out + W));
}

int main(void) {
    /* Record A: payload packed with the envelope-sensitive bytes 0xFF/0xFD/0xFE */
    const uint8_t plA[] = { 0x00, 0xFF, 0xFD, 0xFE, 0x01, 0xFF, 0xFF, 0x80, 0x7F };
    rec(0x11223344u, 0x0102, 56, 0, plA, sizeof plA);

    put(RecordFramer::frameMeta("[MODE clocked wire=2]", out + W));

    /* Continued burst split across two records */
    const uint8_t plB1[] = { 0xAA, 0xBB };
    const uint8_t plB2[] = { 0xCC };
    rec(0x200, 0x10, 0, RecordFramer::FLAG_CONTINUED, plB1, sizeof plB1);
    rec(0x210, 0x08, 0, 0,                            plB2, sizeof plB2);

    put(RecordFramer::frameLoss(out + W));

    for (size_t i = 0; i < W; i++) printf("%02x", out[i]);
    printf("\n");
    return 0;
}
