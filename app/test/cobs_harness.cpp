/* cobs_harness.cpp - exercise src/cobs.hpp from the command line for C++<->JS parity.
 *
 *   ./cobs_harness ff  <hexpayload>   -> prints "<encoded-hex> <roundtrip-ok>"
 *   ./cobs_harness std <hexpayload>   -> idem with standard 0x00 keying
 *
 * roundtrip-ok is 1 if decode(encode(x)) == x, else 0. Empty payload => "".
 * Built and driven by cobs_test.js.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../src/util/cobs.hpp"

static int hexval(int c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s ff|std [hex]\n", argv[0]); return 2; }
    bool ff = strcmp(argv[1], "ff") == 0;

    const char *hex = argc >= 3 ? argv[2] : "";
    size_t hlen = strlen(hex);
    size_t n = hlen / 2;
    uint8_t *in = (uint8_t *)malloc(n ? n : 1);
    for (size_t i = 0; i < n; i++)
        in[i] = (uint8_t)(hexval(hex[2 * i]) << 4 | hexval(hex[2 * i + 1]));

    uint8_t *enc = (uint8_t *)malloc(Cobs::ffMaxEncoded(n) + 8);
    uint8_t *dec = (uint8_t *)malloc(n + 8);
    size_t elen = ff ? Cobs::ffEncode(in, n, enc) : Cobs::encode(in, n, enc);
    size_t dlen = ff ? Cobs::ffDecode(enc, elen, dec) : Cobs::decode(enc, elen, dec);

    int ok = (dlen == n) && (memcmp(in, dec, n) == 0);
    for (size_t i = 0; i < elen; i++) printf("%02x", enc[i]);
    printf(" %d\n", ok);

    free(in); free(enc); free(dec);
    return 0;
}
