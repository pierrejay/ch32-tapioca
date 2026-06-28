// mdio_command.hpp - host->device MDIO master command parser.
//
// The host sends, over the CDC RX line (phytool-style grammar, base-0 numbers):
//
//     !read  <phy>/<reg>            e.g.  !read 1/4      !read 0x1/0x4
//     !write <phy>/<reg> <val16>    e.g.  !write 1/4 0x1A2B
//     !print <phy>                  bulk-read regs 0..31
//
// phy/reg are 0..31 (5-bit Clause-22 addresses), val is 0..0xFFFF. Parsing is STRICT: a
// malformed line - out-of-range field, missing '/', trailing junk - leaves valid=false, so
// CDC line noise can never drive the bus. Clause 22 only (Clause 45 is a later, CPU-only add:
// the wire grammar would gain a <phy>:<dev>/<reg> path form to match phytool).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace MdioCmd {

enum class Op : uint8_t { None, Read, Write, Print };

struct Command {
    bool     valid = false;
    Op       op    = Op::None;
    uint8_t  phy   = 0;           // 0..31
    uint8_t  reg   = 0;           // 0..31 (unused for Print)
    uint16_t val   = 0;           // write data (unused for Read/Print)
};

inline const char* skipWs(const char* p, const char* end) {
    while (p < end && (*p == ' ' || *p == '\t')) p++;
    return p;
}

// Base-0 unsigned parse: "0x.."/"0X.." = hex, else decimal. Advances p past the digits.
// Returns false if no digit is consumed or the value exceeds 'limit' (range-reject).
inline bool parseUint(const char*& p, const char* end, uint32_t limit, uint32_t& out) {
    uint32_t v = 0; bool any = false; uint32_t base = 10;
    if (p + 1 < end && p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) { base = 16; p += 2; }
    while (p < end) {
        uint32_t d;
        char c = *p;
        if (c >= '0' && c <= '9') d = (uint32_t)(c - '0');
        else if (base == 16 && c >= 'a' && c <= 'f') d = (uint32_t)(c - 'a' + 10);
        else if (base == 16 && c >= 'A' && c <= 'F') d = (uint32_t)(c - 'A' + 10);
        else break;
        v = v * base + d;
        if (v > limit) return false;          // out of range -> reject the whole line
        any = true; p++;
    }
    if (!any) return false;
    out = v; return true;
}

// Match a NUL-terminated keyword at p; advances p past it on success.
inline bool verbIs(const char*& p, const char* end, const char* kw) {
    size_t i = 0;
    while (p + i < end && kw[i] && p[i] == kw[i]) i++;
    if (kw[i] != '\0') return false;
    p += i; return true;
}

// Parse one command line (NUL not required; len bounds it). Strict: a valid line is exactly
// one of the three forms with in-range fields and no trailing tokens.
inline Command parse(const char* line, size_t len) {
    Command r;

    // trim trailing CR/LF/space
    while (len && (line[len - 1] == '\r' || line[len - 1] == '\n' ||
                   line[len - 1] == ' '  || line[len - 1] == '\t')) len--;
    const char* p   = line;
    const char* end = line + len;

    p = skipWs(p, end);
    if (p >= end || *p != '!') return r;                  // commands start with '!'
    p++;

    Op op;
    if      (verbIs(p, end, "read"))  op = Op::Read;
    else if (verbIs(p, end, "write")) op = Op::Write;
    else if (verbIs(p, end, "print")) op = Op::Print;
    else return r;                                        // unknown verb
    if (p >= end || (*p != ' ' && *p != '\t')) return r;  // need a space after the verb
    p = skipWs(p, end);

    uint32_t phy = 0, reg = 0, val = 0;
    if (!parseUint(p, end, 31, phy)) return r; // <phy>
    if (op != Op::Print) {
        if (p >= end || *p != '/') return r;              // '/' separator
        p++;
        if (!parseUint(p, end, 31, reg)) return r; // <reg>
    }
    if (op == Op::Write) {
        p = skipWs(p, end);
        if (p >= end) return r;                           // a value is required
        if (!parseUint(p, end, 0xFFFF, val)) return r; // <val16>
    }

    // strict: only trailing whitespace may follow (this drives hardware - reject near-misses)
    p = skipWs(p, end);
    if (p != end) return r;

    r.valid = true; r.op = op;
    r.phy = (uint8_t)phy; r.reg = (uint8_t)reg; r.val = (uint16_t)val;
    return r;
}

} // namespace MdioCmd
