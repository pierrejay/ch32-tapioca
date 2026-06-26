// mode_command.hpp - host->device runtime mode command. Stateless namespace of
// inline helpers; no allocation.
//
// The host sends, over the CDC RX line:
//
//     !mode <rle|clocked>\n
//
// The main loop accumulates a line and calls ModeCmd::parse(); on a valid command that
// changes the active mode it runs the stop->seam->reconfig->start sequence (main.cpp,
// dual-instance selector). Parsing is strict: a malformed line - or ANY trailing token -
// leaves valid=false, so line noise can never trigger a reconfig.
//
//   - Rle     = RLE datapath     (non-clocked: host recovers timing from the data)
//   - Clocked = SAMPLED datapath (external clock; sampled on the clock edge)
//
// The protocol/codec (can/dmx/mdio/spi/...) is purely a HOST concern: the device only
// knows the two capture MODES and never sees a protocol name. Each datapath advertises
// its own fixed identity ([MODE rle|clocked wire=2]); the host owns the
// protocol<->mode mapping and any compatibility warning. No protocol coupling reaches
// the firmware.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace ModeCmd {

enum class CaptureMode : uint8_t { Rle = 0, Clocked = 1 };

struct Command {
    bool        valid = false;                  // true iff the line is a well-formed command
    CaptureMode mode  = CaptureMode::Rle;
};

// case-sensitive token compare against a NUL-terminated keyword, given [p,end).
inline bool tokenIs(const char* p, const char* end, const char* kw) {
    size_t i = 0;
    while (p + i < end && kw[i] && p[i] == kw[i]) i++;
    return kw[i] == '\0' && (p + i == end || p[i] == ' ' || p[i] == '\t');
}

// Parse one command line (NUL not required; len bounds it). Strict: exactly
// `!mode <rle|clocked>` (leading/trailing whitespace tolerated, nothing else). 
inline Command parse(const char* line, size_t len) {
    Command r;

    // trim trailing CR/LF/space
    while (len && (line[len - 1] == '\r' || line[len - 1] == '\n' ||
                   line[len - 1] == ' '  || line[len - 1] == '\t')) len--;
    const char* p = line;
    const char* end = line + len;

    while (p < end && (*p == ' ' || *p == '\t')) p++;          // lead ws
    const char* pre = "!mode";
    size_t i = 0;
    while (p + i < end && pre[i] && p[i] == pre[i]) i++;
    if (pre[i] != '\0') return r;                              // no "!mode"
    p += i;
    if (p >= end || (*p != ' ' && *p != '\t')) return r;       // need a space
    while (p < end && (*p == ' ' || *p == '\t')) p++;          // ws before mode

    if (tokenIs(p, end, "rle"))          { r.mode = CaptureMode::Rle;     p += 3; }
    else if (tokenIs(p, end, "clocked")) { r.mode = CaptureMode::Clocked; p += 7; }
    else return r;                                            // unknown mode

    // strict: nothing but trailing whitespace may follow the mode token. This command
    // triggers a hardware reconfig, so a line that merely starts valid must be rejected. 
    while (p < end && (*p == ' ' || *p == '\t')) p++;
    if (p != end) return r;

    r.valid = true;
    return r;
}

} // namespace ModeCmd
