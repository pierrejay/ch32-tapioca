// mdio_bridge.hpp - USB ASCII command bridge for the active MDIO master.
//
// The host sends phytool-style lines over CDC:
//
//     !read  <phy>/<reg>            e.g.  !read 1/4      !read 0x1/0x4
//     !write <phy>/<reg> <val16>    e.g.  !write 1/4 0x1A2B
//     !read  <phy>:<mmd>/<reg16>    e.g.  !read 1:31/0x0300
//     !write <phy>:<mmd>/<reg16> <val16>
//     !print <phy>                  bulk-read regs 0..31
//
// Parsing is deliberately a private UsbBridge detail. Malformed lines never reach
// Mdio::Master, so CDC line noise cannot drive the bus.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "mdio_master.hpp"

class LedBlinker;
class UsbCdc;

namespace Mdio {

class UsbBridge {
public:
    UsbBridge(UsbCdc& usb, Master& mdio, LedBlinker* led = nullptr)
        : usb_(usb), mdio_(mdio), led_(led) {}

    void tick(); // drain RX, submit MDIO requests, emit completed responses

private:
    enum class Op : uint8_t { None, Read, Write, Print };

    struct Command {
        bool     valid = false;
        Op       op    = Op::None;
        uint8_t  phy   = 0;        // 0..31
        bool     mmdAccess = false;
        uint8_t  mmd   = 0;        // 0..31 (MMD/devad, unused for Clause-22)
        uint16_t reg   = 0;        // 0..31 for Clause-22, 0..0xffff for MMD
        uint16_t val   = 0;        // write data (unused for Read/Print)
    };

    static const char* skipWs(const char* p, const char* end)
    {
        while (p < end && (*p == ' ' || *p == '\t')) p++;
        return p;
    }

    static bool parseUint(const char*& p, const char* end, uint32_t limit,
                          uint32_t& out)
    {
        uint32_t v = 0;
        bool any = false;
        uint32_t base = 10;
        if (p + 1 < end && p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
            base = 16;
            p += 2;
        }

        while (p < end) {
            uint32_t d;
            char c = *p;
            if (c >= '0' && c <= '9') d = (uint32_t)(c - '0');
            else if (base == 16 && c >= 'a' && c <= 'f') d = (uint32_t)(c - 'a' + 10);
            else if (base == 16 && c >= 'A' && c <= 'F') d = (uint32_t)(c - 'A' + 10);
            else break;
            v = v * base + d;
            if (v > limit) return false;
            any = true;
            p++;
        }

        if (!any) return false;
        out = v;
        return true;
    }

    static bool verbIs(const char*& p, const char* end, const char* kw)
    {
        size_t i = 0;
        while (p + i < end && kw[i] && p[i] == kw[i]) i++;
        if (kw[i] != '\0') return false;
        p += i;
        return true;
    }

    static Command parse(const char* line, size_t len)
    {
        Command r;

        while (len && (line[len - 1] == '\r' || line[len - 1] == '\n' ||
                       line[len - 1] == ' '  || line[len - 1] == '\t')) len--;
        const char* p = line;
        const char* end = line + len;

        p = skipWs(p, end);
        if (p >= end || *p != '!') return r;
        p++;

        Op op;
        if      (verbIs(p, end, "read"))  op = Op::Read;
        else if (verbIs(p, end, "write")) op = Op::Write;
        else if (verbIs(p, end, "print")) op = Op::Print;
        else return r;
        if (p >= end || (*p != ' ' && *p != '\t')) return r;
        p = skipWs(p, end);

        uint32_t phy = 0, reg = 0, val = 0;
        bool mmdAccess = false;
        uint32_t mmd = 0;
        if (!parseUint(p, end, 31, phy)) return r;
        if (op != Op::Print) {
            if (p < end && *p == ':') {
                p++;
                if (!parseUint(p, end, 31, mmd)) return r;
                mmdAccess = true;
            }
            if (p >= end || *p != '/') return r;
            p++;
            if (!parseUint(p, end, mmdAccess ? 0xFFFF : 31, reg)) return r;
        }
        if (op == Op::Write) {
            p = skipWs(p, end);
            if (p >= end) return r;
            if (!parseUint(p, end, 0xFFFF, val)) return r;
        }

        p = skipWs(p, end);
        if (p != end) return r;

        r.valid = true;
        r.op = op;
        r.phy = (uint8_t)phy;
        r.mmdAccess = mmdAccess;
        r.mmd = (uint8_t)mmd;
        r.reg = (uint16_t)reg;
        r.val = (uint16_t)val;
        return r;
    }

    bool startRead(const Command& c);
    bool startWrite(const Command& c);
    void handleLine(const char* line, uint16_t len);
    void pollCommand();
    void pollResponse();
    void emitActiveResponse();
    void emitPrintBusy(uint8_t phy);
    void blinkOk();
    void blinkBad();

    static const char* statusErr(Master::Status status);

    UsbCdc&          usb_;
    Master&          mdio_;
    LedBlinker*      led_;
    char             line_[48] = {};
    uint16_t         lineLen_ = 0;
    Master::Response resp_;
    bool             pending_ = false;
    Op               activeOp_ = Op::Read;
    uint8_t          activePhy_ = 0;
    bool             activeMmdAccess_ = false;
    uint8_t          activeMmd_ = 0;
    uint16_t         activeReg_ = 0;
    bool             printActive_ = false;
    bool             printAllOk_ = true;
    uint8_t          printPhy_ = 0;
    uint8_t          printNextReg_ = 0;
};

} // namespace Mdio
