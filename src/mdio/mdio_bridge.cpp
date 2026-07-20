// mdio_bridge.cpp - USB ASCII command layer for the active MDIO master.
#ifdef RUN_MDIO_MASTER

#include "mdio_bridge.hpp"
#include "led_blinker.hpp"
#include "usb_cdc.hpp"

extern "C" {
#include <stdio.h>
}

namespace Mdio {

const char* UsbBridge::statusErr(Master::Status status)
{
    switch (status) {
    case Master::NoResp:  return "noresp";
    case Master::Timeout: return "timeout";
    default:                  return "busy"; // unreachable: only NoResp/Timeout reach here
    }
}

void UsbBridge::blinkOk()
{
    if (led_) led_->blink(50, 50);
}

void UsbBridge::blinkBad()
{
    if (led_) led_->blink(500, 100);
}

bool UsbBridge::startRead(const Command& c)
{
    Master::Result r = c.mmdAccess
        ? mdio_.readMmd(c.phy, c.mmd, c.reg, resp_)
        : mdio_.read(c.phy, (uint8_t)c.reg, resp_);
    if (r != Master::Ok) return false;
    pending_         = true;
    activeOp_        = Op::Read;
    activePhy_       = c.phy;
    activeMmdAccess_ = c.mmdAccess;
    activeMmd_       = c.mmd;
    activeReg_       = c.reg;
    return true;
}

bool UsbBridge::startWrite(const Command& c)
{
    Master::Result r = c.mmdAccess
        ? mdio_.writeMmd(c.phy, c.mmd, c.reg, c.val, resp_)
        : mdio_.write(c.phy, (uint8_t)c.reg, c.val, resp_);
    if (r != Master::Ok) return false;
    pending_         = true;
    activeOp_        = Op::Write;
    activePhy_       = c.phy;
    activeMmdAccess_ = c.mmdAccess;
    activeMmd_       = c.mmd;
    activeReg_       = c.reg;
    return true;
}

void UsbBridge::emitActiveResponse()
{
    const char* verb = (activeOp_ == Op::Read) ? "read" : "write";
    if (resp_.status == Master::Done) {
        if (activeOp_ == Op::Read) {
            if (activeMmdAccess_)
                printf("read %u:%u/0x%04X 0x%04X\r\n", activePhy_, activeMmd_,
                       activeReg_, resp_.value);
            else
                printf("read %u/%u 0x%04X\r\n", activePhy_, activeReg_, resp_.value);
        } else {
            if (activeMmdAccess_)
                printf("write %u:%u/0x%04X ok\r\n", activePhy_, activeMmd_, activeReg_);
            else
                printf("write %u/%u ok\r\n", activePhy_, activeReg_);
        }
    } else {
        if (activeMmdAccess_)
            printf("%s %u:%u/0x%04X err %s\r\n", verb, activePhy_, activeMmd_,
                   activeReg_, statusErr(resp_.status));
        else
            printf("%s %u/%u err %s\r\n", verb, activePhy_, activeReg_,
                   statusErr(resp_.status));
    }
}

void UsbBridge::emitPrintBusy(uint8_t phy)
{
    for (uint8_t reg = 0; reg < 32; reg++)
        printf("read %u/%u err busy\r\n", phy, reg);
}

void UsbBridge::pollResponse()
{
    if (pending_ && resp_.status != Master::Pending) {
        bool ok = (resp_.status == Master::Done);
        emitActiveResponse();
        pending_ = false;

        if (printActive_) {
            if (!ok) printAllOk_ = false;
        } else {
            if (ok) blinkOk(); else blinkBad();
        }
    }

    if (!printActive_ || pending_) return;

    if (printNextReg_ < 32) {
        Command c;
        c.valid = true;
        c.op = Op::Read;
        c.phy = printPhy_;
        c.reg = printNextReg_;
        if (startRead(c))
            printNextReg_++;
        return;
    }

    if (printAllOk_) blinkOk(); else blinkBad();
    printActive_ = false;
}

void UsbBridge::handleLine(const char* line, uint16_t len)
{
    Command c = parse(line, len);
    if (!c.valid) { blinkBad(); return; }

    switch (c.op) {
    case Op::Read:
        if (printActive_ || pending_ || !startRead(c)) {
            if (c.mmdAccess)
                printf("read %u:%u/0x%04X err busy\r\n", c.phy, c.mmd, c.reg);
            else
                printf("read %u/%u err busy\r\n", c.phy, c.reg);
            blinkBad();
        } else {
            blinkOk();
        }
        break;

    case Op::Write:
        if (printActive_ || pending_ || !startWrite(c)) {
            if (c.mmdAccess)
                printf("write %u:%u/0x%04X err busy\r\n", c.phy, c.mmd, c.reg);
            else
                printf("write %u/%u err busy\r\n", c.phy, c.reg);
            blinkBad();
        } else {
            blinkOk();
        }
        break;

    case Op::Print: {
        if (printActive_ || pending_) {
            emitPrintBusy(c.phy);
            blinkBad();
            break;
        }

        printActive_  = true;
        printAllOk_   = true;
        printPhy_     = c.phy;
        printNextReg_ = 0;
        blinkOk();
        Command r;
        r.valid = true;
        r.op = Op::Read;
        r.phy = printPhy_;
        r.reg = printNextReg_;
        if (startRead(r))
            printNextReg_++;
        else {
            printActive_ = false;
            emitPrintBusy(c.phy);
            blinkBad();
        }
        break;
    }

    default:
        blinkBad();
        break;
    }
}

void UsbBridge::pollCommand()
{
    uint8_t b = 0;
    while (usb_.available() && usb_.read(&b, 1) == 1) {
        if (b == '\n' || b == '\r') {
            if (lineLen_) {
                handleLine(line_, lineLen_);
                lineLen_ = 0;
            }
        } else if (lineLen_ < sizeof(line_)) {
            line_[lineLen_++] = (char)b;
        } else {
            lineLen_ = 0; // over-long line -> drop, resync on next newline
        }
    }
}

void UsbBridge::tick()
{
    pollResponse();
    pollCommand();
}

} // namespace Mdio

#endif // RUN_MDIO_MASTER
