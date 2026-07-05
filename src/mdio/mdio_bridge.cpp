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

bool UsbBridge::startRead(uint8_t phy, uint8_t reg)
{
    if (mdio_.read(phy, reg, resp_) != Master::Ok) return false;
    pending_   = true;
    activeOp_  = Op::Read;
    activePhy_ = phy;
    activeReg_ = reg;
    return true;
}

bool UsbBridge::startWrite(uint8_t phy, uint8_t reg, uint16_t val)
{
    if (mdio_.write(phy, reg, val, resp_) != Master::Ok) return false;
    pending_   = true;
    activeOp_  = Op::Write;
    activePhy_ = phy;
    activeReg_ = reg;
    return true;
}

void UsbBridge::emitActiveResponse()
{
    const char* verb = (activeOp_ == Op::Read) ? "read" : "write";
    if (resp_.status == Master::Done) {
        if (activeOp_ == Op::Read)
            printf("read %u/%u 0x%04X\r\n", activePhy_, activeReg_, resp_.value);
        else
            printf("write %u/%u ok\r\n", activePhy_, activeReg_);
    } else {
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
        if (startRead(printPhy_, printNextReg_))
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
        if (printActive_ || pending_ || !startRead(c.phy, c.reg)) {
            printf("read %u/%u err busy\r\n", c.phy, c.reg);
            blinkBad();
        } else {
            blinkOk();
        }
        break;

    case Op::Write:
        if (printActive_ || pending_ || !startWrite(c.phy, c.reg, c.val)) {
            printf("write %u/%u err busy\r\n", c.phy, c.reg);
            blinkBad();
        } else {
            blinkOk();
        }
        break;

    case Op::Print:
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
        if (startRead(printPhy_, printNextReg_))
            printNextReg_++;
        else {
            printActive_ = false;
            emitPrintBusy(c.phy);
            blinkBad();
        }
        break;

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
