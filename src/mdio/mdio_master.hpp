// mdio_master.hpp - active MDIO master driver runtime
//
// Drives the PIOC `mdio_master` blob: the CPU assembles the 4-byte Clause-22 frame, writes
// it + a doorbell to the PIOC data regs, polls for the result. The blob generates MDC,
// drives/samples MDIO and does the read turnaround (see pioc/mdio_master.ASM).
//
// service() drains a CDC RX line, parses it (MdioCmd, strict), runs the transaction and
// writes the phytool-style response (responses echo the request -> self-correlating):
//  read  <phy>/<reg> 0xVAL
//  write <phy>/<reg> ok
//  <verb> <phy>/<reg> err <why>
#pragma once

#include <stdint.h>

#include "led_blinker.hpp"
#include "usb_cdc.hpp"

class MdioMaster {
public:
    // led is optional: pass nullptr to disable the LED activity feedback.
    explicit MdioMaster(UsbCdc& usb, LedBlinker* led = nullptr) : usb_(usb), led_(led) {}

    void begin(uint32_t nowMs);     // load the blob, init MDC/MDIO pins + MDC clock
    void service(uint32_t nowMs);   // drain RX -> parse a line -> transact -> respond

private:
    void loadBlob();                // GPIO (AF_PP) + memcpy blob -> PIOC SRAM + start the eMCU
    void handleLine(const char* line, uint16_t len);

    // One Clause-22 transaction via the blob. Returns true on success; on false, *err is a
    // short reason ("noresp" = TA bit not pulled low / "timeout" = blob never published).
    bool readReg(uint8_t phy, uint8_t reg, uint16_t& out, const char*& err);
    bool writeReg(uint8_t phy, uint8_t reg, uint16_t val, const char*& err);

    // LED feedback (queued, non-blocking)
    void blinkOk()  { if (led_) led_->blink(50, 50); }    // good event = short blink
    void blinkBad() { if (led_) led_->blink(500, 100); }  // bad event = long blink

    UsbCdc&     usb_;
    LedBlinker* led_;
    char        line_[48];  // RX line accumulator ("!write 31/31 0xFFFF" + CRLF fits)
    uint16_t    len_ = 0;
};
