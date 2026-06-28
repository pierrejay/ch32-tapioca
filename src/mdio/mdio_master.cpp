// mdio_master.cpp - MdioMaster implementation
#ifdef RUN_MDIO_MASTER

#include "mdio_master.hpp"
#include "mdio_command.hpp"
#include "time.hpp"

extern "C" {
#include "PIOC_SFR.h"
#include <string.h>
#include <stdio.h>
}

// printf is wrapped to USB-CDC (main.cpp __wrap__write), which drains g_usb as it goes - so
// plain ASCII responses go straight to the host. RX still comes through usb_ (available/read).

// ---- PIOC mailbox (host view of the data-reg file: DR[0x20+n] == DATA_REGn) ------------
static volatile uint8_t* const DR = (volatile uint8_t*)PIOC_SFR_BASE;
enum {                              // data-reg offsets - MUST match pioc/mdio_master.ASM
    DR_CMD0 = 0x22, DR_CMD1 = 0x23, DR_CMD2 = 0x24, DR_CMD3 = 0x25,
    DR_CTRL = 0x26, DR_STATUS = 0x27, DR_RES_H = 0x28, DR_RES_L = 0x29,
};                                  // (DATA_REG0/1 unused: the bit-bang blob has no TMR0 params)
static constexpr uint8_t  CTRL_READ = 0x01, CTRL_GO = 0x80;      // CTRL doorbell bits
static constexpr uint8_t  ST_OK = 0x01, ST_NORESP = 0x80;        // STATUS values the blob writes
static constexpr uint32_t MDIO_TIMEOUT_MS = 5;                   // blob runs in ~60 us; this guards a stall

void MdioMaster::begin(uint32_t /*nowMs*/)
{
#ifdef MDIO_STUB
    printf("# mdio-master ready (clause22, STUB - no PIOC)\r\n");   // protocol-only gate build
#else
    loadBlob();
    DR[DR_STATUS] = 0;                  // clear STATUS (MDC is bit-banged in the blob, no TMR0 setup)
    printf("# mdio-master ready (clause22)\r\n");   // '#' banner: host ignores (no response grammar)
#endif
}

// GPIO (AF_PP, like the sniffer - the blob's SFR_PORT_DIR sets in/out per phase) + load the
// blob into PIOC SRAM and start the eMCU. Same bring-up as ClockedSniffer::loadRingBlob.
void MdioMaster::loadBlob()
{
    static const __attribute__((aligned(16))) unsigned char prog[] =
        #include "../../pioc/mdio_master_inc.h"

    GPIO_InitTypeDef g = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
    g.GPIO_Pin   = GPIO_Pin_18 | GPIO_Pin_19;       // PC18 = MDC, PC19 = MDIO
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &g);

    R8_SYS_CFG = 0;
    memcpy((void*)PIOC_SRAM_BASE, prog, sizeof(prog));
    R8_SYS_CFG |= RB_MST_RESET;
    R8_SYS_CFG  = RB_MST_IO_EN0 | RB_MST_IO_EN1;
    R8_SYS_CFG |= RB_MST_CLK_GATE;
    Delay_Ms(1);
}

void MdioMaster::service(uint32_t /*nowMs*/)
{
    uint8_t b = 0;
    while (usb_.available() && usb_.read(&b, 1) == 1) {
        if (b == '\n' || b == '\r') {
            if (len_) { handleLine(line_, len_); len_ = 0; }
        } else if (len_ < sizeof(line_)) {
            line_[len_++] = (char)b;
        } else {
            len_ = 0;                       // over-long line -> drop, resync on next newline
        }
    }
}

void MdioMaster::handleLine(const char* line, uint16_t len)
{
    MdioCmd::Command c = MdioCmd::parse(line, len);
    if (!c.valid) { blinkBad(); return; }   // parse error: long blink, ignore the line
    blinkOk();                              // decoded a valid command: short blink

    const char* err = nullptr;
    uint16_t    v   = 0;
    switch (c.op) {
    case MdioCmd::Op::Read:
        if (readReg(c.phy, c.reg, v, err)) { printf("read %u/%u 0x%04X\r\n", c.phy, c.reg, v); blinkOk(); }
        else { printf("read %u/%u err %s\r\n", c.phy, c.reg, err); blinkBad(); }
        break;
    case MdioCmd::Op::Write:
        if (writeReg(c.phy, c.reg, c.val, err)) { printf("write %u/%u ok\r\n", c.phy, c.reg); blinkOk(); }
        else { printf("write %u/%u err %s\r\n", c.phy, c.reg, err); blinkBad(); }
        break;
    case MdioCmd::Op::Print: {
        // bulk read regs 0..31, emitting the SAME line shape as a single read (one host
        // parser; the register NAMING stays host-side). Saves 31 USB round-trips. Per-reg
        // blinks would flood the LED queue, so signal ONE aggregate result after the loop:
        // short if every reg answered, long if any reg gave noresp/timeout.
        bool allOk = true;
        for (uint8_t reg = 0; reg < 32; reg++) {
            if (readReg(c.phy, reg, v, err)) printf("read %u/%u 0x%04X\r\n", c.phy, reg, v);
            else { printf("read %u/%u err %s\r\n", c.phy, reg, err); allOk = false; }
        }
        if (allOk) blinkOk(); else blinkBad();
        break;
    }
    default:
        break;
    }
}

// ---- blob transaction --------------------------------------------------------
// Assemble the 32-bit post-preamble Clause-22 frame, hand it + a doorbell to the blob,
// then poll STATUS. STATUS is cleared BEFORE setting GO so a stale prior result can't be
// mistaken for this one. Returns false with *err on no-PHY ("noresp") or a stalled blob
// ("timeout"); the blob normally publishes within ~64 us (64 MDC cycles @ ~1 MHz).
static bool mdioTransact(uint8_t phy, uint8_t reg, bool read, uint16_t wdata,
                         uint16_t& rdata, const char*& err)
{
#ifdef MDIO_STUB
    // Hardware-free protocol gate (no PIOC): canned value echoes the address (obviously a
    // stub); phy 31 fakes a no-resp so the error path is exercisable too. Validates the
    // whole ASCII path over USB so a bench failure later is unambiguously the blob.
    (void)wdata;
    if (phy == 0x1F) { err = "noresp"; return false; }
    if (read) rdata = (uint16_t)((phy << 8) | reg);
    return true;
#else
    // MSB-first: ST(01) OP PHYAD(5) REGAD(5) TA DATA(16) = 32 bits
    uint32_t op    = read ? 0x2u : 0x1u;            // OP: 10 read / 01 write
    uint32_t ta    = read ? 0x0u : 0x2u;            // TA: write drives 10; read is released
    uint32_t frame = (0x1u << 30) | (op << 28)
                   | ((uint32_t)(phy & 0x1F) << 23)
                   | ((uint32_t)(reg & 0x1F) << 18)
                   | (ta << 16)
                   | (read ? 0u : (uint32_t)wdata);
    DR[DR_CMD0] = (uint8_t)(frame >> 24);
    DR[DR_CMD1] = (uint8_t)(frame >> 16);
    DR[DR_CMD2] = (uint8_t)(frame >> 8);
    DR[DR_CMD3] = (uint8_t)(frame);

    DR[DR_STATUS] = 0;                               // busy, cleared before GO (no stale-result race)
    DR[DR_CTRL]   = (read ? CTRL_READ : 0) | CTRL_GO;

    uint32_t t0 = Time::millis();
    uint8_t  st;
    while ((st = DR[DR_STATUS]) == 0) {
        if (Time::millis() - t0 > MDIO_TIMEOUT_MS) { err = "timeout"; return false; }
    }
    if (st == ST_NORESP) { err = "noresp"; return false; }
    if (read) rdata = (uint16_t)((DR[DR_RES_H] << 8) | DR[DR_RES_L]);
    return true;
#endif
}

bool MdioMaster::readReg(uint8_t phy, uint8_t reg, uint16_t& out, const char*& err)
{
    return mdioTransact(phy, reg, true, 0, out, err);
}

bool MdioMaster::writeReg(uint8_t phy, uint8_t reg, uint16_t val, const char*& err)
{
    uint16_t dummy = 0;
    return mdioTransact(phy, reg, false, val, dummy, err);
}

#endif // RUN_MDIO_MASTER
