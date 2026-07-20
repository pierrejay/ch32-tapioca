// mdio_master.cpp - Mdio::Master implementation
#ifdef RUN_MDIO_MASTER

#include "mdio_master.hpp"
#include "ch32_sdk.hpp"

extern "C" {
#include "PIOC_SFR.h"
#include <string.h>
}

// ---- PIOC mailbox (host view of the data-reg file: DR[0x20+n] == DATA_REGn) ------------
static volatile uint8_t* const DR = (volatile uint8_t*)PIOC_SFR_BASE;
enum {                              // data-reg offsets - MUST match pioc/mdio_master.ASM
    DR_CMD0 = 0x22, DR_CMD1 = 0x23, DR_CMD2 = 0x24, DR_CMD3 = 0x25,
    DR_CTRL = 0x26, DR_STATUS = 0x27, DR_RES_H = 0x28, DR_RES_L = 0x29,
};                                  // (DATA_REG0/1 unused: the bit-bang blob has no TMR0 params)
static constexpr uint8_t  CTRL_READ = 0x01, CTRL_GO = 0x80;      // CTRL doorbell bits
static constexpr uint8_t  ST_OK = 0x01, ST_NORESP = 0x80;        // STATUS values the blob writes
static constexpr uint32_t MDIO_TIMEOUT_MS = 5;                   // blob runs in ~60 us; this guards a stall
static constexpr uint8_t  REGCR = 0x0D;                          // Clause-22 MMD access control
static constexpr uint8_t  ADDAR = 0x0E;                          // Clause-22 MMD address/data
static constexpr uint16_t MMD_ADDR = 0x0000;                      // REGCR[15:14] = 00
static constexpr uint16_t MMD_DATA = 0x4000;                      // REGCR[15:14] = 01, no post-inc

namespace Mdio {

void Master::begin()
{
#ifdef MDIO_STUB
    active_ = false;
    started_ = false;
    resp_ = nullptr;
#else
    loadBlob();
    DR[DR_STATUS] = 0;                  // clear STATUS (MDC is bit-banged in the blob, no TMR0 setup)
    active_ = false;
    started_ = false;
    resp_ = nullptr;
#endif
}

// GPIO (AF_PP, like the sniffer - the blob's SFR_PORT_DIR sets in/out per phase) + load the
// blob into PIOC SRAM and start the eMCU. Same bring-up as ClockedSniffer::loadRingBlob.
void Master::loadBlob()
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

Master::Result Master::read(uint8_t phy, uint8_t reg, Response& resp)
{
    return submit(OpRead, phy, 0, reg, 0, resp);
}

Master::Result Master::write(uint8_t phy, uint8_t reg, uint16_t val, Response& resp)
{
    return submit(OpWrite, phy, 0, reg, val, resp);
}

Master::Result Master::readMmd(uint8_t phy, uint8_t mmd, uint16_t reg, Response& resp)
{
    return submit(OpReadMmd, phy, mmd, reg, 0, resp);
}

Master::Result Master::writeMmd(uint8_t phy, uint8_t mmd, uint16_t reg, uint16_t val,
                                Response& resp)
{
    return submit(OpWriteMmd, phy, mmd, reg, val, resp);
}

Master::Result Master::submit(Op op, uint8_t phy, uint8_t mmd, uint16_t reg, uint16_t val,
                              Response& resp)
{
    if (active_) return Busy;

    resp.clear();
    resp.status = Pending;

    active_    = true;
    started_   = false;
    op_        = op;
    step_      = StepStart;
    phy_       = phy;
    mmd_       = mmd;
    reg_       = reg;
    val_       = val;
    wireRead_  = true;
    wireReg_   = 0;
    wireVal_   = 0;
    startedMs_ = 0;
    resp_      = &resp;

    return Ok;
}

void Master::tick(uint32_t nowMs)
{
    if (!active_) return;

    if (!started_) {
        startNextTransaction(nowMs);
#ifdef MDIO_STUB
        // Hardware-free protocol gate (no PIOC): canned value echoes the address; phy 31
        // fakes a no-response so the error path is exercisable without hardware.
        if (phy_ == 0x1F) {
            finish(NoResp);
        } else {
            uint16_t v = 0;
            if (op_ == OpRead) v = (uint16_t)((phy_ << 8) | (reg_ & 0x1F));
            else if (op_ == OpReadMmd) v = (uint16_t)(((mmd_ & 0x1F) << 11) ^ reg_);
            finish(Done, v);
        }
        return;
#else
#endif
    }

#ifndef MDIO_STUB
    uint8_t st = DR[DR_STATUS];
    if (st == 0) {
        if ((uint32_t)(nowMs - startedMs_) > MDIO_TIMEOUT_MS) finish(Timeout);
        return;
    }

    if (st == ST_NORESP) {
        completeTransaction(nowMs, NoResp, 0);
        return;
    }

    if (st != ST_OK) {
        completeTransaction(nowMs, Timeout, 0);
        return;
    }

    uint16_t value = wireRead_ ? (uint16_t)((DR[DR_RES_H] << 8) | DR[DR_RES_L]) : 0;
    completeTransaction(nowMs, Done, value);
#else
    (void)nowMs;
#endif
}

void Master::startNextTransaction(uint32_t nowMs)
{
    started_ = false;

    switch (op_) {
    case OpRead:
    case OpWrite:
        wireRead_ = (op_ == OpRead);
        wireReg_  = (uint8_t)(reg_ & 0x1F);
        wireVal_  = val_;
        step_     = StepDone;
        break;

    case OpReadMmd:
    case OpWriteMmd:
        switch (step_) {
        case StepStart:
            wireRead_ = false;
            wireReg_  = REGCR;
            wireVal_  = (uint16_t)(MMD_ADDR | (mmd_ & 0x1F));
            step_     = StepMmdAddrMode;
            break;
        case StepMmdAddrMode:
            wireRead_ = false;
            wireReg_  = ADDAR;
            wireVal_  = reg_;
            step_     = StepMmdSetAddr;
            break;
        case StepMmdSetAddr:
            wireRead_ = false;
            wireReg_  = REGCR;
            wireVal_  = (uint16_t)(MMD_DATA | (mmd_ & 0x1F));
            step_     = StepMmdDataMode;
            break;
        case StepMmdDataMode:
            wireRead_ = (op_ == OpReadMmd);
            wireReg_  = ADDAR;
            wireVal_  = val_;
            step_     = StepMmdData;
            break;
        default:
            step_ = StepDone;
            break;
        }
        break;

    default:
        step_ = StepDone;
        break;
    }

    if (step_ == StepDone && (op_ == OpReadMmd || op_ == OpWriteMmd)) return;
    startTransaction();
    started_ = true;
    startedMs_ = nowMs;
}

void Master::completeTransaction(uint32_t nowMs, Status status, uint16_t value)
{
    started_ = false;

    if (status != Done) {
        finish(status);
        return;
    }

    if (step_ == StepDone) {
        finish(Done, value);
        return;
    }

    if (step_ == StepMmdData) {
        finish(Done, op_ == OpReadMmd ? value : 0);
        return;
    }

    startNextTransaction(nowMs);
}

// ---- blob transaction --------------------------------------------------------
// Assemble the 32-bit post-preamble Clause-22 frame, hand it + a doorbell to the blob.
// tick() later polls STATUS. STATUS is cleared BEFORE setting GO so a stale prior result
// can't be mistaken for this one.
void Master::startTransaction()
{
#ifndef MDIO_STUB
    // MSB-first: ST(01) OP PHYAD(5) REGAD(5) TA DATA(16) = 32 bits
    bool     isRead = wireRead_;
    uint32_t op     = isRead ? 0x2u : 0x1u;         // OP: 10 read / 01 write
    uint32_t ta     = isRead ? 0x0u : 0x2u;         // TA: write drives 10; read is released
    uint32_t frame = (0x1u << 30) | (op << 28)
                   | ((uint32_t)(phy_ & 0x1F) << 23)
                   | ((uint32_t)(wireReg_ & 0x1F) << 18)
                   | (ta << 16)
                   | (isRead ? 0u : (uint32_t)wireVal_);
    DR[DR_CMD0] = (uint8_t)(frame >> 24);
    DR[DR_CMD1] = (uint8_t)(frame >> 16);
    DR[DR_CMD2] = (uint8_t)(frame >> 8);
    DR[DR_CMD3] = (uint8_t)(frame);

    DR[DR_STATUS] = 0;                               // busy, cleared before GO (no stale-result race)
    DR[DR_CTRL]   = (isRead ? CTRL_READ : 0) | CTRL_GO;
#endif
}

void Master::finish(Status status, uint16_t value)
{
    if (resp_) {
        resp_->value = value;
        resp_->status = status;
    }

    active_ = false;
    started_ = false;
    resp_ = nullptr;
}

} // namespace Mdio

#endif // RUN_MDIO_MASTER
