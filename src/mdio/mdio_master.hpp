// mdio_master.hpp - active MDIO master driver runtime
//
// Drives the PIOC `mdio_master` blob: the CPU assembles the 4-byte Clause-22 frame, writes
// it + a doorbell to the PIOC data regs, then tick() polls for the result. The blob
// generates MDC, drives/samples MDIO and does the read turnaround (see pioc/mdio_master.ASM).
//
// Mdio::Master owns the bus and exposes a single async slot. Higher layers (USB ASCII,
// boot-time init scripts, LED feedback) submit one request and poll Response::status.
#pragma once

#include <stdint.h>

namespace Mdio {

class Master {
public:
    enum Result : uint8_t {
        Ok,
        Busy,
    };

    enum Status : uint8_t {
        Idle,
        Pending,
        Done,
        NoResp,
        Timeout,
    };

    // Client-owned mailbox: read()/write() borrow &resp until the request completes, so
    // resp MUST outlive the transaction - use static/persistent storage, never a temporary.
    struct Response {
        Status   status = Idle;
        uint16_t value  = 0;

        void clear() { status = Idle; value = 0; }
    };

    void begin();                   // load the blob, init MDC/MDIO pins + MDC clock
    Result read(uint8_t phy, uint8_t reg, Response& resp);
    Result write(uint8_t phy, uint8_t reg, uint16_t val, Response& resp);
    void tick(uint32_t nowMs);      // progress the active request, if any
    bool busy() const { return active_; }

private:
    enum Op : uint8_t {
        OpRead,
        OpWrite,
    };

    void loadBlob();                // GPIO (AF_PP) + memcpy blob -> PIOC SRAM + start the eMCU
    Result submit(Op op, uint8_t phy, uint8_t reg, uint16_t val, Response& resp);
    void startTransaction();
    void finish(Status status, uint16_t value = 0);

    bool      active_    = false;
    bool      started_   = false;
    Op        op_        = OpRead;
    uint8_t   phy_       = 0;
    uint8_t   reg_       = 0;
    uint16_t  val_       = 0;
    uint32_t  startedMs_ = 0;
    Response* resp_      = nullptr;
};

} // namespace Mdio
