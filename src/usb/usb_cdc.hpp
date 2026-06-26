// usb_cdc.hpp - Self-contained USB-CDC (ACM) device for the CH32X035 USBFS.
//
// Owns everything USB: endpoint DMA buffers, the control-transfer state machine,
// the CDC line-coding, and two byte rings (host->device and device->host).
// Knows nothing about UART or any application - data crosses the boundary
// through a clean stream API:
//
//     available() / read()  - pull bytes the host sent us (bulk OUT)
//     write() / writeSpace() - queue bytes to send to the host (bulk IN)
//     tick(nowMs)           - drain the TX ring to the endpoint, handle flow ctrl
//     onIrq()               - called from USBFS_IRQHandler
//
// Single instance: a static self-pointer lets the C ISR reach the object.
// The constructor is trivial; all hardware setup happens in init(), so there is
// no static-init-order hazard.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "ring.hpp"
#include "ch32_sdk.hpp"

// Run from RAM (copied out of flash at boot) to dodge flash fetch wait-states. The hot
// bulk-IN path (pump/arm) is called ~4x per CAN drain loop from .highcode service(); left
// in flash it re-introduces the very wait-states .highcode was meant to kill. 
#ifndef __HIGHCODE
#define __HIGHCODE __attribute__((section(".highcode")))
#endif

class UsbCdc
{
public:
    // Parsed CDC line coding (what the host asked the "serial port" to be).
    struct LineCoding
    {
        uint32_t baud     = 115200;
        uint8_t  stopBits = 0;   // 0 = 1 bit, 1 = 1.5 bits, 2 = 2 bits
        uint8_t  parity   = 0;   // 0 none, 1 odd, 2 even, 3 mark, 4 space
        uint8_t  dataBits = 8;   // 5..8 (16 allowed by spec)
    };

    // Ring capacities (power of two). RX larger: host can burst at us.
    static constexpr uint32_t kRxCapacity = 1024;   // host -> device
    static constexpr uint32_t kTxCapacity = 512;    // device -> host

    void init();        // clocks, PHY, endpoints, enable IRQ
    void onIrq();       // USBFS interrupt service (called from extern "C" ISR)
    void tick(uint32_t nowMs); // main-loop step: coalesce TX, re-open OUT flow control

    // Dispatch entry for the C-linkage USBFS_IRQHandler -> the live instance.
    static void handleIrq() { if (self_) self_->onIrq(); }

    bool configured() const { return enumerated_; }

    // Host -> device (bulk OUT)
    uint32_t available() const { return rx_.size(); }
    uint32_t read(uint8_t* dst, uint32_t maxLen) { return rx_.pop(dst, maxLen); }

    // Device -> host (bulk IN). write() queues into the TX ring; tick() sends full
    // packets immediately and flushes a residual short packet after <=1 ms. 
    uint32_t writeSpace() const { return tx_.free(); }
    uint32_t write(const uint8_t* src, uint32_t len) { return tx_.push(src, len); }
    // Zero-copy producer: reserve contiguous TX space, fill it, commit. Lets the
    // drain filter write run-bytes straight into the TX ring (no out[] + push copy).
    uint8_t* txReserve(uint32_t* contig) { *contig = tx_.freeContig(); return tx_.writePtr(); }
    void     txCommit(uint32_t n) { tx_.writeCommit(n); }
    uint32_t txPackets() const { return txPkts_; }   // bulk-IN packets ACKed (USB throughput)
    uint32_t txBytes() const { return txBytes_; }    // REAL data bytes armed (pktHz*64 lies: short packets)
    uint32_t txShort() const { return shortPkts_; }  // <64B packets (transaction efficiency)

    // Lean bulk-IN pump: if a transfer just completed, un-freeze the EP (clear
    // UIF_TRANSFER, INT_BUSY-gated) and re-arm the next packet from tx_. Meant to be
    // called FREQUENTLY (e.g. interleaved inside the CAN drain loop) so the EP's auto-NAK
    // freeze lasts ~one call interval instead of a whole main-loop iteration.
    //
    // CONTRACT (the architectural assumption that makes this safe):
    //  - MAIN-LOOP CONTEXT ONLY. Never call from an ISR (it touches tx_/ep3_/USB regs that
    //    the main loop also owns; there's no locking).
    //  - The USBFS interrupt MUST stay disabled (we poll). pump() relies on the hardware
    //    serializing USB completions (INT_BUSY auto-NAKs until UIF_TRANSFER is cleared), so
    //    exactly one completion is pending per call - no missed-event counting.
    //  - It's the tx_ CONSUMER (tail_); service() is the sole PRODUCER (head_). Interleaving
    //    pump() inside service()'s fill loop is safe: disjoint cursors, SPSC.
    //  - No new interrupt -> the TIM3 RLE drain ISR is never perturbed (the hard constraint).
    //
    // Non-EP3-IN completions fall back to the full onIrq(); see the .cpp for the rare race
    // that fallback can drop and why it self-heals. 
    void pump() __HIGHCODE;

    // CDC line coding: exposed for host-control experiments; the capture firmware
    // currently only tracks whether the host changed it. The flag is sticky until read. 
    const LineCoding& lineCoding() const { return lc_; }
    bool lineCodingChanged()
    {
        bool c = lcChanged_;
        lcChanged_ = false;
        return c;
    }

private:
    // --- USB control-transfer plumbing ---
    void  endpointInit();
    void  handleSetup();
    void  handleEp0In();
    void  handleEp0Out();
    void  busReset();
    void  armEp3(uint16_t len) __HIGHCODE;   // load bulk-IN buffer and ACK
    bool  armEp3FromTx(bool allowShort) __HIGHCODE; // pop one full packet, or an allowed timeout flush
    void  decodeLineCoding();               // wire bytes -> lc_

    // Endpoint DMA buffers (the USBFS engine reads/writes these directly).
    alignas(4) uint8_t ep0_[64];
    alignas(4) uint8_t ep1_[64];   // interrupt IN, unused
    alignas(4) uint8_t ep2_[64];   // bulk OUT staging
    alignas(4) uint8_t ep3_[64];   // bulk IN staging

    Ring<kRxCapacity> rx_;         // produced by ISR (OUT), consumed by read()
    Ring<kTxCapacity> tx_;         // produced by write(), consumed by tick()

    // Pending SETUP transfer state
    const uint8_t* descr_     = nullptr;   // current descriptor read cursor
    uint16_t       setupLen_  = 0;
    uint8_t        setupReqType_ = 0;
    uint8_t        setupReqCode_ = 0;
    uint16_t       setupValue_ = 0;
    uint16_t       setupIndex_ = 0;

    // Device status
    uint8_t  devAddr_    = 0;
    uint8_t  devConfig_  = 0;
    bool     enumerated_ = false;
    uint8_t  sleepStatus_ = 0;

    // CDC line coding: 7 wire bytes (baud32, stop, parity, data) + parsed copy
    uint8_t    lcWire_[7] = { 0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08 }; // 115200 8N1
    LineCoding lc_;
    volatile bool lcChanged_ = false;

    // Bulk endpoint flow control
    volatile bool ep3Busy_ = false;   // a bulk-IN transfer is in flight
    bool          ep2Nak_  = false;   // OUT throttled because RX ring was full
    volatile uint32_t txPkts_ = 0;    // bulk-IN packets ACKed by host (USB throughput)
    volatile uint32_t txBytes_ = 0;   // real data bytes shipped (sum of armed packet lengths)
    volatile uint32_t fullPkts_ = 0;  // 64-byte packets armed
    volatile uint32_t shortPkts_ = 0; // <64-byte packets armed (USB transaction efficiency)
    bool          partialPending_ = false; // TX has 1..63 B waiting for coalesce timeout
    uint32_t      partialSinceMs_ = 0;
    static constexpr uint32_t kPartialFlushMs = 1; // bounded low-rate/log latency

    static UsbCdc* self_;             // for the C ISR to find us
}; // class UsbCdc
