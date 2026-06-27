// clocked_sniffer.hpp - passive clocked-bus sniffer datapath (PIOC ring -> USB-CDC).
//
// The PIOC ("eMCU") runs the validated capture blob: it mid-bit-samples the data
// line on each clock edge (PC18=clock, PC19=data) and streams captured bytes into a
// 16-slot DATA-register FIFO, publishing a monotonic HEAD (torn-read-free, no park).
// This class drains that FIFO every main-loop tick and forwards it to the host over
// USB-CDC as COBS-0xFF framed, timestamped burst records (layout in
// clocked_sniffer.cpp). The host derives polling cadence, effective rate and the clock.
//
// Owns its buffers and the PIOC bring-up; takes the UsbCdc by reference. Usage:
// begin() once, then service(now_ms) every loop iteration. service() NEVER
// blocks - it writes only what the USB TX ring can take and leaves the rest in
// its RAM ring, so the tiny PIOC FIFO is always drained in time.
//
// Passive tap: the blob forces the PIOC IO pins to input; this never drives a bus.
#pragma once

#include "ring.hpp"
#include "usb_cdc.hpp"

class ClockedSniffer
{
public:
    explicit ClockedSniffer(UsbCdc& usb) : usb_(usb) {}

    void begin(uint32_t nowMs);             // load blob, reset state, arm output
    void stop();                            // quiesce for a runtime mode switch: halt PIOC, flush
    void service(uint32_t nowMs);           // drain PIOC FIFO -> USB; call every loop

private:
    // ---- PIOC ring (the eMCU's host-readable 16-slot DATA-reg FIFO) ----------
    static void loadRingBlob();                          // GPIO + blob + start
    static constexpr uint8_t RING_SLOTS = 16;            // DATA_REG0..15
    static constexpr uint8_t RING_HEAD  = 0x3F;          // DATA_REG31 (published count)
    static volatile uint8_t* const DR_;                  // PIOC data-reg file base

    // ---- RAM staging ring (USB backpressure absorber) -----------------------
    // 2048 so a whole max binary record (~783 B encoded) always fits even with
    // some USB backlog -> a record is never dropped for lack of staging room. 
    static constexpr uint16_t RING_SZ   = 2048;          // power of two
    uint16_t ringCount() const { return (uint16_t)ring_.size(); }
    uint16_t ringSpace() const { return (uint16_t)ring_.free(); }
    void     ringPush(const uint8_t* p, uint16_t n);
    uint16_t ringPop(uint8_t* out, uint16_t max);

    // ---- binary burst-record protocol (COBS-0xFF, 0xFF boundary) -------------
    // One COBS-0xFF framed, timestamped record per burst; a [MODE clocked wire=2]
    // block is staged periodically so a host can confirm the mode even on a mute
    // bus. Layout + framing in clocked_sniffer.cpp; envelope in README §"USB wire protocol". 
    static constexpr uint16_t BURST_MAX     = 768;       // payload cap (> a cycle)
    static constexpr uint32_t BURST_TCAP_US = 50000;     // continuous-traffic cap
    static constexpr uint16_t ONSET_N       = 8;         // clock-estimate window
    void     emitRecord(bool continued);
    void     emitModeMarker();               // stage [MODE clocked wire=2] between records
    uint32_t modeHbMs_ = 0;                   // next [MODE] heartbeat time (ms)

    UsbCdc& usb_;

    // RAM ring
    Ring<RING_SZ> ring_;
    // Two loss sources kept distinct so a real capture says WHICH ring overran:
    // ovfRam_ = RAM ring full (USB too slow); ovfPioc_ = PIOC FIFO overrun (loop
    // fell behind). They imply different fixes. 
    uint16_t ovfRam_  = 0;
    uint16_t ovfPioc_ = 0;

    // sniffer drain state
    uint8_t  sniffTail_   = 0;               // our cursor into the PIOC FIFO
    uint32_t sniffLastMs_ = 0;               // ms of the last captured byte

    // current burst
    uint8_t  burstBuf_[BURST_MAX];
    uint16_t burstN_     = 0;
    uint32_t burstT0_    = 0;                 // micros of 1st byte
    uint32_t burstLast_  = 0;                 // micros of last byte
    uint16_t burstOnset_ = 0;                 // micros 1st->ONSET_N-th byte (0 = n/a)
    bool     burstOpen_  = false;

#ifdef DIAG
    // ---- telemetry (only with -D DIAG; off by default so the wire is pure capture).
    // Symmetric with RleSniffer's [T] block, minus the TIM3-drain fields: the clocked
    // path drains from the main loop, so there's no drnCpu / maxGap to report. The
    // per-record flags byte already carries overflow live; diagOvf* re-sum it per
    // [T] window so a single line also shows accumulated loss. 
    uint32_t loopCalls_   = 0;               // service() calls = main-loop iterations
    uint16_t ramHi_       = 0;               // max RAM-ring fill over the window (/RING_SZ)
    uint16_t txMinFree_   = 0xFFFF;          // min tx_ free space (0 = USB EP is the ceiling)
    uint32_t diagOvfPioc_ = 0;               // PIOC FIFO overruns over the window
    uint32_t diagOvfRam_  = 0;               // RAM ring drops over the window
#endif
}; // class ClockedSniffer
