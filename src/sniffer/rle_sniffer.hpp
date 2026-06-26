// rle_sniffer.hpp - passive run-length sniffer datapath (PIOC ring -> USB-CDC).
//
// The PIOC runs the rle capture blob: it times how long the data line stays at a
// level and pushes run-bytes into a 30-slot FIFO. A run may be one terminal byte
// (<128) or several 0x80 continuation bytes plus a terminal byte. All decoding
// (clock recovery, de-stuffing, CRC for CAN, etc.) happens host-side - the device
// is codec-agnostic.
//
// TWO-LEVEL DRAIN (keeps the PIOC FIFO from overflowing at worst-case CAN):
//   1. drainPioc() : from a TIM3 IRQ @ 50 kHz (every 20 us), priority 0 (above the
//      TIM2 millis). Copies PIOC ring -> 2KB RAM ring. ISR-safe, runs from RAM, does
//      not touch USB. The blob itself is untouched (zero added PIOC cycles).
//   2. service()   : from the main loop, drains RAM ring -> USB-CDC. Non
//      time-critical (the RAM ring absorbs USB backpressure).
//
// BINARY format (native): raw run-bytes 0x00..0x80 (the cap-continuation tops out at
// 0x80, so 0x81..0xFF never appear as data -> free sentinels, no escaping). 0xFF =
// frame boundary (idle). 0xFE <ascii> 0xFE = diag block (with -D DIAG). The host
// skips 0xFE blocks for decode and parses them for stats.
//
// Passive-tap safe: the blob CLRs SFR_PORT_DIR (IO pins = input) and SpiGen is never
// configured. On a real bus: PC19 (IO1) <- the bus data line, common ground.
#pragma once

#include "usb_cdc.hpp"

// Place hot drain code in RAM (zero flash wait-states). The .highcode section is
// copied flash->RAM at boot by the existing startup .data loop (see linker script).
// Drain ISR + drainPioc run from RAM so per-fire cost isn't paid in flash fetches. 
#define __HIGHCODE __attribute__((section(".highcode")))

class RleSniffer
{
public:
    explicit RleSniffer(UsbCdc& usb) : usb_(usb) {}

    void begin(uint32_t nowMs);             // load blob, reset state, arm output + start TIM3 drain
    void stop();                            // quiesce for a runtime mode switch: disarm TIM3, halt PIOC, flush
    void service(uint32_t nowMs) __HIGHCODE; // drain RAM ring -> USB-CDC; runs from RAM (flash wait-states were ~4.5us/byte = the throughput wall)
    void drainPioc() __HIGHCODE;            // ISR: drain PIOC ring -> RAM ring. Called from TIM3 @ 50kHz (runs from RAM)

#ifdef DIAG
    void recordTiming(uint32_t svcUs) { svcSum_ += svcUs; }
#endif

#ifdef RUN_RLE_TICK_TEST
    // Test-only raw tap: pop raw run-bytes (no squelch, no USB framing) for the
    // tick-timing bench (src/sniffer/rle_tick_test.cpp). It rides the REAL blob +
    // the validated TIM3 drain; this just consumes the RAM ring verbatim. 
    uint16_t testDrainRaw(uint8_t* out, uint16_t max) { return ringPop(out, max); }
#endif

private:
    // ---- PIOC ring -----------------------------------------------------------
    // The 30-slot ring layout intentionally does NOT use a power-of-two
    // slot count. HEAD is still an 8-bit logical sequence number; sniffSlot_
    // below is the independent physical cursor needed to map it to DATA_REGs. 
    static void loadBlob();                            // GPIO + blob + start
    void        startDrainTimer();                     // TIM3 @ 50 kHz, prio 0

    // Per-level tick period of the blob's counting loop, in CPU cycles. The HIGH and LOW
    // loops differ by one branch, so a tick is NOT the same duration at the two levels.
    // Measured exactly by the constant-level cap bench (env test_rle_tick): HIGH=9, LOW=8
    // (residual ~0.05 = the once-per-128 cap-store). Emitted in [MODE rle] so the host can
    // turn run-byte tick counts into absolute time: baud = fcpu / (UI_level * TICK_*_CYC).
    // The per-run edge overhead k is NOT shipped - it cancels in bit-time differences and
    // the host already absorbs it as fit_ui()'s intercept C. 
    static constexpr uint8_t TICK_HI_CYC = 9;          // line HIGH: 9 CPU cycles / tick
    static constexpr uint8_t TICK_LO_CYC = 8;          // line LOW:  8 CPU cycles / tick

    static constexpr uint8_t RING_SLOTS = 30;          // 30-slot FIFO: DATA_REG2..31
    static constexpr uint8_t RING_BASE  = 0x22;
    static constexpr uint8_t RING_HEAD  = 0x21;        // DATA_REG1 (published head)
    static volatile uint8_t* const DR_;                // PIOC data-reg file base

    // ---- RAM staging ring (USB backpressure absorber) -----------------------
    static constexpr uint16_t RING_SZ   = 2048;        // power of two
    static constexpr uint16_t RING_MASK = RING_SZ - 1;
    uint16_t ringCount() const { return (uint16_t)(head_ - tail_) & RING_MASK; }
    uint16_t ringSpace() const { return RING_MASK - ringCount(); }
    void     ringPush(const uint8_t* p, uint16_t n);
    uint16_t ringPop(uint8_t* out, uint16_t max);   // (unused since 1-pass service; kept for parity with ClockedSniffer)

    bool emitBoundary() __HIGHCODE;                    // write 0xFF boundary; false if TX full -> retry

#ifdef RUN_SNIFFER
    // Self-advertise this datapath's fixed identity ([MODE rle wire=2]) like
    // ClockedSniffer does, but at the CAN safe boundary: the idle gap, where no frame is in
    // flight, so the framed block written straight to USB can't tear a run-byte frame. 
    void     emitModeMarker();
    uint32_t modeHbMs_ = 0;                             // next [MODE] heartbeat time (ms)
#endif

    UsbCdc& usb_;

    // RAM ring
    uint8_t  ring_[RING_SZ];
    // SPSC across the TIM3 ISR / main loop: head_ produced by the drain ISR, tail_ by
    // service(). Single-core + aligned u16 => reads are atomic, but volatile is still
    // needed so the compiler can't hoist/cache them across the ISR boundary (was plain
    // u16 -> a latent visibility UB; the Ring<> class is volatile for the same reason). 
    volatile uint16_t head_ = 0;
    volatile uint16_t tail_ = 0;
    uint16_t ovfRam_  = 0;
    uint16_t ovfPioc_ = 0;

    // sniffer drain state
    uint8_t  sniffTail_   = 0;               // logical cursor into the PIOC FIFO
    uint8_t  sniffSlot_   = 0;               // physical slot, independent of 8-bit HEAD
    uint32_t sniffLastMs_ = 0;               // ms of the last captured byte
    bool     sniffActive_ = false;           // idle-gap tracking

    // ---- drain diagnostics (written from TIM3 ISR, read from service()) ----
    volatile uint8_t  maxGap_      = 0;      // max (head-tail) at drain entry (overflow margin)
#ifdef DIAG
    // drainPioc() self-timing (us, TIM2->CNT read inline - no calls in the fast ISR).
    // Tests the hypothesis that the drain ISR (slow PIOC-SRAM reads x gap) is the
    // CPU hog that crashes loopHz under full-bus load. drnCyc_ = total us in the ISR per
    // [T] window -> ISR-CPU% = drnCyc_*100/window_us. drnMax_ = worst single fire.
    volatile uint32_t drnCyc_ = 0;           // sum of per-fire us over the window
    volatile uint16_t drnMax_ = 0;           // worst single-fire us
#endif

    // ---- loss instrumentation: locate the bottleneck stage (see service() DIAG).
    // ramDrop_ is the silent-loss counter (PIOC->RAM when the RAM ring is full): it
    // surfaces on the wire as a 0xFD marker. ramHi_ = RAM ring fill; txMinFree_ ~ 0 =>
    // the USB endpoint is the ceiling.
    volatile uint32_t ramDrop_   = 0;        // bytes lost PIOC->RAM (RAM ring full)
    uint32_t          lossSeen_  = 0;        // last ramDrop_ we emitted a 0xFD loss marker for
    volatile uint16_t ramHi_     = 0;        // max RAM-ring fill seen (/RING_SZ)
    uint16_t          txMinFree_ = 0xFFFF;   // min tx_ free space seen (0 = tx_ saturates)
    // loopHz vs pktHz: loopHz ~ pktHz => the main loop is slow (something blocks the
    // re-arm); loopHz >> pktHz => loop fast but the single-buffer EP delivers few/s.
    uint32_t          loopCalls_ = 0;        // service() calls = main-loop iterations
    // avg us in service() per [T] window, fed by main.cpp via recordTiming().
    uint32_t          svcSum_  = 0;

    // firehose bound: count consecutive >=128 continuation bytes; below
    // CAP_IDLE_K they pass through (host sums a long in-frame run), at CAP_IDLE_K
    // the run is a long idle -> emit one 0xFF boundary and squelch until the line moves.
    // CAP_IDLE_K is derived from the slowest bitrate we claim to support so a real
    // in-frame run (<=16 bit-times) never reaches it. 0 = bound disabled (raw). 
    uint32_t capRun_   = 0;                   // consecutive continuation bytes
    bool     capPiped_ = false;               // already emitted the 0xFF boundary for this run?
}; // class RleSniffer
