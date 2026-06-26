// clocked_sniffer.cpp - ClockedSniffer implementation. See clocked_sniffer.hpp for the
// datapath overview. Binary output: COBS-0xFF framed, timestamped burst records.
#include "clocked_sniffer.hpp"
#include "record_framer.hpp"        // RecordFramer::frameRecord / frameMeta (COBS-0xFF)
#include "time.hpp"                 // Time::micros() for burst timestamps

extern "C" {
#include "PIOC_SFR.h"
#include <string.h>
#include <stdio.h>
}

// Host view of the PIOC data-register file: DR_[0x20+n] == DATA_REGn, DR_[0x3F]
// == DATA_REG31 (the published HEAD). 
volatile uint8_t* const ClockedSniffer::DR_ = (volatile uint8_t*)PIOC_SFR_BASE;

// ---- RAM staging ring -----------------------------------------------------
void ClockedSniffer::ringPush(const uint8_t* p, uint16_t n)
{
    if (ringSpace() < n) { ovfRam_++; return; }
    uint16_t h = head_;
    for (uint16_t i = 0; i < n; i++) ring_[(h + i) & RING_MASK] = p[i];
    head_ = h + n;
}

uint16_t ClockedSniffer::ringPop(uint8_t* out, uint16_t max)
{
    uint16_t n = ringCount();
    if (n > max) n = max;
    uint16_t t = tail_;
    for (uint16_t i = 0; i < n; i++) out[i] = ring_[(t + i) & RING_MASK];
    tail_ = t + n;
    return n;
}

// ---- PIOC bring-up --------------------------------------------------------
// Loads the DMA-style ring blob and wires PC18/PC19 to the PIOC. Configures ONLY
// the capture pins - never the PA5/PA7 generator - so a passive tap drives
// nothing; the blob itself also CLRs SFR_PORT_DIR (both PIOC IO pins = input). 
void ClockedSniffer::loadRingBlob()
{
    static const __attribute__((aligned(16))) unsigned char prog[] =
        #include "../../pioc/clocked_sniffer_inc.h"

    GPIO_InitTypeDef g = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
    g.GPIO_Pin   = GPIO_Pin_18 | GPIO_Pin_19;
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

// ---- lifecycle ------------------------------------------------------------
void ClockedSniffer::begin(uint32_t nowMs)
{
    loadRingBlob();
    sniffTail_   = DR_[RING_HEAD];
    sniffLastMs_ = nowMs;
    head_ = tail_ = 0;
    ovfRam_ = ovfPioc_ = 0;
    burstOpen_ = false;
    burstN_    = 0;
    // Emit a 0xFF boundary first so the preceding boot text (no 0xFF in it)
    // becomes its own segment the host drops, then announce the mode immediately. 
    { uint8_t ff = 0xFF; ringPush(&ff, 1); }
    emitModeMarker();
    modeHbMs_ = nowMs + 1000;
}

// Quiesce the clocked datapath for a runtime mode switch (dual-instance selector):
// halt the eMCU and drop the RAM staging ring + any open burst. No TIM3 here (clocked
// drains from the main loop), so this is just the PIOC stop. begin() re-initialises all. 
void ClockedSniffer::stop()
{
    R8_SYS_CFG = 0;             // halt the PIOC eMCU (clock gate off)
    head_ = tail_ = 0;          // flush RAM staging ring
    burstOpen_ = false;
    burstN_    = 0;
}

// ---- BINARY format (COBS-0xFF, 0xFF boundary) ==============================
// One COBS-0xFF framed record per "burst" (the run of captured bytes between
// idle gaps), with microsecond timestamps. The RAM ring is reused as the output
// staging / USB-backpressure buffer: a finished record is framed into it whole,
// then drained to USB non-blocking.
//
// Record (pre-frame, little-endian):
//   u8  type      0x01 = data burst
//   u32 t_start   micros() of the first byte of the burst
//   u16 dur_us    micros(last byte) - t_start (capped 0xFFFF)
//   u16 onset_us  micros between the 1st and ONSET_N-th byte (0 = n/a) -> clock est.
//   u8  flags     bit0 = PIOC overflow, bit1 = RAM overflow, bit2 = continued
//   u16 n         payload length
//   u8  payload[n] raw bit-packed capture bytes (host reframes on preamble)
// Records are COBS-0xFF framed with a 0xFF boundary -> the host resyncs at the
// next boundary after any loss (and skips boot text before the first one).
// ========================================================================== 

// Encode the open burst as a record and stage it into the RAM ring (whole or
// not, so a record never tears). Clears the overflow flags only once delivered.
// `continued` = this record was cut at BURST_MAX (more bit-contiguous bytes
// follow) -> the host joins it with the next record before decoding. 
void ClockedSniffer::emitRecord(bool continued)
{
    if (!burstOpen_) return;
    // static (not stack): BURST_MAX is large, and emit is single-threaded.
    static uint8_t rec[12 + BURST_MAX];
    uint16_t k = 0;
    rec[k++] = 0x01;
    rec[k++] = (uint8_t)(burstT0_);       rec[k++] = (uint8_t)(burstT0_ >> 8);
    rec[k++] = (uint8_t)(burstT0_ >> 16); rec[k++] = (uint8_t)(burstT0_ >> 24);
    uint32_t d = burstLast_ - burstT0_;
    uint16_t dur = (d > 0xFFFF) ? 0xFFFF : (uint16_t)d;
    rec[k++] = (uint8_t)(dur);            rec[k++] = (uint8_t)(dur >> 8);
    rec[k++] = (uint8_t)(burstOnset_);    rec[k++] = (uint8_t)(burstOnset_ >> 8);
    uint8_t flags = (ovfPioc_ ? 1 : 0) | (ovfRam_ ? 2 : 0) | (continued ? 4 : 0);
    rec[k++] = flags;
    rec[k++] = (uint8_t)(burstN_);        rec[k++] = (uint8_t)(burstN_ >> 8);
    for (uint16_t i = 0; i < burstN_; i++) rec[k++] = burstBuf_[i];

    static uint8_t enc[12 + BURST_MAX + (12 + BURST_MAX) / 254 + 2];
    uint16_t m = (uint16_t)RecordFramer::frameRecord(rec, k, enc);   // COBS-0xFF + 0xFF boundary
    if (ringSpace() >= m) {
        ringPush(enc, m);
#ifdef DIAG
        diagOvfPioc_ += ovfPioc_; diagOvfRam_ += ovfRam_;   // re-sum delivered loss for [T]
#endif
        ovfPioc_ = ovfRam_ = 0;
    }
    else { ovfRam_++; }                          // record dropped, flag carries on
    burstOpen_ = false;
    burstN_    = 0;
}

// Advertise this datapath's fixed identity. ClockedSniffer IS the clocked datapath - it
// never needs to be told its mode; it just repeats who it is. Staged into the RAM ring
// BETWEEN records (a safe boundary). Cheap and bus-independent: lets the host confirm
// the mode even on a mute line. 
void ClockedSniffer::emitModeMarker()
{
    static const char MARKER[] = "[MODE clocked wire=2]";
    uint8_t out[40];
    size_t w = RecordFramer::frameMeta(MARKER, out);
    if (w && ringSpace() >= w) ringPush(out, (uint16_t)w);
}

// Stage-2 drain tuning, ported from the rle datapath. PUMP_EVERY must be a
// power of two: how many bytes between bulk-IN pumps (smaller = shorter EP auto-NAK
// freeze = higher pktHz, more pump overhead). DRAIN_MAX caps bytes shipped per loop so
// one mode can't starve the other once the selector routes service(). Both were chosen
// to match CAN's validated 467 KB/s drain; re-tune only against ovf=0 + host throughput. 
static constexpr uint16_t PUMP_EVERY = 16;
static constexpr uint16_t DRAIN_MAX  = 192;

void ClockedSniffer::service(uint32_t nowMs)
{
#ifdef DIAG
    loopCalls_++;                                   // main-loop iteration counter (loopHz)
#endif
    if ((int32_t)(nowMs - modeHbMs_) >= 0) { emitModeMarker(); modeHbMs_ = nowMs + 1000; }

    // stage 1: drain PIOC FIFO -> timestamped burst buffer.
    uint8_t h = DR_[RING_HEAD];
    if ((uint8_t)(h - sniffTail_) > RING_SLOTS) {
        ovfPioc_++;                                 // PIOC FIFO lapped us: oldest slots overwritten
        sniffTail_ = (uint8_t)(h - RING_SLOTS);     // skip to the oldest still-valid slot so we drain
// valid bytes only, never a stale wrap (mirror
// rle_sniffer.cpp drainPioc). ovfPioc_ already rides
// the next record's flags byte to the host.
    }
    bool got = false;
    while (sniffTail_ != h) {
        uint8_t b = DR_[0x20 + (sniffTail_ & (RING_SLOTS - 1))];
        sniffTail_++;
        if (!burstOpen_) {
            burstOpen_ = true; burstN_ = 0; burstOnset_ = 0;
            burstT0_ = Time::micros();
        }
        if (burstN_ < BURST_MAX) burstBuf_[burstN_++] = b;
        if (burstN_ == ONSET_N) {                // clock estimate: time of the first ~frame
            uint32_t od = Time::micros() - burstT0_;
            burstOnset_ = (od > 0xFFFF) ? 0xFFFF : (uint16_t)od;
        }
        got = true;
        if (burstN_ >= BURST_MAX) {              // payload cap -> split, continue next byte
            burstLast_ = Time::micros();
            emitRecord(true);                    // continued: more bytes follow
        }
    }
    if (got) { burstLast_ = Time::micros(); sniffLastMs_ = nowMs; }

    // finalize the open burst on an idle gap (>3 ms) or the continuous-traffic cap
    if (burstOpen_ &&
        ((uint32_t)(nowMs - sniffLastMs_) > 3 ||
         (uint32_t)(Time::micros() - burstT0_) > BURST_TCAP_US)) {
        emitRecord(false);                       // terminal: real gap / time cap
    }

    // stage 2: drain RAM ring (encoded record bytes) -> USB verbatim, non-blocking.
    // Ported from the rle datapath: a zero-copy 1-pass drain straight into the
    // TX backing buffer, with an interleaved usb_.pump() every PUMP_EVERY bytes. The
    // pump un-freezes + re-arms the bulk-IN EP MID-drain, so a single service() ships
    // MANY packets instead of one - this breaks the 1-packet/loop INT_BUSY ceiling that
    // caps a continuous clocked bus. Records are already framed, so (unlike CAN) there's
    // no squelch here: a pure verbatim copy. The stream is byte-oriented (host resyncs on
    // the 0xFF boundary), so tearing a record across loops is harmless. 
    uint16_t budget = (uint16_t)usb_.writeSpace();
    if (budget > DRAIN_MAX) budget = DRAIN_MAX;
    if (budget) {
        uint16_t n = ringCount();
        if (n > budget) n = budget;
        if (n) {
            uint32_t cap = 0;
            uint8_t* dst = usb_.txReserve(&cap);     // contiguous writable TX bytes
            if (cap) {
                uint16_t outl = 0;
                uint16_t t = tail_;
                uint16_t i = 0;
                for (; i < n; i++) {
                    if ((i & (PUMP_EVERY - 1)) == 0) usb_.pump();   // re-arm the EP mid-drain
                    if (outl >= cap) break;          // TX run full -> rest next loop
                    dst[outl++] = ring_[(uint16_t)(t + i) & RING_MASK];
                }
                tail_ = t + i;                        // consumed exactly i bytes
                usb_.txCommit(outl);
            }
        }
    }

#ifdef DIAG
    // track RAM-ring high-water + the TX low-water for the [T] window
    { uint16_t fill = ringCount(); if (fill > ramHi_) ramHi_ = fill; }
    { uint16_t f = (uint16_t)usb_.writeSpace(); if (f < txMinFree_) txMinFree_ = f; }

    // ---- ONE telemetry line per second, DIAGNOSTIC ONLY (build with -DDIAG).
    // Symmetric with RleSniffer's [T] but for the clocked datapath: no TIM3 drain here,
    // so no drnCpu/maxGap - this reports loop/USB throughput + accumulated overflow.
    // loopHz ~ pktHz => the loop limits; loopHz >> pktHz => the single-buffer EP is the
    // ceiling; usbB = REAL bytes/s shipped; ovf* > 0 => capture loss (also per-record). 
    static uint32_t lastT = 0, prevLoop = 0, prevPkts = 0, prevBytes = 0, prevShort = 0;
    if ((uint32_t)(nowMs - lastT) >= 1000) {
        uint32_t dt = (uint32_t)(nowMs - lastT); if (!dt) dt = 1;
        lastT = nowMs;
        uint32_t pkts   = usb_.txPackets();
        uint32_t bytes  = usb_.txBytes();
        uint32_t shorts = usb_.txShort();
        uint32_t loopHz = (loopCalls_ - prevLoop) * 1000u / dt;
        uint32_t pktHz  = (pkts - prevPkts) * 1000u / dt;
        uint32_t usbBps = (bytes - prevBytes) * 1000u / dt;     // REAL bytes/s shipped
        uint32_t shortHz = (shorts - prevShort) * 1000u / dt;   // <64B packets/s (waste)
        prevLoop = loopCalls_; prevPkts = pkts; prevBytes = bytes; prevShort = shorts;
        uint16_t rhi = ramHi_; uint32_t ovfP = diagOvfPioc_, ovfR = diagOvfRam_;
        uint16_t txf = (txMinFree_ == 0xFFFF) ? (uint16_t)usb_.writeSpace() : txMinFree_;
        ramHi_ = 0; diagOvfPioc_ = 0; diagOvfRam_ = 0; txMinFree_ = 0xFFFF;
        // Framed via frameMeta (0xFE <ascii> 0xFE 0xFF) and staged through the RAM ring
        // like emitModeMarker - so the block lands on a safe record boundary, never mid
        // record. The payload starts with '[' (frameMeta's contract); diag_monitor.py
        // anchors on 0xFE'[' to resync after a mid-block start. 
        char ascii[160];
        int k = snprintf(ascii, sizeof(ascii),
            "[T t=%lu loopHz=%lu pktHz=%lu shortHz=%lu usbB=%lu ovfPioc=%lu ovfRam=%lu ramHi=%u txFree=%u]",
            (unsigned long)nowMs, (unsigned long)loopHz, (unsigned long)pktHz, (unsigned long)shortHz,
            (unsigned long)usbBps, (unsigned long)ovfP, (unsigned long)ovfR, (unsigned)rhi, (unsigned)txf);
        if (k > 0) {
            uint8_t out[163];
            size_t w = RecordFramer::frameMeta(ascii, out);
            if (w && ringSpace() >= w) ringPush(out, (uint16_t)w);
        }
    }
#endif // DIAG
}
