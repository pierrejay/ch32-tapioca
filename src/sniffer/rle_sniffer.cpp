// rle_sniffer.cpp - RleSniffer implementation. See rle_sniffer.hpp for the
// datapath overview.
//
// Two-level drain (keeps the PIOC FIFO from overflowing at worst-case CAN):
//   - drainPioc()  : TIM3 IRQ @ 50 kHz, PIOC ring -> RAM ring (ISR-safe, runs from RAM)
//   - service()    : main loop, RAM ring -> USB-CDC (binary run-bytes, non-critical)
#include "rle_sniffer.hpp"
#ifdef RUN_SNIFFER
#include "record_framer.hpp"         // RecordFramer::frameMeta for the [MODE rle …] heartbeat
#endif

extern "C" {
#include "PIOC_SFR.h"
#include <string.h>
#include <stdio.h>
}

volatile uint8_t* const RleSniffer::DR_ = (volatile uint8_t*)PIOC_SFR_BASE;

// RLE_MIN_KBPS: the slowest bus we claim to capture. The firehose-bound K
// (consecutive 128-tick continuation bytes that mean "idle") is set so a real
// in-frame mono-level run (<=16 bit-times) at that bitrate never reaches it:
//   K = 16 bits * (48e6 / ~8 cyc-per-tick) / 128 ticks-per-cap / (kbps*1000)
//     ~= 750 / kbps   (floored at 1)
// MIN_KBPS = 0 DISABLES the bound -> raw continuous 0x80, no 0xFF idle boundary (debug/firehose). 
#ifndef RLE_MIN_KBPS
#define RLE_MIN_KBPS 5
#endif
#if (RLE_MIN_KBPS) <= 0
static constexpr uint32_t CAP_IDLE_K = 0;          // 0 = bound disabled
#else
static constexpr uint32_t CAP_IDLE_K =
    (750u / (uint32_t)(RLE_MIN_KBPS)) ? (750u / (uint32_t)(RLE_MIN_KBPS)) : 1u;
#endif

// Singleton pointer so the TIM3 ISR can reach the active RleSniffer instance.
// Set in begin(), never changed after. Safe to read from ISR (ROM-constant). 
static RleSniffer* g_drainTarget = nullptr;

// ---- RAM staging ring (identique à ClockedSniffer) ---------------------------
void RleSniffer::ringPush(const uint8_t* p, uint16_t n)
{
    if (ringSpace() < n) { ovfRam_++; return; }
    uint16_t h = head_;
    for (uint16_t i = 0; i < n; i++) ring_[(h + i) & RING_MASK] = p[i];
    head_ = h + n;
}

uint16_t RleSniffer::ringPop(uint8_t* out, uint16_t max)
{
    uint16_t n = ringCount();
    if (n > max) n = max;
    uint16_t t = tail_;
    for (uint16_t i = 0; i < n; i++) out[i] = ring_[(t + i) & RING_MASK];
    tail_ = t + n;
    return n;
}

// ---- PIOC bring-up --------------------------------------------------------
void RleSniffer::loadBlob()
{
    static const __attribute__((aligned(16))) unsigned char prog[] =
#if defined(RUN_RLE_SNIFFER) || defined(RUN_SNIFFER) || defined(RUN_RLE_TICK_TEST)
        #include "../../pioc/rle_sniffer_inc.h"  // the rle capture blob: 30-slot FIFO, data IO1=PC19
#else
        { 0 };  // this TU is compiled for every env, but RleSniffer is only instantiated
// under RUN_RLE_SNIFFER/RUN_SNIFFER/RUN_RLE_TICK_TEST - loadBlob() is dead here
#endif

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
}

// ---- TIM3 drain timer, prio 0 (above millis TIM2) -------------------------
extern "C" void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __HIGHCODE;
extern "C" void TIM3_IRQHandler(void)
{
    // LEAN handler: poke the flag register directly instead of the SDK's
    // TIM_GetITStatus / TIM_ClearITPendingBit (non-inline, ~3-4 us each -> ~95% CPU at
    // this drain rate). INTFR bit0 = update flag; clearing is a write of ~flag. 
    if (TIM3->INTFR & TIM_IT_Update) {
        TIM3->INTFR = (uint16_t)~TIM_IT_Update;
        if (g_drainTarget) g_drainTarget->drainPioc();
    }
}

void RleSniffer::startDrainTimer()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef timer = {0};
    // 50 kHz drain (20 us period). The LEAN handler (direct INTFR access, no SDK calls)
    // keeps the per-fire cost at ~15-20% CPU; the 30-slot ring then holds 6 slots of real
    // margin at this rate, validated lossless at worst-case CAN (see pioc/README.md). 
    timer.TIM_Period        = 960 - 1;   // 48 MHz / 960 = 50 kHz -> 20 us
    timer.TIM_Prescaler     = 0;         // no prescale: full 48 MHz clock
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_CounterMode   = TIM_CounterMode_Up;
    timer.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &timer);

    TIM_SetCounter(TIM3, 0);
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    NVIC_InitTypeDef nvic = {0};
    nvic.NVIC_IRQChannel                   = TIM3_IRQn;   // CH32X035: single TIM3 IRQ
    nvic.NVIC_IRQChannelPreemptionPriority = 0;   // prio 0: above TIM2 millis (prio 1)
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

// ---- ISR drain : PIOC ring -> RAM ring (called @ 50 kHz) ------------------
// HOT PATH: this runs in TIM3 IRQ @ 50 kHz. It must be FAST — if it takes
// longer than the TIM3 period (20 µs), the next fire is delayed (no self-
// nesting under WCH-Interrupt-fast), and bytes pile up in the PIOC ring ->
// overflow. So: no function calls in the loop, direct ring writes, squelch
// inlined. The previous version called ringPush() per byte (non-inline,
// ringSpace() check each time) = ~90 cyc/byte -> 16 bytes = 30+ µs -> self-
// blocking -> maxGap=31. This version writes directly to the ring buffer. 
void RleSniffer::drainPioc()
{
#ifdef DIAG
    uint16_t _c0 = (uint16_t)TIM2->CNT;        // 0..999 us, inline read (no call)
#endif

    uint8_t h = DR_[RING_HEAD];
    uint8_t rawGap = (uint8_t)(h - sniffTail_);
    uint8_t gap = rawGap;
    if (rawGap > maxGap_) maxGap_ = rawGap;
    uint8_t slot = sniffSlot_;       // physical location of logical sniffTail_
    if (rawGap > RING_SLOTS) {
        ovfPioc_++;
        sniffTail_ = (uint8_t)(h - RING_SLOTS);
        gap = RING_SLOTS;
        // The producer advanced rawGap slots since our old cursor. Its next write
        // position is also the oldest retained slot after an overrun. rawGap is
        // modulo 256, but this ISR's observed delays are far below 256 records. 
        uint8_t advance = rawGap;
        while (advance >= RING_SLOTS) advance -= RING_SLOTS;
        slot = (uint8_t)(slot + advance);
        if (slot >= RING_SLOTS) slot -= RING_SLOTS;
    }

    // DUMB COPY: no squelch, no branches per byte. Just PIOC ring -> RAM ring.
    // ~5 cycles/byte (read SFR + write RAM + inc + check). 16 bytes = ~80 cyc
    // = 1.7 µs. Well under the 20 µs TIM3 period.
    //
    // Idle 80s DO enter the RAM ring (~47 KB/s) but the 2KB ring buffers ~40 ms
    // of idle — service() has plenty of time to squelch + drain to USB. The
    // squelch (first-80-only) is done in service() when reading the RAM ring. 
    uint16_t rh = head_;
    uint16_t rt = tail_;
    uint16_t space = (uint16_t)(RING_MASK - ((uint16_t)(rh - rt) & RING_MASK));
    uint8_t toCopy = gap;
    if (toCopy > space) { toCopy = space; }

    // NB: tried reading the 16-byte PIOC ring as 4x uint32 (R32_DATA_REGn) to cut SFR
    // accesses - it made drnCpu WORSE (28->34%). The PIOC SFR is on the AHB peripheral
    // bus whose access cost is ~proportional to byte count, not fixed per transaction, so
    // 4 words (16 bytes) costs more than gap (~11) byte reads. Byte-by-byte is optimal. 
    // 30 does not divide HEAD's 256-count epoch: never derive a slot with a mask
    // or modulo here. Two contiguous loops avoid a division and keep the per-byte
    // hot path exactly one PIOC SFR read plus one RAM write. 
    uint8_t first = (uint8_t)(RING_SLOTS - slot);
    if (first > toCopy) first = toCopy;
    for (uint8_t i = 0; i < first; i++) {
        ring_[(rh + i) & RING_MASK] = DR_[RING_BASE + slot + i];
    }
    for (uint8_t i = first; i < toCopy; i++) {
        ring_[(rh + i) & RING_MASK] = DR_[RING_BASE + i - first];
    }
    // The logical cursor skips ALL gap bytes when RAM is full. On a normal drain
    // rawGap==gap; after an overrun slot already denotes the post-overrun tail. 
    if (rawGap <= RING_SLOTS) {
        slot = (uint8_t)(slot + rawGap);
        if (slot >= RING_SLOTS) slot -= RING_SLOTS;
    }
    sniffSlot_ = slot;
    sniffTail_ += gap;       // advance past ALL bytes (even if some not copied)
    rh += toCopy;
    head_ = rh;

    // bytes we skipped because the RAM ring was full = silent capture loss.
    if (toCopy < gap) ramDrop_ += (uint32_t)(gap - toCopy);
    uint16_t fill = (uint16_t)(rh - rt) & RING_MASK;   // RAM-ring occupancy after push
    if (fill > ramHi_) ramHi_ = fill;

#ifdef DIAG
    uint16_t _c1 = (uint16_t)TIM2->CNT;
    uint16_t _d  = (_c1 >= _c0) ? (uint16_t)(_c1 - _c0) : (uint16_t)(1000u + _c1 - _c0);
    drnCyc_ += _d;
    if (_d > drnMax_) drnMax_ = _d;
#endif
}

// ---- lifecycle ------------------------------------------------------------
void RleSniffer::begin(uint32_t nowMs)
{
    loadBlob();
    // The blob starts from a known HEAD=0 / slot=0 state (no 1 ms wait in loadBlob),
    // so no unknown number of records can be emitted before these two cursors align. 
    sniffTail_ = 0;
    sniffSlot_ = 0;
    sniffLastMs_ = nowMs;
    sniffActive_ = false;
    head_ = tail_ = 0;
    ovfRam_ = ovfPioc_ = 0;
    ramDrop_ = 0;
    lossSeen_ = 0;
    ramHi_ = 0;
    maxGap_ = 0;
    txMinFree_ = 0xFFFF;
    capRun_ = 0;
    capPiped_ = false;

    g_drainTarget = this;       // arm the TIM3 ISR target before starting the timer

#ifdef RUN_SNIFFER
    // Announce the mode IMMEDIATELY so a clocked->rle switch confirms instantly. The
    // periodic heartbeat (service()) is idle-gated to avoid tearing a frame, which would
    // otherwise defer the first [MODE rle] until the first bus idle gap - so the pill
    // lagged whenever the bus had traffic at switch time. Safe here: capture just
    // (re)started, ring empty, no frame in flight, main.cpp already wrote the 0xFD seam. 
    modeHbMs_ = nowMs + 1000;
    emitModeMarker();
#endif

#ifdef DIAG
    // ASCII banner -> only under DIAG: in binary mode it would be read as run-bytes.
    printf("rle sniffer (30-slot ring; TIM3 @ 50 kHz drain; 0xFF = gap)\r\n");
#endif // DIAG

    startDrainTimer();          // start LAST: TIM3 fires immediately after this
}

// Quiesce the rle datapath for a runtime mode switch (dual-instance selector). Order
// matters: silence the TIM3 drain ISR FIRST so it can't fire drainPioc() on a PIOC the
// other mode is about to repurpose; clearing g_drainTarget makes even a late-pending IRQ
// a no-op (the handler checks it). Then halt the eMCU and drop the RAM staging ring. The
// capture path is left cold - begin() (this or the other class) re-initialises everything. 
void RleSniffer::stop()
{
    TIM_Cmd(TIM3, DISABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
    g_drainTarget = nullptr;
    R8_SYS_CFG = 0;             // halt the PIOC eMCU (clock gate off)
    head_ = tail_ = 0;          // flush RAM staging ring
    sniffActive_ = false;
    capRun_ = 0;
    capPiped_ = false;
}

#ifdef RUN_SNIFFER
// Advertise this datapath's fixed identity. RleSniffer IS the rle datapath - it never
// needs to be told its mode; it just repeats who it is. Written STRAIGHT to USB and only
// from the idle-gap branch of service() (no frame in flight), so the leading 0xFF + meta
// block can't tear a run-byte frame (the CAN RAM ring carries raw run-bytes, not framed
// metadata, so we can't route it through the ring like ClockedSniffer does). 
void RleSniffer::emitModeMarker()
{
    // thi/tlo = per-level tick period in CPU cycles; fcpu = the device clock. Together they
    // let the host convert run-byte tick counts into absolute time (baud = fcpu/(UI*tick)).
    // See TICK_HI_CYC/TICK_LO_CYC in rle_sniffer.hpp. 
    char marker[64];
    int k = snprintf(marker, sizeof marker, "[MODE rle wire=2 thi=%u tlo=%u fcpu=%lu]",
                     (unsigned)TICK_HI_CYC, (unsigned)TICK_LO_CYC, (unsigned long)SystemCoreClock);
    if (k <= 0) return;
    uint8_t out[80];
    out[0] = 0xFF;                                // close the prior (idle) segment
    size_t w = RecordFramer::frameMeta(marker, out + 1);
    if (w && usb_.writeSpace() >= w + 1) usb_.write(out, (uint32_t)(w + 1));
}
#endif

// ---- BINARY format: run-bytes 0x00..0x80 raw, 0xFF = boundary, 0xFE-framed diag ----
bool RleSniffer::emitBoundary()
{
    uint8_t b = 0xFF;
    return usb_.write(&b, 1) == 1;          // whole-or-not: caller keeps sniffActive_ + retries if full
}

// How often (in drained bytes) to pump the bulk-IN EP inside the drain loop. PERFORMANCE
// PARAMETER, not a law - must be a power of 2. Smaller = shorter EP auto-NAK freeze (higher
// pktHz) but more pump overhead; larger = the reverse. 16 (~10us window) was chosen by
// measurement. Re-tune ONLY against the validation criteria: ovf=0, maxGap under margin
// (<RING_SLOTS/RING_SLOTS), RAM ring not pinned-full at lower bus loads, host throughput (raw_throughput.py),
// and shortHz~0 (all packets full). 
static constexpr uint16_t PUMP_EVERY = 16;

void RleSniffer::service(uint32_t nowMs)
{
    loopCalls_++;                              // main-loop iteration counter (loopHz)
    bool sawActivity = false;                  // run data consumed since the last boundary

    // stage 2: drain the RAM ring -> USB (the ISR is just a dumb ~1.7µs copy). Firehose
    // squelch: a run > 128 ticks is a SEQUENCE of >=128 continuation bytes; below
    // CAP_IDLE_K they pass through (host sums a long in-frame run), at CAP_IDLE_K the run
    // is a long idle -> emit one 0xFF boundary and squelch the rest until a terminal (<128)
    // byte. CAP_IDLE_K=0 -> pass everything (raw). 
    uint16_t wfree   = (uint16_t)usb_.writeSpace();
#ifdef DIAG
    uint16_t avail   = (wfree > 160) ? (uint16_t)(wfree - 160) : 0;  // reserve for the 0xFE diag block
#else
    uint16_t avail   = wfree;                                        // binary, no diag: use it all
#endif
    uint16_t budget  = avail;        // 1 run-byte = 1 tx byte; tx_ overflow handled by the cap.
    // Cap = bytes drained per loop. The interleaved pump() ships several packets per loop,
    // so 192 lets service() drain a lot per loop while the pump keeps tx_ drained. Tune vs
    // CPU: drnCpu must stay clear of starving the 50 kHz drain ISR. 
    if (budget > 192) budget = 192;
    if (budget) {
        uint16_t n = ringCount();
        if (n > budget) n = budget;
        if (n) {
            // 1-pass drain: filter the RAM ring STRAIGHT into the TX ring's backing
            // buffer (zero-copy) - no out[] temp, no Ring::push second copy. Squelch
            // state is hoisted into registers (capRun_/capPiped_ are class members ->
            // otherwise a RAM load+store per byte). If the contiguous TX run fills, we
            // stop and leave the rest in the RAM ring (back-pressure), advancing tail_
            // by exactly what we consumed. 
            uint32_t cap = 0;
            uint8_t* dst = usb_.txReserve(&cap);        // contiguous writable TX bytes
            if (cap) {
                uint16_t outl = 0;
                uint16_t t = tail_;
                uint16_t i = 0;
                uint32_t capRun   = capRun_;
                bool     capPiped = capPiped_;
                for (; i < n; i++) {
                    if ((i & (PUMP_EVERY - 1)) == 0) usb_.pump();  // un-freeze+re-arm the EP mid-drain
                    if (outl >= cap) break;             // TX run full -> rest next loop
                    uint8_t b = ring_[(uint16_t)(t + i) & RING_MASK];
                    if (b >= 128) {                     // continuation byte
                        if (CAP_IDLE_K && capPiped) continue;        // past idle -> drop
                        capRun++;
                        if (CAP_IDLE_K && capRun >= CAP_IDLE_K) {
                            dst[outl++] = 0xFF;          // idle boundary inline
                            capPiped = true;
                            sniffActive_ = false;
                            sawActivity = false;
                            continue;
                        }
                        sawActivity = true;              // long run, but still below idle threshold
                        // below threshold: legit long in-frame run -> pass through
                    } else {                            // terminal byte (<128)
                        bool idleTail = capPiped;       // first terminal after a boundary
                        capRun = 0; capPiped = false;
                        sawActivity = true;
                        if (idleTail) continue;          // drop the idle tail (0xFF was the boundary)
                    }
                    dst[outl++] = b;                    // raw run-byte (0x00..0x80)
                } // for loop
                capRun_ = capRun; capPiped_ = capPiped;
                tail_ = t + i;                          // consumed exactly i bytes from the RAM ring
                usb_.txCommit(outl);
            } // if (cap)
        }
    }
    // tx_ low-water: 0 => the USB endpoint can't keep up (it's the drain ceiling).
    { uint16_t f = (uint16_t)usb_.writeSpace(); if (f < txMinFree_) txMinFree_ = f; }

    // Capture-loss marker (works WITHOUT -DDIAG): if the PIOC->RAM ring overflowed
    // since last call (silent data loss), emit one 0xFD sentinel into the stream. 0x81..0xFD
    // are structurally impossible as run-bytes, so the host can mark a discontinuity here
    // instead of silently mis-decoding across the gap. One marker per lossy service() call
    // (not per byte): under the firehose it fires ~every loop = honest "still dropping"; on
    // real traffic that fits, it never fires. A capture must never lose data silently. 
    {
        uint32_t rd = ramDrop_;
        if (rd != lossSeen_) {
            uint8_t mk = 0xFD;
            // advance the watermark ONLY after the 0xFD is really on the wire. If TX is
            // full, lossSeen_ stays behind and we re-emit next service() - the loss
            // marker is never dropped silently (was: lossSeen_=rd before the write). 
            if (usb_.write(&mk, 1) == 1) lossSeen_ = rd;
        }
    }

    // track last-activity ms for the idle-gap separator. Read the volatile tail
    // to detect activity from the ISR without racing (head_ is ISR-written). 
    // head_ (ISR-written) and tail_ (ours) are both aligned u16 -> single-instruction
    // reads on RV32, so ringCount() needs no __disable_irq() guard.
    uint16_t ramCount = ringCount();
    if (sawActivity || ramCount > 0) {
        sniffLastMs_ = nowMs;
        sniffActive_ = true;
    }

    // idle gap -> frame separator (only once RAM ring flushed)
    if (sniffActive_ && ramCount == 0 && (uint32_t)(nowMs - sniffLastMs_) > 3) {
        if (emitBoundary()) sniffActive_ = false;   // TX full -> stay active, retry the 0xFF next loop
    }

#ifdef RUN_SNIFFER
    // mode heartbeat, emitted at the idle boundary (safe: !sniffActive_ => no frame in
    // flight, the last boundary was already written). ~1 s cadence; symmetric with
    // ClockedSniffer's between-records [MODE] so the host confirms the mode on a quiet bus. 
    if (!sniffActive_ && (int32_t)(nowMs - modeHbMs_) >= 0) {
        emitModeMarker();
        modeHbMs_ = nowMs + 1000;
    }
#endif

#ifdef DIAG
    // ---- ONE telemetry line per second, DIAGNOSTIC ONLY (build with -DDIAG).
    // OFF by default so the wire stream is pure capture data. Kept for re-validating
    // the drain on a new bus (e.g. CAN 1 Mbit): loopHz/pktHz/drop/ramHi locate the
    // bottleneck. loopHz >> pktHz => single-buffer EP is the ceiling; loopHz ~ pktHz
    // => the loop itself limits; usbB = REAL bytes shipped/s (txBytes, not pktHz*64 which
    // over-counts when the pump ships short packets); drop>0 => loss. 
    static uint32_t lastT = 0, prevLoop = 0, prevPkts = 0, prevBytes = 0, prevShort = 0;
    if ((uint32_t)(nowMs - lastT) >= 1000) {
        uint32_t dt = (uint32_t)(nowMs - lastT); if (!dt) dt = 1;
        lastT = nowMs;
        uint32_t pkts   = usb_.txPackets();
        uint32_t bytes  = usb_.txBytes();
        uint32_t shorts = usb_.txShort();
        uint32_t loops  = loopCalls_ - prevLoop; if (!loops) loops = 1;
        uint32_t loopHz = (loopCalls_ - prevLoop) * 1000u / dt;
        uint32_t pktHz  = (pkts - prevPkts) * 1000u / dt;
        uint32_t usbBps = (bytes - prevBytes) * 1000u / dt;   // REAL bytes/s shipped
        uint32_t shortHz = (shorts - prevShort) * 1000u / dt;  // <64B packets/s (waste)
        uint32_t avgSvc  = svcSum_  / loops;   // avg us in service() per loop
        prevLoop = loopCalls_; prevPkts = pkts; prevBytes = bytes; prevShort = shorts; svcSum_ = 0;
        __disable_irq();
        uint16_t rhi = ramHi_; uint32_t rdrop = ramDrop_;
        uint8_t  mg  = maxGap_; uint16_t ovf = ovfPioc_;   // PIOC ring: margin + overflow
        uint32_t drnCyc = drnCyc_; uint16_t drnMax = drnMax_;
        ramHi_ = 0; ramDrop_ = 0; maxGap_ = 0; ovfPioc_ = 0;
        drnCyc_ = 0; drnMax_ = 0;
        __enable_irq();
        // ISR-CPU%% over the window: drnCyc us spent in drainPioc / window us.
        uint32_t drnPct = (uint32_t)((uint64_t)drnCyc * 100u / ((uint64_t)dt * 1000u));
        uint16_t txf = (txMinFree_ == 0xFFFF) ? (uint16_t)usb_.writeSpace() : txMinFree_;
        txMinFree_ = 0xFFFF;
        // Framed diag block: 0xFE <ascii> 0xFE. Bracketed by a sentinel (0x81..0xFF never
        // occur as data) so the host skips it for decode, parses it for stats. t=millis ->
        // offline fps/jitter. maxGap/RING_SLOTS = PIOC fill (CAN margin); ovf>0 = source loss.
        // PROTOCOL CONTRACT: the payload MUST start with '[' (0x5B). diag_monitor.py anchors
        // a block open on 0xFE followed by '[' to resync after a mid-block start; any future
        // block type must keep the leading '[' or the host parser desyncs. 
        char m[176];
        m[0] = (char)0xFE;
        int k = snprintf(m + 1, sizeof(m) - 2,
            "[T t=%lu loopHz=%lu pktHz=%lu shortHz=%lu usbB=%lu svcUs=%lu drnCpu=%lu%% drnMax=%u ramHi=%u maxGap=%u/%u ovf=%u txFree=%u drop=%lu]",
            (unsigned long)nowMs, (unsigned long)loopHz, (unsigned long)pktHz, (unsigned long)shortHz, (unsigned long)usbBps,
            (unsigned long)avgSvc, (unsigned long)drnPct, (unsigned)drnMax,
            (unsigned)rhi, (unsigned)mg, (unsigned)RING_SLOTS, (unsigned)ovf, (unsigned)txf, (unsigned long)rdrop);
        if (k > 0) {
            m[1 + k] = (char)0xFE;
            uint32_t total = (uint32_t)k + 2;
            if (total <= usb_.writeSpace()) usb_.write((const uint8_t*)m, total);
        }
    }
#endif // DIAG
} // RleSniffer::service()
