// rle_tick_test.hpp - bench: measure the rle blob's per-level tick period in EXACT
// integer CPU cycles, so the host can turn run-byte counts into real time.
//
// WHY (the host's need)
// ---------------------
// In rle mode the device streams run-bytes = tick counts. To recover ABSOLUTE time
// - e.g. a UART baud rate, where the bitrate is NOT a known constant the way CAN's
// 1 Mbit or DMX's 250 kbaud are - the host must know how long one tick lasts. A tick
// is one iteration of the blob's counting loop = a FIXED integer number of CPU cycles.
// There are TWO of them: the HIGH-level and LOW-level loops differ by one branch, so
// they are not equal (measured: 9 vs 8 cycles).
//
// THE MODEL (why an INTEGER, not a decimal average)
// -------------------------------------------------
// A level interval of real duration D cycles is reported as N ticks where
//
//     D = c*N + k      c = loop cycles per tick (integer, per level)
//                      k = per-edge store overhead (cycles "stolen" from the count
//                          while the line is already at the new level)
//
// So the naive "cycles per tick" = D/N = c + k/N is NOT a constant: it depends on the
// run length N. That decimal (we first measured ~9.39 / ~8.41) is an ARTIFACT, not a
// fractional tick - it folds the per-run offset k into the slope.
//
// The host only needs c, because bit-time is a DIFFERENCE between runs and the offset
// k cancels:  T_bit = UI * c  (UI = ticks/bit, the slope), so
//
//     baud = f_cpu / (UI * c)     per level   (f_cpu = 48 MHz, HSI tolerance ~1%)
//
// And the host already separates the two: fit_ui() in can_rle_decode.py fits
// `counts ~= bits*UI + C`, where C ("the constant push-bias of the capture loop") IS
// this k (in ticks). So it already absorbs k as its intercept - shipping the decimal
// D/N would DOUBLE-count the offset. We ship the integer c per level; k is not needed
// to decode (it cancels), only for the absolute length of a single isolated interval.
//
// THE MEASUREMENT (why CONSTANT level, not a square wave)
// ------------------------------------------------------
// Counting ticks per run on a square wave folds k into every run (~1 tick over only
// ~27) AND beats against the edge phase -> a fractional, noisy number. Instead we HOLD
// the line at one level (drive PA7 -> jumper -> PC19): with no edge the loop free-runs
// and caps every 128 ticks (emits 0x80). We count caps over a window timed by the
// 48 MHz micros() clock:
//
//     c = window_cycles / (128 * caps)
//
// Now k hits once per 128 ticks instead of once per ~27, so its bias on c shrinks to
// ~0.06 cycle -> the reading lands on the integer (e.g. 9.057 -> 9). No edges -> no
// phase beat; a held level is also immune to any feed timing. HIGH (PA7=1) and LOW
// (PA7=0) are measured in alternation.
//
// Wiring: jumper PA7 -> PC19 (same pin as the capture data line). Output: [RLE-TICK ...]
// over USB-CDC, read with any serial terminal. BENCH ONLY (-D RUN_RLE_TICK_TEST).
#pragma once

#include "rle_sniffer.hpp"

class RleTickTest
{
public:
    explicit RleTickTest(RleSniffer& rle) : rle_(rle) {}

    void begin(uint32_t nowMs);     // drive PA7, bring up the real rle blob (TIM3 drain)
    void service(uint32_t nowMs);   // count caps, flip level + report every phase window

private:
    void enterPhase(bool high);     // drive the held level, reset the window
    void report();                  // c = window_cycles / (128 * caps) -> nearest integer

    RleSniffer& rle_;
    bool     measuringHigh_  = true;
    uint32_t capCount_       = 0;   // 0x80 cap bytes (= 128 ticks each) this window
    uint32_t partial_        = 0;   // sub-128 bytes (level transitions); ignored, for sanity
    uint32_t windowStartUs_  = 0;   // micros() at phase entry
    uint32_t phaseUntilMs_   = 0;   // when to report + flip level
};
