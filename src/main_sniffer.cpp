// main_sniffer.cpp - PASSIVE SNIFFER use case (the product).
//
// Envs (see platformio.ini):
//   - RUN_SNIFFER         : both datapaths + the runtime `!mode` switch
//   - RUN_RLE_SNIFFER     : rle datapath standalone
//   - RUN_CLOCKED_SNIFFER : clocked datapath standalone
//
// The board layer (USB-CDC, LED, timebase, printf routing) lives in common/board.*;
// this file only owns the capture datapaths and, for RUN_SNIFFER, the mode control
// plane. The whole file is gated so the other use cases' mains can coexist in src/.
#if defined(RUN_SNIFFER) || defined(RUN_RLE_SNIFFER) || defined(RUN_CLOCKED_SNIFFER)

#include "board.hpp"
#include "ch32_sdk.hpp"
#include "time.hpp"

#if defined(RUN_CLOCKED_SNIFFER) || defined(RUN_SNIFFER)
#include "clocked_sniffer.hpp"
#endif

#if defined(RUN_RLE_SNIFFER) || defined(RUN_SNIFFER)
#include "rle_sniffer.hpp"
#endif

#ifdef RUN_SNIFFER
#include "mode_command.hpp"
#endif

static constexpr uint32_t LED_FLASH_MS  = 50;
static constexpr uint32_t LED_PERIOD_MS = 1000;   // idle heartbeat

#if defined(RUN_CLOCKED_SNIFFER) || defined(RUN_SNIFFER)
static ClockedSniffer g_clocked(Board::usb);
#endif

#if defined(RUN_RLE_SNIFFER) || defined(RUN_SNIFFER)
static RleSniffer g_rle(Board::usb);
#endif

#ifdef RUN_SNIFFER
// Runtime capture mode for the dual-instance selector. ONE datapath is "active"
// (has run begin()) at a time; main() routes service() to it. Default = Clocked;
// the !mode RX command switches it at runtime (stop -> reconfig -> start, below).
static ModeCmd::CaptureMode g_mode = ModeCmd::CaptureMode::Clocked;

// ---- runtime mode-switch control plane --------------------------------------
// The host drives the device mode over the CDC RX line with `!mode <rle|clocked>\n`
// (cross-mode only). We accumulate a line, parse it (ModeCmd::parse, strict), and on a
// real change run  stop(active) -> hard seam (0xFF 0xFD 0xFF) -> begin(other).
//
// The mode ANNOUNCEMENT ([MODE …] block) is deliberately NOT emitted here. Each
// datapath advertises its own mode from INSIDE its service loop, at its own safe
// boundary - ClockedSniffer between records (via its ring), RleSniffer at the idle gap -
// so a periodic heartbeat can never tear an in-flight record/frame. The control plane
// only writes the transport seam; the rest is the datapath's job. This keeps main thin
// and the two modes symmetric.
static char     g_cmdLine[24];  // RX line accumulator ("!mode clocked" + CRLF)
static uint16_t g_cmdLen = 0;

// stop active -> hard seam -> start other. Runs OUT of capture (between service()
// calls), never mid-drain. The new datapath announces its own [MODE …] identity.
static void unifiedSwitchTo(ModeCmd::CaptureMode m)
{
    if (g_mode == ModeCmd::CaptureMode::Rle) g_rle.stop(); else g_clocked.stop();
    // hard seam: leading 0xFF closes any partial segment still in the TX ring (so the
    // 0xFD lands at a segment boundary, structurally), 0xFD = explicit discontinuity, trailing
    // 0xFF = resync point for the new datapath. We run OUT of capture here, so a short bounded
    // pump ships all 3 bytes whole rather than tearing the seam if TX momentarily lacks room.
    static const uint8_t seam[3] = { 0xFF, 0xFD, 0xFF };
    uint32_t off = 0, guard = 0;
    while (off < sizeof(seam) && guard < 100000) {
        uint32_t n = Board::usb.write(seam + off, (uint32_t)(sizeof(seam) - off));
        off += n;
        Board::usb.tick(Time::millis()); // drain toward the host between attempts
        if (n == 0) ++guard; else guard = 0;
    }
    g_mode = m;
    if (m == ModeCmd::CaptureMode::Rle) g_rle.begin(Time::millis());
    else                                g_clocked.begin(Time::millis());
}

// Drain RX, accumulate a line, act on a complete `!mode ...` (cross-mode change).
static void unifiedPollCommand()
{
    uint8_t b = 0;
    while (Board::usb.available() && Board::usb.read(&b, 1) == 1) {
        if (b == '\n' || b == '\r') {
            if (g_cmdLen) {
                ModeCmd::Command c = ModeCmd::parse(g_cmdLine, g_cmdLen);
                if (c.valid && c.mode != g_mode) unifiedSwitchTo(c.mode);
                g_cmdLen = 0;
            }
        } else if (g_cmdLen < sizeof(g_cmdLine)) {
            g_cmdLine[g_cmdLen++] = (char)b;
        } else {
            g_cmdLen = 0; // over-long line -> drop, resync on next newline
        }
    }
}
#endif // RUN_SNIFFER

int main(void)
{
    Board::init();

#ifdef RUN_CLOCKED_SNIFFER
    g_clocked.begin(Time::millis());
#endif

#ifdef RUN_RLE_SNIFFER
    g_rle.begin(Time::millis());
#endif

#ifdef RUN_SNIFFER
    // Bring up ONLY the active datapath: begin() loads that mode's PIOC blob (the two
    // blobs are mutually exclusive on the shared eMCU) and, for rle, arms the TIM3
    // drain (g_drainTarget = &g_rle). The inactive instance's buffers just sit in .bss.
    if (g_mode == ModeCmd::CaptureMode::Rle) g_rle.begin(Time::millis());
    else                                     g_clocked.begin(Time::millis());
#endif

    uint32_t nextLedFlashMs = Time::millis();

    while (1)
    {
        uint32_t now = Time::millis();
        Board::usb.tick(now);
#if defined(DIAG) && (defined(RUN_RLE_SNIFFER) || defined(RUN_SNIFFER))
        // service()-time sample for the [T] line (recordTiming's sole reader). Gated so
        // production pays neither the micros() read nor the accumulation. Integrity diags
        // (0xFD loss, record OVF flags) are NOT here - always-on, in-band, per datapath.
        uint32_t _tsvc = Time::micros();
#endif

#ifdef RUN_CLOCKED_SNIFFER
        g_clocked.service(now); // drain ring -> USB; never blocks long
#endif

#ifdef RUN_RLE_SNIFFER
        g_rle.service(now);     // drain PIOC ring -> USB; never blocks
#ifdef DIAG
        g_rle.recordTiming(Time::micros() - _tsvc);  // us in service()
#endif
#endif

#ifdef RUN_SNIFFER
        if (g_mode == ModeCmd::CaptureMode::Rle) {
            g_rle.service(now);
#ifdef DIAG
            g_rle.recordTiming(Time::micros() - _tsvc);
#endif
        } else {
            g_clocked.service(now); // clocked: drain ring -> USB
        }
        unifiedPollCommand();       // host !mode -> stop/reconfig/start, out of capture
#endif

        if ((int32_t)(now - nextLedFlashMs) >= 0)
        {
            Board::led.blink(LED_FLASH_MS);
            nextLedFlashMs = now + LED_PERIOD_MS;
        }

        Board::led.tick();
    }
}

#endif // sniffer use case
