// main.cpp - CH32X035 firmware base (generic rle / clocked sniffer).
//
// Keeps the USB-CDC device (UsbCdc) alive as the byte-stream link, plus an LED
// heartbeat. The old USART2<->USB UartBridge was removed, freeing USART2 for the
// debug console (-D DEBUG=2 -> PA2). Runtime mode is picked at build time:
//   - RUN_SNIFFER : both datapaths + runtime !mode switch (the product)
//   - RUN_CLOCKED_SNIFFER    : clocked datapath standalone
//   - RUN_RLE_SNIFFER     : rle datapath standalone
#include "ch32_sdk.hpp"
#include "led_blinker.hpp"
#include "time.hpp"
#include "usb_cdc.hpp"

#if defined(RUN_CLOCKED_SNIFFER) || defined(RUN_SNIFFER)
#include "clocked_sniffer.hpp"     // ClockedSniffer
#endif

#ifdef RUN_SNIFFER
#include "rle_sniffer.hpp"         // RleSniffer too: both datapaths, runtime mode_ select
#include "mode_command.hpp"        // ModeCmd::parse / CaptureMode
#include "record_framer.hpp"       // RecordFramer::frameMeta / frameLoss for the switch seam
#endif

#ifdef RUN_RLE_SNIFFER
#include "rle_sniffer.hpp"         // RleSniffer
#endif

#ifdef RUN_RLE_TICK_TEST
#include "rle_sniffer.hpp"         // real rle blob + TIM3 drain, tapped raw by the bench
#include "rle_tick_test.hpp"       // RleTickTest: constant-level cap timing -> integer cyc/tick
#endif

// Heartbeat LED: PB12. Clear of USB PC16/PC17, debug PA2 (DEBUG=2) and PIOC PC18/PC19.
#ifndef LED_GPIO_PORT
#define LED_GPIO_PORT GPIOB
#endif
#ifndef LED_GPIO_PIN
#define LED_GPIO_PIN GPIO_Pin_12
#endif
#ifndef LED_GPIO_RCC
#define LED_GPIO_RCC RCC_APB2Periph_GPIOB
#endif
#ifndef LED_ACTIVE_HIGH
#define LED_ACTIVE_HIGH 1
#endif

static constexpr uint32_t LED_FLASH_MS = 50;
static constexpr uint32_t LED_PERIOD_MS = 1000;

// File-scope so they live in .bss with static storage duration, but are
// constructed trivially - all hardware init happens in init(), called from
// main(), so there is no global-constructor ordering hazard. 
static UsbCdc     g_usb;
static LedBlinker g_led(LED_GPIO_PORT,
                        LED_GPIO_PIN,
                        LED_GPIO_RCC,
                        LED_ACTIVE_HIGH != 0);

#if defined(RUN_CLOCKED_SNIFFER) || defined(RUN_SNIFFER)
static ClockedSniffer g_clocked(g_usb);   // drains the PIOC FIFO to USB each loop
#endif

#if defined(RUN_RLE_SNIFFER) || defined(RUN_SNIFFER) || defined(RUN_RLE_TICK_TEST)
static RleSniffer  g_rle(g_usb);       // continuous rle passive tap -> USB
#endif

#ifdef RUN_RLE_TICK_TEST
static RleTickTest g_tick(g_rle);      // bench: measure the blob's per-level tick cycles
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
// only writes the transport seam; the rest is the datapath's job. This keeps main.cpp
// thin and the two modes symmetric. 
static char     g_cmdLine[24];                        // RX line accumulator ("!mode clocked" + CRLF)
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
        uint32_t n = g_usb.write(seam + off, (uint32_t)(sizeof(seam) - off));
        off += n;
        g_usb.tick(Time::millis());                   // drain toward the host between attempts
        if (n == 0) ++guard; else guard = 0;
    }
    g_mode = m;
    if (m == ModeCmd::CaptureMode::Rle) g_rle.begin(Time::millis());
    else                                  g_clocked.begin(Time::millis());
}

// Drain RX, accumulate a line, act on a complete `!mode ...` (cross-mode change).
static void unifiedPollCommand()
{
    uint8_t b = 0;
    while (g_usb.available() && g_usb.read(&b, 1) == 1) {
        if (b == '\n' || b == '\r') {
            if (g_cmdLen) {
                ModeCmd::Command c = ModeCmd::parse(g_cmdLine, g_cmdLen);
                if (c.valid && c.mode != g_mode) unifiedSwitchTo(c.mode);
                g_cmdLen = 0;
            }
        } else if (g_cmdLen < sizeof(g_cmdLine)) {
            g_cmdLine[g_cmdLen++] = (char)b;
        } else {
            g_cmdLen = 0;                                   // over-long line -> drop, resync on next newline
        }
    }
}
#endif // RUN_SNIFFER

// Route printf (newlib _write) over USB-CDC instead of the UART. The linker flag
// -Wl,--wrap=_write redirects all printf output here; pump g_usb so it drains to
// the host. Bounded by `guard` so it can't hang if the host isn't reading. 
extern "C" int __wrap__write(int fd, char *buf, int size)
{
    (void)fd;
    int      sent  = 0;
    uint32_t guard = 0;
    while (sent < size && guard < 200000)
    {
        uint32_t n = g_usb.write((const uint8_t *)buf + sent, (uint32_t)(size - sent));
        sent += (int)n;
        g_usb.tick(Time::millis());            // drain toward the host
        if (n == 0) ++guard; else guard = 0;
    }
    return size;
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    Time::init();        // TIM2-backed monotonic millis()/micros() for app timing
    g_usb.init();        // USB-CDC; printf is routed here via __wrap__write
    g_led.init();

    // Wait for the host to enumerate and open the CDC port before logging, so the
    // boot output isn't lost. Pump USB while waiting. 
    {
        uint32_t t0 = Time::millis();
        while (Time::millis() - t0 < 4000) g_usb.tick(Time::millis());
    }

    printf("SystemClk:%u\r\n", (unsigned)SystemCoreClock);
    printf("ChipID:%08x\r\n", (unsigned)DBGMCU_GetCHIPID());

#ifdef RUN_CLOCKED_SNIFFER
    g_clocked.begin(Time::millis());
#endif

#ifdef RUN_RLE_SNIFFER
    g_rle.begin(Time::millis());
#endif

#ifdef RUN_RLE_TICK_TEST
    g_tick.begin(Time::millis());         // constant level on PA7 + real rle blob (TIM3 drain)
#endif

#ifdef RUN_SNIFFER
    // Bring up ONLY the active datapath: begin() loads that mode's PIOC blob (the two
    // blobs are mutually exclusive on the shared eMCU) and, for rle, arms the TIM3
    // drain (g_drainTarget = &g_rle). The inactive instance's buffers just sit in .bss. 
    if (g_mode == ModeCmd::CaptureMode::Rle) g_rle.begin(Time::millis());
    else                                       g_clocked.begin(Time::millis());
#endif

    uint32_t nextLedFlashMs = Time::millis();

    while (1)
    {
        uint32_t now = Time::millis();
        g_usb.tick(now);
#if defined(DIAG) && (defined(RUN_RLE_SNIFFER) || defined(RUN_SNIFFER))
        // service()-time sample for the [T] line (recordTiming's sole reader). Gated so
        // production pays neither the micros() read nor the accumulation. Integrity diags
        // (0xFD loss, record OVF flags) are NOT here - always-on, in-band, per datapath. 
        uint32_t _tsvc = Time::micros();
#endif

#ifdef RUN_CLOCKED_SNIFFER
        g_clocked.service(now);               // drain ring -> USB; never blocks long
#endif

#ifdef RUN_RLE_SNIFFER
        g_rle.service(now);                   // drain PIOC ring -> USB; never blocks
#ifdef DIAG
        g_rle.recordTiming(Time::micros() - _tsvc);  // us in service()
#endif
#endif

#ifdef RUN_RLE_TICK_TEST
        g_tick.service(now);                  // count caps on the held level, flip + report per phase
#endif

#ifdef RUN_SNIFFER
        // route to the active datapath only - never service both at once.
        if (g_mode == ModeCmd::CaptureMode::Rle) {
            g_rle.service(now);
#ifdef DIAG
            g_rle.recordTiming(Time::micros() - _tsvc);
#endif
        } else {
            g_clocked.service(now);           // clocked: drain ring -> USB (now pump-driven)
        }
        unifiedPollCommand();                 // host !mode -> stop/reconfig/start, out of capture
#endif

        if ((int32_t)(now - nextLedFlashMs) >= 0)
        {
            g_led.blink(LED_FLASH_MS);
            nextLedFlashMs = now + LED_PERIOD_MS;
#if !defined(RUN_CLOCKED_SNIFFER) && !defined(RUN_RLE_SNIFFER) && !defined(RUN_SNIFFER) && !defined(RUN_RLE_TICK_TEST)
            printf("Blink.\r\n");             // heartbeat text would pollute the binary capture stream
#endif
        }

        g_led.tick();
    }
}
