// main_tick_test.cpp - BENCH use case: measure the rle blob's per-level tick period
// in exact integer CPU cycles (see rle_tick_test.hpp for the model and the wiring).
//
// Env (see platformio.ini): RUN_RLE_TICK_TEST.
//
// The board layer (USB-CDC, LED, timebase, printf routing) lives in common/board.*.
// The whole file is gated so the other use cases' mains can coexist in src/.
#ifdef RUN_RLE_TICK_TEST

#include "board.hpp"
#include "ch32_sdk.hpp"
#include "time.hpp"

#include "rle_sniffer.hpp"
#include "rle_tick_test.hpp"

static constexpr uint32_t LED_FLASH_MS  = 50;
static constexpr uint32_t LED_PERIOD_MS = 1000;   // idle heartbeat

static RleSniffer g_rle(Board::usb);
static RleTickTest g_tick(g_rle);  // bench: measure the blob's per-level tick cycles

int main(void)
{
    Board::init();

    g_tick.begin(Time::millis());  // constant level on PA7 + real rle blob (TIM3 drain)

    uint32_t nextLedFlashMs = Time::millis();

    while (1)
    {
        uint32_t now = Time::millis();
        Board::usb.tick(now);

        g_tick.service(now);   // count caps on the held level, flip + report per phase

        if ((int32_t)(now - nextLedFlashMs) >= 0)
        {
            Board::led.blink(LED_FLASH_MS);
            nextLedFlashMs = now + LED_PERIOD_MS;
        }

        Board::led.tick();
    }
}

#endif // RUN_RLE_TICK_TEST
