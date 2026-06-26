// rle_tick_test.cpp - see rle_tick_test.hpp. Built only under -D RUN_RLE_TICK_TEST;
// otherwise this translation unit is empty (compiled in every env, like the blobs).
#ifdef RUN_RLE_TICK_TEST

#include "rle_tick_test.hpp"
#include "ch32_sdk.hpp"          // GPIO + RCC + SystemCoreClock
#include "time.hpp"              // Time::micros() - the 48 MHz-derived timebase

extern "C" {
#include <stdio.h>
}

static constexpr uint32_t CAP_TICKS = 128;      // the blob caps at CNT bit 7 -> 0x80 per cap
static constexpr uint32_t PHASE_MS  = 2000;     // measure each held level for 2 s

// Drive PA7 (push-pull) to a constant level -> jumper -> PC19 (the blob's data line).
static void driveLevel(bool high)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_7, high ? Bit_SET : Bit_RESET);
}

void RleTickTest::begin(uint32_t nowMs)
{
    rle_.begin(nowMs);                          // real rle blob + TIM3 drain into the RAM ring

    // PA7 as a plain output: the held level we feed into the capture pin. (SpiGen used
    // PA5/PA7 as SPI AF; here we just need a steady DC level, so plain GPIO is enough.) 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin   = GPIO_Pin_7;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &g);

    printf("[RLE-TICK] start: jumper PA7 -> PC19. Constant-level cap timing, "
           "Fcpu=%u Hz, cap=%u ticks, %u ms/level.\r\n",
           (unsigned)SystemCoreClock, (unsigned)CAP_TICKS, (unsigned)PHASE_MS);

    enterPhase(true);                           // start measuring HIGH
    phaseUntilMs_ = nowMs + PHASE_MS;
}

void RleTickTest::enterPhase(bool high)
{
    measuringHigh_ = high;
    driveLevel(high);
    capCount_ = 0;
    partial_  = 0;
    windowStartUs_ = Time::micros();
}

void RleTickTest::service(uint32_t nowMs)
{
    uint8_t buf[64];
    uint16_t n;
    while ((n = rle_.testDrainRaw(buf, sizeof buf)) > 0) {
        for (uint16_t i = 0; i < n; i++) {
            if (buf[i] >= 128) capCount_++;     // 0x80 cap = 128 ticks of the held level
            else               partial_++;      // the lone transition run at a level flip
        }
        if (n < sizeof buf) break;
    }

    if ((int32_t)(nowMs - phaseUntilMs_) >= 0) {
        report();
        enterPhase(!measuringHigh_);            // flip HIGH <-> LOW
        phaseUntilMs_ = nowMs + PHASE_MS;
    }
}

void RleTickTest::report()
{
    uint32_t elapsedUs = Time::micros() - windowStartUs_;
    const char* lvl = measuringHigh_ ? "HIGH" : "LOW ";

    if (capCount_ < 100 || elapsedUs < 1000) {
        printf("[RLE-TICK] %s warming up: caps=%lu (check the PA7->PC19 jumper)\r\n",
               lvl, (unsigned long)capCount_);
        return;
    }

    // c = window_cycles / (128 * caps).  window_cycles = elapsedUs * (Fcpu / 1e6).
    uint32_t cycPerUs = SystemCoreClock / 1000000u;             // 48
    uint64_t cycles   = (uint64_t)elapsedUs * cycPerUs;
    uint32_t c_x1000  = (uint32_t)((cycles * 1000u) / ((uint64_t)CAP_TICKS * capCount_));
    uint32_t c_int    = (c_x1000 + 500u) / 1000u;               // nearest integer
    uint32_t resid    = c_x1000 - c_int * 1000u;                // ~+0.06: the cap-store overhead

    printf("[RLE-TICK] %s c=%lu.%03lu cyc/tick  => %lu  (caps=%lu, %lu ms, residual=%lu.%03lu, partial=%lu)\r\n",
           lvl, (unsigned long)(c_x1000 / 1000), (unsigned long)(c_x1000 % 1000),
           (unsigned long)c_int, (unsigned long)capCount_, (unsigned long)(elapsedUs / 1000),
           (unsigned long)(resid / 1000), (unsigned long)(resid % 1000), (unsigned long)partial_);
}

#endif // RUN_RLE_TICK_TEST
