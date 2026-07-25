// board.cpp - see board.hpp. Built into every env (all use cases need it).
#include "board.hpp"

#include "ch32_sdk.hpp"
#include "time.hpp"

// Heartbeat / activity LED. Overridable from build_flags for another board rev.
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

namespace Board {

UsbCdc     usb;
LedBlinker led(LED_GPIO_PORT,
               LED_GPIO_PIN,
               LED_GPIO_RCC,
               LED_ACTIVE_HIGH != 0);

void init()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    Time::init();        // TIM2-backed monotonic millis()/micros() for app timing
    usb.init();          // USB-CDC; printf is routed here via __wrap__write
    led.init();

    // Wait for the host to enumerate and open the CDC port before logging, so the
    // boot output isn't lost. Pump USB while waiting.
    uint32_t t0 = Time::millis();
    while (Time::millis() - t0 < 2000) usb.tick(Time::millis());

    printf("SystemClk:%u\r\n", (unsigned)SystemCoreClock);
    printf("ChipID:%08x\r\n", (unsigned)DBGMCU_GetCHIPID());
}

} // namespace Board

// Route printf (newlib _write) over USB-CDC instead of the UART. The linker flag
// -Wl,--wrap=_write redirects all printf output here; pump the CDC so it drains to
// the host. Bounded by `guard` so it can't hang if the host isn't reading.
extern "C" int __wrap__write(int fd, char *buf, int size)
{
    (void)fd;
    int      sent  = 0;
    uint32_t guard = 0;
    while (sent < size && guard < 200000)
    {
        uint32_t n = Board::usb.write((const uint8_t *)buf + sent, (uint32_t)(size - sent));
        sent += (int)n;
        Board::usb.tick(Time::millis());       // drain toward the host
        if (n == 0) ++guard; else guard = 0;
    }
    return size;
}
