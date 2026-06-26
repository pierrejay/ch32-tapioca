// interrupts.cpp - core fault handlers.
//
// Peripheral IRQ handlers live next to the code they drive:
//   - USBFS in usb_cdc.cpp (polled path, full handler fallback)
//   - TIM2 timebase in time.cpp
//   - TIM3 RLE drain in rle_sniffer.cpp (hot path in RAM)
#include "ch32_sdk.hpp"

extern "C" void NMI_Handler(void)       __attribute__((interrupt("WCH-Interrupt-fast")));
extern "C" void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

extern "C" void NMI_Handler(void)
{
    while (1) { }
}

extern "C" void HardFault_Handler(void)
{
    NVIC_SystemReset();
    while (1) { }
}
