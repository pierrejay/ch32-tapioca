// board.hpp - the board layer shared by every firmware use case.
//
// Everything that is NOT use-case logic lives here: the USB-CDC device, the LED,
// the clock/timebase bring-up and the printf-over-CDC plumbing. Each use case
// then gets its own thin `main`:
//
//     src/main_sniffer.cpp    - passive bus sniffer   (RUN_SNIFFER / _RLE_ / _CLOCKED_)
//     src/main_mdio.cpp       - active MDIO master    (RUN_MDIO_MASTER)
//     src/main_tick_test.cpp  - rle tick bench        (RUN_RLE_TICK_TEST)
//
// Those three files are each wrapped in the build flag of their env, so exactly
// one of them defines main() in a given build and the others compile to nothing.
// No build_src_filter needed: the whole of src/ is always compiled.
#pragma once

#include <stdint.h>

#include "led_blinker.hpp"
#include "usb_cdc.hpp"

namespace Board {

// Single instances, owned here. `usb` is also the sink of printf (see
// __wrap__write in board.cpp), so it must outlive everything - hence globals.
extern UsbCdc     usb;
extern LedBlinker led;

// NVIC/clocks/Delay, Time (TIM2 millis/micros), usb.init(), led.init(), then wait
// ~2 s for the host to enumerate and open the port (pumping USB) so the boot
// banner isn't lost. Prints SystemClk/ChipID. Call it first from main().
void init();

} // namespace Board
