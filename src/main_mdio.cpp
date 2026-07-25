// main_mdio.cpp - ACTIVE MDIO MASTER use case (the driver).
//
// Envs (see platformio.ini):
//   - RUN_MDIO_MASTER            : drives a real PHY over the PIOC dongle (Clause 22 + MMD)
//   - RUN_MDIO_MASTER + MDIO_STUB: protocol-only build, ASCII layer over USB, no PIOC
//
// The board layer (USB-CDC, LED, timebase, printf routing) lives in common/board.*;
// this file only owns the MDIO master and its USB ASCII command bridge. The whole
// file is gated so the other use cases' mains can coexist in src/.
#ifdef RUN_MDIO_MASTER

#include "board.hpp"
#include "ch32_sdk.hpp"
#include "time.hpp"

#include "mdio_bridge.hpp"
#include "mdio_master.hpp"

static Mdio::Master    g_mdio;  // active MDIO bus owner; USB command layer below
static Mdio::UsbBridge g_mdioUsb(Board::usb, g_mdio, &Board::led);
// No idle heartbeat here: the LED is the bridge's activity signal (per transaction).

int main(void)
{
    Board::init();

    g_mdio.begin();
#ifdef MDIO_STUB
    printf("# mdio-master ready (clause22+mmd, STUB - no PIOC)\r\n"); // '#' banner: host ignores
#else
    printf("# mdio-master ready (clause22+mmd)\r\n");                 // '#' banner: host ignores
#endif

    while (1)
    {
        uint32_t now = Time::millis();
        Board::usb.tick(now);

        g_mdio.tick(now);      // progress the active MDIO request, if any
        g_mdioUsb.tick();      // USB ASCII command layer -> submit/respond

        Board::led.tick();
    }
}

#endif // RUN_MDIO_MASTER
