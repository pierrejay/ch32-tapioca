/* mdio_cmd_harness.cpp - drive src/mdio/mdio_bridge.hpp from the CLI for mdio_command_test.js.
 *
 *   ./mdio_cmd_harness parse "<line>"   -> "0 -"                  (invalid)
 *                                       -> "1 <op> <phy[:mmd]> <reg|-> <val|->" (valid)
 *   ./mdio_cmd_harness frame "<bytes>"  -> one "ready:<line>" or "invalid" event per line
 *
 * phy/mmd/reg are decimal; val (write only) is decimal; '-' marks a field the op doesn't carry.
 */
#include <stdio.h>
#include <string.h>
#define private public
#include "../../src/mdio/mdio_bridge.hpp"
#undef private

class UsbCdc {};

int main(int argc, char **argv) {
    if (argc >= 3 && strcmp(argv[1], "parse") == 0) {
        Mdio::UsbBridge::Command r = Mdio::UsbBridge::parse(argv[2], strlen(argv[2]));
        if (!r.valid) { printf("0 -\n"); return 0; }
        const char *op = r.op == Mdio::UsbBridge::Op::Read  ? "read"
                       : r.op == Mdio::UsbBridge::Op::Write ? "write" : "print";
        char path[16];
        char reg[8] = "-", val[8] = "-";
        if (r.mmdAccess) snprintf(path, sizeof path, "%u:%u", r.phy, r.mmd);
        else snprintf(path, sizeof path, "%u", r.phy);
        if (r.op != Mdio::UsbBridge::Op::Print) snprintf(reg, sizeof reg, "%u", r.reg);
        if (r.op == Mdio::UsbBridge::Op::Write) snprintf(val, sizeof val, "%u", r.val);
        printf("1 %s %s %s %s\n", op, path, reg, val);
        return 0;
    }
    if (argc >= 3 && strcmp(argv[1], "frame") == 0) {
        UsbCdc usb;
        Mdio::Master mdio;
        Mdio::UsbBridge bridge(usb, mdio);
        const char *p = argv[2];
        while (*p) {
            Mdio::UsbBridge::LineEvent event = bridge.pushCommandByte((uint8_t)*p++);
            if (event == Mdio::UsbBridge::LineEvent::Ready) {
                printf("ready:%.*s\n", (int)bridge.lineLen_, bridge.line_);
                bridge.lineLen_ = 0;
            } else if (event == Mdio::UsbBridge::LineEvent::Invalid) {
                printf("invalid\n");
            }
        }
        return 0;
    }
    fprintf(stderr, "usage: %s <parse|frame> <input>\n", argv[0]);
    return 2;
}
