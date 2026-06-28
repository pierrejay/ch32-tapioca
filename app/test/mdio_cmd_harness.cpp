/* mdio_cmd_harness.cpp - drive src/mdio/mdio_command.hpp from the CLI for mdio_command_test.js.
 *
 *   ./mdio_cmd_harness parse "<line>"   -> "0 -"                  (invalid)
 *                                       -> "1 <op> <phy> <reg|-> <val|->"   (valid)
 *
 * phy/reg are decimal; val (write only) is decimal; '-' marks a field the op doesn't carry.
 */
#include <stdio.h>
#include <string.h>
#include "../../src/mdio/mdio_command.hpp"

int main(int argc, char **argv) {
    if (argc >= 3 && strcmp(argv[1], "parse") == 0) {
        MdioCmd::Command r = MdioCmd::parse(argv[2], strlen(argv[2]));
        if (!r.valid) { printf("0 -\n"); return 0; }
        const char *op = r.op == MdioCmd::Op::Read  ? "read"
                       : r.op == MdioCmd::Op::Write ? "write" : "print";
        char reg[8] = "-", val[8] = "-";
        if (r.op != MdioCmd::Op::Print) snprintf(reg, sizeof reg, "%u", r.reg);
        if (r.op == MdioCmd::Op::Write) snprintf(val, sizeof val, "%u", r.val);
        printf("1 %s %u %s %s\n", op, r.phy, reg, val);
        return 0;
    }
    fprintf(stderr, "usage: %s parse <line>\n", argv[0]);
    return 2;
}
