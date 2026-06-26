/* mode_cmd_harness.cpp - drive src/mode_command.hpp from the CLI for mode_command_test.js.
 *
 *   ./mode_cmd_harness parse "<line>"   -> "<valid> <mode|->"
 *
 * The device knows MODES only (rle/clocked); protocol/codec is a host concern and
 * never reaches mode_command.hpp, so parse() returns just {valid, mode}.
 */
#include <stdio.h>
#include <string.h>
#include "../../src/sniffer/mode_command.hpp"

int main(int argc, char **argv) {
    if (argc >= 3 && strcmp(argv[1], "parse") == 0) {
        ModeCmd::Command r = ModeCmd::parse(argv[2], strlen(argv[2]));
        const char *mode = r.mode == ModeCmd::CaptureMode::Clocked ? "clocked" : "rle";
        printf("%d %s\n", r.valid ? 1 : 0, r.valid ? mode : "-");
        return 0;
    }
    fprintf(stderr, "usage: %s parse <line>\n", argv[0]);
    return 2;
}
