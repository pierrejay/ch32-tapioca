# scripts/

Python **test / validation** scripts for the sniffer — and a starting point if you want to
interface with the device from your own code. No third-party deps.

The **browser app** (`../app/index.html`) is the primary, up-to-date decoder — these scripts are for CLI/CI/scripting, not the main UI.

| file | what |
|---|---|
| `can_rle_decode.py` / `dmx_rle_decode.py` | decode a CAN / DMX capture file → frames. `--selftest` runs without hardware. `can_rle_decode.py --live <dev>` also has an experimental realtime stdout mode. |
| `mdio_decode.py` / `mdio_lib.py` | MDIO decoder (Clause-22 + MMD fold) for the current wire=2 envelope. `mdio_lib.py` is also the parity oracle for the JS codec test. |
| `diag_monitor.py` | tail the device's `[MODE]` + `[T]` telemetry blocks (`-D DIAG`). Works in both modes — the `[T]` fields differ (RLE vs clocked); see the script header. |
| `raw_throughput.py` | measure the USB stream's raw read throughput : answers "is the host reader the cap, or the device?" |

Most decoders are **batch** (decode a capture FILE); to grab live data, `timeout 5 cat
/dev/cu.usbmodemXXXX > cap.bin` then decode the file. CAN also has a realtime `--live` mode.

The current codec (COBS-0xFF, record framing, `!mode` parsing) is tested host-side,
no hardware, in `../app/test/` — run `bash ../app/test/run_all.sh`.
