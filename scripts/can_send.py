#!/usr/bin/env python3
"""Send simple Classic CAN frames through python-can.

Useful for exercising Tapioca's CAN/RLE capture path with a known transmitter.

Examples:
    python3 scripts/can_send.py --channel /dev/ttyACM0 --bitrate 500000 \
        --id 0x123 DE AD BE EF 01 02 03 04

    python3 scripts/can_send.py --channel /dev/ttyACM0 --bitrate 500000 \
        --id 0x123 --count 100 --period-ms 10 --counter

    python3 scripts/can_send.py --interface socketcan --channel can0 \
        --id 0x123 01020304
"""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Sequence


DEFAULT_PAYLOAD = bytes.fromhex("DE AD BE EF 01 02 03 04")


def _parse_can_id(text: str) -> int:
    value = int(text, 0)
    if value < 0:
        raise argparse.ArgumentTypeError("CAN id must be positive")
    return value


def _parse_payload(parts: Sequence[str]) -> bytes:
    """Parse hex bytes from either spaced tokens or a compact hex string.

    Accepted forms:
      DE AD BE EF
      0xDE 0xAD 0xBE 0xEF
      DE:AD:BE:EF
      deadbeef
    """
    if not parts:
        return DEFAULT_PAYLOAD

    text = " ".join(parts)
    for sep in ",:-_":
        text = text.replace(sep, " ")

    out: list[int] = []
    for token in text.split():
        token = token.strip()
        if not token:
            continue
        if token.lower().startswith("0x"):
            token = token[2:]

        if len(token) > 2:
            if len(token) % 2:
                raise argparse.ArgumentTypeError(f"odd-length hex payload token: {token!r}")
            out.extend(int(token[i : i + 2], 16) for i in range(0, len(token), 2))
        else:
            value = int(token, 16)
            if value > 0xFF:
                raise argparse.ArgumentTypeError(f"payload byte out of range: {token!r}")
            out.append(value)

    if len(out) > 8:
        raise argparse.ArgumentTypeError("Classic CAN payload is limited to 8 bytes")
    return bytes(out)


def _open_bus(interface: str, channel: str, bitrate: int | None):
    try:
        import can
    except ImportError as exc:
        raise SystemExit(
            "python-can is required: install it with `python3 -m pip install python-can pyserial`"
        ) from exc

    kwargs: dict[str, object] = {"interface": interface, "channel": channel}
    if bitrate is not None:
        kwargs["bitrate"] = bitrate
    return can.Bus(**kwargs)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send Classic CAN frames through a python-can interface, defaulting to slcan.",
    )
    parser.add_argument(
        "data",
        nargs="*",
        help="payload as hex bytes or compact hex string; default: DE AD BE EF 01 02 03 04",
    )
    parser.add_argument(
        "--interface",
        default="slcan",
        help="python-can interface/backend, e.g. slcan or socketcan (default: slcan)",
    )
    parser.add_argument(
        "--channel",
        default="/dev/ttyACM0",
        help="serial dongle path for slcan, or can0-style name for socketcan (default: /dev/ttyACM0)",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=None,
        help="bus bitrate in bit/s; slcan defaults to 500000 when omitted",
    )
    parser.add_argument(
        "--id",
        dest="arbitration_id",
        type=_parse_can_id,
        default=0x123,
        help="arbitration id, decimal or hex (default: 0x123)",
    )
    parser.add_argument(
        "--extended",
        action="store_true",
        help="send a 29-bit extended-id frame instead of an 11-bit standard frame",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="number of frames to send; 0 means forever (default: 1)",
    )
    parser.add_argument(
        "--period-ms",
        type=float,
        default=0.0,
        help="delay between frames in milliseconds (default: 0)",
    )
    parser.add_argument(
        "--counter",
        action="store_true",
        help="increment the last payload byte on each frame, useful for spotting fresh traffic",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="python-can send timeout in seconds (default: 1.0)",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.count < 0:
        parser.error("--count must be >= 0")
    if args.period_ms < 0:
        parser.error("--period-ms must be >= 0")

    if not args.extended and args.arbitration_id > 0x7FF:
        parser.error("standard CAN id must fit in 11 bits; use --extended for 29-bit ids")
    if args.extended and args.arbitration_id > 0x1FFFFFFF:
        parser.error("extended CAN id must fit in 29 bits")

    bitrate = args.bitrate
    if bitrate is None and args.interface == "slcan":
        bitrate = 500_000

    try:
        payload = bytearray(_parse_payload(args.data))
    except (ValueError, argparse.ArgumentTypeError) as exc:
        parser.error(str(exc))

    if args.counter and not payload:
        payload.append(0)

    bus = _open_bus(args.interface, args.channel, bitrate)

    try:
        import can

        sent = 0
        period_s = args.period_ms / 1000.0

        while args.count == 0 or sent < args.count:
            if args.counter:
                payload[-1] = sent & 0xFF

            msg = can.Message(
                arbitration_id=args.arbitration_id,
                data=bytes(payload),
                is_extended_id=args.extended,
            )
            bus.send(msg, timeout=args.timeout)
            sent += 1
            print(
                f"sent {sent}: id=0x{args.arbitration_id:X} "
                f"{'ext' if args.extended else 'std'} data={bytes(payload).hex(' ').upper()}",
                flush=True,
            )

            if period_s > 0 and (args.count == 0 or sent < args.count):
                time.sleep(period_s)
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 130
    finally:
        bus.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
