#!/usr/bin/env python3
"""Read (or set) the RoboClaw serial timeout.

The RoboClaw stops its motors on its own if no serial command arrives within
this timeout. 0 disables it, which means a killed ROS node leaves the last
duty cycle latched and the rover keeps driving. Resolution is 0.1 s, max 25.5 s.

Stop any running launch first — the port can only be open once.

  python3 scripts/serial_timeout.py             # read
  python3 scripts/serial_timeout.py --set 0.2   # set 200 ms
"""
import argparse
import sys

from basicmicro import Basicmicro

PORT = "/dev/gerbil_roboclaw"
BAUD = 38400
ADDRESSES = [0x80]  # Gerbil drives both wheels from one board


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--set", type=float, metavar="SECONDS",
                        help="write this timeout to the board (0 disables)")
    parser.add_argument("--port", default=PORT)
    args = parser.parse_args()

    # Sent to the board as a single byte in tenths of a second
    if args.set is not None and not 0.0 <= args.set <= 25.5:
        parser.error("--set must be between 0 and 25.5 seconds")

    controller = Basicmicro(args.port, BAUD)
    if not controller.Open():
        sys.exit(f"Could not open {args.port} (is a launch still running?)")
    try:
        for address in ADDRESSES:
            if args.set is not None:
                ok = controller.SetTimeout(address, args.set)
                print(f"{address}: set {args.set}s -> {'ok' if ok else 'FAILED'}")
            ok, timeout = controller.GetTimeout(address)
            if not ok:
                print(f"{address}: read failed (board not responding?)")
            elif timeout == 0:
                print(f"{address}: timeout DISABLED — motors latch if commands stop")
            else:
                print(f"{address}: timeout {timeout}s")
    finally:
        controller.close()


if __name__ == "__main__":
    main()
