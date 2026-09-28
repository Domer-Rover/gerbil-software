#!/usr/bin/env python3
"""Read (or set) the RoboClaw serial timeout on all three boards.

The RoboClaw stops its motors on its own if no serial command arrives within
this timeout. 0 disables it, which means a killed ROS node leaves the last
duty cycle latched and the rover keeps driving. Resolution is 0.1 s.

Stop any running launch first — the port can only be open once.

  python3 scripts/serial_timeout.py             # read all three
  python3 scripts/serial_timeout.py --set 0.2   # set 200 ms on all three
"""
import argparse

from basicmicro import Basicmicro

PORT = "/dev/gerbil_roboclaw"
BAUD = 38400
ADDRESSES = [0x80]  # Gerbil drives both wheels from one board


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--set", type=float, metavar="SECONDS",
                        help="write this timeout to every board (0 disables)")
    parser.add_argument("--port", default=PORT)
    args = parser.parse_args()

    controller = Basicmicro(args.port, BAUD)
    controller.Open()
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
