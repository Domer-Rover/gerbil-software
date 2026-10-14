#!/usr/bin/env python3
"""Measure qppr: encoder counts per one revolution of the wheel.

qppr lives in gerbil_description/urdf/mobile_base.ros2_control.xacro and
converts encoder ticks to radians. Wrong qppr means the robot drives fine but
misreports distance, which quietly breaks odometry and Nav2.

This does not move the motors. You turn the wheel by hand.

  python3 scripts/measure_qppr.py            # M1, 10 turns
  python3 scripts/measure_qppr.py --motor M2 --turns 10
"""
import argparse
import time

from basicmicro import Basicmicro

PORT = "/dev/gerbil_roboclaw"
BAUD = 38400
ADDRESS = 0x80


def read(controller, motor):
    ok, value, _ = (controller.ReadEncM1 if motor == "M1" else controller.ReadEncM2)(ADDRESS)
    if not ok:
        raise SystemExit(f"could not read the {motor} encoder")
    return value


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--motor", choices=["M1", "M2"], default="M1")
    ap.add_argument("--turns", type=float, default=10.0)
    ap.add_argument("--port", default=PORT)
    args = ap.parse_args()

    controller = Basicmicro(args.port, BAUD)
    controller.Open()
    try:
        print(f"Mark the {args.motor} tire and the chassis so you can line them up.")
        input("Press Enter when the marks are aligned... ")
        start = read(controller, args.motor)
        print(f"start = {start}")
        print(f"Now turn the wheel exactly {args.turns:g} full revolutions, forward.")
        print("Live count (Ctrl-C when you are back at the mark):")
        try:
            while True:
                print(f"  {read(controller, args.motor) - start:+d}   ", end="\r", flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print()
        end = read(controller, args.motor)

        delta = end - start
        qppr = abs(delta) / args.turns
        print(f"\nend   = {end}")
        print(f"delta = {delta} over {args.turns:g} turns")
        print(f"qppr  = {qppr:.1f}  (use {round(qppr)} in the xacro)")
        if delta < 0:
            print("NOTE: the count went DOWN. Forward motion should count up;")
            print("      check the encoder direction for this channel.")
        for ratio, counts in (("71.2:1", 1993.6), ("99.5:1", 2786.2)):
            if abs(qppr - counts) / counts < 0.05:
                print(f"      matches the {ratio} goBILDA gearbox ({counts} PPR)")
    finally:
        controller.close()


if __name__ == "__main__":
    main()
