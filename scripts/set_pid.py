#!/usr/bin/env python3
"""
set_pid.py
Write velocity + position PID to both channels (M1, M2) of a RoboClaw 2x15A,
read them back to verify, and optionally save to NVM.

Usage:
    python set_pid.py            # apply to RAM only (lost on power cycle)
    python set_pid.py --save     # apply and save to NVM (survives power cycle)
    python set_pid.py --read     # just read what's currently on the board

Get the numbers from BasicMicro Motion Studio auto-tune, run separately per
motor (velocity tune AND position tune), with the real load attached.
"""

import sys
from basicmicro import Basicmicro as Roboclaw

SERIAL  = "/dev/ttyACM0"        # Linux: "/dev/ttyACM0"
BAUD    = 38400
ADDRESS = 0x80

# Position limits: RoboClaw CLAMPS position targets to [min, max].
# For a tracker that can keep turning (shortest-path accumulates counts),
# keep these wide. For a mechanically limited axis, set real limits.
POS_MIN = -2_000_000_000
POS_MAX =  2_000_000_000

# ── Per-motor config (replace with your Motion Studio auto-tune values) ──────
MOTORS = {
    "M1": {
        "velocity": {"p": 10.44217, "i": 0.62657, "d": 0.0, "qpps": 4290},
        "position": {"p": 0.0, "i": 0.0, "d": 0.0,
                     "imax": 0, "deadzone": 0, "min": POS_MIN, "max": POS_MAX},
    },
    "M2": {
        "velocity": {"p": 10.44217, "i": 0.62657, "d": 0.0, "qpps": 4290},
        "position": {"p": 0.0, "i": 0.0, "d": 0.0,
                     "imax": 0, "deadzone": 0, "min": POS_MIN, "max": POS_MAX},
    },
}

TOL = 0.01  # float readback tolerance (board stores fixed-point, so values round)


def connect():
    rc = Roboclaw(SERIAL, BAUD)
    if rc.Open() != 1:
        sys.exit(f"ERROR: could not open {SERIAL}")
    ver = rc.ReadVersion(ADDRESS)
    if not ver[0]:
        sys.exit("ERROR: no response from RoboClaw (address/baud/wiring?)")
    print(f"Connected: {ver[1].strip()}")
    return rc


def write_motor(rc, name, cfg):
    v, p = cfg["velocity"], cfg["position"]
    if name == "M1":
        ok_v = rc.SetM1VelocityPID(ADDRESS, v["p"], v["i"], v["d"], v["qpps"])
        ok_p = rc.SetM1PositionPID(ADDRESS, p["p"], p["i"], p["d"],
                                   p["imax"], p["deadzone"], p["min"], p["max"])
    else:
        ok_v = rc.SetM2VelocityPID(ADDRESS, v["p"], v["i"], v["d"], v["qpps"])
        ok_p = rc.SetM2PositionPID(ADDRESS, p["p"], p["i"], p["d"],
                                   p["imax"], p["deadzone"], p["min"], p["max"])
    print(f"{name}: velocity write {'OK' if ok_v else 'FAILED'}, "
          f"position write {'OK' if ok_p else 'FAILED'}")
    return bool(ok_v and ok_p)


def read_motor(rc, name):
    if name == "M1":
        v = rc.ReadM1VelocityPID(ADDRESS)
        p = rc.ReadM1PositionPID(ADDRESS)
        e = rc.ReadEncM1(ADDRESS)
    else:
        v = rc.ReadM2VelocityPID(ADDRESS)
        p = rc.ReadM2PositionPID(ADDRESS)
        e = rc.ReadEncM2(ADDRESS)
    if not (v[0] and p[0]):
        print(f"{name}: READ FAILED")
        return None
    vel = {"p": v[1], "i": v[2], "d": v[3], "qpps": v[4]}
    pos = {"p": p[1], "i": p[2], "d": p[3], "imax": p[4],
           "deadzone": p[5], "min": p[6], "max": p[7]}
    enc = e[1] if e[0] else "read failed"
    print(f"{name}: VEL  P={vel['p']:.5f} I={vel['i']:.5f} D={vel['d']:.5f} QPPS={vel['qpps']}")
    print(f"{name}: POS  P={pos['p']:.5f} I={pos['i']:.5f} D={pos['d']:.5f} "
          f"iMax={pos['imax']} dz={pos['deadzone']} min={pos['min']} max={pos['max']}")
    print(f"{name}: encoder = {enc}")
    return {"velocity": vel, "position": pos}


def matches(want, got):
    for section in ("velocity", "position"):
        for k, w in want[section].items():
            g = got[section][k]
            if abs(float(w) - float(g)) > max(TOL, abs(w) * 0.001):
                print(f"  mismatch {section}.{k}: wanted {w}, board has {g}")
                return False
    return True


def main():
    args = set(sys.argv[1:])
    rc = connect()

    if "--read" in args:
        for name in MOTORS:
            read_motor(rc, name)
        return

    all_ok = True
    for name, cfg in MOTORS.items():
        all_ok &= write_motor(rc, name, cfg)

    print("\nVerifying readback:")
    for name, cfg in MOTORS.items():
        got = read_motor(rc, name)
        if got is None or not matches(cfg, got):
            all_ok = False

    if not all_ok:
        sys.exit("\nNot all values verified — NOT saving to NVM.")

    if "--save" in args:
        if rc.WriteNVM(ADDRESS):
            print("\nSaved to NVM.")
        else:
            print("\nWriteNVM FAILED.")
    else:
        print("\nApplied to RAM only. Run with --save to persist.")


if __name__ == "__main__":
    main()
