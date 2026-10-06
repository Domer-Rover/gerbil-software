#!/usr/bin/env python3
"""
getPid_2.py
Test, write, print and save the velocity + position PID of a RoboClaw 2x20A
driving two goBILDA 5204 Yellow Jacket motors (M1, M2).

What a full run does:
  1. reads everything the board reports (PID, QPPS, encoders, voltages, ...)
  2. spins each motor at full duty, both directions, to measure the real QPPS
  3. writes the velocity + position PID below (velocity with the measured
     QPPS) and verifies readback
  4. commands a few speeds under PID and reports target vs. actual
  5. prints a report and saves it all to scripts/pid_logs/roboclaw_latest.json
     (each run replaces the previous file)

WHEELS MUST BE OFF THE GROUND: steps 2 and 4 run the motors at full speed.
Stop any running launch first — the port can only be open once.

Usage:
    python3 scripts/getPid_2.py              # test + write to RAM (lost on power cycle)
    python3 scripts/getPid_2.py --save       # same, then save to NVM if everything passed
    python3 scripts/getPid_2.py --read       # no motion, no writes: just print + save file
    python3 scripts/getPid_2.py --no-test    # no motion: write the constants below, verify
    python3 scripts/getPid_2.py --port /dev/gerbil_roboclaw
"""

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

from basicmicro import Basicmicro as Roboclaw

PORT    = "/dev/ttyACM0"
BAUD    = 38400
ADDRESS = 0x80

# Encoder counts per output-shaft revolution: 28 counts/motor rev x gear ratio.
# 1993.6 = 71.2:1 (223 RPM). Only used to print RPM next to QPPS — change it to
# match the ratio printed on your motors.
COUNTS_PER_REV = 1993.6

# Position limits: RoboClaw CLAMPS position targets to [min, max]. Wheels turn
# continuously, so keep these wide.
POS_MIN = -2_000_000_000
POS_MAX =  2_000_000_000

# ── Per-motor config (replace with your Motion Studio auto-tune values) ──────
# Same layout as old_scripts/set_pid.py. velocity.qpps is only used with --no-test; a
# normal run measures it.
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

# ── Test settings ────────────────────────────────────────────────────────────
TEST_DUTY      = 32767             # full duty; QPPS is defined as speed at 100%
SPINUP_S       = 1.5               # time to reach steady speed before sampling
SAMPLE_S       = 1.0               # averaging window
POLL_S         = 0.05              # also keeps the board's serial timeout fed
MIN_SPEED      = 50                # counts/s; below this the encoder isn't counting
STEP_FRACTIONS = (0.25, 0.5, 0.75, -0.5)   # of QPPS; negative = reverse
STEP_TOLERANCE = 0.05              # pass if within 5% of target

TOL     = 0.01  # float readback tolerance (board stores fixed-point, so values round)
LOG_FILE = Path(__file__).resolve().parent / "pid_logs" / "roboclaw_latest.json"


class TestFailed(Exception):
    pass


def signed32(v):
    return v - (1 << 32) if v >= (1 << 31) else v


def connect(port):
    rc = Roboclaw(port, BAUD)
    if rc.Open() != 1:
        sys.exit(f"ERROR: could not open {port}")
    ver = rc.ReadVersion(ADDRESS)
    if not ver[0]:
        rc.close()
        sys.exit("ERROR: no response from RoboClaw (address/baud/wiring?)")
    print(f"Connected: {ver[1].strip()}")
    return rc


def decode_status(status):
    """Status word → text. Low 16 bits are errors, high 16 bits warnings.
    (basicmicro 2.0.10's own decode_error_status raises NameError.)"""
    text = [f"ERROR: {d}" for bit, d in Roboclaw.ERROR_DESCRIPTIONS.items()
            if status & 0xFFFF & bit]
    text += [f"warning: {d}" for bit, d in Roboclaw.WARNING_DESCRIPTIONS.items()
             if (status >> 16) & bit]
    return text or ["none"]


def stop(rc):
    rc.DutyM1M2(ADDRESS, 0, 0)


# ── Reading ──────────────────────────────────────────────────────────────────
def rd(fn):
    """Call a read; return its values without the success flag, or None."""
    try:
        r = fn(ADDRESS)
    except Exception:
        return None
    return list(r[1:]) if r[0] else None


def snapshot(rc):
    def one(fn, scale=None):
        r = rd(fn)
        if r is None:
            return None
        return r[0] / scale if scale else r[0]

    snap = {}
    ver = rd(rc.ReadVersion)
    snap["firmware"] = ver[0].strip() if ver else None

    enc = rd(rc.GetEncoders)
    modes = rd(rc.ReadEncoderModes)
    amps = rd(rc.ReadCurrents)
    accels = rd(rc.GetDefaultAccels)
    for n, name in enumerate(MOTORS):
        m1 = name == "M1"
        v = rd(rc.ReadM1VelocityPID if m1 else rc.ReadM2VelocityPID)
        p = rd(rc.ReadM1PositionPID if m1 else rc.ReadM2PositionPID)
        lim = rd(rc.ReadM1MaxCurrent if m1 else rc.ReadM2MaxCurrent)
        snap[name] = {
            "velocity_pid": v and dict(zip(("p", "i", "d", "qpps"), v)),
            "position_pid": p and dict(zip(
                ("p", "i", "d", "imax", "deadzone", "min", "max"),
                p[:5] + [signed32(p[5]), signed32(p[6])])),
            "encoder": enc and signed32(enc[n]),
            "encoder_mode": modes and modes[n],
            "current_a": amps and amps[n] / 100.0,
            "max_current_a": lim and lim[0] / 100.0,
            "default_accel": accels and accels[2 * n],
            "default_decel": accels and accels[2 * n + 1],
        }

    snap["main_battery_v"] = one(rc.ReadMainBatteryVoltage, 10.0)
    snap["logic_battery_v"] = one(rc.ReadLogicBatteryVoltage, 10.0)
    mm = rd(rc.ReadMinMaxMainVoltages)
    snap["main_battery_min_v"] = mm and mm[0] / 10.0
    snap["main_battery_max_v"] = mm and mm[1] / 10.0
    snap["temp_c"] = one(rc.ReadTemp, 10.0)
    snap["temp2_c"] = one(rc.ReadTemp2, 10.0)
    snap["serial_timeout_s"] = one(rc.GetTimeout)
    snap["config"] = one(rc.GetConfig)
    err = one(rc.ReadError)
    snap["error"] = err
    snap["error_text"] = None if err is None else decode_status(err)
    return snap


def fmt(v, spec=""):
    return "read failed" if v is None else format(v, spec)


def print_snapshot(title, snap):
    print(f"\n── {title} " + "─" * (60 - len(title)))
    print(f"Firmware      : {fmt(snap['firmware'])}")
    print(f"Main battery  : {fmt(snap['main_battery_v'])} V "
          f"(limits {fmt(snap['main_battery_min_v'])}–{fmt(snap['main_battery_max_v'])} V)")
    print(f"Logic battery : {fmt(snap['logic_battery_v'])} V")
    print(f"Temperature   : {fmt(snap['temp_c'])} °C / {fmt(snap['temp2_c'])} °C")
    print(f"Serial timeout: {fmt(snap['serial_timeout_s'])} s   config: {fmt(snap['config'])}")
    print(f"Errors        : {', '.join(snap['error_text']) if snap['error_text'] else 'read failed'}")
    for name in MOTORS:
        m = snap[name]
        v, p = m["velocity_pid"], m["position_pid"]
        if v:
            print(f"{name}: VEL  P={v['p']:.5f} I={v['i']:.5f} D={v['d']:.5f} QPPS={v['qpps']}")
            if v["qpps"] == 0:
                print(f"{name}: WARNING — QPPS is 0, velocity PID is not configured")
        else:
            print(f"{name}: VEL  read failed")
        if p:
            print(f"{name}: POS  P={p['p']:.5f} I={p['i']:.5f} D={p['d']:.5f} "
                  f"iMax={p['imax']} dz={p['deadzone']} min={p['min']} max={p['max']}")
        else:
            print(f"{name}: POS  read failed")
        print(f"{name}: encoder={fmt(m['encoder'])} mode={fmt(m['encoder_mode'])} "
              f"current={fmt(m['current_a'])} A (limit {fmt(m['max_current_a'])} A) "
              f"accel/decel={fmt(m['default_accel'])}/{fmt(m['default_decel'])}")


# ── Motion tests ─────────────────────────────────────────────────────────────
def read_speed(rc, name):
    r = (rc.ReadSpeedM1 if name == "M1" else rc.ReadSpeedM2)(ADDRESS)
    if not r[0]:
        return None
    speed = signed32(r[1])
    # Some firmware reports magnitude + a direction byte instead of a signed value.
    if speed > 0 and r[2] == 1:
        speed = -speed
    return speed


def steady_speed(rc, name):
    """Poll speed for SPINUP_S + SAMPLE_S; return the mean over the last SAMPLE_S."""
    samples = []
    start = time.monotonic()
    while (t := time.monotonic() - start) < SPINUP_S + SAMPLE_S:
        s = read_speed(rc, name)
        if s is not None and t >= SPINUP_S:
            samples.append(s)
        time.sleep(POLL_S)
    if not samples:
        raise TestFailed(f"{name}: could not read speed")
    return sum(samples) / len(samples)


def measure_qpps(rc, name):
    duty = rc.DutyM1 if name == "M1" else rc.DutyM2
    result = {}
    for label, d in (("forward", TEST_DUTY), ("reverse", -TEST_DUTY)):
        print(f"{name}: full duty {label}...", end=" ", flush=True)
        duty(ADDRESS, d)
        try:
            result[label] = round(steady_speed(rc, name))
        finally:
            duty(ADDRESS, 0)
        print(f"{result[label]} counts/s")
        time.sleep(1.0)  # coast down before reversing

    fwd, rev = result["forward"], result["reverse"]
    if abs(fwd) < MIN_SPEED or abs(rev) < MIN_SPEED:
        raise TestFailed(f"{name}: encoder is not counting (check encoder wiring/power)")
    if fwd < 0 or rev > 0:
        raise TestFailed(f"{name}: encoder counts backwards relative to the motor "
                         "(swap encoder A/B or the motor leads) — PID would run away")
    result["qpps"] = min(fwd, -rev)
    result["rpm"] = round(result["qpps"] / COUNTS_PER_REV * 60.0, 1)
    print(f"{name}: QPPS = {result['qpps']}  (~{result['rpm']} RPM at the output shaft)")
    return result


def step_test(rc, name, qpps):
    speed = rc.SpeedAccelM1 if name == "M1" else rc.SpeedAccelM2
    duty = rc.DutyM1 if name == "M1" else rc.DutyM2
    rows = []
    try:
        for frac in STEP_FRACTIONS:
            target = int(qpps * frac)
            speed(ADDRESS, qpps * 2, target)
            actual = steady_speed(rc, name)
            err = (actual - target) / target
            rows.append({"target": target, "actual": round(actual),
                         "error_pct": round(err * 100.0, 2),
                         "ok": abs(err) <= STEP_TOLERANCE})
            r = rows[-1]
            print(f"{name}: target {r['target']:>7}  actual {r['actual']:>7}  "
                  f"error {r['error_pct']:>6.2f}%  {'OK' if r['ok'] else 'FAIL'}")
    finally:
        duty(ADDRESS, 0)
    time.sleep(1.0)
    return rows


# ── Writing ──────────────────────────────────────────────────────────────────
def write_pid(rc, name, cfg):
    v, p = cfg["velocity"], cfg["position"]
    if name == "M1":
        ok_v = rc.SetM1VelocityPID(ADDRESS, v["p"], v["i"], v["d"], v["qpps"])
        ok_p = rc.SetM1PositionPID(ADDRESS, p["p"], p["i"], p["d"],
                                   p["imax"], p["deadzone"], p["min"], p["max"])
    else:
        ok_v = rc.SetM2VelocityPID(ADDRESS, v["p"], v["i"], v["d"], v["qpps"])
        ok_p = rc.SetM2PositionPID(ADDRESS, p["p"], p["i"], p["d"],
                                   p["imax"], p["deadzone"], p["min"], p["max"])
    print(f"{name}: VEL  P={v['p']} I={v['i']} D={v['d']} QPPS={v['qpps']} "
          f"→ {'OK' if ok_v else 'FAILED'}")
    print(f"{name}: POS  P={p['p']} I={p['i']} D={p['d']} iMax={p['imax']} "
          f"dz={p['deadzone']} min={p['min']} max={p['max']} "
          f"→ {'OK' if ok_p else 'FAILED'}")
    return bool(ok_v and ok_p)


def matches(name, want, got):
    ok = True
    for section in ("velocity", "position"):
        have = got[f"{section}_pid"]
        if have is None:
            print(f"{name}: {section} readback FAILED")
            ok = False
            continue
        for k, w in want[section].items():
            if abs(float(w) - float(have[k])) > max(TOL, abs(w) * 0.001):
                print(f"{name}: mismatch {section}.{k}: wanted {w}, board has {have[k]}")
                ok = False
    return ok


def save_record(record):
    LOG_FILE.parent.mkdir(exist_ok=True)
    LOG_FILE.write_text(json.dumps(record, indent=2) + "\n")
    return LOG_FILE


def main():
    parser = argparse.ArgumentParser(
        description="Test, write, print and save the RoboClaw velocity + position PID.")
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--read", action="store_true",
                        help="no motion, no writes: print the board state and save it")
    parser.add_argument("--no-test", action="store_true",
                        help="no motion: write the constants (including velocity qpps) and verify")
    parser.add_argument("--save", action="store_true",
                        help="save to NVM if everything passed (survives power cycle)")
    parser.add_argument("--yes", action="store_true",
                        help="skip the wheels-off-the-ground prompt")
    args = parser.parse_args()
    moving = not (args.read or args.no_test)

    rc = connect(args.port)
    record = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "port": args.port,
        "address": ADDRESS,
        "counts_per_rev": COUNTS_PER_REV,
        "firmware": None,
        "before": None,
        "after": None,
        "qpps_test": None,
        "written": None,
        "readback_ok": None,
        "step_test": None,
        "saved_to_nvm": False,
        "result": "failed: did not finish",
    }

    try:
        record["before"] = snapshot(rc)
        record["firmware"] = record["before"]["firmware"]
        print_snapshot("Board state", record["before"])
        if args.read:
            record["result"] = "ok"
            return

        cfgs = {name: {k: dict(v) for k, v in cfg.items()} for name, cfg in MOTORS.items()}

        if moving:
            if not args.yes:
                answer = input("\nMotors will run at FULL SPEED in both directions.\n"
                               "Are the wheels off the ground? [y/N] ")
                if answer.strip().lower() not in ("y", "yes"):
                    raise TestFailed("cancelled at the safety prompt")
            print("\n── Measuring QPPS ─────────────────────────────────────────────")
            record["qpps_test"] = {}
            for name in MOTORS:
                record["qpps_test"][name] = measure_qpps(rc, name)
                cfgs[name]["velocity"]["qpps"] = record["qpps_test"][name]["qpps"]

        print("\n── Writing velocity + position PID ────────────────────────────")
        record["written"] = cfgs
        wrote = all([write_pid(rc, name, cfg) for name, cfg in cfgs.items()])
        after = snapshot(rc)
        record["readback_ok"] = wrote and all(
            [matches(name, cfg, after[name]) for name, cfg in cfgs.items()])
        if not record["readback_ok"]:
            raise TestFailed("written values did not verify — NOT saving to NVM")
        print("Readback verified.")

        if moving:
            print("\n── Step test (closed loop) ────────────────────────────────────")
            record["step_test"] = {}
            for name, cfg in cfgs.items():
                record["step_test"][name] = step_test(rc, name, cfg["velocity"]["qpps"])
            if not all(r["ok"] for rows in record["step_test"].values() for r in rows):
                raise TestFailed(f"step test outside ±{STEP_TOLERANCE:.0%} — the PID is in "
                                 "RAM but was NOT saved to NVM; retune P/I and rerun")

        if args.save:
            if not rc.WriteNVM(ADDRESS):
                raise TestFailed("WriteNVM failed")
            record["saved_to_nvm"] = True
            print("\nSaved to NVM.")
        else:
            print("\nApplied to RAM only. Run with --save to persist.")

        record["result"] = "ok"
    except TestFailed as e:
        record["result"] = f"failed: {e}"
    except KeyboardInterrupt:
        record["result"] = "failed: interrupted"
    finally:
        try:
            if not args.read:
                stop(rc)
                if record["written"]:
                    record["after"] = snapshot(rc)
                    print_snapshot("Board state after", record["after"])
        finally:
            rc.close()
            path = save_record(record)
            print(f"\nResult: {record['result']}")
            print(f"Saved:  {path}")

    if record["result"] != "ok":
        sys.exit(1)


if __name__ == "__main__":
    main()
