# RoboClaw settings (Motion Studio)

Gerbil: one RoboClaw 2x60A, address 128, both wheels.
Record changes here — the lack of a record is why these had to be
reverse-engineered once already.

Battery: **Gens Ace 6800 mAh 6S 100C 22.8 V LiHV** (hardcase, EC5).
LiHV means 4.35 V/cell, so full charge is 26.1 V, not the 25.2 V of a
standard 6S LiPo. Nominal 22.8 V, and 3.0 V/cell (18.0 V) is the damage floor.

## Communication — must match the code

| Setting | Value | Why |
|---|---|---|
| Control mode | Packet Serial | `roboclaw_serial` speaks only this |
| Baud rate | 38400 | hardcoded in `roboclaw_serial/device.hpp` |
| Packet Serial Address | 128 | `mobile_base.ros2_control.xacro` |
| Multi-Unit Mode | Off | single board (Capybara's 3 boards need it on) |
| Serial timeout | 0.2 s | motors stop if ROS dies; `scripts/serial_timeout.py --set 0.2` |
| M1 | right wheel | `base_right_wheel_joint` |
| M2 | left wheel | `base_left_wheel_joint` |
| Encoder mode | Quadrature (both) | interface reads `READ_M1_M2_ENC` |

## Main battery (6S LiHV)

| Setting | Value | Reasoning |
|---|---|---|
| Maximum | **27.0-27.6 V** | above the 26.1 V full charge, below the board's limit |
| Minimum | **19.8 V** | 3.3 V/cell. The factory default of 10 V would let the pack discharge to destruction |

Stop driving around 21.0 V (3.5 V/cell) rather than relying on the 19.8 V
cutoff — that threshold is damage protection, not a discharge target.

## Logic battery

The DC-DC converter powers the Jetson only, so the RoboClaw's logic runs off
the main battery (onboard main-to-logic path, or a B+ to LB+ jumper). The logic
voltage it measures is therefore the pack: 22.8 V nominal, 26.1 V full.

| Setting | Value |
|---|---|
| Logic Battery Maximum | **27.0 V** |
| Logic Battery Minimum | **20.0 V** |

A maximum below 26.1 V causes the "Logic Battery High" fault — error LED blinks
twice, motors freewheel until reset. That was the fault seen on 2026-09-30.

Before raising it, confirm the logic input's rated range in the 2x60A manual.
If 26 V is outside it, feed LB+ 12 V from the converter instead of raising the
threshold.

## Current limits

Motors: **goBILDA 5304-8002-0100** (99.5:1, 180 RPM @ 24 V, RS-560).
Rated current 11 A, stall 60 A, no-load 1.3 A. 24 V nominal, so the 6S pack
needs no duty limiting.

| Setting | Value |
|---|---|
| M1 / M2 current limit | **15 A each** (above 11 A rated, below 60 A stall) |
| Motor-path fuse | **20-25 A slow-blow** (a 5 A fuse is below the rated draw and will blow) |

The pack is 6800 mAh at 100C, so it can source hundreds of amps: the battery
is not the limit, your wiring and fuses are. Set the board's electronic limit
low and raise it only after reading real currents:

```bash
python3 -c "
from basicmicro import Basicmicro
c = Basicmicro('/dev/gerbil_roboclaw', 38400); c.Open()
print('err', c.ReadError(0x80), 'V', c.ReadMainBatteryVoltage(0x80), 'I', c.ReadCurrents(0x80))
c.close()"
```

## Velocity PID (closed loop only)

Measure first, tune second:

1. **qppr = 2786** from the datasheet (2786.2 PPR at the output shaft), already
   set in the xacro. Verify once: mark the tire, zero the encoder, turn 10 full
   revolutions, expect ~27,862.
2. **QPPS ~8400** (180 RPM = 3 rev/s x 2786). Prefer the live counts/sec that
   Motion Studio shows at full duty.
3. **Encoder polarity** — rolling a wheel forward must make the count go **up**.
   If it counts down, swap that channel's A/B encoder wires. In velocity mode a
   reversed encoder becomes positive feedback and the motor runs away at full
   throttle.
4. Auto-tune velocity PID, or start at P 0.2 / I 0.1 / D 0.

Use a shielded USB cable with a ferrite bead for tuning. Motor noise drops the
USB link mid-tune, which can leave half-written settings on the board.

## Known wiring quirks

- M2's encoder A/B are crossed (or reversed in Motion Studio) so that forward
  motion counts up — confirmed 2026-10-02. Do not "fix" this back.
- Power-on produces an inrush arc at the fuse holder. Press the e-stop before
  unplugging the battery, and consider an XT90-S anti-spark connector or a
  pre-charge resistor. Check the e-stop's **DC** rating, not its AC rating.

## Measured values (fill in)

| Value | M1 (right) | M2 (left) |
|---|---|---|
| qppr | 2786 (datasheet) | 2786 (datasheet) |
| QPPS | | |
| P / I / D | | |
| Current at cruise | | |
| Current limit set | | |
