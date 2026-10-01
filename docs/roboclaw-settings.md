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
| Maximum | **27.0 V** | above the 26.1 V full charge, below the board's limit |
| Minimum | **19.8 V** | 3.3 V/cell. Low enough that sag under load does not nuisance-trip |

Stop driving around 21.0 V (3.5 V/cell) rather than relying on the 19.8 V
cutoff — that threshold is damage protection, not a discharge target.

## Logic battery

The "Logic Battery High" fault (error LED blinks twice) means the measured
logic voltage exceeded this maximum. Measure LB+ to LB− before setting it.

| If LB+ measures | Minimum | Maximum |
|---|---|---|
| ~12 V (from the DC-DC converter) | 10.0 V | 16.0 V |
| ~22-26 V (fed from the pack, or jumpered to main) | 20.0 V | 27.0 V |

## Current limits

| Setting | Value |
|---|---|
| M1 / M2 current limit | **5 A each to start** |

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

1. **qppr** — mark the tire, zero the encoder, turn the wheel 10 full
   revolutions, read the count, divide by 10. Put it in the xacro (both joints).
2. **QPPS** — run the motor at full duty in Motion Studio and read live
   counts/sec. That is the maximum for the PID.
3. **Encoder polarity** — rolling a wheel forward must make the count go **up**.
   If it counts down, swap that channel's A/B encoder wires. In velocity mode a
   reversed encoder becomes positive feedback and the motor runs away at full
   throttle.
4. Auto-tune velocity PID, or start at P 0.2 / I 0.1 / D 0.

Use a shielded USB cable with a ferrite bead for tuning. Motor noise drops the
USB link mid-tune, which can leave half-written settings on the board.

## Measured values (fill in)

| Value | M1 (right) | M2 (left) |
|---|---|---|
| qppr | | |
| QPPS | | |
| P / I / D | | |
| Current at cruise | | |
| Current limit set | | |
