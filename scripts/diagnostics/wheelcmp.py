#!/usr/bin/env python3
"""Pre-flight wheel/encoder check. Run this BEFORE every closed-loop drive.

Why it exists: on 2026-11-10 M2's encoder ground pin came off its jumper. In
velocity-PID mode the loop saw feedback stuck at zero, wound up, and saturated
duty -- the wheel ran away at full speed. That looks exactly like a tuning
problem in a log, and it is not. The signature is unmistakable but only if you
look for it: one wheel reads EXACTLY 0.0, not a small number, while the other
tracks fine and the wheel is visibly spinning.

That jumper is a known weak point and will fail again on rough ground.

Usage, with the stack up and the WHEELS OFF THE GROUND:

    # 1. drive the wheels from another terminal (teleop, or a cmd_vel pub)
    # 2. then, while they are turning:
    python3 scripts/diagnostics/wheelcmp.py

    python3 scripts/diagnostics/wheelcmp.py --duration 10
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

LEFT = 'base_left_wheel_joint'    # RoboClaw M1
RIGHT = 'base_right_wheel_joint'  # RoboClaw M2


class WheelCompare(Node):

    def __init__(self, duration):
        super().__init__('wheelcmp')
        self.duration = duration
        self.samples = {LEFT: [], RIGHT: []}
        self.missing_joints = False
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 50)
        self.start = self.get_clock().now()

    def on_joint_state(self, msg):
        if not msg.velocity:
            return
        for name in (LEFT, RIGHT):
            if name in msg.name:
                i = msg.name.index(name)
                if i < len(msg.velocity):
                    self.samples[name].append(msg.velocity[i])
            else:
                self.missing_joints = True

    def done(self):
        elapsed = (self.get_clock().now() - self.start).nanoseconds / 1e9
        return elapsed >= self.duration


def stats(values):
    if not values:
        return None
    n = len(values)
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / n
    peak = max(values, key=abs)
    exact_zeros = sum(1 for v in values if v == 0.0)
    return {
        'n': n, 'mean': mean, 'std': var ** 0.5,
        'peak': peak, 'exact_zeros': exact_zeros,
        'all_exactly_zero': exact_zeros == n,
    }


def report(left, right, missing_joints):
    print()
    print('=' * 68)
    if missing_joints:
        print('WARNING: a wheel joint was absent from /joint_states.')
        print('         joint_state_broadcaster may not be spawned.')
        print('=' * 68)

    if left is None or right is None:
        print('NO DATA on /joint_states for one or both wheels.')
        print()
        print('  left  (M1):', 'no samples' if left is None else f"{left['n']} samples")
        print('  right (M2):', 'no samples' if right is None else f"{right['n']} samples")
        print()
        print('VERDICT: cannot judge the wheels. Check that the stack is up:')
        print('  ros2 topic hz /joint_states')
        print('=' * 68)
        return 2

    for label, s in (('left  (M1)', left), ('right (M2)', right)):
        print(f'  {label}  mean {s["mean"]:+8.4f} rad/s   '
              f'peak {s["peak"]:+8.4f}   std {s["std"]:6.4f}   '
              f'n={s["n"]}  exact-zeros={s["exact_zeros"]}')
    print()

    moving = 0.02  # rad/s; below this a wheel is not meaningfully turning
    l_moving = abs(left['peak']) > moving
    r_moving = abs(right['peak']) > moving

    # The dead-encoder case FIRST. A real stationary wheel still shows
    # quantisation noise, so a channel that is bit-for-bit zero across every
    # sample is reading nothing at all, not reading a stopped wheel.
    dead = [name for name, s in (('left (M1)', left), ('right (M2)', right))
            if s['all_exactly_zero']]
    if dead and (l_moving or r_moving):
        print(f'VERDICT: *** {" and ".join(dead)} read EXACTLY 0.0 across every '
              f'sample while the other wheel turned. ***')
        print()
        print('  That is a DEAD ENCODER CHANNEL, not a PID problem. Do not')
        print('  retune anything. Check the encoder jumper -- the ground pin')
        print('  has come off before.')
        print()
        print('  In velocity-PID mode this wheel WILL run away at full duty.')
        print('  Launch with use_duty_cycle:=true until it is fixed.')
        print('=' * 68)
        return 1

    if not l_moving and not r_moving:
        print('VERDICT: neither wheel moved. Inconclusive -- this script needs')
        print('         the wheels turning. Drive them and run it again.')
        print('=' * 68)
        return 2

    if l_moving != r_moving:
        still = 'right (M2)' if l_moving else 'left (M1)'
        print(f'VERDICT: {still} did not turn while the other did.')
        print('         Could be a dead encoder reading small nonzero noise, a')
        print('         motor not driving, or a loose power lead. Compare')
        print('         against what you can SEE the wheels doing.')
        print('=' * 68)
        return 1

    # Both turning: how well do they agree?
    ratio = abs(left['mean']) / abs(right['mean']) if right['mean'] else float('inf')
    same_sign = (left['mean'] > 0) == (right['mean'] > 0)
    print(f'  magnitude ratio |L|/|R| = {ratio:.4f}')
    print(f'  same direction          = {same_sign}')
    print()

    if not same_sign:
        print('VERDICT: the wheels turned in OPPOSITE directions. Expected if')
        print('         you commanded a rotation; a fault if you commanded a')
        print('         straight line.')
        print('=' * 68)
        return 0

    if abs(ratio - 1.0) > 0.15:
        print(f'VERDICT: both wheels turning but {abs(ratio - 1.0) * 100:.0f}% apart.')
        print('         Real asymmetry. Check the PID values per motor and that')
        print('         both encoders are on the same mode and QPPS.')
        print('=' * 68)
        return 1

    print('VERDICT: both encoders live and agreeing within 15%. Good to drive.')
    print('=' * 68)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=float, default=5.0,
                    help='seconds to collect (default 5)')
    args = ap.parse_args()

    rclpy.init()
    node = WheelCompare(args.duration)
    print(f'Collecting /joint_states for {args.duration:.0f} s -- '
          f'drive the wheels NOW if they are not already turning.')
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('\ninterrupted, reporting on what was collected')

    code = report(stats(node.samples[LEFT]), stats(node.samples[RIGHT]),
                  node.missing_joints)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
