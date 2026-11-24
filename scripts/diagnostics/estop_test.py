#!/usr/bin/env python3
"""Measure how long the wheels keep turning after commands stop.

This is the "virtual e-stop" check. There are two independent timeouts between
a stopped command stream and stopped wheels:

  twist_mux            topic timeout, 0.5 s   (config/twist_mux.yaml)
  diff_drive_controller cmd_vel_timeout, 0.5 s (config/gerbil_controllers.yaml)
  RoboClaw             serial failsafe, ~200 ms, in firmware -- the backstop
                       that catches the whole stack dying at once

This script exercises the first two by simply STOPPING its own publishing,
which is the common case: an operator lets go, a node crashes, Wi-Fi drops.
It does not test the RoboClaw failsafe -- for that, kill the launch and watch,
since /joint_states dies with it and nothing can observe the stop from ROS.

!!! WHEELS OFF THE GROUND !!!  This drives the robot.

    python3 scripts/diagnostics/estop_test.py --confirm
    python3 scripts/diagnostics/estop_test.py --confirm --speed 0.15 --drive 3
"""
import argparse
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

LEFT = 'base_left_wheel_joint'
RIGHT = 'base_right_wheel_joint'
STOPPED = 0.02  # rad/s, below this the wheel is stopped


class EstopTest(Node):

    def __init__(self, topic, speed, drive, timeout):
        super().__init__('estop_test')
        self.speed = speed
        self.drive = drive
        self.timeout = timeout
        self.pub = self.create_publisher(Twist, topic, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 50)
        self.wheel_speed = None
        self.ever_moved = False

    def on_joint_state(self, msg):
        if not msg.velocity:
            return
        worst = 0.0
        for name in (LEFT, RIGHT):
            if name in msg.name:
                i = msg.name.index(name)
                if i < len(msg.velocity):
                    worst = max(worst, abs(msg.velocity[i]))
        self.wheel_speed = worst
        if worst > STOPPED * 3:
            self.ever_moved = True

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def spin_until(self, deadline, predicate=None):
        while rclpy.ok() and self.now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if predicate and predicate():
                return True
        return False

    def run(self):
        twist = Twist()
        twist.linear.x = self.speed

        print(f'Driving at {self.speed} m/s for {self.drive:.0f} s ...')
        end = self.now() + self.drive
        while rclpy.ok() and self.now() < end:
            self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)

        if not self.ever_moved:
            print()
            print('The wheels never moved, so there is no stop to time.')
            print('Check that the command path is live:')
            print('  ros2 topic echo /diff_drive_controller/cmd_vel_unstamped')
            print('If twist_mux is off, publish straight to that topic instead')
            print('with --topic /diff_drive_controller/cmd_vel_unstamped')
            return 2

        released = self.now()
        print(f'Commands STOPPED at t=0 (last wheel speed '
              f'{self.wheel_speed:.3f} rad/s). Waiting for the wheels ...')

        stopped = self.spin_until(
            released + self.timeout,
            lambda: self.wheel_speed is not None and self.wheel_speed < STOPPED)
        elapsed = self.now() - released

        print()
        print('=' * 68)
        if not stopped:
            print(f'FAIL: wheels still turning {elapsed:.2f} s after commands '
                  f'stopped ({self.wheel_speed:.3f} rad/s).')
            print()
            print('  Cut power. Something is latching the last command -- that')
            print('  is the failure mode the timeouts exist to prevent.')
            print('=' * 68)
            return 1

        print(f'PASS: wheels stopped {elapsed:.3f} s after the last command.')
        print()
        print('  Expected roughly 0.5-1.0 s: twist_mux drops the stale topic at')
        print('  0.5 s, then diff_drive_controller zeroes at its own 0.5 s')
        print('  cmd_vel_timeout. Much longer means a timeout is not firing.')
        print()
        print('  The stop is abrupt by design -- halt() bypasses the')
        print('  acceleration limiter. Raising cmd_vel_timeout would NOT soften')
        print('  the jolt, only delay the stop.')
        print('=' * 68)
        return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--confirm', action='store_true',
                    help='required: confirms the wheels are off the ground')
    ap.add_argument('--topic', default='/cmd_vel_key',
                    help='where to publish (default /cmd_vel_key, through twist_mux)')
    ap.add_argument('--speed', type=float, default=0.1, help='m/s (default 0.1)')
    ap.add_argument('--drive', type=float, default=2.0, help='seconds (default 2)')
    ap.add_argument('--timeout', type=float, default=5.0,
                    help='how long to wait for the stop (default 5)')
    args = ap.parse_args()

    if not args.confirm:
        print('This DRIVES THE ROBOT. Get the wheels off the ground, then')
        print('re-run with --confirm.')
        sys.exit(2)

    rclpy.init()
    node = EstopTest(args.topic, args.speed, args.drive, args.timeout)
    try:
        code = node.run()
    except KeyboardInterrupt:
        print('\ninterrupted')
        code = 2
    finally:
        try:
            node.pub.publish(Twist())  # best effort: command zero on the way out
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
