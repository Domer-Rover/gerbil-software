#!/usr/bin/env python3
"""Sim only: pass the ZED VIO stand-in through, minus physically impossible samples.

Gazebo's OdometryPublisher differentiates the model pose, and its FIRST sample
differences the spawn pose against the world origin. In school_loop (spawn
y = -10.67) that one message reports vy of about -333 m/s. The EKF fuses ZED
vy with the wrapper's zero covariance, i.e. as near-certain truth (the same
flaw as the real ZED, see gerbil_bringup/config/ekf.yaml), so that single
spike teleported the EKF 13 m sideways. Every goal then fell outside the
rolling global costmap and Nav2 dropped into spin/backup recovery.

A start-up delay was tried first and lost the race under CPU load, so this
filters on the value instead. Gerbil cannot exceed ~0.3 m/s or ~0.5 rad/s, so
anything above MAX_LINEAR / MAX_ANGULAR is a sim artefact and is dropped.
Everything else is republished unchanged, covariance included (still zero,
deliberately, to match the real wrapper).

  /sim/zed_odom_raw (bridged from gz)  ->  /zed/zed_node/odom (what the EKF reads)

The real robot never runs this; its ZED wrapper publishes /zed/zed_node/odom
directly.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

MAX_LINEAR = 3.0    # m/s, 10x Gerbil's top speed
MAX_ANGULAR = 5.0   # rad/s, 10x Gerbil's top yaw rate


class ZedOdomSimRelay(Node):

    def __init__(self):
        super().__init__('zed_odom_sim_relay')
        self.pub = self.create_publisher(Odometry, '/zed/zed_node/odom', 10)
        self.create_subscription(Odometry, '/sim/zed_odom_raw', self.on_odom, 10)
        self.dropped = 0

    def on_odom(self, msg):
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular
        speed = math.hypot(lin.x, lin.y)
        if speed > MAX_LINEAR or abs(ang.z) > MAX_ANGULAR:
            self.dropped += 1
            self.get_logger().warn(
                f'dropped impossible VIO sample (|v| {speed:.1f} m/s, wz {ang.z:.1f} rad/s), '
                f'{self.dropped} so far. Expected once at spawn.')
            return
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ZedOdomSimRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
