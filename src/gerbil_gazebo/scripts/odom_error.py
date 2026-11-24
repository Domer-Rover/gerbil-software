#!/usr/bin/env python3
"""Grade Gerbil's odometry against Gazebo ground truth.

  ros2 run gerbil_gazebo odom_error.py
  ros2 run gerbil_gazebo odom_error.py --ros-args -p estimate_topic:=/diff_drive_controller/odom

Subscribes to the estimate (default /odometry/filtered, the EKF output Nav2
uses) and to /sim/ground_truth/odom. The two start in different frames (the
EKF starts at 0,0,0 in odom wherever the robot is, the truth is in the world
frame), so both are re-expressed relative to an origin pose taken the moment
BOTH topics have reported: the first message of the later topic, paired with
the latest message of the earlier one. Leave the robot still until the first
report prints, or that pairing absorbs any motion in between.

Every `period` seconds (default 2) it prints position error, yaw error and
the position error as a percentage of the distance actually travelled (path
length of the truth, not straight-line). On Ctrl-C it prints a final line.

It also publishes /sim/ground_truth/odom_aligned: the true pose re-expressed
in the ESTIMATE's frame (odom), starting where the estimate started. Gazebo's
TF is never bridged, so RViz cannot place the raw truth in odom; this topic
can be drawn next to /odometry/filtered and the gap between the two arrows IS
the odometry drift. sim.launch.py runs this node for that reason.

Planar only: z, roll and pitch are ignored, as they are by the EKF
(two_d_mode). Sim-only tool; nothing on the robot depends on it.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _pose2d(msg):
    p = msg.pose.pose.position
    return (p.x, p.y, _yaw(msg.pose.pose.orientation))


def _compose(origin, rel):
    """Inverse of _relative: place rel (in origin's frame) back in the parent."""
    ox, oy, oyaw = origin
    c, s = math.cos(oyaw), math.sin(oyaw)
    return (ox + c * rel[0] - s * rel[1], oy + s * rel[0] + c * rel[1], _wrap(oyaw + rel[2]))


def _relative(origin, pose):
    """pose expressed in the frame of origin (both x, y, yaw)."""
    ox, oy, oyaw = origin
    dx, dy = pose[0] - ox, pose[1] - oy
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    return (c * dx - s * dy, s * dx + c * dy, _wrap(pose[2] - oyaw))


class OdomError(Node):

    def __init__(self):
        super().__init__('odom_error')
        self.estimate_topic = self.declare_parameter(
            'estimate_topic', '/odometry/filtered').value
        self.truth_topic = self.declare_parameter(
            'truth_topic', '/sim/ground_truth/odom').value
        aligned_topic = self.declare_parameter(
            'aligned_topic', '/sim/ground_truth/odom_aligned').value
        period = float(self.declare_parameter('period', 2.0).value)
        self.est_frame = 'odom'  # replaced by the estimate's own frame_id

        self.est = None          # latest (x, y, yaw), raw
        self.truth = None
        self.est_origin = None   # alignment origins, set once
        self.truth_origin = None
        self.distance = 0.0      # truth path length since alignment
        self.last_truth_rel = None
        self.max_pos_err = 0.0

        self.create_subscription(
            Odometry, self.estimate_topic, self._on_est, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.truth_topic, self._on_truth, qos_profile_sensor_data)
        self.aligned_pub = self.create_publisher(Odometry, aligned_topic, 10)
        self.create_timer(period, self._report)
        self.get_logger().info(
            f'comparing {self.estimate_topic} against {self.truth_topic}; '
            'waiting for both')

    def _try_align(self):
        if self.est_origin is None and self.est is not None and self.truth is not None:
            self.est_origin = self.est
            self.truth_origin = self.truth
            self.last_truth_rel = (0.0, 0.0, 0.0)
            self.get_logger().info('both topics live, origins aligned. Drive.')

    def _on_est(self, msg):
        self.est = _pose2d(msg)
        self.est_frame = msg.header.frame_id or self.est_frame
        self._try_align()

    def _on_truth(self, msg):
        self.truth = _pose2d(msg)
        self._try_align()
        if self.truth_origin is not None:
            rel = _relative(self.truth_origin, self.truth)
            self.distance += math.hypot(rel[0] - self.last_truth_rel[0],
                                        rel[1] - self.last_truth_rel[1])
            self.last_truth_rel = rel
            self._publish_aligned(msg, rel)

    def _publish_aligned(self, msg, rel):
        x, y, yaw = _compose(self.est_origin, rel)
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.est_frame
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Body-frame twist is frame-independent in the plane, copy it as is.
        out.twist = msg.twist
        self.aligned_pub.publish(out)

    def errors(self):
        """(est_rel, truth_rel, pos_err, yaw_err) or None before alignment."""
        if self.est_origin is None:
            return None
        e = _relative(self.est_origin, self.est)
        t = _relative(self.truth_origin, self.truth)
        pos_err = math.hypot(e[0] - t[0], e[1] - t[1])
        yaw_err = _wrap(e[2] - t[2])
        self.max_pos_err = max(self.max_pos_err, pos_err)
        return e, t, pos_err, yaw_err

    def line(self):
        r = self.errors()
        if r is None:
            missing = [name for name, v in ((self.estimate_topic, self.est),
                                            (self.truth_topic, self.truth)) if v is None]
            return f'waiting for {", ".join(missing)}'
        e, t, pos_err, yaw_err = r
        pct = (f'{100.0 * pos_err / self.distance:6.2f}% of {self.distance:.2f} m'
               if self.distance > 0.05 else f'  n/a (travelled {self.distance:.2f} m)')
        return (f'pos err {pos_err:6.3f} m  ({pct})  '
                f'yaw err {math.degrees(yaw_err):+7.2f} deg  '
                f'max pos err {self.max_pos_err:.3f} m  |  '
                f'est ({e[0]:+.2f}, {e[1]:+.2f})  truth ({t[0]:+.2f}, {t[1]:+.2f})')

    def _report(self):
        self.get_logger().info(self.line())


def main():
    rclpy.init()
    node = OdomError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Printed, not logged: the logger may already be torn down by Ctrl-C.
        print(f'[odom_error] FINAL  {node.line()}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
