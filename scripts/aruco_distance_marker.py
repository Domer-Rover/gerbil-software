#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import math

class ArucoDistanceMarker(Node):
    def __init__(self):
        super().__init__('aruco_distance_marker')
        self.marker_pub = self.create_publisher(Marker, '/aruco_marker_distance', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco_marker_publisher/pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        # Compute distance from robot base to marker
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        dist = math.sqrt(x**2 + y**2 + z**2)

        marker = Marker()
        marker.header = msg.header
        marker.ns = 'aruco_distance'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z + 0.2  # Offset above marker
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.15
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"{dist:.2f} m"
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = ArucoDistanceMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
