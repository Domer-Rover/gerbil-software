#!/usr/bin/env python3
"""Lightweight ArUco marker detector using OpenCV.

Subscribes to the ZED left camera image and camera_info, detects ArUco
markers (DICT_6X6_250), and publishes:
  - /aruco/markers        (PoseArray of all detected markers)
  - /aruco/marker_distance (visualization_msgs/Marker with distance text)
  - /aruco/image_debug    (annotated image with drawn markers)
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('dictionary_id', 10)  # DICT_6X6_250
        self.declare_parameter('camera_frame', 'zed_left_camera_optical_frame')

        self.marker_size = self.get_parameter('marker_size').value
        dict_id = self.get_parameter('dictionary_id').value
        self.camera_frame = self.get_parameter('camera_frame').value

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscribers
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            CameraInfo,
            '/zed/zed_node/rgb/color/rect/camera_info',
            self._camera_info_cb,
            qos,
        )
        self.create_subscription(
            Image,
            '/zed/zed_node/rgb/color/rect/image',
            self._image_cb,
            qos,
        )

        # Publishers
        self.pose_array_pub = self.create_publisher(PoseArray, '/aruco/markers', 10)
        self.marker_vis_pub = self.create_publisher(Marker, '/aruco/marker_distance', 10)
        self.debug_img_pub = self.create_publisher(Image, '/aruco/image_debug', 5)

        self.get_logger().info(
            f'ArUco detector started (dict={dict_id}, size={self.marker_size}m)'
        )

    def _camera_info_cb(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera info received')

    def _image_cb(self, msg: Image):
        if self.camera_matrix is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = self.detector.detectMarkers(cv_image)

        pose_array = PoseArray()
        pose_array.header = msg.header
        pose_array.header.frame_id = self.camera_frame

        if ids is not None:
            # Estimate pose for each marker
            for i, marker_id in enumerate(ids.flatten()):
                obj_points = np.array([
                    [-self.marker_size / 2,  self.marker_size / 2, 0],
                    [ self.marker_size / 2,  self.marker_size / 2, 0],
                    [ self.marker_size / 2, -self.marker_size / 2, 0],
                    [-self.marker_size / 2, -self.marker_size / 2, 0],
                ], dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    obj_points, corners[i][0], self.camera_matrix, self.dist_coeffs
                )
                if not success:
                    continue

                # Convert rotation vector to quaternion
                rot_mat, _ = cv2.Rodrigues(rvec)
                quat = self._rotation_matrix_to_quaternion(rot_mat)

                pose = Pose()
                pose.position.x = float(tvec[0])
                pose.position.y = float(tvec[1])
                pose.position.z = float(tvec[2])
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]
                pose_array.poses.append(pose)

                # Distance visualization marker
                dist = math.sqrt(tvec[0] ** 2 + tvec[1] ** 2 + tvec[2] ** 2)
                vis = Marker()
                vis.header = msg.header
                vis.header.frame_id = self.camera_frame
                vis.ns = 'aruco_distance'
                vis.id = int(marker_id)
                vis.type = Marker.TEXT_VIEW_FACING
                vis.action = Marker.ADD
                vis.pose = pose
                vis.pose.position.z = float(tvec[2]) + 0.1
                vis.scale.z = 0.08
                vis.color.r = 0.0
                vis.color.g = 1.0
                vis.color.b = 1.0
                vis.color.a = 1.0
                vis.text = f'ID:{marker_id} {dist:.2f}m'
                vis.lifetime.sec = 0
                vis.lifetime.nanosec = 500000000  # 0.5s
                self.marker_vis_pub.publish(vis)

            # Draw markers on debug image
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        self.pose_array_pub.publish(pose_array)

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_img_pub.publish(debug_msg)

    @staticmethod
    def _rotation_matrix_to_quaternion(R):
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return (x, y, z, w)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
