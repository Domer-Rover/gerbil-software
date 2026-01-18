#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class IMUTestSubscriber(Node):
    def __init__(self):
        super().__init__('imu_test_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        self.get_logger().info("IMU Test Subscriber started - listening to /imu/data")
        
    def imu_callback(self, msg):
        """Display IMU data in a readable format"""
        
        # Linear acceleration
        lin_acc = msg.linear_acceleration
        acc_magnitude = math.sqrt(lin_acc.x**2 + lin_acc.y**2 + lin_acc.z**2)
        
        # Angular velocity
        ang_vel = msg.angular_velocity
        ang_magnitude = math.sqrt(ang_vel.x**2 + ang_vel.y**2 + ang_vel.z**2)
        
        # Orientation (if available)
        orient = msg.orientation
        
        self.get_logger().info(
            f"\n"
            f"=== IMU Data ===\n"
            f"Linear Acceleration (m/s²):\n"
            f"  X: {lin_acc.x:7.3f}  Y: {lin_acc.y:7.3f}  Z: {lin_acc.z:7.3f}\n"
            f"  Magnitude: {acc_magnitude:7.3f}\n"
            f"Angular Velocity (rad/s):\n"
            f"  X: {ang_vel.x:7.3f}  Y: {ang_vel.y:7.3f}  Z: {ang_vel.z:7.3f}\n"
            f"  Magnitude: {ang_magnitude:7.3f}\n"
            f"Orientation (quaternion):\n"
            f"  X: {orient.x:7.3f}  Y: {orient.y:7.3f}  Z: {orient.z:7.3f}  W: {orient.w:7.3f}\n"
            f"================"
        )

def main(args=None):
    rclpy.init(args=args)
    subscriber = IMUTestSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
