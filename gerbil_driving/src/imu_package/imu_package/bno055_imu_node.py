#!/usr/bin/env python3
import os

os.environ['BLINKA_FORCEGENERIC'] = '1'

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import time
import sys

class BNO055IMUNode(Node):
    def __init__(self):
        super().__init__("bno055_imu_node")
        
        # Declare parameters
        self.declare_parameter('i2c_bus', 7)          # Default to the bus we found
        self.declare_parameter('i2c_address', 0x28)   # Default to the address we found
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('frame_id', 'imu_link')
        
        # Get parameters
        self.i2c_bus_id = self.get_parameter('i2c_bus').value
        self.i2c_addr = self.get_parameter('i2c_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Create publisher
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        
        # Setup IMU connection
        self.imu = None
        self.setup_i2c_imu()
        
        # Create timer
        if self.imu:
            timer_period = 1.0 / self.publish_rate
            self.timer = self.create_timer(timer_period, self.publish_imu_data)
            self.get_logger().info(f"BNO055 IMU Node started on Bus {self.i2c_bus_id} at {self.publish_rate} Hz")
        else:
            self.get_logger().error("Could not start IMU node. Exiting.")
            raise SystemExit

    def setup_i2c_imu(self):
        """Setup using Adafruit CircuitPython library with Extended Bus"""
        try:
            # Dynamic imports to ensure libraries are installed
            from adafruit_extended_bus import ExtendedI2C as I2C
            import adafruit_bno055
            
            self.get_logger().info(f"Attempting to connect to I2C Bus {self.i2c_bus_id}...")
            
            # Initialize the specific I2C Bus (e.g., /dev/i2c-7)
            i2c = I2C(self.i2c_bus_id)
            
            # Initialize the sensor
            self.imu = adafruit_bno055.BNO055_I2C(i2c, address=self.i2c_addr)
            self.get_logger().info("BNO055 initialized successfully.")
            
        except ImportError:
            self.get_logger().error("Missing libraries! Run: pip3 install adafruit-circuitpython-bno055 adafruit-extended-bus")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize BNO055: {e}")
            self.imu = None

    def publish_imu_data(self):
        """Read and publish IMU data"""
        try:
            msg = Imu()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            
            # --- QUATERNION FIX ---
            # Adafruit library returns (w, x, y, z)
            # ROS message expects x, y, z, w
            quaternion = self.imu.quaternion
            if quaternion is not None:
                msg.orientation.w = quaternion[0] # W is index 0 in Adafruit
                msg.orientation.x = quaternion[1]
                msg.orientation.y = quaternion[2]
                msg.orientation.z = quaternion[3]
                msg.orientation_covariance[0] = 0.01 
            else:
                return # Skip publishing if data is bad

            # Get angular velocity (rad/s)
            gyro = self.imu.gyro
            if gyro is not None:
                msg.angular_velocity.x = gyro[0]
                msg.angular_velocity.y = gyro[1]
                msg.angular_velocity.z = gyro[2]
                msg.angular_velocity_covariance[0] = 0.01

            # Get linear acceleration (m/s^2)
            accel = self.imu.linear_acceleration
            if accel is not None:
                msg.linear_acceleration.x = accel[0]
                msg.linear_acceleration.y = accel[1]
                msg.linear_acceleration.z = accel[2]
                msg.linear_acceleration_covariance[0] = 0.01
            
            self.publisher_.publish(msg)

        except OSError:
            # I2C read errors happen occasionally, just log a warning and skip this frame
            self.get_logger().warning("I2C Read Error (skipping frame)")
        except Exception as e:
            self.get_logger().error(f"Error reading IMU data: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        imu_node = BNO055IMUNode()
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
