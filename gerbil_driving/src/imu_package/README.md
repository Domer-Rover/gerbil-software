# BNO055 IMU ROS2 Package

ROS2 package for the Bosch BNO055 9-DOF IMU sensor on Jetson.

## Features
- Publishes sensor_msgs/Imu messages
- Supports both I2C (Adafruit library) and Serial connection
- Configurable publish rate
- Includes test subscriber for verification
- Launch files and parameter configuration

## Dependencies

Install the Adafruit BNO055 library (optional, for I2C):
```bash
pip3 install adafruit-circuitpython-bno055
```

Install pyserial (for serial connection):
```bash
pip3 install pyserial
```

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select imu_package
source install/setup.bash
```

## Usage

### Launch the IMU node:
```bash
ros2 launch imu_package bno055_imu.launch.py
```

Or with custom serial port:
```bash
ros2 launch imu_package bno055_imu.launch.py serial_port:=/dev/ttyUSB0
```

### Run the test subscriber:
```bash
ros2 run imu_package imu_test_sub
```

### View topics:
```bash
ros2 topic list
ros2 topic echo /imu/data
ros2 topic hz /imu/data
```

## Configuration

Edit `config/bno055_params.yaml` to configure:
- Serial port
- Baud rate
- Publishing rate
- Frame ID
- I2C vs Serial mode

## Topics

- `/imu/data` (sensor_msgs/Imu) - IMU data with linear acceleration, angular velocity, and orientation

## Notes on EKF

ROS2 includes `robot_localization` package with an Extended Kalman Filter (EKF) that can fuse IMU data with other sensors. To use it:

```bash
sudo apt install ros-humble-robot-localization
```

The BNO055 has a built-in sensor fusion algorithm that provides orientation quaternions directly, but you can also use ROS2's `robot_localization` for additional filtering or fusing with other sensors like wheel odometry.

## Troubleshooting

### Finding the IMU serial port:
```bash
ls -l /dev/ttyACM*
ls -l /dev/ttyUSB*
```

### Check IMU connection:
```bash
sudo dmesg | grep tty
```

### Give permission to serial port:
```bash
sudo chmod 666 /dev/ttyACM1
```
