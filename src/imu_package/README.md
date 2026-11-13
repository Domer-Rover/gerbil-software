# BNO055 IMU ROS2 Package

ROS2 package for the Bosch BNO055 9-DOF IMU sensor on Jetson.

## Features
- Publishes sensor_msgs/Imu messages
- Connects over I2C (Adafruit CircuitPython library); there is no serial mode
- Configurable publish rate
- Includes test subscriber for verification
- Launch files and parameter configuration

## Dependencies

Install the Adafruit BNO055 and extended-bus libraries:
```bash
pip3 install adafruit-circuitpython-bno055 adafruit-extended-bus
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

Or as part of the robot bringup: `launch_imu:=true` on any `gerbil_bringup`
launch file.

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
- I2C bus (`i2c_bus`, default 7, i.e. `/dev/i2c-7`)
- I2C address (`i2c_address`, default 0x28)
- Publishing rate
- Frame ID

## Topics

- `/imu/data` (sensor_msgs/Imu) - IMU data with linear acceleration, angular velocity, and orientation

## Notes on EKF

ROS2 includes `robot_localization` package with an Extended Kalman Filter (EKF) that can fuse IMU data with other sensors. To use it:

```bash
sudo apt install ros-humble-robot-localization
```

The BNO055 has a built-in sensor fusion algorithm that provides orientation quaternions directly, but you can also use ROS2's `robot_localization` for additional filtering or fusing with other sensors like wheel odometry.

Gerbil's EKF does not fuse it yet: the node fills only the first covariance
entry (see `gerbil_bringup/config/ekf.yaml`).

## Troubleshooting

### Find the IMU on the I2C bus:
```bash
sudo i2cdetect -y -r 7      # expect 28 (or 29 if ADR is pulled high)
```

### Permission to the I2C bus:
The user must be in the `i2c` group (`id -nG`); `scripts/add_dev_user.sh`
adds it.
