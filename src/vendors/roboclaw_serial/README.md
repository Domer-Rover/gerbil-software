# RoboClaw Serial

A header-only C++ interface for the BasicMicro RoboClaw (packet serial).

In this workspace it is consumed by the ROS 2 `roboclaw_hardware_interface` package.

## Build (colcon)

```bash
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select roboclaw_serial
source install/setup.bash
```

## Test

```bash
cd ~/ros2_ws
colcon test --packages-select roboclaw_serial
colcon test-result --verbose
```
