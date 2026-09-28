# gerbil-software (ROS 2)

Software for **Gerbil**, the Domer Rover two-wheel test robot: ROS 2 Humble on a
Jetson, ZED2i, BNO055 IMU, one RoboClaw driving both wheels with encoders.

Gerbil is the indoor testbed for the Capybara rover's navigation stack: same
packages, same launch layout, smaller and easier to carry upstairs. Nav2 tuning
proven here gets ported to [capybara-software](https://github.com/Domer-Rover/capybara-software).

## Quick start

```bash
cd ~/gerbil-software
colcon build --symlink-install && source install/setup.bash
ros2 launch gerbil_bringup gerbil_foxglove.launch.py use_joystick:=true
```

## Layout

| Path | Contents |
|---|---|
| `src/gerbil_bringup` | Launch files, Nav2/SLAM/EKF/controller configs |
| `src/gerbil_description` | URDF and ros2_control block |
| `src/gerbil_hw` | RoboClaw ros2_control hardware interface |
| `src/imu_package` | BNO055 IMU driver |
| `src/vendors` | ZED ROS 2 wrapper, roboclaw_serial |
| `scripts` | Dev onboarding and hardware test scripts |
| `udev` | Stable serial device names |

## Docs

- [Jetson setup](docs/jetson-setup.md): accounts, SSH keys, serial devices
- [Build and run](docs/build-and-run.md): build, launch, closed-loop encoders, Nav2
