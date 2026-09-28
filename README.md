<h1 align="center">
  gerbil-software
  <br>
</h1>

<p align="center">
  Software stack for <b>Gerbil</b>, the two-wheel test robot built by <a href="https://github.com/Domer-Rover">Domer Rover</a> for developing the University Rover Challenge navigation stack indoors.
  <br />
  Built on <b>ROS 2 Humble</b> on a Jetson, with a ZED2i camera, LD19 LIDAR, u-blox GPS, and a RoboClaw driving both wheels with encoders.
</p>

<p align="center">
  <a href="https://github.com/Domer-Rover/gerbil-software/blob/main/LICENSE"><img src="https://img.shields.io/badge/license-MIT-blue" alt="License"></a>
  <img src="https://img.shields.io/badge/Software%20Lead-Brandon%20Martinez-C1E1C1" alt="Software Lead">
  <img src="https://img.shields.io/badge/CI-In%20Progress-yellow" alt="CI Status">
</p>

<p align="center">
  <a href="https://github.com/Domer-Rover">Domer Rover</a>
  ·
  <a href="https://github.com/Domer-Rover/gerbil-software/tree/main/docs">Documentation</a>
  ·
  <a href="https://github.com/Domer-Rover/capybara-software">capybara-software</a>
  ·
  <a href="https://github.com/Domer-Rover/gerbil-software/issues">Report an Issue</a>
</p>

---

## Overview

Gerbil is the indoor testbed for [Capybara](https://github.com/Domer-Rover/capybara-software), Domer Rover's **University Rover Challenge (URC)** entry. Same packages, same launch layout, same hardware interface — a smaller robot that is easier to carry, plug into, and drive around a hallway. Navigation tuning proven on Gerbil is ported to Capybara.

Unlike Capybara, Gerbil has wheel encoders, so it runs closed-loop velocity control and fuses wheel odometry with ZED visual odometry.

## Directory Structure

| Path | Description |
| --- | --- |
| `src/gerbil_bringup` | Launch files, Nav2/SLAM/EKF/controller configs |
| `src/gerbil_description` | URDF and `ros2_control` block |
| `src/gerbil_hw` | RoboClaw hardware interface for `ros2_control` |
| `src/imu_package` | BNO055 driver (unused; the ZED2i IMU is used) |
| `src/vendors` | ZED ROS 2 wrapper, `roboclaw_serial`, `ldlidar_stl_ros2` |
| `scripts` | Developer onboarding and hardware test scripts |
| `udev` | Stable serial device names |
| `docs` | Setup and usage guides |

## Getting Started

Fresh Jetson? Work through [docs/first-setup.md](docs/first-setup.md) — device
names, motor safety, encoders, LIDAR check, then Nav2. Accounts and SSH keys are
in [Jetson setup](docs/jetson-setup.md).

```bash
ssh <username>@<jetson>
cd ~/gerbil-software
colcon build --symlink-install && source install/setup.bash
ros2 launch gerbil_bringup gerbil_foxglove.launch.py use_joystick:=true
```

Connect Foxglove to `ws://<jetson>:8765`. See [Build and run](docs/build-and-run.md) for the other launch files, the closed-loop encoder procedure, and checks.

## Built With

- **ROS 2 Humble**: middleware for every package in this repo
- **ros2_control**: hardware abstraction and control
- **Nav2**: autonomous navigation
- **robot_localization**: wheel + visual odometry fusion
- **ZED SDK**: visual-inertial odometry

## About Domer Rover

[Domer Rover](https://github.com/Domer-Rover) is the University of Notre Dame's rover team, competing at the University Rover Challenge (URC). This repository is maintained by the team's software subgroup.

## Contributing

Bug reports and pull requests are welcome. Check the `.github` folder for the PR template, and open an [issue](https://github.com/Domer-Rover/gerbil-software/issues) if you run into a problem.

## License

This project is licensed under the [MIT License](https://github.com/Domer-Rover/gerbil-software/blob/main/LICENSE).
