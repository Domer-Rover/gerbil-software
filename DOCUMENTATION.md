# Gerbil Robot Documentation

## Jetson Serial Port Setup

Configure UART for RoboClaw motor controller communication.

```bash
# Check if port exists
ls -l /dev/ttyTHS1

# Stop serial console service
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty

# Add user to dialout group
sudo usermod -a -G dialout $USER

# Reboot to apply group changes
sudo reboot

# Or set permissions temporarily for current session
sudo chmod 666 /dev/ttyTHS1
```

---

## Docker Container Usage

### Build Container Images

```bash
# Development image (Mac/Windows with VNC)
docker-compose build ros-dev

# Jetson image (headless)
docker-compose build ros-jetson
```

### Run Containers

```bash
# Development container (foreground with logs)
docker-compose up ros-dev
# Access via browser: http://localhost:6080

# Jetson container (detached/background mode)
docker-compose up -d ros-jetson
# -d flag runs container in background without blocking terminal

# Enter running Jetson container
docker exec -it gerbil-jetson bash
```

### Stop Containers

```bash
# Stop specific service
docker-compose stop ros-dev

# Stop and remove containers
docker-compose down

# Force rebuild (no cache)
docker-compose build --no-cache ros-dev
```

---

## ZED2i Camera Setup

### Prerequisites

ZED SDK is automatically installed during Docker image build:
- Development image (amd64 only): x86_64 SDK with CUDA
- Jetson image: L4T SDK for Jetson boards
- Mac ARM64: Skipped (not supported by Stereolabs)

### Verify ZED Installation

```bash
# Inside container, check SDK version
/usr/local/zed/tools/ZED_Explorer

# Check if camera is detected (camera must be plugged in)
ls /dev/video*

# Test camera access
/usr/local/zed/tools/ZED_Diagnostic
```

### Build ZED ROS2 Wrapper

The wrapper is included as a git submodule and builds with the workspace:

```bash
cd ~/ros2_ws

# Build ZED packages
colcon build --packages-select zed_wrapper zed_components zed_ros2

# Source workspace
source install/setup.bash

# Verify packages installed
ros2 pkg list | grep zed
```

### Launch ZED Camera

```bash
# ZED2i camera node
ros2 launch zed_wrapper zed2i.launch.py

# Check published topics
ros2 topic list | grep zed

# View camera feed in RViz
rviz2
```

### ZED Topics

Common topics published by ZED wrapper:
- `/zed/zed_node/rgb/image_rect_color` - RGB image
- `/zed/zed_node/depth/depth_registered` - Depth map
- `/zed/zed_node/point_cloud/cloud_registered` - Point cloud
- `/zed/zed_node/odom` - Visual odometry
- `/zed/zed_node/pose` - Camera pose

---

## ROS2 Workspace Build

### Inside Container

```bash
# Navigate to workspace
cd ~/ros2_ws

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
colcon build --symlink-install

# Build specific packages
colcon build --packages-select gerbil_hw gerbil_description gerbil_bringup

# Build with release optimizations
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

---

## Launch Arguments

### Main Launch File

```bash
ros2 launch gerbil_bringup gerbil.launch.xml
```

### Available Arguments

| Argument | Default | Options | Description |
|----------|---------|---------|-------------|
| `use_mock_hardware` | `true` | `true`, `false` | Enable mock hardware for testing without physical robot |
| `launch_rviz` | `true` | `true`, `false` | Launch RViz visualization |
| `launch_imu` | `false` | `true`, `false` | Launch IMU node |

### Common Launch Commands

```bash
# Development: Mock hardware with RViz
ros2 launch gerbil_bringup gerbil.launch.xml use_mock_hardware:=true launch_rviz:=true

# Development: Mock hardware without RViz (headless)
ros2 launch gerbil_bringup gerbil.launch.xml use_mock_hardware:=true launch_rviz:=false

# Jetson: Real hardware without RViz
ros2 launch gerbil_bringup gerbil.launch.xml use_mock_hardware:=false launch_rviz:=false

# Jetson: Real hardware with IMU
ros2 launch gerbil_bringup gerbil.launch.xml use_mock_hardware:=false launch_imu:=true
```

---

## Teleoperation

```bash
# Standard teleop

# This will be SLOW but controllable
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_cont/cmd_vel_unstamped -p speed:=0.02 -p turn:=0.2

```

---

## Verification Commands

### Check Topics

```bash
# List all topics
ros2 topic list

# Check controller command topic
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped

# Check odometry
ros2 topic echo /diff_drive_controller/odom

# Check joint states
ros2 topic echo /joint_states
```

### Check Controllers

```bash
# List loaded controllers
ros2 control list_controllers

# Check controller status
ros2 control list_hardware_interfaces
```

### Check TF Tree

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo odom base_link
```

---

## Troubleshooting

### Controller Manager Not Ready

```bash
# Check if controller_manager is running
ros2 service list | grep controller_manager

# Manually spawn controllers
ros2 run controller_manager spawner diff_drive_controller
ros2 run controller_manager spawner joint_state_broadcaster
```

### Serial Port Permission Denied

```bash
# Check current permissions
ls -l /dev/ttyTHS1

# Fix temporarily
sudo chmod 666 /dev/ttyTHS1

# Fix permanently
sudo usermod -a -G dialout $USER
sudo reboot
```

### RoboClaw Communication Test

```bash
# Test script location
cd ~/ros2_ws/src/gerbil-software/scripts

# Run test
python3 test_roboclaw.py
```
