# Vendored Dependencies

Third-party ROS2 packages vendored for build consistency and offline development.

## Packages

### roboclaw_serial
- Source: Custom implementation
- Purpose: Serial communication for BasicMicro RoboClaw motor controllers
- License: See package LICENSE

### zed_wrapper
- Source: [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
- Version: Vendored Feb 4, 2024
- Purpose: ROS2 wrapper for ZED2i camera
- License: See package LICENSE
- Contains: `zed_components`, `zed_wrapper`, `zed_debug`, `zed_ros2`

### ldlidar_stl_ros2
- Source: [ldrobotSensorTeam/ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2)
- Version: vendored 2026-09-28 (upstream bf668a8)
- Purpose: driver for the LD19 LIDAR (configured as LDLiDAR_LD06 — same sensor core)
- License: See package LICENSE
- Not in the ROS index, which is why it is vendored rather than installed with rosdep

## Updating

```bash
cd /tmp
git clone <repo-url> <package>
cd <package>
git checkout <commit-or-tag>
rm -rf .git*

cp -r /tmp/<package> /path/to/gerbil-software/src/vendors/
cd /path/to/gerbil-software
colcon build --packages-select <package>

git add src/vendors/<package>
git commit -m "vendor: Update <package> to version X"
```
