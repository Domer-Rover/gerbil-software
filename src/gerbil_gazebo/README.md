# gerbil_gazebo

Gazebo **Fortress** (Ignition, the Humble pairing) simulation of Gerbil. It runs in the
`gerbil-sim` noVNC container on Apple-Silicon Macs with software rendering. See `docs/simulation.md`.

## Interface contract

All sim work is built against this contract. Change it only in a deliberate, reviewed commit.

### Robot description
- xacro arg `sim_mode` takes `none` or `gazebo` and defaults to `none`.
  - `none` behaves exactly as before: `use_mock_hardware` picks `mock_components` or `gerbil_hw`.
  - `gazebo` uses `ign_ros2_control/IgnitionSystem` as the ros2_control hardware, and
    `gerbil_description/urdf/gerbil.gazebo.xacro` is included, which carries the plugins, sensors and
    friction.
- Gazebo mode adds no links or joints that the real robot lacks, so the TF tree is identical.

### Topics (sim publishes exactly what the robot publishes)
| Topic | Type | Source in sim |
|---|---|---|
| `/clock` | rosgraph_msgs/Clock | ros_gz_bridge |
| `/scan_raw` | sensor_msgs/LaserScan, frame `laser_frame` | gpu_lidar to bridge, then the **existing** laser_filter produces `/scan` |
| `/zed/zed_node/odom` | nav_msgs/Odometry | VIO stand-in (gz OdometryPublisher plus noise), ~25 Hz |
| `/zed/zed_node/point_cloud/cloud_registered` | sensor_msgs/PointCloud2 | optional depth camera, off by default |
| `/diff_drive_controller/odom` | nav_msgs/Odometry | the real diff_drive_controller, via ign_ros2_control |
| `/sim/ground_truth/odom` | nav_msgs/Odometry | true pose, for evaluation **only**, never fused |

### Gazebo-side topic names (set in `gerbil.gazebo.xacro`, bridged in `config/bridge.yaml`)
| gz topic | gz type | bridged to ROS |
|---|---|---|
| `/clock` (world) | ignition.msgs.Clock | `/clock` |
| `/scan_raw` | ignition.msgs.LaserScan | `/scan_raw` |
| `/zed/zed_node/odom` | ignition.msgs.Odometry | `/zed/zed_node/odom` |
| `/sim/ground_truth/odom` | ignition.msgs.Odometry | `/sim/ground_truth/odom` |
| `/sim/zed_depth/points` (optional) | ignition.msgs.PointCloudPacked | `/zed/zed_node/point_cloud/cloud_registered` |

Gazebo's own TF topics (e.g. `/model/gerbil/tf`) are **never** bridged.
The spawned model is named `gerbil`.

### World registry: `config/worlds.yaml` (owned by the worlds workstream)
```yaml
school_loop:              # key == launch world:= value == <world name=...> in the SDF
  file: school_loop.sdf   # relative to share/gerbil_gazebo/worlds
  spawn: {x: 0.0, y: 0.0, z: 0.05, yaw: 0.0}
```
The world SDF must load these systems: Physics, UserCommands, SceneBroadcaster, and Sensors with
`render_engine ogre2`. The robot does **not** load them.

### Frames
`odom -> base_footprint` is published by the EKF only. Below that are `base_link`, then `laser_frame`
(yaw -pi/2, unchanged, so the laser_filter.yaml bounds stay valid) and `zed_camera_link`.
In sim, the ZED-stand-in odom message uses `child_frame_id: zed_camera_link`, matching the real
wrapper.

### Time
Every node runs with `use_sim_time: true` in sim. The real-robot launch defaults are unchanged
(`use_sim_time` defaults to false).

### Launch
`ros2 launch gerbil_gazebo sim.launch.py world:=school_loop|mdrs_utah|indiana_dunes rviz:=true nav2:=true`

### Worlds
| World | What |
|---|---|
| `school_loop` | Rectangular ring corridor, 3.66 m (12 ft) wide, 40 x 25 m outer by default |
| `mdrs_utah` | URC site terrain (38.40547, -110.79002) from a USGS DEM heightmap |
| `indiana_dunes` | Dune/beach terrain patch (stretch) |

Drop an obstacle live with `ros2 run gerbil_gazebo spawn_obstacle.sh <box|cone|trash_can|block> x y`.
