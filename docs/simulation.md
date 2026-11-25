# Simulation (Gazebo Fortress, in a browser)

This is the guide for running Gerbil in simulation on an Apple-Silicon Mac. It
assumes no ROS or Gazebo experience. Every command can be copied as is.

> **Status: first-run checks pending.** The sim has been written and reviewed
> but has not yet run end to end in Gazebo. Nothing on this page is a measured
> number from the sim. Where the page says "you should see", that is what the
> code is designed to do. If you see something different, it is a bug: report
> it with the terminal output.

## 1. What this is

One command starts Gazebo **Fortress** with Gerbil in it, and runs the **real**
robot software on top:

- the real URDF (`gerbil_description`), with a Gazebo-only add-on
  (`urdf/gerbil.gazebo.xacro`) for the sensors and plugins
- the real `diff_drive_controller` and `joint_state_broadcaster`, through
  ros2_control
- the real EKF (`ekf.yaml`), twist_mux, laser_filters and Nav2
  (`nav2_params.yaml`), loaded from `src/gerbil_bringup/config/`

`src/gerbil_gazebo/launch/sim.launch.py` includes the robot's own
`gerbil_nav2.launch.py` and changes only the sim switches, such as
`use_sim:=true` and `use_controllers:=false`. A tuning change you make in
`gerbil_bringup/config/` therefore applies in sim and on the robot alike.

All of this runs in a Docker container (`gerbil-sim`). The desktop is shown in
your browser through noVNC. There is no GPU, so everything is drawn by the CPU
(Mesa llvmpipe).

What is simulated and how:

| Real part | In sim | Notes |
|---|---|---|
| RoboClaw and motors | `ign_ros2_control` drives the two wheel joints | Same controller and config as the robot |
| LD19 LiDAR | `gpu_lidar` sensor, 455 rays, 10 Hz, 12 m, 1 cm noise | Publishes `/scan_raw`. The real laser_filters chain turns it into `/scan` |
| ZED2i visual odometry | Gazebo `OdometryPublisher` plus white noise (0.01 per twist component), 25 Hz, on `/zed/zed_node/odom` | A **stand-in**: no cameras, no tracking loss, no scale drift |
| ZED2i depth | Off by default. `use_sim_depth:=true` adds a 320x180, 5 Hz depth cloud | Costs CPU |
| GPS | Not simulated | |
| Ground truth | `/sim/ground_truth/odom` | **For grading only.** Nothing may fuse it |

## 2. Setup (once)

### Docker Desktop

1. Install Docker Desktop for Mac (Apple Silicon).
2. Settings > Resources: **Memory 8 GB or more**. CPUs: as many as you can
   spare, at least 4. Software rendering is CPU-bound, so more cores give a
   faster sim. Apply and restart.

### Build and start the container

From the repo root on your Mac:

```bash
docker compose build gerbil-sim          # first build downloads several GB
docker compose up -d gerbil-sim
```

`ros-dev` uses port 6080 too. If it is running, stop it first with
`docker compose stop ros-dev`.

### Open the desktop and build the workspace

1. Open <http://localhost:6080> and log in. The VNC password is `ubuntu`.
2. Double-click the **Gerbil Workspace** icon on the desktop. It opens a
   terminal in `~/ros2_ws`.
3. Build:

   ```bash
   gerbil-build
   source install/setup.bash
   ```

`gerbil-build` runs `colcon build --symlink-install` and skips the packages that
need real hardware or the ZED SDK. The list is in
`docker/sim/colcon_defaults.yaml`. New terminals source the workspace
automatically. Only the terminal you built in needs the `source` line.

Your repo's `src/` is mounted into the container, so you edit files on the Mac
as usual. Because of `--symlink-install`, YAML and launch edits take effect on
the next launch without a rebuild. A new file or a `CMakeLists.txt` change
needs `gerbil-build` again.

To get a shell from the Mac terminal instead, run:
`docker exec -it -u ubuntu gerbil-sim bash`. Use `-u ubuntu`, or GUI apps
cannot reach the VNC display.

**After a change to `docker/Dockerfile.sim` or the dependencies**, rebuild the
image and wipe the build volumes, which hold the old build:

```bash
docker compose down -v
docker compose build gerbil-sim && docker compose up -d gerbil-sim
```

Then run `gerbil-build` again inside the container.

## 3. First run

```bash
ros2 launch gerbil_gazebo sim.launch.py world:=school_loop
```

What should happen, in order:

1. A **Gazebo** window opens with a corridor (blue outer walls, brick inner
   walls) and the rover on a yellow start line.
2. An **RViz** window opens in the `odom` frame. It shows the robot model, the
   LiDAR scan, the local and global costmaps, and the EKF odometry arrow.
3. The terminal prints `Configured and activated diff_drive_controller`. The
   sim is ready from that point. Before it, there are no wheels to drive and
   no `odom` frame, so Nav2 warnings about a missing transform are normal.
4. Every 2 s an `odom_error` line prints. See [section 7](#7-grading-odometry).

Stop everything with Ctrl-C in the launch terminal. Closing the Gazebo window
stops the whole launch too.

### Launch arguments

From `src/gerbil_gazebo/launch/sim.launch.py`. Pass them as `name:=value`.

| Argument | Default | Meaning |
|---|---|---|
| `world` | `school_loop` | World key in `config/worlds.yaml`: `school_loop`, `mdrs_utah` or `indiana_dunes` |
| `use_nav2` | `true` | Run Nav2 on top of the base bringup. `false` gives driving only |
| `use_rviz` | `true` | RViz with `rviz/sim.rviz`. Needs a display |
| `use_foxglove` | `false` | foxglove_bridge on port 8765 (`ws://localhost:8765` from the Mac) |
| `use_slam` | `false` | slam_toolbox with the robot's config, building `/map` alongside. Nav2 still plans in `odom` |
| `use_sim_depth` | `false` | Simulated ZED depth cloud on `/zed/zed_node/point_cloud/cloud_registered`. Costs CPU |
| `use_gui` | `true` | `false` runs the Gazebo server only (`ign gazebo -s`), with no Gazebo window |
| `use_headless` | `false` | With `use_gui:=false` and **no** X display at all: render the sensors through EGL. Only for CI. Not needed in the container |
| `use_odom_error` | `true` | Run `odom_error.py`: EKF vs ground truth every 2 s, plus the aligned truth arrow in RViz |
| `gz_args` | (empty) | Extra args for `ign gazebo`, e.g. `gz_args:="-v 4"` to debug plugin loading |

## 4. Driving by hand

In a **second** terminal (the launch must keep running):

```bash
ros2 run gerbil_bringup teleop_keyboard.sh
```

That script is the one the robot uses. It runs:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel_key -p speed:=0.3 -p turn:=1.0
```

Keys: `i` forward, `,` back, `j`/`l` turn, `k` stop. The terminal must have
keyboard focus.

Commands go to `/cmd_vel_key`. **twist_mux** (`config/twist_mux.yaml`) forwards
the highest-priority live input to the controller: joystick 100, keyboard 90,
Nav2 10. So the keyboard overrides Nav2 while you press keys. About 0.5 s after
you let go, Nav2 takes over again. Letting go **does not cancel** a goal.

The controller clamps turning to 0.5 rad/s (`gerbil_controllers.yaml`), so
`turn:=1.0` gives 0.5.

## 5. Sending goals

### Why goals are in `odom`, relative to where the robot started

Gerbil navigates **odom-only**: no map and no AMCL. Both costmaps and the
planner work in the `odom` frame. The EKF starts `odom` at (0, 0, 0) wherever
the robot is at boot, which in sim is the spawn pose. So:

- a goal at `x: 3.0` in `odom` means "3 m ahead of where the robot started,
  facing the way it started"
- odom drifts, so a goal is only as accurate as the odometry
- the Gazebo world coordinates (used by `spawn_obstacle.sh`) are **not** odom
  coordinates. For `school_loop`, see the conversion in
  [section 6](#6-obstacles)

### RViz

Click **2D Goal Pose** in the RViz toolbar. Then click on the map where the
robot should go and drag to set the heading. RViz is already in the `odom`
frame.

### Command line: a 3 m goal

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

It should end with `Goal finished with status: SUCCEEDED`. Ctrl-C cancels the
goal.

The fire-and-forget version from the launch banner does the same, but gives no
result:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 3.0}, orientation: {w: 1.0}}}"
```

### Around the first corner of `school_loop`

The geometry, from the SDF header:

- The corridor centreline is a 36.34 x 21.34 m rectangle centred on the world
  origin.
- The robot spawns at world (0, -10.67), facing +x, in the middle of the south
  corridor. Driving forward laps counter-clockwise.
- The spawn yaw is 0, so odom = world shifted by the spawn: **odom x = world
  x, odom y = world y + 10.67**.
- The first corner (centre of the south-east corner square) is at world
  (18.17, -10.67), which is **odom (18.17, 0)**: about 18 m straight ahead,
  then the corridor turns left (+y).

**Known limit: a goal more than about 10 m away fails.** The global costmap is
a 20 x 20 m window that moves with the robot (`nav2_params.yaml`,
`global_costmap`). A goal outside it is outside the planner's grid, and
planning fails with an unhelpful error. This is a known Stage-1 review red
flag: see [exercise 9b](#9-tuning-exercises).

Until that is fixed, send the corner as **hops of less than 10 m**, each one
after the last succeeds:

```bash
G='ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose'
$G "{pose: {header: {frame_id: odom}, pose: {position: {x: 9.0, y: 0.0}, orientation: {w: 1.0}}}}"
$G "{pose: {header: {frame_id: odom}, pose: {position: {x: 18.17, y: 0.0}, orientation: {z: 0.7071, w: 0.7071}}}}"
$G "{pose: {header: {frame_id: odom}, pose: {position: {x: 18.17, y: 6.0}, orientation: {z: 0.7071, w: 0.7071}}}}"
```

The last two face +y (90 degrees left), up the east corridor.

`/navigate_through_poses` is also running. Its default tree plans one path
through **all** the poses at once, against the same 20 x 20 m costmap. So it
probably fails the same way when any pose is more than 10 m from the robot.
That is untested. To check it:

```bash
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
  "{poses: [{header: {frame_id: odom}, pose: {position: {x: 9.0}, orientation: {w: 1.0}}}, {header: {frame_id: odom}, pose: {position: {x: 18.17}, orientation: {z: 0.7071, w: 0.7071}}}]}"
```

## 6. Obstacles

Drop a static obstacle into the running world:

```bash
ros2 run gerbil_gazebo spawn_obstacle.sh <box|cone|trash_can|block> <x> <y> [yaw] [name]
```

**x and y are Gazebo world coordinates, not odom.** In `school_loop` the
origin is the centre of the classroom block, which is solid, so `box 5 0` would
land inside the block. To put an obstacle at odom (ox, oy) in `school_loop`,
use world (ox, oy - 10.67):

```bash
ros2 run gerbil_gazebo spawn_obstacle.sh box 5 -10.67        # 5 m straight ahead of the spawn
ros2 run gerbil_gazebo spawn_obstacle.sh cone 7 -10.0        # 7 m ahead, 0.67 m left of centre
ros2 run gerbil_gazebo spawn_obstacle.sh trash_can 18.17 -5  # in the east corridor, past the first corner
```

The south corridor runs from world y = -12.5 to y = -8.84, so keep y in that
range. That is the clear width, 3.66 m.

Details:

- Every obstacle gets an `obs_` prefix. Remove them all with
  `ros2 run gerbil_gazebo clear_obstacles.sh`, or one with
  `ros2 run gerbil_gazebo clear_obstacles.sh obs_<name>`.
- The models are **static**: the robot cannot push them, it can only stall
  against them.
- On the terrain worlds, set `Z` to the ground height or the obstacle floats or
  is buried. Near the spawn, the ground is about the spawn z minus 0.1 m:
  `Z=1.45 ros2 run gerbil_gazebo spawn_obstacle.sh box 3 0` for `mdrs_utah`,
  whose spawn is world (-2, 0).
- If the scripts cannot find the world, set it: `WORLD=school_loop`.

A good first test: send the 3 m goal, then spawn `box 2 -10.67` while the robot
drives. The box is in the robot's path, 2 m from the spawn. Watch what Nav2
does: see [exercise 9a](#9-tuning-exercises).

## 7. Grading odometry

`odom_error.py` starts with the sim (`use_odom_error:=false` turns it off). It
compares the EKF output, `/odometry/filtered`, with the ground truth. Both are
measured from the pose where both topics first reported, so **leave the robot
still until the first report prints**. Every 2 s it prints a line like:

```
pos err  0.123 m  (  1.23% of 10.00 m)  yaw err   +1.50 deg  max pos err 0.150 m  |  est (+9.95, +0.10)  truth (+10.02, +0.03)
```

(The numbers above show the format only. They are not a measurement.)

- `pos err`: distance between where the EKF thinks the robot is and where it
  is.
- `% of N m`: that error as a share of the distance actually driven (the path
  length of the truth). This is the number to compare between tuning changes.
- `yaw err`: heading error. Small yaw errors turn into large position errors
  over a long drive, so watch it.
- `max pos err`: the worst so far.
- On Ctrl-C it prints a `FINAL` line.

In RViz, the white `/sim/ground_truth/odom_aligned` arrow is the truth drawn in
`odom`. The gap between it and the EKF arrow is the drift.

To grade only the wheels, without the EKF:

```bash
ros2 run gerbil_gazebo odom_error.py --ros-args -p estimate_topic:=/diff_drive_controller/odom
```

(Run it with `use_odom_error:=false` on the launch, or two copies print.)

**What good looks like.** The sim does not yet have a baseline. It is
expected to be optimistic: the floor is flat and grippy, the wheel radius in
the URDF and the controller are identical, and the ZED stand-in has no scale
error. The real-robot calibration (`ekf.yaml`) measured wheel odometry at +0.6%
over a 3 m tape and the ZED at -7.9%. Treat a sim error close to zero as "the
fusion is not broken", not as "the robot will do this". Record the first
numbers for a lap of `school_loop` as the baseline, then compare against it
after each EKF change.

## 8. Worlds

| `world:=` | What | Size and relief | Spawn (world) |
|---|---|---|---|
| `school_loop` | Ring corridor around a solid classroom block, the hallway Gerbil drives in. 12 ft (3.66 m) clear width, 6 door recesses (1.0 m wide, 0.25 m deep), 2.4 m walls | 40 x 25 m outer, flat. **Lap 115.36 m** on the centreline | (0, -10.67), facing +x |
| `mdrs_utah` | URC site near Hanksville, UT (MDRS). USGS 3DEP **1 m lidar** DEM, with rocks | 200 x 200 m, **12.8 m relief**, median slope 2.1 deg | (-2, 0, 1.55) |
| `indiana_dunes` | West Beach, Indiana Dunes NP: beach, foredune, interdunal flat, dune toe to the south. USGS 3DEP 1 m lidar DEM, with driftwood | 200 x 200 m, 14.5 m relief, median slope 8.6 deg, **steep faces** (90th percentile 22.4 deg) | (2, 0, 1.86) |

Data sources and credits are in
`src/gerbil_gazebo/models/{mdrs,dunes}_terrain/README.md`.

On the terrain worlds, the LiDAR scan plane is only 0.221 m above the ground.
Expect it to hit rising ground and mark it as an obstacle. That is realistic,
and it matters for URC.

Regenerate the worlds. You rarely need to, and both scripts write into
`src/gerbil_gazebo`:

```bash
# school_loop, standard library only. With no args it reproduces the committed SDF.
python3 src/gerbil_gazebo/scripts/gen_loop_world.py --outer-length 40 --outer-width 25 --corridor 3.66

# terrain: needs numpy, tifffile and Pillow, and downloads from USGS
python3 -m venv /tmp/demvenv && /tmp/demvenv/bin/pip install numpy tifffile Pillow
/tmp/demvenv/bin/python src/gerbil_gazebo/scripts/fetch_dem.py --site mdrs_utah --download-dir /tmp/dem-mdrs
```

If you change the `school_loop` size, update its spawn in `config/worlds.yaml`
to match.

## 9. Tuning exercises

These map to the known Stage-1 review red flags. Each one gives the symptom you
will see in sim, the file to change, and what to try. Make the change in
`src/gerbil_bringup/config/`. That is the real config, so a fix proven here is
a fix for the robot. Commit sim-only experiments on a branch.

### 9a. Blocked path makes the robot "spin crazily"

- **Symptom.** Send the 3 m goal and spawn `box 2 -10.67` in the way, or fully
  block the corridor with two or three `block`s. Nav2 runs its **default**
  recovery: clear both costmaps, then Spin, Wait and BackUp in a round robin,
  retrying up to 6 times. The robot pivots on the spot and reverses blind,
  because `/scan` covers only the front 180 degrees. This is the behaviour the
  team wants to eliminate.
- **Where.** `nav2_params.yaml`, `bt_navigator`. No
  `default_nav_to_pose_bt_xml` is set, so Humble's shipped
  `navigate_to_pose_w_replanning_and_recovery.xml` runs.
- **Try.** Write a tree that stops, waits, clears only the local costmap,
  replans and retries a few times, then aborts and holds still. For example,
  save this as `src/gerbil_bringup/config/bt/navigate_hold.xml`. The `config/`
  directory is already installed, so this needs one `gerbil-build` to create the
  install symlink.

  ```xml
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <!-- 3 retries of: wait, clear local costmap, replan. Then abort.
           No Spin, no BackUp. On abort the controller stops, so the robot holds. -->
      <RecoveryNode number_of_retries="3" name="NavigateRecovery">
        <PipelineSequence name="NavigateWithReplanning">
          <RateController hz="1.0">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
          </RateController>
          <FollowPath path="{path}" controller_id="FollowPath"/>
        </PipelineSequence>
        <Sequence name="StopWaitClear">
          <Wait wait_duration="3"/>
          <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
        </Sequence>
      </RecoveryNode>
    </BehaviorTree>
  </root>
  ```

  Then, under `bt_navigator: ros__parameters:` in `nav2_params.yaml`:

  ```yaml
  default_nav_to_pose_bt_xml: /home/ubuntu/ros2_ws/install/gerbil_bringup/share/gerbil_bringup/config/bt/navigate_hold.xml
  ```

  Humble takes an absolute path here. That path is right for the container. The
  robot needs its own, so a launch-file substitution is the proper fix before
  this goes to the robot. Once the tree no longer calls Spin or BackUp, revisit
  `behavior_plugins`: the comments there explain why both are loaded today.
  Check: blocked, the robot should stop, wait, retry, then report `ABORTED`
  and stay still. Unblocked (`clear_obstacles.sh`) during a wait, it should
  carry on.

### 9b. Goals more than about 10 m away fail

- **Symptom.** A goal at odom (18, 0) fails at once with a planner error. Goals
  under about 9 m succeed.
- **Where.** `nav2_params.yaml`, `global_costmap`: `width: 20`, `height: 20`,
  `resolution: 0.05`. The window is rolling and centred on the robot, so the
  reach is 10 m in x and in y.
- **Try.** `width: 40`, `height: 40`, `resolution: 0.10`. That is the same
  400 x 400 = 160k cells, so a similar cost, with twice the reach. Then send
  the corner goal in one go. Watch for paths that now cut corners: 0.10 m cells
  are coarse next to a 0.70 m inflation. Leave `local_costmap` alone: it is
  4 x 4 m @ 0.05 and the controller wants fine cells.

### 9c. The EKF trusts the ZED absolutely

- **Symptom.** The sim reproduces the real flaw on purpose. The plain
  `/zed/zed_node/odom` message carries an all-zero twist covariance, as the real
  ZED wrapper's does. robot_localization reads zero as "perfectly certain" and
  fuses ZED vy and vyaw as near-truth. The wheel yaw rate counts for almost
  nothing. See the `CAUTION - UNRESOLVED` note in `ekf.yaml`. In sim the
  stand-in is well behaved, so the error stays small. The danger is a ZED that
  loses tracking, whose garbage is still stamped "certain".
- **Where.** `ekf.yaml`, `odom1` (currently `/zed/zed_node/odom`).
- **Try.** Write the relay that the `ekf.yaml` note asks for: a small node that
  subscribes to `/zed/zed_node/odom` and republishes it, for example on
  `/zed/zed_node/odom_cov`. It stamps a sane twist covariance (vy about 0.02,
  vyaw about 0.01) and drops messages while tracking is bad. On the robot that
  is `pos_tracking_status`. In sim, fake a loss, for example by dropping or
  corrupting messages for a few seconds. Point `odom1` at the relayed topic,
  then compare `odom_error` before and after with a faked loss.

  For comparison, Gazebo also publishes the stand-in **with** covariance on the
  gz topic `/zed/zed_node/odom_with_covariance`, with sigma squared on the
  diagonal (`gerbil.gazebo.xacro`). It is not bridged. To try it, add an entry
  to `src/gerbil_gazebo/config/bridge.yaml` that maps it to a new ROS topic
  such as `/sim/zed_odom_with_covariance`, with gz type
  `ignition.msgs.OdometryWithCovariance` and ROS type `nav_msgs/msg/Odometry`.
  First check that your ros_gz_bridge version supports that pair (unverified).
  Never rename it to the real topic name.

### 9d. Nothing stops the robot independently of the planner

- **Symptom.** Spawn a box right in front of a moving robot
  (`spawn_obstacle.sh box 1 -10.67` just after it starts). Only the
  planner and controller react, at their own rates. There is no reflex layer
  between twist_mux and the wheels. In sim the motors also have unlimited
  torque (see [section 12](#12-sim-vs-real)), so nothing stalls on contact
  either.
- **Where.** A new node. `nav2_collision_monitor` is installed in the image.
- **Try.** Run the collision monitor between twist_mux and the controller.
  Remap twist_mux's `cmd_vel_out` (in `gerbil.launch.xml`) to, say,
  `/cmd_vel_mux`. Set the monitor's `cmd_vel_in_topic: /cmd_vel_mux` and
  `cmd_vel_out_topic: /diff_drive_controller/cmd_vel_unstamped`. Use
  `base_frame_id: base_footprint`, `odom_frame_id: odom` and one `stop`
  polygon just ahead of the footprint (which spans x -0.59 to +0.05), with
  observation source `/scan`. It is a lifecycle node, so it needs a lifecycle
  manager. Check the Humble parameter names in the Nav2 docs: some were
  renamed in later releases. Placed after the mux, it also stops keyboard and
  joystick driving, which is the point.

### 9e. Inflation 0.70 m in a 3.66 m corridor

- **Symptom.** Watch the global costmap in RViz. Inflation spreads 0.70 m off
  each wall, so only about 2.26 m of the 3.66 m corridor is free of cost.
  Regulated Pure Pursuit slows down in high cost, and the planner pulls toward
  the middle. At each door recess the wall steps back 0.25 m for 1.0 m, and the
  path may wobble there. The rover cannot fit through any door: 1.0 m is narrower
  than the swept turning circle.
- **Where.** `nav2_params.yaml`, `inflation_layer` in both costmaps:
  `inflation_radius: 0.70` and `cost_scaling_factor: 1.5`.
- **Try.** Read the long comment above `inflation_radius` first. 0.70 is
  derived from the padded circumscribed radius (0.676 m), and going below it
  lets the 0.59 m tail clip walls when the robot turns in place. So try the
  other lever: raise `cost_scaling_factor` (2.0, 2.5, 3.0) and watch the path
  and speed past the doors. In the Gazebo window, check that the tail never
  touches a wall during in-place turns.

## 10. Performance tips (software rendering)

Everything is drawn by the CPU, so rendering competes with physics. In order
of impact:

1. **`use_gui:=false`**: no Gazebo window, and RViz shows you everything Nav2
   sees. This is the biggest saving. Inside the container, do **not** add
   `use_headless:=true`: the VNC display already serves the sensors.
2. **`use_rviz:=false`** when you only need the CLI. The usual setup is
   `use_gui:=false` with RViz on.
3. Keep **`use_sim_depth:=false`** (the default). It renders a second camera.
4. If you keep the Gazebo GUI, close the side panels (entity tree, component
   inspector) and make the window small.
5. Give Docker more CPUs (section 2).

**Real-time factor (RTF)** is sim seconds per wall second. 1.0 is real time,
and 0.5 means the sim runs at half speed. With the GUI, RTF shows at the
bottom right of the Gazebo window. Without it:

```bash
timeout 5 ign topic -e -t /world/school_loop/stats | grep -m1 real_time_factor
```

Every node runs on sim time, so a low RTF makes the sim slow but not wrong.
Nav2 timeouts are counted in sim time too. Report the RTF you see with any
performance issue.

## 11. Troubleshooting

| Symptom | Likely cause | What to do |
|---|---|---|
| Gazebo window is black or fails to create a GL context | ogre2 cannot get a GL context from llvmpipe | Per `docker/README.md`: try `MESA_GL_VERSION_OVERRIDE=4.5 ros2 launch ...`, or the old render engine for the GUI with `gz_args:="--render-engine ogre"`. Or run `use_gui:=false` and use RViz |
| `diff_drive_controller` never reaches active, spawner errors | The spawners wait up to 120 s for `/controller_manager`, which exists only after the robot is spawned | Check `[gerbil sim] gerbil spawned` printed. If spawning failed, the launch shuts itself down with a message. Rerun with `gz_args:="-v 4"` and look for `ign_ros2_control` errors. Check with `ros2 control list_controllers` |
| No `/scan` | Bridge or laser_filters chain | `ros2 topic hz /scan_raw` (bridge, from the gpu_lidar). If that is fine, `ros2 topic hz /scan` (laser_filters). No `/scan_raw` usually means the sensors did not render: see the black-window row |
| Nav2 says "Timed out waiting for transform from base_footprint to odom" | The EKF has no input yet | Normal until the controllers are active. If it persists, check `ros2 topic hz /diff_drive_controller/odom` and `/zed/zed_node/odom` |
| A goal fails at once | Goal outside the 20 x 20 m global costmap, or inside a wall | Section 5 and exercise 9b. Remember that goals are odom, spawn-relative |
| Robot sinks into or flies off the terrain, or its height looks wrong | Terrain mesh orientation or scale (regenerate with `scripts/dem_to_mesh.py --all`) | **Report it** with the world name and a screenshot. Check whether the terrain looks mirrored against the site README |
| `spawn_obstacle.sh`: "no running world found" | Sim not up, or `ign` cannot see it | Set `WORLD=<name>` and check `ign service -l` lists `/world/<name>/create` |
| Port 6080 already in use | `ros-dev` is running | `docker compose stop ros-dev` |
| Strange build errors after an image update | Stale build volumes | `docker compose down -v`, then `gerbil-build` |
| Everything is very slow | Low RTF | Section 10 |

## 12. Sim vs real

Do not carry a result to the robot without these in mind:

- **Unlimited wheel torque.** ign_ros2_control sets wheel velocities directly,
  with no effort limit. The rover never stalls, never slips on a slope it
  should not climb, and drives into an obstacle at full force. The spawned
  obstacles are static, so it cannot push them. It can only grind against them.
- **Perfect floor.** Wheel friction is mu 1.0 on a flat, even floor. There is no
  slip, no carpet and no ramps. Wheel odometry is near-perfect.
- **The ZED is a stand-in.** It has white noise only: no scale drift (the real
  one reads -7.9% over 3 m), no tracking loss and no latency spikes. No depth
  unless `use_sim_depth:=true`, and that cloud is a low-res Gazebo depth camera,
  not ZED stereo.
- **The LiDAR sees visuals**, not collision shapes. That makes no difference
  for the primitive models here.
- **Ground truth exists.** `/sim/ground_truth/odom` is for grading only. If
  anything fuses it, the sim result is worthless.
- **Rendering time.** A low RTF slows everything consistently, but CPU-heavy
  nodes such as the planner get relatively more time per sim second than on
  the Jetson.

## CI smoke test

`.github/workflows/sim.yml` runs `src/gerbil_gazebo/test/sim_smoke_test.sh` on
pull requests that touch the sim, the description, the bringup or the sim image.
The test launches `school_loop` headless, waits for the clock, scan, EKF,
controllers and bt_navigator, then drives a 3 m goal and checks against the
ground truth that the robot really moved.

To run it yourself, in the container after `gerbil-build`:

```bash
~/ros2_ws/src/gerbil_gazebo/test/sim_smoke_test.sh
```

It uses its own `ROS_DOMAIN_ID` and Gazebo partition, so it does not interfere
with a sim you already have open. The logs go to `~/ros2_ws/log/sim_smoke/`.
