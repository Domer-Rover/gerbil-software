# Build and Run

## Build

```bash
cd ~/domerrover/gerbil-software
colcon build --symlink-install
source install/setup.bash
```

First time on a machine (admin, installs the packages the launch files run):

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Launch

```bash
ros2 launch gerbil_bringup gerbil_foxglove.launch.py   # driving + Foxglove
ros2 launch gerbil_bringup gerbil_nav2.launch.py       # Nav2, odom frame, no map
ros2 launch gerbil_bringup gerbil_slam.launch.py       # build a map
```

Common args: `use_mock_hardware:=true`, `use_joystick:=true`, `launch_zed:=false`,
`launch_lidar:=true`, `launch_gps:=true`, `use_duty_cycle:=false` (closed-loop
velocity PID; see below). The Nav2 and SLAM launches turn the LIDAR on
themselves.

Odometry: `robot_localization` fuses wheel odometry with ZED VIO and publishes
`odom -> base_footprint` on `/odometry/filtered`. The ZED node runs with
`publish_tf:=false` and the controller with `enable_odom_tf: false`, so exactly
one node owns that transform.

## Closed-loop (encoders)

Duty cycle is open loop: commands are PWM percentages, so actual speed varies
with battery and load. Velocity PID uses the encoders and holds the commanded
speed. Before switching:

1. Tune the RoboClaw velocity PID in Motion Studio (QPPS first, then P/I/D).
2. Measure `qppr` (encoder pulses per wheel revolution) and set it in
   `gerbil_description/urdf/mobile_base.ros2_control.xacro`.
3. Set the serial timeout so a killed node cannot latch the motors:
   `python3 scripts/serial_timeout.py --set 0.2`
4. Wheels off the ground, e-stop in hand, launch with `use_duty_cycle:=false`
   and command one wheel slowly.

A backwards-wired encoder turns velocity PID into positive feedback and the
motor runs away at full speed. Test one motor at a time.

## Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped -p speed:=0.2 -p turn:=0.5
```

Joystick (`use_joystick:=true`): hold **L1** as the deadman, left stick to drive.

## Indoor Nav2 test

```bash
ros2 launch gerbil_bringup gerbil_nav2.launch.py use_joystick:=true
```

Check `/scan` in Foxglove first: objects in front of the robot must appear in
front. The LIDAR mount rotation in the URDF is a guess copied from Capybara,
and `laser_filter.yaml` crops the rear half, so a wrong assumption shows up as
a scan that is 90 or 180 degrees off. Then send a 3 m goal:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 3.0}, orientation: {w: 1.0}}}"
```

Record it:

```bash
mkdir -p ~/bags
ros2 bag record -o ~/bags/nav2_$(date +%F_%H%M) \
  /scan /odometry/filtered /diff_drive_controller/odom /zed/zed_node/odom \
  /fix /tf /tf_static /diff_drive_controller/cmd_vel_unstamped /plan /goal_pose
```

Odometry sanity check: drive a closed loop by hand and compare where
`/odometry/filtered` says you ended up against where you actually are. Compare
`/diff_drive_controller/odom` (wheels) and `/zed/zed_node/odom` (VIO) in the
same bag to see which one the filter should trust more.

## Checks

```bash
ros2 control list_controllers
ros2 topic hz /zed/zed_node/odom
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_footprint
```

## Running detached (survives losing WiFi)

A launch started over SSH dies when the SSH connection drops, and with the
RoboClaw serial timeout at 0 the rover keeps driving on its last command. Start
it inside `tmux` so it keeps running when SSH, Foxglove, or both go away — the
joystick still works because it runs on the rover.

```bash
tmux new -s rover          # start a named session
# ... source the workspace and run the launch as usual ...
# detach with: Ctrl-b then d
```

Reattach after reconnecting, from any SSH session:

```bash
tmux attach -t rover       # Ctrl-c inside stops the launch
tmux ls                    # list sessions
```

If tmux is missing: `sudo apt install tmux` (admin).

## Checklist before driving

1. `ls -l /dev/gerbil_*` — udev names exist
2. `ros2 topic echo /scan --once` — LIDAR alive, front arc correct in Foxglove
3. `python3 scripts/serial_timeout.py` — not `DISABLED`
4. `ros2 control list_controllers` — both controllers active
5. `ros2 topic hz /odometry/filtered` — EKF publishing
6. `ros2 run tf2_ros tf2_echo odom base_footprint` — one publisher, no jumps
6. Joystick drives before sending any Nav2 goal

## Motor safety

The RoboClaws stop on their own only if their serial timeout is set. With it at
0, killing a launch leaves the last command latched and the rover keeps driving.
Stop all launches first (the port opens once), then:

```bash
python3 scripts/serial_timeout.py            # read all three boards
python3 scripts/serial_timeout.py --set 0.2  # 200 ms, resolution is 0.1 s
```

## Troubleshooting

- Permission denied on a serial port or ZED: `id -nG` must include `dialout`, `video`, `zed`. Log out and back in after groups change.
- Port busy: another launch is running. `sudo fuser -v /dev/gerbil_roboclaw` (admin).
- Motors don't move: check you didn't pass `use_mock_hardware:=true`, and `ros2 control list_controllers` shows both controllers active.
