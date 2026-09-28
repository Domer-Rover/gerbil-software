# First setup on Gerbil's Jetson

One-time checklist after pulling `main`. Everything in this repo is untested on
Gerbil's hardware, so expect to fix things as you go.

## 1. Pull and build

```bash
cd ~/gerbil-software
git checkout main && git pull
sudo apt install ros-humble-robot-localization ros-humble-laser-filters \
                 ros-humble-ublox-gps ros-humble-joy ros-humble-teleop-twist-joy tmux
colcon build --symlink-install && source install/setup.bash
```

`ldlidar_stl_ros2` is vendored in `src/vendors`, so it builds with the workspace.

## 2. Device names

```bash
ls -l /dev/serial/by-id/
udevadm info -q property -n /dev/ttyUSB0 | grep ID_SERIAL=   # repeat per device
```

Put each `ID_SERIAL` value into `udev/99-gerbil.rules` (replacing `REPLACE_ME_*`), then:

```bash
sudo cp udev/99-gerbil.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/gerbil_*        # expect roboclaw, lidar, gps
```

## 3. Motor safety before anything spins

```bash
python3 scripts/serial_timeout.py            # reads all boards
python3 scripts/serial_timeout.py --set 0.2  # motors stop if commands stop
```

## 4. Drive in duty cycle (open loop) first

```bash
ros2 launch gerbil_bringup gerbil_foxglove.launch.py use_joystick:=true
```

Hold **L1** to drive. Confirm: both controllers active, wheels turn the correct
direction, `/odometry/filtered` publishes, and `tf2_echo odom base_footprint`
shows one publisher with no jumps.

## 5. Encoders and closed loop

1. Measure `qppr`: mark the tire, zero the encoder, turn the wheel 10 full
   revolutions, read the count, divide by 10. Set it for both joints in
   `gerbil_description/urdf/mobile_base.ros2_control.xacro` (currently 500,
   a placeholder).
2. Tune the RoboClaw velocity PID in Motion Studio (QPPS first, then P/I/D).
3. Wheels off the ground, e-stop in hand, one wheel at a time:

```bash
ros2 launch gerbil_bringup gerbil_foxglove.launch.py use_duty_cycle:=false
```

A backwards-wired encoder makes velocity PID run away at full speed. Test one
motor at a time.

## 6. LIDAR

```bash
ros2 launch gerbil_bringup gerbil_foxglove.launch.py launch_lidar:=true
```

In Foxglove, an object in front of the robot must appear in front on `/scan`.
The mount rotation in the URDF and the rear-half crop in `laser_filter.yaml`
are both copied from Capybara and unverified. If the scan is rotated, fix the
`lidar_joint` rpy; if the wrong half is missing, swap the angles in
`laser_filter.yaml`.

## 7. Re-measure the URDF

With the robot together, check wheel radius, wheel separation, chassis size,
and the LIDAR, ZED, and IMU mount positions against
`gerbil_description/urdf/mobile_base.xacro`, and the footprint in
`config/nav2_params.yaml` (currently 0.62 x 0.46).

## 8. Then Nav2

```bash
ros2 launch gerbil_bringup gerbil_nav2.launch.py use_joystick:=true
```

See [build-and-run.md](build-and-run.md) for goals, recording, and checks.
