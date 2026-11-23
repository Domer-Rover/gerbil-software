#!/usr/bin/env bash
# Joystick teleop for Gerbil, in its own terminal.
#
# This IS how you drive Gerbil with a pad. gerbil.launch.xml deliberately does
# not start joy_node or teleop_twist_joy, and has no use_joystick argument --
# that one is on Capybara. Teleop lives outside the launch file because a pad
# gets plugged in and unplugged independently of the robot stack, so attaching
# one should not mean restarting everything.
#
#   ./teleop_joystick.sh               # /dev/input/js0
#   JOY_DEV=/dev/input/js1 ./teleop_joystick.sh
#
# Hold L1 (button 4) as the deadman; left stick drives. Publishes to
# /cmd_vel_joy, which twist_mux takes at priority 100 -- the highest, so it
# preempts both keyboard and Nav2 instantly.
#
# Scales come from teleop_common.sh, identical to the keyboard script.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/teleop_common.sh"
: "${JOY_DEV:=/dev/input/js0}"
# SDL index for joy_node, taken from the jsN number. js0 -> 0, js1 -> 1.
JOY_ID="$(echo "$JOY_DEV" | grep -oE '[0-9]+$')"
: "${JOY_ID:=0}"

if [ ! -e "$JOY_DEV" ]; then
    echo "ERROR: no joystick at $JOY_DEV" >&2
    echo "Devices present:" >&2
    ls /dev/input/js* 2>/dev/null >&2 || echo "  (none -- is the pad plugged in / paired?)" >&2
    exit 1
fi

if ros2 node list 2>/dev/null | grep -q teleop_twist_joy; then
    echo "ERROR: teleop_twist_joy is already running, probably this script" >&2
    echo "       in another terminal. Two instances would both publish to" >&2
    echo "       /cmd_vel_joy and fight. Use the one already running, or" >&2
    echo "       stop it first." >&2
    exit 1
fi
check_mux "joystick commands"

echo "Joystick teleop  ->  /cmd_vel_joy   (priority 100, beats everything)"
echo "device=${JOY_DEV} (SDL id ${JOY_ID})   speed=${SPEED} m/s   turn=${TURN} rad/s (clamped to 0.5)"
echo "Hold L1 (button 4) as the deadman, then use the left stick."
echo

# autorepeat_rate keeps a HELD stick from going stale against the mux's 0.5s
# timeout; joy_node otherwise only publishes on change.
# device_id, NOT device_name. ROS 2 Humble's joy_node is SDL2-based and
# device_name expects the joystick's SDL NAME (here "Wireless Controller"),
# not a device path. Passing /dev/input/js0 as the name failed with
#   Could not get joystick with name /dev/input/js0
# device_id is the SDL index, which matches the jsN number for a single pad.
# Set JOY_NAME to use the name instead, if you ever run two pads at once.
if [ -n "${JOY_NAME:-}" ]; then
    JOY_SEL=(-p "device_name:=${JOY_NAME}")
else
    JOY_SEL=(-p "device_id:=${JOY_ID}")
fi

ros2 run joy joy_node --ros-args \
    "${JOY_SEL[@]}" -p autorepeat_rate:=20.0 -p deadzone:=0.05 &
JOY_PID=$!
# Make sure joy_node dies with this script, including on Ctrl-C.
trap 'kill $JOY_PID 2>/dev/null || true' EXIT INT TERM

exec ros2 run teleop_twist_joy teleop_node --ros-args \
    -r cmd_vel:=/cmd_vel_joy \
    -p enable_button:=4 -p axis_linear.x:=1 -p axis_angular.yaw:=0 \
    -p "scale_linear.x:=${SPEED}" -p "scale_angular.yaw:=${TURN}"
