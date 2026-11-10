#!/usr/bin/env bash
# Joystick teleop for Gerbil, in its own terminal.
#
# Normally you do NOT need this: gerbil.launch.xml already starts joy_node and
# teleop_twist_joy when launched with use_joystick:=true. Use this script to
# attach a controller to an ALREADY-RUNNING robot without restarting the stack,
# or to test a pad on its own.
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

if [ ! -e "$JOY_DEV" ]; then
    echo "ERROR: no joystick at $JOY_DEV" >&2
    echo "Devices present:" >&2
    ls /dev/input/js* 2>/dev/null >&2 || echo "  (none -- is the pad plugged in / paired?)" >&2
    exit 1
fi

if ros2 node list 2>/dev/null | grep -q teleop_twist_joy; then
    echo "ERROR: teleop_twist_joy is already running (launched with" >&2
    echo "       use_joystick:=true). Two instances would both publish to" >&2
    echo "       /cmd_vel_joy and fight. Relaunch with use_joystick:=false," >&2
    echo "       or just use the one already running." >&2
    exit 1
fi
check_mux "joystick commands"

echo "Joystick teleop  ->  /cmd_vel_joy   (priority 100, beats everything)"
echo "device=${JOY_DEV}   speed=${SPEED} m/s   turn=${TURN} rad/s (clamped to 0.5)"
echo "Hold L1 (button 4) as the deadman, then use the left stick."
echo

# autorepeat_rate keeps a HELD stick from going stale against the mux's 0.5s
# timeout; joy_node otherwise only publishes on change.
ros2 run joy joy_node --ros-args \
    -p "device_name:=${JOY_DEV}" -p autorepeat_rate:=20.0 -p deadzone:=0.05 &
JOY_PID=$!
# Make sure joy_node dies with this script, including on Ctrl-C.
trap 'kill $JOY_PID 2>/dev/null || true' EXIT INT TERM

exec ros2 run teleop_twist_joy teleop_node --ros-args \
    -r cmd_vel:=/cmd_vel_joy \
    -p enable_button:=4 -p axis_linear.x:=1 -p axis_angular.yaw:=0 \
    -p "scale_linear.x:=${SPEED}" -p "scale_angular.yaw:=${TURN}"
