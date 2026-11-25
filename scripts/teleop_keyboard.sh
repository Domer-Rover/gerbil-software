#!/usr/bin/env bash
# Keyboard teleop for Gerbil, in its own terminal.
#
# WHY A SCRIPT AND NOT A LAUNCH ARGUMENT: teleop_twist_keyboard reads raw
# keystrokes from stdin, and a node started by a launch file owns no terminal,
# so it would receive no keys at all. It has to be run interactively.
#
#   ./teleop_keyboard.sh            # 0.3 m/s, 1.0 rad/s (clamped to 0.5)
#   SPEED=0.15 ./teleop_keyboard.sh # half speed, for tight indoor testing
#
# Publishes to /cmd_vel_key, which twist_mux takes at priority 90: it overrides
# Nav2 (10) but yields to the joystick (100). Releasing the keys hands control
# back to Nav2 after the 0.5s mux timeout -- it does NOT cancel the goal.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/teleop_common.sh"
check_mux "keyboard commands"
echo "Keyboard teleop  ->  /cmd_vel_key   (priority 90, beats Nav2)"
echo "speed=${SPEED} m/s   turn=${TURN} rad/s (clamped to 0.5 by the controller)"
echo "Keep this terminal FOCUSED; keys are only read while it has focus."
echo
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel_key \
    -p "speed:=${SPEED}" -p "turn:=${TURN}"
