# Shared teleop settings. Sourced by teleop_keyboard.sh and teleop_joystick.sh
# so the two input methods can never drift apart in feel.
#
# Override per-run without editing:  SPEED=0.15 ./teleop_keyboard.sh
: "${SPEED:=0.3}"   # m/s   linear
: "${TURN:=1.0}"    # rad/s angular
#
# NOTE: TURN is clamped by diff_drive_controller's angular.z.max_velocity,
# which is 0.5 in config/gerbil_controllers.yaml. Asking for 1.0 therefore
# yields 0.5 on the robot. Raise BOTH if you want more; raising only this one
# changes nothing.

# Make `ros2` available even in a fresh terminal that was never sourced.
if ! command -v ros2 >/dev/null 2>&1; then
    [ -f /opt/ros/humble/setup.bash ] && . /opt/ros/humble/setup.bash
fi
# Overlay this workspace, found relative to the script rather than hardcoded,
# so the scripts work from a clone in any directory.
_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [ -f "$_WS/install/setup.bash" ]; then
    . "$_WS/install/setup.bash"
elif [ -f "$HOME/gerbil-software/install/setup.bash" ]; then
    . "$HOME/gerbil-software/install/setup.bash"
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 not on PATH and no setup.bash found. Source your workspace first." >&2
    exit 1
fi

# twist_mux must be running or nothing forwards these commands to the wheels.
# It is launched by gerbil.launch.xml with use_twist_mux:=true (the default).
check_mux() {
    if ! ros2 node list 2>/dev/null | grep -q twist_mux; then
        echo "WARNING: twist_mux is not running, so nothing is forwarding" >&2
        echo "         $1 to /diff_drive_controller/cmd_vel_unstamped." >&2
        echo "         Start the robot first (gerbil.launch.xml, or the" >&2
        echo "         foxglove/nav2 wrapper) and leave it running." >&2
        echo >&2
    fi
}
