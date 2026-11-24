#!/usr/bin/env bash
# Sim smoke test: bring Gerbil up in school_loop headless, check the stack is
# alive, drive a 3 m Nav2 goal and confirm against ground truth that it moved.
#
#   src/gerbil_gazebo/test/sim_smoke_test.sh            # from the workspace root
#   ~/ros2_ws/src/gerbil_gazebo/test/sim_smoke_test.sh  # in the gerbil-sim container
#
# Run from the SOURCE tree (it is not installed). Needs a built workspace; it
# sources <ws>/install/setup.bash itself if gerbil_gazebo is not found, where
# <ws> is three directories above this script.
#
# Every wait is bounded, so it cannot hang. Exit 0 = pass. Env overrides:
#   WORLD                world key (school_loop)
#   STARTUP_TIMEOUT      s, wall time for the whole bring-up (180)
#   GOAL_TIMEOUT         s, wall time for the goal to finish (120)
#   GOAL_X               m, goal distance ahead in odom (3.0)
#   MIN_TRUE_DISPLACEMENT m, ground-truth distance the robot must cover (GOAL_X - 0.8)
#   LOG_DIR              where logs go (<ws>/log/sim_smoke)
#   HEADLESS_RENDERING   true/false; default true only when DISPLAY is unset
#   SMOKE_ROS_DOMAIN_ID  ROS_DOMAIN_ID for the test (73), so an open sim is not disturbed
#   IGN_PARTITION        Gazebo transport partition (gerbil_smoke_<pid>)
set -uo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS=$(cd "$SCRIPT_DIR/../../.." && pwd)

WORLD=${WORLD:-school_loop}
STARTUP_TIMEOUT=${STARTUP_TIMEOUT:-180}
GOAL_TIMEOUT=${GOAL_TIMEOUT:-120}
GOAL_X=${GOAL_X:-3.0}
MIN_TRUE_DISPLACEMENT=${MIN_TRUE_DISPLACEMENT:-$(awk -v g="$GOAL_X" 'BEGIN{printf "%.2f", g - 0.8}')}
LOG_DIR=${LOG_DIR:-$WS/log/sim_smoke}
if [[ -z ${HEADLESS_RENDERING:-} ]]; then
    if [[ -n ${DISPLAY:-} ]]; then HEADLESS_RENDERING=false; else HEADLESS_RENDERING=true; fi
fi

mkdir -p "$LOG_DIR"
LAUNCH_LOG=$LOG_DIR/launch.log
GOAL_LOG=$LOG_DIR/goal.log
: > "$LAUNCH_LOG"

export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=${GALLIUM_DRIVER:-llvmpipe}
export ROS_DOMAIN_ID=${SMOKE_ROS_DOMAIN_ID:-73}
export IGN_PARTITION=${IGN_PARTITION:-gerbil_smoke_$$}
export ROS_LOG_DIR=$LOG_DIR/ros
export RCUTILS_COLORIZED_OUTPUT=0
export PYTHONUNBUFFERED=1

T0=$(date +%s)
log() { printf '[smoke %4ds] %s\n' "$(( $(date +%s) - T0 ))" "$*"; }
fail() {
    log "FAIL: $*"
    echo "::error::sim smoke test: $*" >&2
    if [[ -s $LAUNCH_LOG ]]; then
        echo "----- last 80 lines of $LAUNCH_LOG -----"
        tail -n 80 "$LAUNCH_LOG"
        echo "----------------------------------------"
    fi
    exit 1
}

# ---------------------------------------------------------------- environment
# colcon's setup.bash trips over set -u; it is not set here, but keep it that way.
if ! command -v ros2 >/dev/null 2>&1; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
if ! ros2 pkg prefix gerbil_gazebo >/dev/null 2>&1; then
    [[ -f $WS/install/setup.bash ]] || fail "gerbil_gazebo not found and no $WS/install/setup.bash; build first"
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
    ros2 pkg prefix gerbil_gazebo >/dev/null 2>&1 || fail "gerbil_gazebo still not found after sourcing $WS/install/setup.bash"
fi
log "workspace $WS, world $WORLD, ROS_DOMAIN_ID $ROS_DOMAIN_ID, IGN_PARTITION $IGN_PARTITION"
log "headless_rendering:=$HEADLESS_RENDERING, logs in $LOG_DIR"

# ---------------------------------------------------------------- launch
# setsid puts the launch and everything it starts (ign gazebo included) in one
# process group, so cleanup can signal all of it at once.
setsid ros2 launch gerbil_gazebo sim.launch.py \
    world:="$WORLD" gui:=false rviz:=false foxglove:=false \
    headless_rendering:="$HEADLESS_RENDERING" odom_error:=true \
    >"$LAUNCH_LOG" 2>&1 < /dev/null &
LAUNCH_PID=$!
LAUNCH_PGID=$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ')
LAUNCH_PGID=${LAUNCH_PGID:-$LAUNCH_PID}

cleanup() {
    local rc=$?
    trap - EXIT INT TERM
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
        log "stopping launch (pgid $LAUNCH_PGID)"
        kill -INT -- "-$LAUNCH_PGID" 2>/dev/null
        for _ in $(seq 1 30); do
            kill -0 "$LAUNCH_PID" 2>/dev/null || break
            sleep 1
        done
        if kill -0 "$LAUNCH_PID" 2>/dev/null; then
            log "launch still running after 30 s, sending TERM"
            kill -TERM -- "-$LAUNCH_PGID" 2>/dev/null
            sleep 5
        fi
        kill -KILL -- "-$LAUNCH_PGID" 2>/dev/null
    fi
    wait "$LAUNCH_PID" 2>/dev/null
    # No pkill by name: inside the container that would also kill a sim the
    # user has open. The process-group kill above covers ign gazebo.
    local last
    last=$(grep -a 'pos err' "$LAUNCH_LOG" 2>/dev/null | tail -n 1)
    [[ -n $last ]] && log "last odom_error: ${last#*] }"
    exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

DEADLINE=$(( T0 + STARTUP_TIMEOUT ))

# wait_for <description> <command...>: retry until success or the startup
# deadline. Fails at once if the launch has died.
wait_for() {
    local what=$1; shift
    log "waiting for $what"
    while :; do
        if "$@"; then
            log "ok: $what"
            return 0
        fi
        kill -0 "$LAUNCH_PID" 2>/dev/null || fail "launch exited while waiting for $what"
        (( $(date +%s) < DEADLINE )) || fail "timed out (${STARTUP_TIMEOUT}s startup budget) waiting for $what"
        sleep 2
    done
}

# ---------------------------------------------------------------- checks
clock_now() {
    # Prints sim time as sec.nanosec, or nothing.
    timeout 10 ros2 topic echo --once /clock 2>/dev/null \
        | awk '/^ *sec:/{s=$2} /^ *nanosec:/{n=$2} END{if (s != "") printf "%d.%09d\n", s, n}'
}
clock_advancing() {
    local a b
    a=$(clock_now); [[ -n $a ]] || return 1
    sleep 1
    b=$(clock_now); [[ -n $b ]] || return 1
    awk -v a="$a" -v b="$b" 'BEGIN{exit !(b > a)}'
}
topic_once() {
    timeout 15 ros2 topic echo --once "$1" >/dev/null 2>&1
}
controllers_active() {
    local out
    out=$(timeout 15 ros2 control list_controllers 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g') || return 1
    grep -Eq '^ *joint_state_broadcaster[[:space:][].*[[:space:]]active' <<<"$out" \
        && grep -Eq '^ *diff_drive_controller[[:space:][].*[[:space:]]active' <<<"$out"
}
lifecycle_active() {
    timeout 15 ros2 lifecycle get "$1" 2>/dev/null | grep -q '^active'
}
truth_xy() {
    # Prints "x y" of the ground truth, or nothing. Python because the CLI
    # cannot pick a nested field reliably in Humble.
    timeout 20 python3 - <<'EOF' 2>/dev/null
import sys, time
import rclpy
from nav_msgs.msg import Odometry
rclpy.init()
node = rclpy.create_node('sim_smoke_truth')
got = []
node.create_subscription(Odometry, '/sim/ground_truth/odom', got.append, 10)
end = time.monotonic() + 15.0
while not got and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
if got:
    p = got[-1].pose.pose.position
    print(f'{p.x:.4f} {p.y:.4f}')
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if got else 1)
EOF
}

wait_for "/clock advancing"                  clock_advancing
wait_for "/scan publishing"                  topic_once /scan
wait_for "controllers active"                controllers_active
wait_for "/odometry/filtered publishing"     topic_once /odometry/filtered
wait_for "bt_navigator active"               lifecycle_active /bt_navigator

# Let the costmaps take in a few scans before planning.
sleep 5

# ---------------------------------------------------------------- goal
read -r X0 Y0 <<<"$(truth_xy)" || true
[[ -n ${X0:-} ]] || fail "no /sim/ground_truth/odom before the goal"
log "truth before goal: ($X0, $Y0)"

GOAL="{pose: {header: {frame_id: odom}, pose: {position: {x: $GOAL_X, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
log "sending NavigateToPose x=$GOAL_X in odom (timeout ${GOAL_TIMEOUT}s)"
timeout --signal=INT --kill-after=10 "$GOAL_TIMEOUT" \
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$GOAL" \
    >"$GOAL_LOG" 2>&1
goal_rc=$?
cat "$GOAL_LOG"
if ! grep -q 'Goal finished with status: SUCCEEDED' "$GOAL_LOG"; then
    (( goal_rc == 124 )) && fail "goal did not finish within ${GOAL_TIMEOUT}s"
    fail "goal did not succeed (send_goal exit $goal_rc)"
fi
log "goal SUCCEEDED"

read -r X1 Y1 <<<"$(truth_xy)" || true
[[ -n ${X1:-} ]] || fail "no /sim/ground_truth/odom after the goal"
moved=$(awk -v a="$X0" -v b="$Y0" -v c="$X1" -v d="$Y1" 'BEGIN{printf "%.3f", sqrt((c-a)^2 + (d-b)^2)}')
log "truth after goal: ($X1, $Y1), moved $moved m (need >= $MIN_TRUE_DISPLACEMENT m)"
awk -v m="$moved" -v n="$MIN_TRUE_DISPLACEMENT" 'BEGIN{exit !(m >= n)}' \
    || fail "Nav2 reported success but the robot truly moved only $moved m"

log "PASS"
