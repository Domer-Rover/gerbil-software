#!/usr/bin/env bash
# Record a Nav2 run on Gerbil for replay in Foxglove, instead of streaming it
# live through foxglove_bridge (which costs ~0.7 of a Jetson core).
#
#   scripts/record_run.sh                  # nav topics, until Ctrl-C
#   scripts/record_run.sh -n box_offset    # name the run
#   scripts/record_run.sh -d 120 --camera  # 2 min, plus compressed RGB
#   scripts/record_run.sh --list           # print the topics and exit
#
# Output: ~/bags/<YYYY-mm-dd_HHMMSS>_<name>/ as MCAP, which Foxglove opens
# directly (File > Open local file). Copy it off with
#   scp -r gerbil:~/bags/<run> .
#
# The default set is everything needed to understand a Nav2 run after the
# fact: TF, scan, the three odometry sources, commands at every stage, plans,
# costmaps and footprint, the action status and BT log. Topics that do not
# exist (yet) are picked up when they appear. Images and point clouds are
# opt-in: they are by far the most expensive things to record on the Jetson.
#
# MCAP needs ros-humble-rosbag2-storage-mcap. Without it this falls back to
# the default sqlite3 storage, which Foxglove can also open.
set -euo pipefail

usage() {
    sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'
    cat <<'EOF'

Options:
  -n, --name NAME     suffix for the bag folder (default: run)
  -o, --out DIR       parent folder (default: ~/bags)
  -d, --duration S    stop after S seconds (default: until Ctrl-C)
      --camera        add compressed ZED RGB (heavy)
      --cloud         add the ZED point cloud (very heavy)
      --list          print the topic list and exit
  -h, --help
EOF
}

NAME=run
OUT="$HOME/bags"
DURATION=""
CAMERA=false
CLOUD=false
LIST=false
while [[ $# -gt 0 ]]; do
    case $1 in
        -n|--name) NAME=$2; shift 2 ;;
        -o|--out) OUT=$2; shift 2 ;;
        -d|--duration) DURATION=$2; shift 2 ;;
        --camera) CAMERA=true; shift ;;
        --cloud) CLOUD=true; shift ;;
        --list) LIST=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "record_run: unknown option $1" >&2; usage >&2; exit 2 ;;
    esac
done

TOPICS=(
    # Frames and robot model
    /tf /tf_static /robot_description /joint_states
    # Sensors
    /scan /scan_raw
    # Odometry: fused, wheels, ZED (compare them in a Foxglove plot)
    /odometry/filtered /diff_drive_controller/odom /zed/zed_node/odom
    # ZED tracking state: shows when VIO lost tracking (issue #26)
    /zed/zed_node/pose/status
    # Commands at every stage: nav2 -> twist_mux -> controller, plus teleop
    /cmd_vel_nav /cmd_vel_joy /cmd_vel_key /diff_drive_controller/cmd_vel_unstamped
    # Plans and controller internals
    /goal_pose /plan /received_global_plan /lookahead_point
    # Costmaps (full maps plus incremental updates) and the footprint
    /global_costmap/costmap /global_costmap/costmap_updates
    /local_costmap/costmap /local_costmap/costmap_updates
    /local_costmap/published_footprint
    # What Nav2 decided and why
    /navigate_to_pose/_action/status /behavior_tree_log
    # Logs and health
    /rosout /diagnostics
)
# zed_wrapper 5.x topic names (4.x used rgb/image_rect_color)
$CAMERA && TOPICS+=(/zed/zed_node/rgb/color/rect/image/compressed /zed/zed_node/rgb/color/rect/camera_info)
$CLOUD && TOPICS+=(/zed/zed_node/point_cloud/cloud_registered)

if $LIST; then
    printf '%s\n' "${TOPICS[@]}"
    exit 0
fi

if ! command -v ros2 >/dev/null; then
    echo "record_run: ros2 not found; source /opt/ros/humble/setup.bash first" >&2
    exit 1
fi

STORAGE=(-s mcap)
if ! ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1; then
    echo "record_run: MCAP storage not installed, using sqlite3." >&2
    echo "            sudo apt install ros-humble-rosbag2-storage-mcap  for MCAP." >&2
    STORAGE=()
fi

mkdir -p "$OUT"
DEST="$OUT/$(date +%Y-%m-%d_%H%M%S)_${NAME}"
echo "record_run: recording ${#TOPICS[@]} topics to $DEST"
[[ -n $DURATION ]] && echo "record_run: stopping after ${DURATION}s" || echo "record_run: Ctrl-C to stop"

CMD=(ros2 bag record "${STORAGE[@]}" -o "$DEST" "${TOPICS[@]}")
if [[ -n $DURATION ]]; then
    # SIGINT, not SIGTERM, so rosbag2 closes the file cleanly
    timeout -s INT "$DURATION" "${CMD[@]}" || true
else
    "${CMD[@]}"
fi
echo "record_run: saved $DEST"
