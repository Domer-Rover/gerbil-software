#!/usr/bin/env bash
set -euo pipefail

# Save map into the same directory as this script with a timestamped name.
MAP_DIR="$(cd "$(dirname "$0")" && pwd)"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BASENAME="map_${TIMESTAMP}"
OUT="$MAP_DIR/$BASENAME"

echo "Saving map to: $OUT"

# Prefer slam_toolbox service if available, otherwise use nav2 map_saver_cli
if ros2 service list 2>/dev/null | grep -q '/slam_toolbox/save_map'; then
  echo "Detected slam_toolbox service — calling save_map service"
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: '$OUT'}"
else
  echo "Using nav2 map_saver_cli"
  ros2 run nav2_map_server map_saver_cli -f "$OUT"
fi

echo "Done. Files written: ${OUT}.pgm and ${OUT}.yaml" 
