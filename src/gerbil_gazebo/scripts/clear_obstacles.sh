#!/usr/bin/env bash
# Remove obstacles added with spawn_obstacle.sh from the running Gazebo world.
#
#   ros2 run gerbil_gazebo clear_obstacles.sh            # remove every obs_* model
#   ros2 run gerbil_gazebo clear_obstacles.sh obs_box_1  # remove just these
#
# WORLD=<name> overrides world auto-detection.
set -euo pipefail

ign_cli=$(command -v ign || command -v gz || true)
[[ -n $ign_cli ]] || { echo "neither 'ign' nor 'gz' is on PATH" >&2; exit 1; }
# Fortress messages live in the ignition.msgs package.
pkg=ignition.msgs
[[ $(basename "$ign_cli") == gz ]] && pkg=gz.msgs

world=${WORLD:-}
if [[ -z $world ]]; then
  mapfile -t worlds < <("$ign_cli" service -l 2>/dev/null | sed -n 's#^/world/\([^/]*\)/remove$#\1#p' | sort -u)
  if [[ ${#worlds[@]} -ne 1 ]]; then
    echo "found ${#worlds[@]} running worlds (${worlds[*]:-}); set WORLD=<name>" >&2
    exit 1
  fi
  world=${worlds[0]}
fi

if [[ $# -gt 0 ]]; then
  names=("$@")
else
  # The scene graph lists model, link and visual names; only spawned models start with obs_.
  mapfile -t names < <("$ign_cli" service -s "/world/$world/scene/info" \
      --reqtype "$pkg.Empty" --reptype "$pkg.Scene" --timeout 3000 --req '' 2>/dev/null \
    | sed -n 's/^ *name: "\(obs_[^"]*\)".*/\1/p' | sort -u)
fi

if [[ ${#names[@]} -eq 0 ]]; then
  echo "no obstacles to remove in world '$world'"
  exit 0
fi

for n in "${names[@]}"; do
  echo "removing $n"
  "$ign_cli" service -s "/world/$world/remove" --reqtype "$pkg.Entity" --reptype "$pkg.Boolean" \
    --timeout 3000 --req "name: \"$n\" type: MODEL" >/dev/null || echo "  failed to remove $n" >&2
done
