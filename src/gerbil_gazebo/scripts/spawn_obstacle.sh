#!/usr/bin/env bash
# Drop a static obstacle into the running Gazebo world.
#
#   ros2 run gerbil_gazebo spawn_obstacle.sh <box|cone|trash_can|block> <x> <y> [yaw] [name]
#
# x, y in metres and yaw in radians, in the world frame (school_loop: origin at the centre of the
# classroom block, x along the long side). Env vars:
#   WORLD  world name, if auto-detection (via `ign service -l`) fails or finds several worlds
#   Z      spawn height, default 0 (obstacle origins are on their base; on the terrain worlds set
#          Z to the ground height at x, y)
# Every name gets an "obs_" prefix so clear_obstacles.sh can find them again.
set -euo pipefail

usage() {
  echo "usage: $(basename "$0") <model> <x> <y> [yaw] [name]" >&2
  echo "models: ${MODELS:-box cone trash_can block}" >&2
  exit 2
}

[[ $# -ge 3 && $# -le 5 ]] || usage
model=$1 x=$2 y=$3 yaw=${4:-0} name=${5:-}
z=${Z:-0}
num='^-?[0-9]+([.][0-9]+)?$'
for v in "$x" "$y" "$yaw" "$z"; do
  [[ $v =~ $num ]] || { echo "not a number: $v" >&2; usage; }
done

prefix=$(ros2 pkg prefix gerbil_gazebo) || { echo "gerbil_gazebo not found; source the workspace" >&2; exit 1; }
models_dir=$prefix/share/gerbil_gazebo/models
file=$models_dir/$model/model.sdf
[[ -f $file ]] || { echo "unknown model '$model' (no $file)" >&2; usage; }

ign_cli=$(command -v ign || command -v gz || true)
world=${WORLD:-}
if [[ -z $world && -n $ign_cli ]]; then
  mapfile -t worlds < <("$ign_cli" service -l 2>/dev/null | sed -n 's#^/world/\([^/]*\)/create$#\1#p' | sort -u)
  if [[ ${#worlds[@]} -eq 1 ]]; then
    world=${worlds[0]}
  elif [[ ${#worlds[@]} -gt 1 ]]; then
    echo "several worlds running (${worlds[*]}); set WORLD=<name>" >&2
    exit 1
  fi
fi
[[ -n $world ]] || { echo "no running world found; is the sim up? (or set WORLD=<name>)" >&2; exit 1; }

if [[ -z $name ]]; then
  name=${model}_$(date +%H%M%S)_$RANDOM
fi
[[ $name == obs_* ]] || name=obs_$name

echo "spawning $model as '$name' in world '$world' at x=$x y=$y z=$z yaw=$yaw"
# key=value form so negative coordinates are not mistaken for flags
exec ros2 run ros_gz_sim create -world="$world" -file="$file" -name="$name" \
  -x="$x" -y="$y" -z="$z" -Y="$yaw"
