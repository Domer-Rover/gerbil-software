# shellcheck shell=bash
# Environment for interactive shells in the gerbil-sim container.
# Sourced from /etc/bash.bashrc and (last) from ~ubuntu/.bashrc, so the workspace
# overlay always ends up on top of /opt/ros/humble.

# Software rendering: Docker on macOS has no GPU. Mesa llvmpipe gives a GL 4.5 core
# context, which covers ogre2's GL 3.3 requirement, so no version override is needed.
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

# The VNC desktop is display :1, so `docker exec` shells can open windows in it too.
export DISPLAY="${DISPLAY:-:1}"
if [ -z "${XAUTHORITY:-}" ] && [ -r /home/ubuntu/.Xauthority ]; then
    export XAUTHORITY=/home/ubuntu/.Xauthority
fi

export COLCON_DEFAULTS_FILE=/opt/gerbil/colcon_defaults.yaml

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f /home/ubuntu/ros2_ws/install/setup.bash ]; then
    # shellcheck disable=SC1091
    source /home/ubuntu/ros2_ws/install/setup.bash
fi
