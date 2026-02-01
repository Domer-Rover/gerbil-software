FROM tiryoh/ros2-desktop-vnc:humble

# Switch to root to install software:
USER root

# Update and install "plugins" or dependencies:
RUN apt-get update && apt-get install -y \
    nano \
    git \
    python3-pip \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-teleop-twist-keyboard \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-ros-pkgs \ 
    # gazebo depdencies above, not sure if I need it
    libserial-dev \
    && rm -rf /var/lib/apt/lists/*

# # Set TurtleBot3 model (burger, waffle, waffle_pi)
# ENV TURTLEBOT3_MODEL=burger
# ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

# (Optional) Install Python packages:
RUN pip3 install numpy pandas basicmicro --no-cache-dir