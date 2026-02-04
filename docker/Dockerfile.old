FROM tiryoh/ros2-desktop-vnc:humble 

# Switch to root to install software:
USER root

# Update and install dependencies:
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
    libserial-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo only on amd64 (people with linux or windows)
# Skip on arm64 (apple silicon not supported)
ARG TARGETPLATFORM
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ] || [ "$(uname -m)" = "x86_64" ]; then \
    apt-get update && \
    apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gz && \
    rm -rf /var/lib/apt/lists/*; \
    fi

# # Set TurtleBot3 model (tutorial)
ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
RUN echo "export TURTLEBOT3_MODEL=waffle" >> /etc/bash.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:\$GAZEBO_MODEL_PATH" >> /etc/bash.bashrc

# (Optional) Install python packages using pip:
RUN pip3 install numpy pandas basicmicro pyserial 