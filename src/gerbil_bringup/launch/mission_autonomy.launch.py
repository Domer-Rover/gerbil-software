#!/usr/bin/env python3
"""URC Autonomy mission: Nav2 + ArUco.

Includes gerbil_nav2.launch.py (LIDAR + ZED forced on) in velocity-PID mode
for path tracking, plus GPS and the ArUco detector. GPS is published for
logging and display only: it is not fused yet, so Nav2 goals are still in the
odom frame. Started by the mission launcher GUI (tools/mission_launcher), or
by hand:

  ros2 launch gerbil_bringup mission_autonomy.launch.py

For a joystick override, run `ros2 run gerbil_bringup teleop_joystick.sh` in a
second terminal: twist_mux ranks the joystick above Nav2.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Use mock hardware for bench testing'
    )

    launch_imu_arg = DeclareLaunchArgument(
        'launch_imu',
        default_value='false',
        description='Launch the BNO055 IMU (not fused by the EKF)'
    )

    gerbil_bringup_share = FindPackageShare('gerbil_bringup')

    nav2_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                gerbil_bringup_share, 'launch', 'gerbil_nav2.launch.py'
            ])
        ]),
        launch_arguments={
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'launch_imu': LaunchConfiguration('launch_imu'),
            'use_duty_cycle': 'false',
            'launch_gps': 'true',
        }.items()
    )

    aruco_detector = Node(
        package='gerbil_bringup',
        executable='aruco_detector.py',
        name='aruco_detector',
        output='screen',
    )

    return LaunchDescription([
        use_mock_hardware_arg,
        launch_imu_arg,
        nav2_launch,
        aruco_detector,
    ])
