#!/usr/bin/env python3
"""URC Astrobiology mission: teleop to a site, image it, tag its coordinates.

Joystick + ZED + GPS on top of the Foxglove bringup. Started by the mission
launcher GUI (tools/mission_launcher), or by hand:

  ros2 launch gerbil_bringup mission_astrobiology.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    foxglove_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                gerbil_bringup_share, 'launch', 'gerbil_foxglove.launch.py'
            ])
        ]),
        launch_arguments={
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'launch_imu': LaunchConfiguration('launch_imu'),
            'use_joystick': 'true',
            'launch_zed': 'true',
            'launch_gps': 'true',
            'launch_lidar': 'false',
        }.items()
    )

    # TODO(capybara): science payload nodes (instruments, sample cache)

    return LaunchDescription([
        use_mock_hardware_arg,
        launch_imu_arg,
        foxglove_launch,
    ])
