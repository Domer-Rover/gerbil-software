#!/usr/bin/env python3
"""URC Delivery mission: long-range teleop to astronauts across rough terrain.

ZED + GPS on top of the Foxglove bringup, with the ArUco detector optional
for finding marked drop-off points. Started by the mission launcher GUI
(tools/mission_launcher), or by hand:

  ros2 launch gerbil_bringup mission_delivery.launch.py launch_aruco:=true

Teleop is not part of the launch: drive with
`ros2 run gerbil_bringup teleop_joystick.sh` (or teleop_keyboard.sh) in a
second terminal; twist_mux forwards it to the controller.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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

    launch_aruco_arg = DeclareLaunchArgument(
        'launch_aruco',
        default_value='false',
        description='Launch the ArUco marker detector'
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
            'launch_zed': 'true',
            'launch_gps': 'true',
            'launch_lidar': 'false',
        }.items()
    )

    aruco_detector = Node(
        package='gerbil_bringup',
        executable='aruco_detector.py',
        name='aruco_detector',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_aruco')),
    )

    return LaunchDescription([
        use_mock_hardware_arg,
        launch_imu_arg,
        launch_aruco_arg,
        foxglove_launch,
        aruco_detector,
    ])
