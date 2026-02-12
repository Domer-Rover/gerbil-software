#!/usr/bin/env python3

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
        description='Use mock hardware for simulation'
    )

    launch_zed_arg = DeclareLaunchArgument(
        'launch_zed',
        default_value='true',
        description='Launch ZED2i camera'
    )

    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Foxglove WebSocket port'
    )

    gerbil_bringup_share = FindPackageShare('gerbil_bringup')

    gerbil_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                gerbil_bringup_share,
                'launch',
                'gerbil.launch.xml'
            ])
        ]),
        launch_arguments={
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'launch_rviz': 'false',
            'launch_zed': LaunchConfiguration('launch_zed'),
        }.items()
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': LaunchConfiguration('foxglove_port'),
            'address': '0.0.0.0',
            'num_threads': 4,
            'max_qos_depth': 10,
            'send_buffer_limit': 41943040,
        }],
        output='screen'
    )

    # ArUco marker detection (OpenCV, DICT_6X6_250)
    aruco_detector = Node(
        package='gerbil_bringup',
        executable='aruco_detector.py',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'marker_size': 0.15,
            'dictionary_id': 10,
            'camera_frame': 'zed_left_camera_optical_frame',
        }],
    )

    return LaunchDescription([
        use_mock_hardware_arg,
        launch_zed_arg,
        foxglove_port_arg,
        gerbil_launch,
        foxglove_bridge,
        aruco_detector,
    ])
