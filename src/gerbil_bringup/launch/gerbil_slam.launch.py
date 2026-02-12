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

    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Foxglove WebSocket port'
    )

    gerbil_bringup_share = FindPackageShare('gerbil_bringup')

    # Base robot launch (controllers, ZED, robot_state_publisher)
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
            'launch_zed': 'true',
        }.items()
    )

    # Foxglove bridge for remote visualization
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

    # Convert ZED pointcloud to 2D laser scan for SLAM Toolbox
    # Uses full 3D cloud with height filtering — sees obstacles at all heights
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.1,
            'max_height': 1.5,
            'angle_min': -1.5708,           # -90 degrees
            'angle_max': 1.5708,            # +90 degrees
            'angle_increment': 0.0087,      # ~0.5 degree resolution
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        remappings=[
            ('cloud_in', '/zed/zed_node/point_cloud/cloud_registered'),
            ('scan', '/scan'),
        ],
        output='screen'
    )

    # SLAM Toolbox for mapping and localization
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            PathJoinSubstitution([
                gerbil_bringup_share,
                'config',
                'slam_toolbox.yaml'
            ])
        ],
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
        foxglove_port_arg,
        gerbil_launch,
        foxglove_bridge,
        pointcloud_to_laserscan,
        slam_toolbox,
        aruco_detector,
    ])