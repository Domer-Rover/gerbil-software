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
            'launch_imu': 'false',
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
            'send_buffer_limit': 10000000,
        }],
        output='screen'
    )

    # Convert ZED depth image to 2D laser scan for SLAM Toolbox
    depthimage_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_node',
        parameters=[
            PathJoinSubstitution([
                gerbil_bringup_share,
                'config',
                'depthimage_to_laserscan.yaml'
            ])
        ],
        remappings=[
            ('depth', '/zed/zed_node/depth/depth_registered'),
            ('depth_camera_info', '/zed/zed_node/depth/camera_info'),
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

    return LaunchDescription([
        use_mock_hardware_arg,
        foxglove_port_arg,
        gerbil_launch,
        foxglove_bridge,
        depthimage_to_laserscan,
        slam_toolbox,
    ])