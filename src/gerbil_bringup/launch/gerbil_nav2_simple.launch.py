#!/usr/bin/env python3
"""Nav2 without map/AMCL — odom-only navigation for testing.

Navigates purely in the odom frame using ZED visual odometry.
No SLAM map, no AMCL localization. Just send a goal relative to
where the robot started (odom origin).

Usage:
  ros2 launch gerbil_bringup gerbil_nav2_simple.launch.py use_mock_hardware:=false

Then send a goal (e.g. 2m forward) in Foxglove on /goal_pose (frame: odom),
or from CLI:
  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: 'odom'}, pose: {position: {x: 2.0}, orientation: {w: 1.0}}}"
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
        description='Use mock hardware for simulation'
    )

    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Foxglove WebSocket port'
    )

    gerbil_bringup_share = FindPackageShare('gerbil_bringup')

    nav2_params = PathJoinSubstitution([
        gerbil_bringup_share, 'config', 'nav2_odom_only_params.yaml'
    ])

    # Base robot launch (controllers, ZED, robot_state_publisher)
    gerbil_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                gerbil_bringup_share, 'launch', 'gerbil.launch.xml'
            ])
        ]),
        launch_arguments={
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'launch_rviz': 'false',
            'launch_zed': 'true',
        }.items()
    )

    # Foxglove bridge
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

    # Convert ZED depth image to 2D laser scan for costmaps
    depthimage_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_node',
        parameters=[
            PathJoinSubstitution([
                gerbil_bringup_share, 'config', 'depthimage_to_laserscan.yaml'
            ])
        ],
        remappings=[
            ('depth', '/zed/zed_node/depth/depth_registered'),
            ('depth_camera_info', '/zed/zed_node/depth/camera_info'),
            ('scan', '/scan'),
        ],
        output='screen'
    )

    # --- Nav2 (odom-only, no map/AMCL) ---

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
        }],
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
        # Robot base
        gerbil_launch,
        foxglove_bridge,
        depthimage_to_laserscan,
        # Navigation (no localization needed)
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,
        # ArUco detection
        aruco_detector,
    ])
