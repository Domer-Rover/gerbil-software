#!/usr/bin/env python3
"""Nav2 for Gerbil: odom-only navigation, indoors.

No map, no AMCL — goals are relative to wherever the robot booted. Obstacles
come from the LD19 LIDAR on /scan (driver + laser_filters live in
gerbil.launch.xml, launched here with launch_lidar:=true).

  ros2 launch gerbil_bringup gerbil_nav2.launch.py use_duty_cycle:=false

Send a 3 m goal:
  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \\
    "{header: {frame_id: 'odom'}, pose: {position: {x: 3.0}, orientation: {w: 1.0}}}"
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

    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='true',
        description='Joystick override (recommended outdoors)'
    )

    launch_gps_arg = DeclareLaunchArgument(
        'launch_gps',
        default_value='false',
        description='Launch the u-blox GPS node (publishes /fix)'
    )

    use_duty_cycle_arg = DeclareLaunchArgument(
        'use_duty_cycle',
        default_value='true',
        description='false = RoboClaw velocity PID (needs encoders + tuned PID)'
    )

    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Foxglove WebSocket port'
    )

    gerbil_bringup_share = FindPackageShare('gerbil_bringup')

    nav2_params = PathJoinSubstitution([
        gerbil_bringup_share, 'config', 'nav2_params.yaml'
    ])

    # Base robot launch (controllers, ZED, robot_state_publisher)
    capybara_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                gerbil_bringup_share, 'launch', 'gerbil.launch.xml'
            ])
        ]),
        launch_arguments={
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'launch_rviz': 'false',
            'launch_zed': 'true',
            'use_joystick': LaunchConfiguration('use_joystick'),
            'use_duty_cycle': LaunchConfiguration('use_duty_cycle'),
            'launch_lidar': 'true',
            'launch_gps': LaunchConfiguration('launch_gps'),
        }.items()
    )

    # Foxglove bridge
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[
            PathJoinSubstitution([gerbil_bringup_share, 'config', 'foxglove_bridge.yaml']),
            {'port': LaunchConfiguration('foxglove_port')},
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
    

    return LaunchDescription([
        use_mock_hardware_arg,
        use_joystick_arg,
        use_duty_cycle_arg,
        launch_gps_arg,
        foxglove_port_arg,
        # Robot base (controllers + ZED + LIDAR)
        capybara_launch,
        foxglove_bridge,
        # Navigation (odom-only, no map/AMCL)
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,
        # Future object-detection feature:  
    ])
