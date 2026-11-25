#!/usr/bin/env python3
"""Nav2 for Gerbil: odom-only navigation, indoors.

No map, no AMCL — goals are relative to wherever the robot booted. Obstacles
come from the LD19 LIDAR on /scan (driver + laser_filters live in
gerbil.launch.xml, launched here with use_lidar:=true).

  ros2 launch gerbil_bringup gerbil_nav2.launch.py use_duty_cycle:=false

Send a 3 m goal:
  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \\
    "{header: {frame_id: 'odom'}, pose: {position: {x: 3.0}, orientation: {w: 1.0}}}"

The sim-only arguments (use_sim, use_sim_depth, use_controllers) all default
to the real robot. gerbil_gazebo/launch/sim.launch.py is the only thing that
should change them; see gerbil_gazebo/README.md.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Launch arguments forwarded unchanged into gerbil.launch.xml. Each default is
# today's real-robot value, so a launch that names none of them behaves
# exactly as before. (name, default, description)
_FORWARDED_ARGS = [
    ('use_zed', 'true', 'Start the ZED2i wrapper (never under use_sim)'),
    ('use_lidar', 'true',
     'Start the LIDAR chain: LD19 driver plus laser_filters (/scan_raw -> /scan)'),
    ('use_sim', 'false',
     'Run under Gazebo: sim time, Gazebo URDF, no ros2_control_node, no LD19 '
     'driver, no ZED'),
    ('use_sim_depth', 'false', 'Add the simulated ZED depth cloud (use_sim only)'),
    ('use_controllers', 'true',
     'Spawn the controllers. The sim launch spawns them itself once the robot '
     'exists'),
]


def generate_launch_description():
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Use mock hardware for simulation'
    )

    use_gps_arg = DeclareLaunchArgument(
        'use_gps',
        default_value='false',
        description='Launch the u-blox GPS node (publishes /fix)'
    )

    use_duty_cycle_arg = DeclareLaunchArgument(
        'use_duty_cycle',
        default_value='true',
        description='false = RoboClaw velocity PID (needs encoders + tuned PID)'
    )

    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='false',
        description=(
            'Bring the pad up with the stack. It publishes to /cmd_vel_joy, '
            'which twist_mux takes at priority 100, so holding L1 and moving '
            'the left stick overrides Nav2 immediately. Note this PAUSES the '
            'mission rather than cancelling it: the goal stays active and Nav2 '
            'resumes about 0.5 s after you let go.'
        )
    )

    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Foxglove WebSocket port'
    )

    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='true',
        description='Start foxglove_bridge with the stack'
    )

    forwarded_args = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in _FORWARDED_ARGS
    ]

    # A LaunchConfiguration is a string ('true' / 'false'). ParameterValue
    # with value_type=bool makes it a real bool parameter, so a typo fails
    # loudly at launch instead of becoming a string parameter.
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim'), value_type=bool)

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
            'use_rviz': 'false',
            'use_duty_cycle': LaunchConfiguration('use_duty_cycle'),
            'use_gps': LaunchConfiguration('use_gps'),
            'use_joystick': LaunchConfiguration('use_joystick'),
            # use_zed and use_lidar used to be hard-coded 'true' here. They
            # are arguments now, and their defaults are still 'true'.
            **{name: LaunchConfiguration(name)
               for name, _, _ in _FORWARDED_ARGS},
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
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_foxglove')),
    )

    # --- Nav2 (odom-only, no map/AMCL) ---

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        # Nav2 publishes to the twist_mux input, NOT straight to the
        # controller. twist_mux gives navigation priority 10, so a
        # joystick (100) or keyboard (90) command preempts it instantly.
        # Previously this remapped onto the controller topic directly,
        # alongside teleop, and the robot obeyed whichever arrived last.
        remappings=[('cmd_vel', '/cmd_vel_nav')],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        # Nav2 publishes to the twist_mux input, NOT straight to the
        # controller. twist_mux gives navigation priority 10, so a
        # joystick (100) or keyboard (90) command preempts it instantly.
        # Previously this remapped onto the controller topic directly,
        # alongside teleop, and the robot obeyed whichever arrived last.
        remappings=[('cmd_vel', '/cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
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
        use_duty_cycle_arg,
        use_gps_arg,
        use_joystick_arg,
        foxglove_port_arg,
        use_foxglove_arg,
        *forwarded_args,
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
