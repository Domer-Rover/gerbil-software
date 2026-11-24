#!/usr/bin/env python3
"""Gerbil in Gazebo Fortress, one command.

  ros2 launch gerbil_gazebo sim.launch.py
  ros2 launch gerbil_gazebo sim.launch.py world:=mdrs_utah slam:=true
  ros2 launch gerbil_gazebo sim.launch.py gui:=false rviz:=false   # CI

This is NOT a fork of the robot bringup. It includes the real
gerbil_bringup/launch/gerbil_nav2.launch.py (or gerbil.launch.xml with
nav2:=false) and flips only the sim arguments:

  use_sim_time             true   every node follows the bridged /clock
  sim_mode                 gazebo xacro swaps RoboClaw for ign_ros2_control
  launch_ros2_control_node false  the plugin IS the controller_manager
  launch_spawners          false  spawned here, after the robot exists
  launch_zed               false  the ZED odom comes from the bridge
  launch_lidar_driver      false  /scan_raw comes from the bridge, and the
                                  real laser_filters chain still makes /scan

So the EKF, twist_mux, laser_filters, controllers and Nav2 all run with the
robot's own config files. Topics and frames are pinned in ../README.md.

Startup order:
  1. Gazebo (server, plus GUI unless gui:=false) loads the world with -r.
  2. In parallel: the bridge, the real bringup (robot_state_publisher, EKF,
     twist_mux, laser_filters, Nav2), RViz and SLAM if asked.
  3. `create` waits for /robot_description, then spawns the model `gerbil`.
     Loading it starts ign_ros2_control and its /controller_manager.
  4. Only when `create` has EXITED cleanly do the controller spawners start.
     Before that there is no controller_manager to talk to.
  5. Once diff_drive_controller is active, /diff_drive_controller/odom and
     the bridged ZED odom feed the EKF, odom -> base_footprint appears, and
     Nav2's costmaps (which wait for that transform) come up.
"""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node

# Seconds a spawner waits for /controller_manager after `create` returns.
# The plugin normally answers within a second or two; software rendering on a
# loaded Mac can be far slower. Bounded so a broken plugin fails loudly.
CONTROLLER_MANAGER_TIMEOUT = '120'

# Same controllers, same order, as gerbil.launch.xml on the real robot.
CONTROLLERS = ['joint_state_broadcaster', 'diff_drive_controller']

DEFAULT_SPAWN = {'x': 0.0, 'y': 0.0, 'z': 0.05, 'yaw': 0.0}


def _flag(context, name):
    value = context.launch_configurations[name].strip().lower()
    if value in ('true', '1', 'yes', 'on'):
        return True
    if value in ('false', '0', 'no', 'off'):
        return False
    raise RuntimeError(f"sim.launch.py: {name}:={value!r} is not true or false")


def _scoped(include):
    # In Humble an include's launch_arguments are written into the CALLER's
    # launch configurations and stay there, so they would leak into every
    # sibling. A scoped group pops them again once the include is done.
    return GroupAction([include], scoped=True)


def _resolve_world(gazebo_share, world):
    """Look the world up in config/worlds.yaml. Returns (sdf path, spawn dict)."""
    registry_path = os.path.join(gazebo_share, 'config', 'worlds.yaml')
    try:
        with open(registry_path) as f:
            registry = yaml.safe_load(f) or {}
    except FileNotFoundError:
        raise RuntimeError(
            f'sim.launch.py: world registry not found at {registry_path}. '
            'Rebuild gerbil_gazebo (colcon build) so config/worlds.yaml is installed.')
    if world not in registry:
        raise RuntimeError(
            f'sim.launch.py: unknown world {world!r}. '
            f'Known worlds in {registry_path}: {", ".join(sorted(registry))}')
    entry = registry[world]
    sdf = os.path.join(gazebo_share, 'worlds', entry['file'])
    if not os.path.isfile(sdf):
        raise RuntimeError(f'sim.launch.py: world {world!r} points at {sdf}, which does not exist')
    spawn = dict(DEFAULT_SPAWN)
    spawn.update(entry.get('spawn') or {})
    return sdf, spawn


def _launch_setup(context, *args, **kwargs):
    gazebo_share = get_package_share_directory('gerbil_gazebo')
    bringup_share = get_package_share_directory('gerbil_bringup')

    world = context.launch_configurations['world']
    nav2 = _flag(context, 'nav2')
    rviz = _flag(context, 'rviz')
    foxglove = _flag(context, 'foxglove')
    slam = _flag(context, 'slam')
    depth_camera = _flag(context, 'depth_camera')
    gui = _flag(context, 'gui')
    headless_rendering = _flag(context, 'headless_rendering')
    odom_error = _flag(context, 'odom_error')
    gz_extra = context.launch_configurations['gz_extra_args'].strip()

    world_sdf, spawn = _resolve_world(gazebo_share, world)

    actions = [LogInfo(msg=f'[gerbil sim] world {world}: {world_sdf}, spawn {spawn}')]

    # ---------------- 1. Gazebo ----------------
    # Humble's ros_gz_sim gz_sim.launch.py takes gz_args and, with its default
    # gz_version 6, runs `ign gazebo` (Fortress). -r starts unpaused, -s is
    # server only. on_exit_shutdown: closing Gazebo stops the whole launch.
    gz_args = ['-r']
    if not gui:
        gz_args.append('-s')
        if headless_rendering:
            # EGL rendering with no X display at all. Not needed when DISPLAY
            # is set (e.g. inside the noVNC container), only for bare CI.
            gz_args.append('--headless-rendering')
    if gz_extra:
        gz_args.append(gz_extra)
    gz_args.append(world_sdf)
    actions.append(_scoped(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ' '.join(gz_args),
            'on_exit_shutdown': 'true',
        }.items(),
    )))

    # ---------------- 2a. Bridge ----------------
    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gerbil_sim_bridge',
        parameters=[{
            'config_file': os.path.join(gazebo_share, 'config', 'bridge.yaml'),
            'use_sim_time': True,
        }],
        output='screen',
    ))
    if depth_camera:
        actions.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gerbil_sim_depth_bridge',
            parameters=[{
                'config_file': os.path.join(gazebo_share, 'config', 'bridge_depth_camera.yaml'),
                'use_sim_time': True,
            }],
            output='screen',
        ))

    # ---------------- 2b. The REAL bringup, sim args flipped ----------------
    # robot_state_publisher runs exactly once, inside this include, and is the
    # only source of /robot_description (which `create` spawns from and the
    # ign_ros2_control plugin reads).
    bringup_args = {
        'use_sim_time': 'true',
        'sim_mode': 'gazebo',
        'sim_depth_camera': str(depth_camera).lower(),
        'launch_ros2_control_node': 'false',
        'launch_spawners': 'false',
        'launch_zed': 'false',
        'launch_lidar': 'true',
        'launch_lidar_driver': 'false',
        'launch_gps': 'false',
        'launch_rviz': 'false',     # ours is below, with the sim config
        'use_mock_hardware': 'false',  # ignored by xacro when sim_mode is gazebo
    }
    if nav2:
        bringup_args['launch_foxglove'] = str(foxglove).lower()
        bringup_file = os.path.join(bringup_share, 'launch', 'gerbil_nav2.launch.py')
    else:
        bringup_file = os.path.join(bringup_share, 'launch', 'gerbil.launch.xml')
    actions.append(_scoped(IncludeLaunchDescription(
        AnyLaunchDescriptionSource(bringup_file),
        launch_arguments=bringup_args.items(),
    )))

    if foxglove and not nav2:
        # gerbil.launch.xml has no foxglove of its own; this mirrors the node
        # in gerbil_nav2.launch.py.
        actions.append(Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[
                os.path.join(bringup_share, 'config', 'foxglove_bridge.yaml'),
                {'port': 8765, 'use_sim_time': True},
            ],
            output='screen',
        ))

    if rviz:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(gazebo_share, 'rviz', 'sim.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ))

    if slam:
        # The robot's own slam_toolbox.yaml: odom / map / base_footprint on
        # /scan, all of which exist unchanged in sim. It publishes map -> odom.
        # Nav2 here plans in odom, so SLAM only builds the map alongside.
        actions.append(Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                os.path.join(bringup_share, 'config', 'slam_toolbox.yaml'),
                {'use_sim_time': True},
            ],
            output='screen',
        ))

    if odom_error:
        # Prints EKF-vs-truth error every 2 s and publishes the truth in the
        # odom frame for RViz (/sim/ground_truth/odom_aligned). Started now so
        # it aligns while the robot is still sitting at its spawn pose.
        actions.append(Node(
            package='gerbil_gazebo',
            executable='odom_error.py',
            name='odom_error',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ))

    # ---------------- 3. Spawn the robot ----------------
    create = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_gerbil',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'gerbil',
            '-x', str(spawn['x']),
            '-y', str(spawn['y']),
            '-z', str(spawn['z']),
            '-Y', str(spawn['yaw']),
        ],
        output='screen',
    )
    actions.append(create)

    # ---------------- 4. Controllers, only once gerbil exists ----------------
    def _after_create(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f'[gerbil sim] spawning the robot FAILED (create exited '
                            f'{event.returncode}). Is the world loaded and named as '
                            'in config/worlds.yaml? Shutting down.'),
                EmitEvent(event=Shutdown(reason='robot spawn failed')),
            ]
        return [
            LogInfo(msg='[gerbil sim] gerbil spawned, starting controller spawners'),
        ] + [
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    name,
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', CONTROLLER_MANAGER_TIMEOUT,
                ],
                output='screen',
            )
            for name in CONTROLLERS
        ]

    actions.append(RegisterEventHandler(
        OnProcessExit(target_action=create, on_exit=_after_create)))

    # spawn_obstacle.sh takes WORLD coordinates, so "5 m ahead" has to be
    # computed from the spawn pose rather than hard-coded as (5, 0).
    ahead_x = spawn['x'] + 5.0 * math.cos(spawn['yaw'])
    ahead_y = spawn['y'] + 5.0 * math.sin(spawn['yaw'])
    actions.append(LogInfo(msg=(
        '\n[gerbil sim] Ready once "Configured and activated diff_drive_controller" appears.\n'
        '  Send a 3 m goal (or use the RViz "2D Goal Pose" tool, frame odom):\n'
        '    ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '
        '"{header: {frame_id: odom}, pose: {position: {x: 3.0}, orientation: {w: 1.0}}}"\n'
        f'  Drop an obstacle 5 m ahead:  ros2 run gerbil_gazebo spawn_obstacle.sh box '
        f'{ahead_x:.2f} {ahead_y:.2f}   (world coordinates)\n'
        '  Odometry vs truth:           the odom_error lines (odom_error:=false to silence)\n'
        '  Drive by keyboard:           ros2 run gerbil_bringup teleop_keyboard.sh'
    )))
    return actions


# (name, default, description)
_ARGS = [
    ('world', 'school_loop',
     'Key in config/worlds.yaml: school_loop, mdrs_utah, indiana_dunes'),
    ('nav2', 'true', 'Run Nav2 (gerbil_nav2.launch.py) on top of the base bringup'),
    ('rviz', 'true', 'RViz with rviz/sim.rviz. Needs a display'),
    ('foxglove', 'false', 'foxglove_bridge on port 8765'),
    ('slam', 'false', 'slam_toolbox with the robot config, building /map alongside'),
    ('depth_camera', 'false',
     'Simulated ZED depth cloud on /zed/zed_node/point_cloud/cloud_registered '
     '(costs CPU)'),
    ('gui', 'true', 'false = Gazebo server only (ign gazebo -s), for CI'),
    ('headless_rendering', 'false',
     'With gui:=false and NO X display: render sensors through EGL'),
    ('odom_error', 'true',
     'Run scripts/odom_error.py: EKF vs ground truth every 2 s, plus the '
     'aligned truth for RViz'),
    ('gz_extra_args', '',
     'Extra args for ign gazebo, e.g. "-v 4" to debug plugin loading'),
]


def generate_launch_description():
    args = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in _ARGS]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
