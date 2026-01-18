#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # in case urdf is .xacro, process it, otherwise read in urdf
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('gerbil_description'), 'urdf', 'my_robot.urdf.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # controller configuration file
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('gerbil_hw'),
            'config',
            'controller_manager.yaml',
        ]
    )

    # controller manager node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
    )

    # robot state publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # join state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # diff drive
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gerbil_base_controller', '--controller-manager', '/controller_manager'],
    )

    # delay launches
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)