# launch/bno055_imu.launch.py
#
# The BNO055 is on I2C (bus and address in config/bno055_params.yaml); the
# node opens no serial port.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('imu_package')

    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'bno055_params.yaml')

    # Declare launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='IMU data publishing rate in Hz'
    )

    # BNO055 IMU node
    imu_node = Node(
        package='imu_package',
        executable='bno055_imu',
        name='bno055_imu_node',
        output='screen',
        parameters=[config_file, {
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        remappings=[
            ('/imu/data', '/imu/data'),
        ]
    )

    return LaunchDescription([
        publish_rate_arg,
        imu_node
    ])
