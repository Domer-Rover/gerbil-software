# launch/bno055_imu.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('imu_package')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'bno055_params.yaml')
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM1',
        description='Serial port for BNO055 IMU'
    )
    
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
            'serial_port': LaunchConfiguration('serial_port'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        remappings=[
            ('/imu/data', '/imu/data'),
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        publish_rate_arg,
        imu_node
    ])
