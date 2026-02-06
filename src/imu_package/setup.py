from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'imu_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='drnd@nd.edu',
    description='BNO055 IMU driver for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno055_imu = imu_package.bno055_imu_node:main',
            'imu_test_sub = imu_package.imu_test_subscriber:main',
        ],
    },
)
