from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slam_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maxence',
    maintainer_email='maxence@todo.todo',
    description='ROS 2 package for SLAM and Autonomous Navigation in Gazebo Harmonic.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'waypoint_follower = slam_robot.waypoint_follower:main',
            'nav_controller = slam_robot.nav_controller:main',
            'map_server = slam_robot.map_server:main',
            'dynamic_actors = slam_robot.dynamic_actors:main'
        ],
    },
)
