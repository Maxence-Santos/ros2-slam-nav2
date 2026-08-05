import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('slam_robot')
    config_file = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    default_map_file = os.path.join(pkg_share, 'maps', 'warehouse_map')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='SLAM mode: mapping or localization'
    )

    map_file_arg = DeclareLaunchArgument(
        'map_file_name',
        default_value=default_map_file,
        description='Full path to map file without extension for localization'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': True,
                'autostart': True,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'mode': LaunchConfiguration('mode'),
                'map_file_name': LaunchConfiguration('map_file_name')
            }
        ]
    )

    return LaunchDescription([
        mode_arg,
        map_file_arg,
        slam_toolbox_node
    ])
