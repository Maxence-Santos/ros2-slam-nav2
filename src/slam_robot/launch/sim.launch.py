import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('slam_robot')

    world_type_arg = DeclareLaunchArgument(
        'world_type',
        default_value='dynamic',
        description='World type: "static" (for SLAM without characters) or "dynamic" (for Nav with characters)'
    )
    
    world_type = LaunchConfiguration('world_type')

    # Process URDF file
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    
    # Gazebo Launch (Static world for SLAM, Dynamic world for Nav)
    world_static = os.path.join(pkg_share, 'worlds', 'warehouse_static.sdf')
    world_dynamic = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')

    gazebo_static = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_static],
        condition=IfCondition(PythonExpression(["'", world_type, "' == 'static'"])),
        output='screen'
    )

    gazebo_dynamic = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_dynamic],
        condition=UnlessCondition(PythonExpression(["'", world_type, "' == 'static'"])),
        output='screen'
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf, 'use_sim_time': True}]
    )
    
    # Spawn Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{'use_sim_time': True}],
        arguments=['-topic', 'robot_description',
                   '-name', 'slam_robot',
                   '-z', '0.1'],
        output='screen'
    )
    
    # Bridge for ROS 2 -> Gazebo topics
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/warehouse/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/slam_robot/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/slam_robot/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/person_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/person_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/person_3/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/slam_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/slam_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            ('/world/warehouse/clock', '/clock'),
            ('/model/slam_robot/scan', '/scan'),
            ('/model/slam_robot/imu/data', '/imu/data'),
            ('/model/slam_robot/odometry', '/odom'),
            ('/model/slam_robot/tf', '/tf')
        ],
        output='screen'
    )

    # Dynamic Actors Controller (Only launched when world_type == 'dynamic')
    dynamic_actors_node = Node(
        package='slam_robot',
        executable='dynamic_actors',
        name='dynamic_actors',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(PythonExpression(["'", world_type, "' == 'static'"])),
        output='screen'
    )

    # RViz
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam_nav.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        world_type_arg,
        gazebo_static,
        gazebo_dynamic,
        robot_state_publisher_node,
        spawn_entity,
        bridge_node,
        dynamic_actors_node,
        rviz_node
    ])
