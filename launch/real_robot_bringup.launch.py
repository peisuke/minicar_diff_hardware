#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Launch arguments
    robot_ns_arg = DeclareLaunchArgument(
        'robot_ns',
        default_value='real_robot',
        description='Robot namespace'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    rplidar_serial_port_arg = DeclareLaunchArgument(
        'rplidar_serial_port', 
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLiDAR'
    )
    
    # Launch configurations
    robot_ns = LaunchConfiguration('robot_ns')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rplidar_serial_port = LaunchConfiguration('rplidar_serial_port')
    
    return LaunchDescription([
        robot_ns_arg,
        use_sim_time_arg,
        rplidar_serial_port_arg,
        
        # Robot State Publisher (実機用URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_ns,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': PathJoinSubstitution([
                    FindPackageShare('minicar_diff_hardware'),
                    'config', 'minicar_diff_real.xacro'
                ])
            }]
        ),
        
        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            namespace=robot_ns,
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('minicar_diff_hardware'),
                    'config', 'minicar_hardware_config.yaml'
                ]),
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', ["/", robot_ns, "/controller_manager"]],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Diff Drive Controller
        Node(
            package='controller_manager', 
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', ["/", robot_ns, "/controller_manager"]],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # RPLiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rplidar_ros2'),
                    'launch',
                    'rplidar.launch.py'
                ])
            ]),
            launch_arguments={
                'serial_port': rplidar_serial_port,
                'frame_id': 'laser'
            }.items()
        ),
    ])