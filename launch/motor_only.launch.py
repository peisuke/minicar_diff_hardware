#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

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
    
    # Launch configurations
    robot_ns = LaunchConfiguration('robot_ns')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # URDF/XACRO file for hardware
    xacro_file = PathJoinSubstitution([
        FindPackageShare('minicar_diff_hardware'),
        'config', 'minicar_diff_real.xacro'
    ])
    
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' robot_ns:=', robot_ns]), value_type=str)
    
    # Hardware config file
    hardware_config = PathJoinSubstitution([
        FindPackageShare('minicar_diff_hardware'),
        'config', 'minicar_hardware_config.yaml'
    ])
    
    return LaunchDescription([
        robot_ns_arg,
        use_sim_time_arg,
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_ns,
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),
        
        # Controller Manager (hardware interface) - with robot_description parameter
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=robot_ns,
            output='screen',
            parameters=[
                hardware_config,
                {'use_sim_time': use_sim_time,
                 'robot_description': robot_description}
            ]
        ),
        
        # Joint State Broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", ["/", robot_ns, "/controller_manager"]],
            output="screen",
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
    ])