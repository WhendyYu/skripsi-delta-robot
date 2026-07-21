"""Launch file for delta robot kinematics service"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('kinematics')
    
    # Path to config file
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'kinematics.yaml'
    ])
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to configuration file'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logging'
    )
    
    # IK Service Node
    ik_service_node = Node(
        package='kinematics',
        executable='ik_service',
        name='ik_service',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        arguments=['--ros-args', '--log-level', 
                  ['debug' if LaunchConfiguration('debug') else 'info']]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        debug_arg,
        ik_service_node
    ])