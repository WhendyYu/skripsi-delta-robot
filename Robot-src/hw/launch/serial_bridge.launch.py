from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('hw')
    
    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'config_file',
        #     default_value=config_file,
        #     description='Path to config file'
        # ),
        
        Node(
            package='hw',
            executable='serial_bridge',
            name='serial_bridge',
            # parameters=[LaunchConfiguration('config_file')],
            output='screen',
            respawn=True,
            respawn_delay=2.0
        )
    ])