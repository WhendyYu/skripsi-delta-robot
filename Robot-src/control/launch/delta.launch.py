from launch import LaunchDescription

from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():

    config = os.path.join(
            get_package_share_directory('control'),
            'config',
            'config.yaml'
        )

    with open(config, 'r') as f:
        cfg = yaml.safe_load(f)

    vision_enabled = cfg['delta_controller']['ros__parameters']['vision_enabled']

    return LaunchDescription([

        # ======================================================
        # Controller
        # ======================================================

        Node(
            package='control',
            executable='controller',
            name='delta_controller',
            output='screen',
            parameters=[config]
        ),

        # ======================================================
        # Serial Bridge
        # ======================================================

        Node(
            package='hw',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[config]
        ),

        # ======================================================
        # IK Service
        # ======================================================

        Node(
            package='kinematics',
            executable='ik_service',
            name='ik_service',
            output='screen',
            parameters=[config]
        ),

        # ======================================================
        # ODETE
        # ======================================================

        Node(
            package='odete',
            executable='odete',
            name='odete',
            output='screen',
            parameters=[config],
            condition=IfCondition(
                str(vision_enabled)
            )
        )
    ])