from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(

            package="odete",

            executable="detector_node",

            name="detector_node",

            output="screen",

            emulate_tty=True,

            parameters=[
                "/home/quack/scriptchi/delta/src/odete/config/detector_params.yaml"
            ]
        )
    ])