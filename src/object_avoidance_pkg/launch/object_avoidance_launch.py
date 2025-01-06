from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="object_avoidance_pkg",
            executable="robot_avoidance_node",
            output="screen",
            emulate_tty=True
        ),
    ])
