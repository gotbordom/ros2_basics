from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="exercise42_pkg",
            executable="exercise42_service_node",
            output="screen",
            emulate_tty=True
        ),
    ])