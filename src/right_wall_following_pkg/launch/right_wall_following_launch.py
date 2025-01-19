from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="right_wall_following_pkg",
            executable="right_wall_following",
            output="screen",
            emulate_tty=True
,        )
    ])