from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="topic_publisher_pkg",
            executable="simple_topic_publisher",
            output="screen",
            emulate_tty="true"
        ),
    ])