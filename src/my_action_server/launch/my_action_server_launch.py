from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="my_action_server",
            executable="my_action_server_node",
            output="screen",
            emulate_tty=True
        ),
    ])