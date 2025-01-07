from launch import LaunchDescription
from launch_ros import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="actions_quiz",
            executable="actions_quiz_server_node",
            output="screen",
            emulate_tty=True
        ),
    ])