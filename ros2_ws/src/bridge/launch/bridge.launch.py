from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="bridge",
            executable="bridge",
            name="bridge",
            output="screen",
        ),
        Node(
            package="sensors",
            executable="qwiic_ultrasonic",
            name="qwiic_ultrasonic",
            output="screen",
        ),
    ])
