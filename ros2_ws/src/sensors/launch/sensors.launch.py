from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sensors",
            executable="robot_gps",
            name="robot_gps",
            output="screen",
        ),
        Node(
            package="sensors",
            executable="qwiic_ultrasonic",
            name="qwiic_ultrasonic",
            output="screen",
        ),
    ])
