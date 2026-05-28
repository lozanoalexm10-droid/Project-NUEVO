from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="bridge",
            executable="bridge",
            name="bridge",
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("rplidar_ros"), "launch", "rplidar_c1.launch.py"]
                )
            ),
            launch_arguments={
                "serial_port": "/dev/rplidar",
                "serial_baudrate": "460800",
                "frame_id": "laser_frame",
                "topic_name": "scan",
                "scan_mode": "Standard",
                "angle_compensate": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("sensors"), "launch", "sensors.launch.py"]
                )
            ),
        ),
    ])
