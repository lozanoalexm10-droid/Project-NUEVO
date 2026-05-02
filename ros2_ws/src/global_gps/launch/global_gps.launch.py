"""
Launch file for the Global GPS stack on the Jetson Nano.

Starts:
  1. realsense2_camera  — RealSense D4xx driver (colour or IR)
  2. ground_localizer   — ArUco field localizer, publishes /global_gps/tag_detections

Usage (inside the Jetson Docker container):
    ros2 launch global_gps global_gps.launch.py
    ros2 launch global_gps global_gps.launch.py marker_size:=0.15 corner_ids:=[0,1,2,3]
    ros2 launch global_gps global_gps.launch.py image_source:=infra1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# image_source -> realsense flags + topic remappings + camera profile.
# Color  : 1280x720 RGB, 69° × 42° FOV. Default.
# Infra1 : 1280x720 mono IR, 87° × 58° FOV (wider). IR projector is disabled
#          so its dot pattern doesn't confuse ArUco.
_IMAGE_SOURCE_CONFIGS = {
    "color": {
        "rs_args": {
            "enable_color": "true",
            "enable_depth": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "align_depth.enable": "false",
            "rgb_camera.color_profile": "1280x720x15",
        },
        "remappings": [
            ("image_raw", "/camera/camera/color/image_raw"),
            ("camera_info", "/camera/camera/color/camera_info"),
        ],
    },
    "infra1": {
        "rs_args": {
            "enable_color": "false",
            "enable_depth": "false",
            "enable_infra1": "true",
            "enable_infra2": "false",
            "align_depth.enable": "false",
            "depth_module.emitter_enabled": "0",
            # IR shares the depth_module.depth_profile setting (same sensor).
            # Camera-advertised profiles depend on USB speed; query with:
            #   ros2 param describe /camera/camera depth_module.depth_profile
            # On USB 2.1 the widest 16:9 mode at usable framerate is 848x480x10
            # (full horizontal FOV ~87°). Move to USB 3 to unlock higher rates.
            "depth_module.depth_profile": "848x480x10",
        },
        "remappings": [
            ("image_raw", "/camera/camera/infra1/image_rect_raw"),
            ("camera_info", "/camera/camera/infra1/camera_info"),
        ],
    },
}


def _build_nodes(context, *args, **kwargs):
    source = LaunchConfiguration("image_source").perform(context)
    if source not in _IMAGE_SOURCE_CONFIGS:
        raise ValueError(
            f"image_source must be one of {list(_IMAGE_SOURCE_CONFIGS)}, got '{source}'"
        )
    cfg = _IMAGE_SOURCE_CONFIGS[source]

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("realsense2_camera"),
            "/launch/rs_launch.py",
        ]),
        launch_arguments=cfg["rs_args"].items(),
    )

    localizer_node = Node(
        package="global_gps",
        executable="ground_localizer",
        name="ground_localizer",
        output="screen",
        parameters=[{
            "marker_size": LaunchConfiguration("marker_size"),
            "corner_ids": LaunchConfiguration("corner_ids"),
            "rover_ids": LaunchConfiguration("rover_ids"),
            "tcp_port": LaunchConfiguration("tcp_port"),
            "calibration_layout_file": LaunchConfiguration("calibration_layout_file"),
        }],
        remappings=cfg["remappings"],
    )

    return [realsense_launch, localizer_node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "marker_size",
            default_value="0.10",
            description="Physical side length of ArUco markers in metres.",
        ),
        DeclareLaunchArgument(
            "corner_ids",
            default_value="[0, 1, 2, 3]",
            description="Marker IDs used as fixed field-corner anchors.",
        ),
        DeclareLaunchArgument(
            "rover_ids",
            default_value="[11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27]",
            description="Marker IDs that appear on rovers.",
        ),
        DeclareLaunchArgument(
            "tcp_port",
            default_value="7777",
            description="TCP port for the robot push server (NAT-friendly delivery).",
        ),
        DeclareLaunchArgument(
            "calibration_layout_file",
            default_value="",
            description=(
                "Path to a YAML file with world-frame coordinates of the four "
                "calibration tags. Empty -> use packaged default at "
                "<global_gps share>/config/calibration_layout.yaml."
            ),
        ),
        DeclareLaunchArgument(
            "image_source",
            default_value="color",
            choices=list(_IMAGE_SOURCE_CONFIGS),
            description=(
                "Which camera stream to feed the localizer. "
                "'color' = RGB (69°×42° FOV); "
                "'infra1' = left IR camera (87°×58° FOV, wider, IR emitter off)."
            ),
        ),
        OpaqueFunction(function=_build_nodes),
    ])
