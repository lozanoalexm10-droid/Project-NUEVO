"""
lidar_renderer.py — subscribes to /scan and writes a top-down PNG every second.

Run standalone:
    ros2 run robot lidar_renderer
or with a custom output path:
    ros2 run robot lidar_renderer --ros-args -p output_path:=/tmp/lidar.png
"""
from __future__ import annotations

import os
import threading
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _HAS_MPL = True
except ImportError:
    _HAS_MPL = False

_DEFAULT_OUT = os.environ.get(
    "NUEVO_LIDAR_OUTPUT_DIR", "/runtime_output/lidar"
) + "/latest.png"

RANGE_MAX_M = 8.0   # clip far returns
FIG_SIZE    = 5     # inches, square


class LidarRenderer(Node):
    def __init__(self) -> None:
        super().__init__("lidar_renderer")
        self.declare_parameter("output_path", _DEFAULT_OUT)
        self._out = Path(self.get_parameter("output_path").get_parameter_value().string_value)
        self._out.parent.mkdir(parents=True, exist_ok=True)

        self._lock = threading.Lock()
        self._latest: LaserScan | None = None
        self._dirty = False

        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        # Render at ~1 Hz regardless of scan rate
        self.create_timer(1.0, self._render)

        if not _HAS_MPL:
            self.get_logger().error("matplotlib not found — lidar_renderer will not produce images")

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._latest = msg
            self._dirty = True

    def _render(self) -> None:
        if not _HAS_MPL:
            return
        with self._lock:
            if not self._dirty or self._latest is None:
                return
            msg = self._latest
            self._dirty = False

        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Filter invalid / out-of-range points
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < min(msg.range_max, RANGE_MAX_M))
        r = ranges[valid]
        a = angles[valid]

        xs = r * np.cos(a)
        ys = r * np.sin(a)

        fig, ax = plt.subplots(figsize=(FIG_SIZE, FIG_SIZE), facecolor="#111827")
        ax.set_facecolor("#111827")

        # Grid rings
        for radius in [1, 2, 3, 4]:
            circle = plt.Circle((0, 0), radius, color="#374151", fill=False, linewidth=0.5)
            ax.add_patch(circle)
            ax.text(0, radius, f"{radius}m", color="#6B7280", fontsize=6, ha="center", va="bottom")

        # Cross-hairs
        ax.axhline(0, color="#374151", linewidth=0.5)
        ax.axvline(0, color="#374151", linewidth=0.5)

        # Robot marker
        ax.plot(0, 0, "o", color="#F59E0B", markersize=6, zorder=5)

        # Scan points
        ax.scatter(xs, ys, s=1.5, c="#34D399", alpha=0.85, linewidths=0)

        lim = min(RANGE_MAX_M, 5.0)
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_aspect("equal")
        ax.tick_params(colors="#6B7280", labelsize=6)
        for spine in ax.spines.values():
            spine.set_edgecolor("#374151")
        ax.set_xlabel("x (m)", color="#9CA3AF", fontsize=7)
        ax.set_ylabel("y (m)", color="#9CA3AF", fontsize=7)
        ax.set_title(f"LiDAR  ·  {len(r)} pts", color="#D1D5DB", fontsize=8, pad=4)

        tmp = str(self._out) + ".tmp"
        fig.savefig(tmp, dpi=120, bbox_inches="tight", facecolor=fig.get_facecolor())
        plt.close(fig)
        os.replace(tmp, self._out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarRenderer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
