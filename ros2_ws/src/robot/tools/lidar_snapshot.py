#!/usr/bin/env python3
"""
lidar_snapshot.py — grab one /scan message and save a top-down PNG.

Usage (inside the ros2_runtime container, after sourcing the workspace):
    python3 /ros2_ws/src/robot/tools/lidar_snapshot.py [output_path]

Default output: /runtime_output/lidar_scan.png  (visible on the host at
ros2_ws/runtime_output/lidar_scan.png since /runtime_output is bind-mounted).
"""
from __future__ import annotations

import math
import sys

import matplotlib
matplotlib.use("Agg")  # no display needed
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class Snapshot(Node):
    def __init__(self, out_path: str):
        super().__init__("lidar_snapshot")
        self.out_path = out_path
        self.got_scan = False
        # RPLIDAR publishes BEST_EFFORT; matching is required or we miss messages.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos)
        self.get_logger().info("Waiting for /scan...")

    def _on_scan(self, msg: LaserScan) -> None:
        if self.got_scan:
            return
        self.got_scan = True

        ranges = np.array(msg.ranges, dtype=float)
        n = len(ranges)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment

        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        r = ranges[valid]
        a = angles[valid]
        # Robot frame: x = forward, y = left
        x = r * np.cos(a)
        y = r * np.sin(a)

        fig, ax = plt.subplots(figsize=(8, 8), facecolor="#1a1a2e")
        ax.set_facecolor("#12121f")
        ax.scatter(x, y, s=4, c="#5af0c2")
        ax.plot([0], [0], marker="o", markersize=8, color="#ff5577")  # robot
        ax.arrow(0, 0, 0.3, 0, head_width=0.08, color="#ff5577")       # forward
        rmax = float(min(msg.range_max, 6.0))
        ax.set_xlim(-rmax, rmax)
        ax.set_ylim(-rmax, rmax)
        ax.set_aspect("equal")
        ax.set_xlabel("x forward (m)", color="white")
        ax.set_ylabel("y left (m)", color="white")
        ax.set_title(
            f"LiDAR scan — {int(valid.sum())}/{n} pts, "
            f"angle [{math.degrees(msg.angle_min):.0f}°, {math.degrees(msg.angle_max):.0f}°]",
            color="white",
        )
        ax.tick_params(colors="white")
        for s in ax.spines.values():
            s.set_edgecolor("#555577")
        ax.grid(True, color="#333355", linestyle=":")
        fig.tight_layout()
        fig.savefig(self.out_path, dpi=120, facecolor=fig.get_facecolor())
        plt.close(fig)
        self.get_logger().info(f"Saved {self.out_path} ({int(valid.sum())} points)")


def main() -> None:
    out_path = sys.argv[1] if len(sys.argv) > 1 else "/runtime_output/lidar_scan.png"
    rclpy.init()
    node = Snapshot(out_path)
    try:
        while rclpy.ok() and not node.got_scan:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
