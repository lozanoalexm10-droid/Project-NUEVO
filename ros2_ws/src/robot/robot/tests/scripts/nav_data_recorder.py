"""
nav_data_recorder.py — passive ROS2 data recorder for navigation analysis.

Run this ALONGSIDE (not instead of) the driving script to capture data for
post-run plotting.  It is read-only: it subscribes to existing topics and
never commands the robot.

Usage (in a container shell, after sourcing ROS2):
    python3 nav_data_recorder.py [--out /runtime_output/nav_run.json]

Press Ctrl+C to stop.  The JSON file is written on shutdown.

Topics consumed:
    /sensor_kinematics   — raw odometry (x, y, theta)
    /fused_pose          — GPS-fused pose (x, y, theta, gps_active)
    /scan                — raw lidar scan (stored as one snapshot per second)
    /vision/detections   — detection events (class, confidence, attributes)
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from bridge_interfaces.msg import FusedPose, SensorKinematics, VisionDetectionArray

# Downsample rates — odometry and fused pose are ~30 Hz; we save at 5 Hz
_ODOM_SAVE_HZ   = 5.0
_LIDAR_SAVE_HZ  = 1.0   # one full scan snapshot per second


class NavRecorder(Node):
    def __init__(self, out_path: Path) -> None:
        super().__init__("nav_data_recorder")
        self._out = out_path
        self._t0  = time.monotonic()

        self._odom:        list[dict] = []
        self._fused:       list[dict] = []
        self._lidar:       list[dict] = []
        self._detections:  list[dict] = []

        self._last_odom_t  = -999.0
        self._last_lidar_t = -999.0
        self._latest_fused: dict | None = None  # for lidar world-frame projection

        self.create_subscription(SensorKinematics,    "/sensor_kinematics",   self._on_odom,   10)
        self.create_subscription(FusedPose,           "/fused_pose",          self._on_fused,  10)
        self.create_subscription(LaserScan,           "/scan",                self._on_scan,   10)
        self.create_subscription(VisionDetectionArray,"/vision/detections",   self._on_vision, 10)

        self.get_logger().info(f"[recorder] recording — output: {out_path}")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _t(self) -> float:
        return time.monotonic() - self._t0

    def _on_odom(self, msg: SensorKinematics) -> None:
        now = self._t()
        if now - self._last_odom_t < 1.0 / _ODOM_SAVE_HZ:
            return
        self._last_odom_t = now
        self._odom.append({
            "t":     round(now, 3),
            "x":     round(float(msg.x), 1),
            "y":     round(float(msg.y), 1),
            "theta": round(float(msg.theta), 4),
        })

    def _on_fused(self, msg: FusedPose) -> None:
        now = self._t()
        entry = {
            "t":          round(now, 3),
            "x":          round(float(msg.x), 1),
            "y":          round(float(msg.y), 1),
            "theta":      round(float(msg.theta), 4),
            "gps_active": bool(msg.gps_active),
        }
        self._latest_fused = entry
        # Save at the same rate as odom (already downsampled via _on_odom timer
        # but fused_pose fires at its own rate — use a separate gate)
        if not self._fused or now - self._fused[-1]["t"] >= 1.0 / _ODOM_SAVE_HZ:
            self._fused.append(entry)

    def _on_scan(self, msg: LaserScan) -> None:
        now = self._t()
        if now - self._last_lidar_t < 1.0 / _LIDAR_SAVE_HZ:
            return
        self._last_lidar_t = now

        # Convert polar scan to Cartesian robot frame, then world frame using
        # the latest fused pose (best available position estimate).
        angles  = msg.angle_min + (
            [i * msg.angle_increment for i in range(len(msg.ranges))]
            if msg.angle_increment else []
        )
        ranges  = list(msg.ranges)

        rx_list, ry_list = [], []
        for r, a in zip(ranges, angles):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            rx_list.append(r * math.cos(a) * 1000.0)  # m → mm
            ry_list.append(r * math.sin(a) * 1000.0)

        # Project to world frame if we have a fused pose
        p = self._latest_fused
        if p and rx_list:
            ct, st = math.cos(p["theta"]), math.sin(p["theta"])
            wx = [round(p["x"] + rx * ct - ry * st, 0) for rx, ry in zip(rx_list, ry_list)]
            wy = [round(p["y"] + rx * st + ry * ct, 0) for rx, ry in zip(rx_list, ry_list)]
            robot_x, robot_y = p["x"], p["y"]
        else:
            wx, wy = [round(v, 0) for v in rx_list], [round(v, 0) for v in ry_list]
            robot_x = robot_y = 0.0

        self._lidar.append({
            "t":       round(now, 3),
            "robot_x": round(robot_x, 1),
            "robot_y": round(robot_y, 1),
            "xs":      wx,
            "ys":      wy,
        })

    def _on_vision(self, msg: VisionDetectionArray) -> None:
        now = self._t()
        for det in msg.detections:
            attrs = {}
            for name, val in zip(det.attribute_names, det.attribute_values):
                attrs[name] = val
            self._detections.append({
                "t":          round(now, 3),
                "class_name": det.class_name,
                "confidence": round(float(det.confidence), 3),
                "x": int(det.x), "y": int(det.y),
                "w": int(det.width), "h": int(det.height),
                "attrs": attrs,
            })

    # ── Save ──────────────────────────────────────────────────────────────────

    def save(self) -> None:
        self._out.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "meta": {
                "recorded_at": datetime.now().isoformat(timespec="seconds"),
                "duration_s":  round(self._t(), 1),
                "n_odom":      len(self._odom),
                "n_fused":     len(self._fused),
                "n_lidar":     len(self._lidar),
                "n_detections":len(self._detections),
            },
            "odom":       self._odom,
            "fused":      self._fused,
            "lidar":      self._lidar,
            "detections": self._detections,
        }
        self._out.write_text(json.dumps(data, indent=2))
        self.get_logger().info(
            f"[recorder] saved {len(self._odom)} odom, {len(self._fused)} fused, "
            f"{len(self._lidar)} lidar, {len(self._detections)} detections → {self._out}"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out",
        default=f"/runtime_output/nav_run_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json",
        help="Output JSON path",
    )
    args = parser.parse_args()
    out_path = Path(args.out)

    rclpy.init()
    node = NavRecorder(out_path)

    def _shutdown(sig, frame):  # noqa: ARG001
        node.save()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print(f"[recorder] recording to {out_path} — Ctrl+C to stop and save")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
