"""
test_position_fusion.py — GPS-anchored position fusion live test
================================================================
Drives the robot in a straight line for a fixed distance, recording raw
odometry and fused position at every kinematics tick.  After the run,
matplotlib plots:
  1. X position over time — raw odometry vs fused
  2. Y position over time — raw odometry vs fused
  3. 2D trajectory — raw odometry path vs fused path
  4. Fused − odometry position error magnitude over time, with GPS-active
     regions shaded in green

The GPS offset must be set before running if the GPS frame origin does not
coincide with the robot start corner.  Edit GPS_OFFSET_X_MM / GPS_OFFSET_Y_MM
below once the offset has been measured.

Usage:
    ros2 launch robot test_position_fusion.launch.py
"""

from __future__ import annotations

import math
import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

from robot.robot import FirmwareState, Robot
from robot.hardware_map import DEFAULT_FSM_HZ

# ── Tunable parameters ────────────────────────────────────────────────────────

DRIVE_DISTANCE_MM = 500.0    # mm — straight-line travel distance
DRIVE_SPEED_MM_S  = 100.0    # mm/s forward speed
POS_FUSION_ALPHA  = 0.10     # GPS weight for complementary filter (0–1)

# GPS frame → arena frame offset (mm).
# Set these to the measured offset once calibrated; leave at 0 to trigger the
# "offset not configured" warning and test uncompensated behaviour.
GPS_OFFSET_X_MM   = 0.0
GPS_OFFSET_Y_MM   = 0.0

# ── Plot output path ──────────────────────────────────────────────────────────
# /host_home is the host $HOME bind-mounted in docker-compose.rpi.yml.
# Fall back to the container's own home if the mount is absent (e.g. vm target).
_HOST_HOME = "/host_home"
_PLOT_PATH = (
    os.path.join(_HOST_HOME, "position_fusion_test_result.png")
    if os.path.isdir(_HOST_HOME)
    else os.path.expanduser("~/position_fusion_test_result.png")
)


# =============================================================================
# Data collection
# =============================================================================

class _Record:
    def __init__(self) -> None:
        self.t:          list[float] = []
        self.odom_x:     list[float] = []
        self.odom_y:     list[float] = []
        self.fused_x:    list[float] = []
        self.fused_y:    list[float] = []
        self.gps_active: list[bool]  = []

    def append(self, t, odom_x, odom_y, fused_x, fused_y, gps_active) -> None:
        self.t.append(t)
        self.odom_x.append(odom_x)
        self.odom_y.append(odom_y)
        self.fused_x.append(fused_x)
        self.fused_y.append(fused_y)
        self.gps_active.append(gps_active)

    def arrays(self):
        return (
            np.array(self.t),
            np.array(self.odom_x),
            np.array(self.odom_y),
            np.array(self.fused_x),
            np.array(self.fused_y),
            np.array(self.gps_active, dtype=bool),
        )


# =============================================================================
# Straight-line drive
# =============================================================================

def _drive_straight(robot: Robot, rec: _Record) -> None:
    """Drive forward until DRIVE_DISTANCE_MM is reached, recording every tick."""
    period = 1.0 / float(DEFAULT_FSM_HZ)

    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=1.0)

    t_start = time.monotonic()
    next_tick = t_start

    print(
        f"[pos_fusion_test] Driving straight {DRIVE_DISTANCE_MM:.0f} mm "
        f"at {DRIVE_SPEED_MM_S:.0f} mm/s\n"
        f"[pos_fusion_test] GPS offset: ({GPS_OFFSET_X_MM:.1f}, {GPS_OFFSET_Y_MM:.1f}) mm"
    )

    while True:
        robot.wait_for_pose_update(timeout=period * 2)

        # Raw odometry (private, accessed here for diagnostic purposes only)
        with robot._lock:
            odom_x, odom_y, _ = robot._pose

        fused_x_mm, fused_y_mm, _ = robot._get_pose_mm()
        gps_on = robot.is_gps_active()

        elapsed = time.monotonic() - t_start
        rec.append(elapsed, odom_x, odom_y, fused_x_mm, fused_y_mm, gps_on)

        dist_traveled = math.hypot(odom_x, odom_y)
        if dist_traveled >= DRIVE_DISTANCE_MM:
            break

        robot.set_velocity(DRIVE_SPEED_MM_S, 0.0)

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()

    robot.stop()
    print(
        f"[pos_fusion_test] Done — travelled {math.hypot(odom_x, odom_y):.1f} mm "
        f"in {time.monotonic() - t_start:.1f} s"
    )


# =============================================================================
# Plotting
# =============================================================================

def _shade_gps_regions(ax, t: np.ndarray, gps_active: np.ndarray) -> None:
    """Shade time periods when GPS was active green."""
    in_region = False
    t_start = 0.0
    for i, active in enumerate(gps_active):
        if active and not in_region:
            t_start = t[i]
            in_region = True
        elif not active and in_region:
            ax.axvspan(t_start, t[i], alpha=0.15, color="green", label="_nolegend_")
            in_region = False
    if in_region:
        ax.axvspan(t_start, t[-1], alpha=0.15, color="green", label="_nolegend_")


def _plot_results(rec: _Record) -> None:
    t, odom_x, odom_y, fused_x, fused_y, gps_active = rec.arrays()

    error_mm = np.hypot(fused_x - odom_x, fused_y - odom_y)

    try:
        import matplotlib
        matplotlib.use("TkAgg")
    except Exception:
        pass

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(
        f"Position fusion test — straight line {DRIVE_DISTANCE_MM:.0f} mm  "
        f"(α={POS_FUSION_ALPHA}, offset=({GPS_OFFSET_X_MM:.0f}, {GPS_OFFSET_Y_MM:.0f}) mm)",
        fontsize=12,
    )
    gps_patch = mpatches.Patch(color="green", alpha=0.3, label="GPS active")

    # ── Panel 1: X position over time ─────────────────────────────────────────
    ax = axes[0, 0]
    _shade_gps_regions(ax, t, gps_active)
    ax.plot(t, odom_x,  lw=1.2, color="steelblue", label="Odometry X")
    ax.plot(t, fused_x, lw=1.5, color="tomato", linestyle="--", label="Fused X")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("X position (mm)")
    ax.set_title("X position over time")
    ax.legend(handles=[*ax.get_legend_handles_labels()[0][:2], gps_patch], fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── Panel 2: Y position over time ─────────────────────────────────────────
    ax = axes[0, 1]
    _shade_gps_regions(ax, t, gps_active)
    ax.plot(t, odom_y,  lw=1.2, color="steelblue", label="Odometry Y")
    ax.plot(t, fused_y, lw=1.5, color="tomato", linestyle="--", label="Fused Y")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("Y position (mm)")
    ax.set_title("Y position over time")
    ax.legend(handles=[*ax.get_legend_handles_labels()[0][:2], gps_patch], fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── Panel 3: 2D trajectory ────────────────────────────────────────────────
    ax = axes[1, 0]
    ax.plot(odom_x,  odom_y,  lw=1.2, color="steelblue", label="Odometry path")
    ax.plot(fused_x, fused_y, lw=1.5, color="tomato", linestyle="--", label="Fused path")
    ax.plot(odom_x[0],  odom_y[0],  "go", markersize=8, label="start")
    ax.plot(odom_x[-1], odom_y[-1], "rs", markersize=8, label="end (odom)")
    ax.plot(fused_x[-1], fused_y[-1], "r^", markersize=8, label="end (fused)")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title("2D trajectory")
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── Panel 4: Position error magnitude ────────────────────────────────────
    ax = axes[1, 1]
    _shade_gps_regions(ax, t, gps_active)
    ax.fill_between(t, error_mm, alpha=0.3, color="darkorange")
    ax.plot(t, error_mm, lw=1.2, color="darkorange", label="|fused − odom|")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("error (mm)")
    ax.set_title("Fused − odometry error magnitude")
    ax.legend(handles=[*ax.get_legend_handles_labels()[0][:1], gps_patch], fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    try:
        plt.savefig(_PLOT_PATH, dpi=150)
        print(f"[pos_fusion_test] Plot saved → {_PLOT_PATH}")
    except Exception as exc:
        print(f"[pos_fusion_test] Could not save plot: {exc}")

    try:
        plt.show()
    except Exception:
        pass


# =============================================================================
# Entry point
# =============================================================================

def run(robot: Robot) -> None:
    robot.set_gps_offset(GPS_OFFSET_X_MM, GPS_OFFSET_Y_MM)
    robot.set_pos_fusion_alpha(POS_FUSION_ALPHA)

    rec = _Record()
    _drive_straight(robot, rec)
    _plot_results(rec)


# =============================================================================
# ROS2 node entry point
# =============================================================================

def main(args=None) -> None:
    import signal
    import threading

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    from robot.robot import Robot as _Robot

    class _TestNode(Node):
        def __init__(self) -> None:
            super().__init__("position_fusion_test")
            self.robot = _Robot(self)

    node = _TestNode()

    def _spin() -> None:
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    def _raise_keyboard_interrupt(signum, frame):
        raise KeyboardInterrupt()

    old_sigint  = signal.getsignal(signal.SIGINT)
    old_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT,  _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    try:
        run(node.robot)
    except KeyboardInterrupt:
        node.get_logger().info("test interrupted; shutting down")
    finally:
        try:
            node.robot.shutdown()
        except Exception:
            pass
        signal.signal(signal.SIGINT,  old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
