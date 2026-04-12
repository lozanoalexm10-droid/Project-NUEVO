"""
test_orientation_fusion.py — complementary-filter orientation test
==================================================================
Drives the robot in two circles, recording odometry theta and the
complementary-filter (magnetometer-blended) heading at every tick.
After the run, matplotlib plots:
  1. Trajectory — x/y path from odometry, coloured by fused-heading drift
  2. Heading comparison — raw odometry vs fused heading over time
  3. Fusion correction — per-sample difference (fused − odometry)

Usage (replace the run() import in main.py, or use the dedicated launch file):
    ros2 launch robot test_orientation_fusion.launch.py

Tuning parameters at the top of this file:
    CIRCLE_RADIUS_MM  — circle radius (mm)
    DRIVE_SPEED_MM_S  — forward speed (mm/s)
    NUM_LAPS          — how many full circles to complete
    FUSION_ALPHA      — complementary-filter mag weight (0 = pure odometry)
"""

from __future__ import annotations

import math
import time

import matplotlib
matplotlib.use("Agg")   # headless-safe; switched to TkAgg below if a display exists
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

from robot.robot import FirmwareState, Robot
from robot.hardware_map import DEFAULT_FSM_HZ

# ── Tunable parameters ────────────────────────────────────────────────────────

CIRCLE_RADIUS_MM  = 300.0    # mm — tighter circles stress heading fusion more
DRIVE_SPEED_MM_S  = 100.0    # mm/s forward speed
NUM_LAPS          = 2        # number of full circles to complete
FUSION_ALPHA      = 0.02     # complementary-filter magnetometer weight

# Derived motion command
_CIRCLE_PERIOD_S  = 2.0 * math.pi * CIRCLE_RADIUS_MM / DRIVE_SPEED_MM_S
_ANGULAR_DEG_S    = math.degrees(DRIVE_SPEED_MM_S / CIRCLE_RADIUS_MM)  # CCW positive

# ── Plot output path ──────────────────────────────────────────────────────────
# /host_home is the host $HOME bind-mounted in docker-compose.rpi.yml.
# Fall back to the container's own home if the mount is absent (e.g. vm target).
import os
_HOST_HOME = "/host_home"
_PLOT_PATH = (
    os.path.join(_HOST_HOME, "fusion_test_result.png")
    if os.path.isdir(_HOST_HOME)
    else os.path.expanduser("~/fusion_test_result.png")
)


# =============================================================================
# Data collection helpers
# =============================================================================

class _Record:
    """Lightweight ring-buffer replacement — just appends to lists."""
    def __init__(self) -> None:
        self.t:           list[float] = []
        self.x:           list[float] = []
        self.y:           list[float] = []
        self.odom_deg:    list[float] = []
        self.fused_deg:   list[float] = []

    def append(self, t, x, y, odom_deg, fused_deg) -> None:
        self.t.append(t)
        self.x.append(x)
        self.y.append(y)
        self.odom_deg.append(odom_deg)
        self.fused_deg.append(fused_deg)

    # Convenience: arrays for numpy/matplotlib
    def arrays(self):
        return (
            np.array(self.t),
            np.array(self.x),
            np.array(self.y),
            np.array(self.odom_deg),
            np.array(self.fused_deg),
        )


# =============================================================================
# Circle drive
# =============================================================================

def _drive_circle(robot: Robot, rec: _Record) -> None:
    """Command the robot to drive circles and record sensor data."""
    total_duration = _CIRCLE_PERIOD_S * NUM_LAPS
    period = 1.0 / float(DEFAULT_FSM_HZ)

    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.5)

    t_start = time.monotonic()
    next_tick = t_start

    print(
        f"[fusion_test] Driving {NUM_LAPS} circle(s): "
        f"R={CIRCLE_RADIUS_MM:.0f} mm, v={DRIVE_SPEED_MM_S:.0f} mm/s, "
        f"ω={_ANGULAR_DEG_S:.1f}°/s, α={FUSION_ALPHA}\n"
        f"[fusion_test] Expected duration: {total_duration:.1f} s"
    )

    while True:
        elapsed = time.monotonic() - t_start
        if elapsed >= total_duration:
            break

        x, y, odom_deg = robot.get_pose()
        fused_deg = robot.get_fused_orientation()
        rec.append(elapsed, x, y, odom_deg, fused_deg)

        robot.set_velocity(DRIVE_SPEED_MM_S, _ANGULAR_DEG_S)

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()

    robot.stop()
    print("[fusion_test] Circle complete — stopping.")


# =============================================================================
# Plotting
# =============================================================================

def _plot_results(rec: _Record) -> None:
    t, x, y, odom_deg, fused_deg = rec.arrays()

    correction = fused_deg - odom_deg
    # Wrap correction to [-180, 180]
    correction = (correction + 180.0) % 360.0 - 180.0

    try:
        import matplotlib
        matplotlib.use("TkAgg")
    except Exception:
        pass  # stay with Agg if no display

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle(
        f"Complementary-filter fusion test  "
        f"(α={FUSION_ALPHA}, R={CIRCLE_RADIUS_MM:.0f} mm, "
        f"{NUM_LAPS} lap{'s' if NUM_LAPS != 1 else ''})",
        fontsize=12,
    )

    # ── Panel 1: Trajectory coloured by fusion correction magnitude ──────────
    ax = axes[0]
    corr_abs = np.abs(correction)
    sc = ax.scatter(x, y, c=corr_abs, cmap="plasma", s=6, zorder=2)
    ax.plot(x[0], y[0], "go", markersize=8, label="start", zorder=3)
    ax.plot(x[-1], y[-1], "rs", markersize=8, label="end", zorder=3)
    fig.colorbar(sc, ax=ax, label="|fusion correction| (°)")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title("Trajectory")
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── Panel 2: Heading over time ────────────────────────────────────────────
    ax = axes[1]
    ax.plot(t, odom_deg,  lw=1.2, label="Odometry θ", color="steelblue")
    ax.plot(t, fused_deg, lw=1.5, label="Fused θ (CF)",  color="tomato",  linestyle="--")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("heading (°)")
    ax.set_title("Heading: odometry vs fused")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ── Panel 3: Fusion correction over time ──────────────────────────────────
    ax = axes[2]
    ax.axhline(0, color="gray", lw=0.8, linestyle=":")
    ax.fill_between(t, correction, alpha=0.3, color="tomato")
    ax.plot(t, correction, lw=1.2, color="tomato", label="fused − odom")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("correction (°)")
    ax.set_title("Fusion correction (fused − odometry)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    try:
        plt.savefig(_PLOT_PATH, dpi=150)
        print(f"[fusion_test] Plot saved → {_PLOT_PATH}")
    except Exception as exc:
        print(f"[fusion_test] Could not save plot: {exc}")

    try:
        plt.show()
    except Exception:
        pass  # headless — plot already saved


# =============================================================================
# Entry point
# =============================================================================

def run(robot: Robot) -> None:
    robot.set_fusion_alpha(FUSION_ALPHA)

    rec = _Record()
    _drive_circle(robot, rec)
    _plot_results(rec)


# =============================================================================
# ROS2 node entry point (mirrors robot_node.py, but calls this module's run())
# =============================================================================

def main(args=None) -> None:
    import signal
    import threading

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    # Reuse the Robot wrapper via a minimal node
    from robot.robot import Robot as _Robot

    class _TestNode(Node):
        def __init__(self) -> None:
            super().__init__("orientation_fusion_test")
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
