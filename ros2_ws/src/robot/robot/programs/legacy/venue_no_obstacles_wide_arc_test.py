"""
venue_no_obstacles_wide_arc_test.py
====================================
Wide-arc variant of the venue no-obstacles test.  Designed to exploit the
30 mm narrower front omni-wheel axle footprint that lets the robot start
further to the left of the lane-1 centreline.

Key differences from venue_no_obstacles_test.py
------------------------------------------------
  - Robot is placed 30 mm to the left of the old lane-1 centre.
    Odometry resets to (0, 0) at the new start position, so all
    downstream lane centres shift 30 mm to the right in odometry space:
        lane 2 centre → 640 mm   (was 610 mm)
        obs-zone mid  → 1555 mm  (was 1525 mm)
        lane 5 centre → 2470 mm  (was 2440 mm)

  - All three U-turns use *elliptical* arcs — horizontal half-span Rx and
    wall-approach depth Ry are set independently.  This lets the arc swing
    further toward the wall without changing the start/end lane positions.

  - Y_TOP_WIDE and Y_BOT_WIDE are pushed 100-120 mm closer to the walls
    to give the robot a longer straight run-up before each arc.

Wall clearances at arc extremes (robot centre-line):
    Top arcs peak  at Y_TOP_WIDE + Ry_TOP = 3150 + 430 = 3580 mm
                   → 270 mm clear of top wall (3850 mm)           ✓
    Bottom arc dip at Y_BOT_WIDE - Ry_BOT = 1100 - 500 = 600 mm
                   → 290 mm clear of bottom wall (310 mm)         ✓

Compared with v1 arc dimensions:
    Arc           v1 Rx / Ry          wide Rx / Ry
    ─────────────────────────────────────────────────
    Top 1          305 / 305           320 / 430
    Bottom         457.5 / 457.5       457.5 / 500
    Top 2          457.5 / 457.5       457.5 / 430
"""
from __future__ import annotations

import math
import time

from robot.hardware_map import (
    Button,
    DCPidLoop,
    DEFAULT_FSM_HZ,
    LED,
    INITIAL_THETA_DEG,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    TAG_ID,
    VELOCITY_KD,
    VELOCITY_KI,
    VELOCITY_KP,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline


# ── Venue / lane geometry ─────────────────────────────────────────────────────
#
# All coordinates in odometry space. (0, 0) = robot start position, which is
# 30 mm to the left of the old lane-1 centre.  Lane-to-lane spacing is still
# 610 mm, but the first arc spans 640 mm because the robot starts 30 mm left
# of the physical lane-1 centre.
#
#   x positions (odometry):         physical lane in venue:
#     0        ← robot start         30 mm left of lane-1 centre
#     640      ← lane 2 centre       lane-2 centre
#     1555     ← obs-zone mid        midline of lanes 3+4
#     2470     ← lane 5 centre       lane-5 centre
#
LANE_MM       = 610.0        # physical lane centre-to-centre spacing (mm)
X_START       = 0.0          # odometry origin (30 mm left of old lane-1 centre)
X_LANE2       = 640.0        # lane 2 centre in odometry space
X_OBS_MID     = 1555.0       # obstacle-zone centreline (lanes 3+4 mid)
X_LANE5       = 2470.0       # lane 5 centre

Y_MAX         = 3850.0       # top wall
Y_MIN_WALL    = 310.0        # bottom wall
Y_TOP_WIDE    = 3150.0       # straight-run end / arc approach, top side
Y_BOT_WIDE    = 1100.0       # straight-run end / arc approach, bottom side
FINISH_Y      = 700.0        # final stop point on lane 5

# ── Arc radii (horizontal Rx, depth Ry) ──────────────────────────────────────
#
# Each arc is an ellipse: horizontal half-span = Rx, depth toward wall = Ry.
# A pure semicircle is the special case Rx == Ry.

# Top arc 1 — lane 1 start → lane 2 centre (spans 640 mm)
RX_ARC1 = (X_LANE2 - X_START) / 2   # 320 mm
RY_ARC1 = 430.0                       # depth toward top wall; peak at 3580 mm

# Bottom arc — lane 2 → obs-zone mid (spans 915 mm)
RX_BOT  = (X_OBS_MID - X_LANE2) / 2  # 457.5 mm
RY_BOT  = 500.0                        # depth toward bottom wall; dip to 600 mm

# Top arc 2 — obs-zone mid → lane 5 centre (spans 915 mm)
RX_ARC2 = (X_LANE5 - X_OBS_MID) / 2  # 457.5 mm
RY_ARC2 = 430.0                        # depth toward top wall; peak at 3580 mm


# ── Arc generators ────────────────────────────────────────────────────────────

def _elliptic_top_arc(
    x0: float, y0: float, Rx: float, Ry: float
) -> list[tuple[float, float]]:
    """Elliptical U-turn toward the top wall.

    Entry:  (x0, y0)       heading +y
    Peak:   (x0+Rx, y0+Ry)
    Exit:   (x0+2Rx, y0)  heading -y

    36 waypoints at 5-degree intervals give ~Ry*π/36 ≈ 37 mm spacing at peak.
    """
    return [
        (x0 + Rx * (1.0 - math.cos(math.radians(d))),
         y0 + Ry *        math.sin(math.radians(d)))
        for d in range(5, 181, 5)
    ]


def _elliptic_bot_arc(
    x0: float, y0: float, Rx: float, Ry: float
) -> list[tuple[float, float]]:
    """Elliptical U-turn toward the bottom wall.

    Entry:  (x0, y0)       heading -y
    Dip:    (x0+Rx, y0-Ry)
    Exit:   (x0+2Rx, y0)  heading +y
    """
    return [
        (x0 + Rx * (1.0 - math.cos(math.radians(d))),
         y0 - Ry *        math.sin(math.radians(d)))
        for d in range(5, 181, 5)
    ]


# ── Full wide-arc path ────────────────────────────────────────────────────────

FULL_PATH_WIDE: list[tuple[float, float]] = (
    # ── lane 1 up (x = 0, 30 mm left of old lane-1 centre) ───────────────────
    [(X_START, 0.0), (X_START, Y_TOP_WIDE)]

    # ── top arc 1: → (640, 3150) heading -y ──────────────────────────────────
    + _elliptic_top_arc(X_START, Y_TOP_WIDE, RX_ARC1, RY_ARC1)

    # ── lane 2 down ───────────────────────────────────────────────────────────
    + [(X_LANE2, Y_BOT_WIDE)]

    # ── bottom arc: → (1555, 1100) heading +y ────────────────────────────────
    + _elliptic_bot_arc(X_LANE2, Y_BOT_WIDE, RX_BOT, RY_BOT)

    # ── obstacle-zone centreline up ───────────────────────────────────────────
    + [(X_OBS_MID, Y_TOP_WIDE)]

    # ── top arc 2: → (2470, 3150) heading -y ─────────────────────────────────
    + _elliptic_top_arc(X_OBS_MID, Y_TOP_WIDE, RX_ARC2, RY_ARC2)

    # ── lane 5 down to finish ─────────────────────────────────────────────────
    + [(X_LANE5, FINISH_Y)]
)


# ── Robot setup helpers ───────────────────────────────────────────────────────

def _configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    robot.enable_gps()
    robot.set_tracked_tag_id(TAG_ID)
    robot.set_orientation_fusion_alpha(0.0)
    robot.set_position_fusion_alpha(0.1)


def _start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.set_pid_gains(LEFT_WHEEL_MOTOR,  DCPidLoop.VELOCITY, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD)
    robot.set_pid_gains(RIGHT_WHEEL_MOTOR, DCPidLoop.VELOCITY, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD)
    time.sleep(0.2)
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=3.0):
        print("[WARN] odometry reset not confirmed within 3 s — pose may be stale")
        robot.wait_for_pose_update(timeout=1.0)
    print(f"[CONFIG] pose after reset: {robot.get_odometry_pose()}")


# ── Entry point ───────────────────────────────────────────────────────────────

def run(robot: Robot) -> None:
    _configure_robot(robot)

    state = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    print("[WIDE-ARC] arc geometry:")
    print(f"  Arc 1  — Rx={RX_ARC1:.1f} mm, Ry={RY_ARC1:.1f} mm, "
          f"peak y={Y_TOP_WIDE + RY_ARC1:.0f} mm  (wall clearance "
          f"{Y_MAX - (Y_TOP_WIDE + RY_ARC1):.0f} mm)")
    print(f"  Bot    — Rx={RX_BOT:.1f} mm,   Ry={RY_BOT:.1f} mm, "
          f"dip  y={Y_BOT_WIDE - RY_BOT:.0f} mm  (wall clearance "
          f"{(Y_BOT_WIDE - RY_BOT) - Y_MIN_WALL:.0f} mm)")
    print(f"  Arc 2  — Rx={RX_ARC2:.1f} mm, Ry={RY_ARC2:.1f} mm, "
          f"peak y={Y_TOP_WIDE + RY_ARC2:.0f} mm  (wall clearance "
          f"{Y_MAX - (Y_TOP_WIDE + RY_ARC2):.0f} mm)")

    while True:
        if state == "INIT":
            _start_robot(robot)
            print("[FSM] INIT (odometry reset)")
            path = densify_polyline(FULL_PATH_WIDE, spacing=50.0)
            print(f"[FSM] wide-arc path ready — {len(path)} densified waypoints")
            state = "IDLE"

        elif state == "IDLE":
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 255)
            time.sleep(3)

            if robot.get_button(Button.BTN_2):
                print("[FSM] BTN_2 pressed — stopping.")
                robot.shutdown()
                return

            print("[FSM] IDLE — auto-starting wide-arc path.")
            state = "MOVING"

        elif state == "MOVING":
            robot.set_led(LED.ORANGE, 0)
            robot.set_led(LED.GREEN, 255)
            robot.purepursuit_follow_path(
                waypoints=path,
                velocity=125.0,
                lookahead=160.0,       # 10 mm larger than v1 — wider arcs preview more curve
                tolerance=20.0,
                max_angular_rad_s=1.5,
                advance_radius=80.0,
                blocking=True,
            )
            print("[FSM] Wide-arc venue route complete.")
            robot.stop()
            state = "DONE"

        elif state == "DONE":
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 255)
            robot.stop()

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
