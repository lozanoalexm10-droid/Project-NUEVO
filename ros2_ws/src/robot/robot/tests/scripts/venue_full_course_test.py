"""
venue_full_course_test.py — full venue route with lane-switch obstacle avoidance
=================================================================================
Drives the complete venue course in three segments using _nav_follow_pp_path_loop
for all segments (same architecture as full_venue_route.py):

  1. PRE_OBSTACLE  — straight up lane 1 → U-turn → lane 2 down → obs zone entry
                     (obstacle avoidance OFF)
  2. OBSTACLE      — obstacle zone centreline up
                     (LiDAR lane-switch avoidance ON)
  3. POST_OBSTACLE — exit U-turn → lane 5 down to finish
                     (obstacle avoidance OFF)

U-turns are sharp corners in the waypoint list — the nav loop's lookahead
rounds them naturally (effective turn radius ≈ lookahead_distance).

No vision-based stop-sign or traffic-light detection.
Auto-starts 3 seconds after launch.  BTN_2 = emergency stop (checked in IDLE).
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


def configure_robot(robot: Robot) -> None:
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
    robot.enable_lidar()
    robot.enable_gps()
    robot.set_tracked_tag_id(TAG_ID)
    robot.set_orientation_fusion_alpha(0.0)
    robot.set_position_fusion_alpha(0.0)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.ORANGE, 255)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 255)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

    robot.set_pid_gains(
        motor_id=LEFT_WHEEL_MOTOR,
        loop_type=DCPidLoop.VELOCITY,
        kp=VELOCITY_KP,
        ki=VELOCITY_KI,
        kd=VELOCITY_KD,
    )
    robot.set_pid_gains(
        motor_id=RIGHT_WHEEL_MOTOR,
        loop_type=DCPidLoop.VELOCITY,
        kp=VELOCITY_KP,
        ki=VELOCITY_KI,
        kd=VELOCITY_KD,
    )
    time.sleep(0.2)

    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=3.0):
        print("[WARN] odometry reset not confirmed within 3 s — pose may be stale")
        robot.wait_for_pose_update(timeout=1.0)
    print(f"[CONFIG] pose after reset: {robot.get_odometry_pose()}")


# ── Path geometry ─────────────────────────────────────────────────────────────
#
# Waypoints use actual course lane centres. U-turns are represented as
# 90° corners — the nav loop's lookahead_distance controls how smoothly
# (and how wide) each corner is taken.
#
# Robot power-on position: (X_LANE1, -150) — 60 mm left of lane-1 centre.

X_LANE1   = 0.0      # robot start x (60 mm left of lane-1 centre)
X_LANE2   = 610.0      # lane-2 centre
X_OBS_MID = 1525.0     # obstacle zone centreline
X_LANE5   = 2440.0     # lane-5 centre

Y_MAX      = 3850.0    # top wall
Y_MIN_WALL = 310.0     # bottom wall
Y_TOP      = 3300.0    # approach height before top U-turns
Y_BOT      = 610.0     # approach height before bottom U-turn
FINISH_Y   = 700.0     # lane-5 finish position

LANE_MM = 610.0
R       = (LANE_MM / 2) - 40.0  # 265 mm — single-lane U-turn radius


def _top_arc(x0: float, y0: float) -> list[tuple[float, float]]:
    """R=305 semicircle: (x0, y0) heading +y → (x0+610, y0) heading -y.
    Peaks at (x0+305, y0+305).  Safe when y0+305 < Y_MAX."""
    return [
        (x0 + R * (1 - math.cos(math.radians(d))),
         y0 + R * math.sin(math.radians(d)))
        for d in range(5, 181, 5)
    ]


# Pre-obstacle: straight up lane 1 → arc U-turn → down lane 2 → obs zone entry
# Lane-1 centre is x=0; robot starts 60 mm left at X_LANE1=-60.
# _top_arc peaks at y = Y_TOP+305 = 3705 mm  (145 mm from wall — safe).
PRE_OBSTACLE_WAYPOINTS = (
    [(X_LANE1, -150.0), (X_LANE1-130.0, Y_TOP-300.0), (X_LANE1-130.0, Y_TOP)]
    + _top_arc(0.0, Y_TOP)               # (0, Y_TOP) → (610, Y_TOP), heading -y
    + [(X_LANE2, Y_TOP), (X_LANE2+60.0, Y_BOT), (X_OBS_MID, Y_BOT)]
)

# Obstacle zone: straight up the centreline
OBS_WAYPOINTS = [
    (X_OBS_MID, Y_BOT),
    (X_OBS_MID, Y_TOP),
]

# Post-obstacle: sharp corner U-turn → down lane 5 to finish
# _top_arc_obs (R=457.5) would peak at 3857 mm — clips the wall, so use corners.
POST_OBSTACLE_WAYPOINTS = [
    (X_OBS_MID, Y_TOP),
    (X_LANE5,   Y_TOP),
    (X_LANE5,   FINISH_Y),
]

# lookahead_distance controls the effective U-turn radius for non-obstacle segments
# (increased from full_venue_route's 100 mm for wider, smoother turns)
LOOKAHEAD_MM = 150.0
LINEAR_SPEED = 140.0


def _load_segment(
    robot: Robot,
    control_points: list[tuple[float, float]],
    *,
    obstacle_avoidance: bool,
    x_L: float = 0.0,
    spacing: float = 50.0,
) -> None:
    """Configure nav controller and arm a path segment."""
    path = densify_polyline(control_points, spacing=spacing)

    if obstacle_avoidance:
        robot._nav_follow_pp_path(
            lookahead_distance=LOOKAHEAD_MM,
            max_linear_speed=LINEAR_SPEED,
            max_angular_speed=1.0,
            goal_tolerance=30.0,
            obstacles_range=550.0,
            view_angle=math.radians(65.0),
            safe_dist=360.0,
            avoidance_delay=420,
            alpha_Ld=1.3,
            offset=250.0,
            lane_width=400.0,
            obstacle_avoidance=True,
            x_L=x_L,
        )
    else:
        robot._nav_follow_pp_path(
            lookahead_distance=LOOKAHEAD_MM,
            max_linear_speed=LINEAR_SPEED,
            max_angular_speed=1.5,
            goal_tolerance=20.0,
            obstacles_range=450.0,
            view_angle=math.radians(70.0),
            safe_dist=250.0,
            avoidance_delay=150,
            alpha_Ld=0.7,
            offset=305.0,
            lane_width=500.0,
            obstacle_avoidance=False,
            x_L=x_L,
        )

    robot._set_obstacle_avoidance_path(path)


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            _load_segment(robot, PRE_OBSTACLE_WAYPOINTS, obstacle_avoidance=False)
            print("[FSM] INIT complete. Entering IDLE.")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            robot._draw_lidar_obstacles()
            print("[FSM] Auto-starting full course in 3 seconds.")
            time.sleep(3)

            if robot.get_button(Button.BTN_2):
                print("[FSM] BTN_2 — aborting.")
                robot.shutdown()
                return

            print("[FSM] PRE_OBSTACLE")
            state = "PRE_OBSTACLE"

        elif state == "PRE_OBSTACLE":
            show_moving_leds(robot)
            nav_state = robot._nav_follow_pp_path_loop()
            if nav_state == "IDLE":
                print("[FSM] Pre-obstacle complete. Loading obstacle segment.")
                _load_segment(
                    robot, OBS_WAYPOINTS,
                    obstacle_avoidance=True,
                    x_L=X_OBS_MID,
                    spacing=400.0,
                )
                state = "OBSTACLE"

        elif state == "OBSTACLE":
            show_moving_leds(robot)
            nav_state = robot._nav_follow_pp_path_loop()
            if nav_state == "IDLE":
                print("[FSM] Obstacle zone complete. Loading post-obstacle segment.")
                _load_segment(robot, POST_OBSTACLE_WAYPOINTS, obstacle_avoidance=False)
                state = "POST_OBSTACLE"

        elif state == "POST_OBSTACLE":
            show_moving_leds(robot)
            nav_state = robot._nav_follow_pp_path_loop()
            if nav_state == "IDLE":
                print("[FSM] Full course complete.")
                robot.stop()
                state = "DONE"

        elif state == "DONE":
            show_idle_leds(robot)
            robot.stop()

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
