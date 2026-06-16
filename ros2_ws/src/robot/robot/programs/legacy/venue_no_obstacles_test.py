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

    robot.enable_gps()
    robot.set_tracked_tag_id(TAG_ID)

    robot.set_orientation_fusion_alpha(0.0)
    robot.set_position_fusion_alpha(0.1)


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
        print("[WARN] odometry reset not confirmed within 3s — pose may be stale")
        robot.wait_for_pose_update(timeout=1.0)
    print(f"[CONFIG] pose after reset: {robot.get_odometry_pose()}")


# ── Path geometry ─────────────────────────────────────────────────────────────
#
# Route:
#   Lane 1 (x=0)    up   → top arc R=305  → Lane 2 (x=610) down
#   → bottom arc R=457.5 (1.5 lanes) → middle of obstacle zone (x=1525) up
#   → top arc R=457.5 (1.5 lanes)    → Lane 5 (x=2440) down to finish
#
# Obstacle zone (lanes 3+4 combined, x=1220–1830):
#   Robot drives straight up the centerline (x=1525) in this test.
#   When obstacle avoidance is added: enter at x=1525, pick a lane, avoid
#   obstacles, return to x=1525 before the exit arc.
#
# Arc radii:
#   R     = 305 mm  — single-lane (610 mm shift)
#   R_OBS = 457.5 mm — 1.5-lane (915 mm shift), used for obstacle zone entry/exit
#
# Y stops:
#   Y_TOP = 2950  — approach for top arcs (top wall at Y_MAX=3850, ~440 mm clearance)
#   Y_BOT = 1220  — approach for bottom arc (bottom wall at 310, ~452 mm clearance)

LANE_MM    = 610.0
R          = LANE_MM / 2            # 305 mm
R_OBS      = LANE_MM * 1.5 / 2      # 457.5 mm — 1.5-lane arc
X_OBS_MID  = LANE_MM * 2.5          # 1525 mm — centerline between lanes 3 and 4
Y_MAX      = 3850.0                 # top wall
Y_MIN_WALL = 310.0                  # bottom wall
Y_TOP      = 3050.0                 # approach for top arcs
Y_BOT      = 1220.0                 # approach for bottom arc
FINISH_Y   = 700.0                  # end of lane 5


def _top_arc(x0: float, y0: float) -> list[tuple[float, float]]:
    """R=305, +y → -y heading, +610 mm in x."""
    return [
        (x0 + R * (1 - math.cos(math.radians(d))),
         y0 + R * math.sin(math.radians(d)))
        for d in range(5, 181, 5)
    ]


def _top_arc_obs(x0: float, y0: float) -> list[tuple[float, float]]:
    """R=457.5, +y → -y heading, +915 mm in x (1.5 lanes)."""
    return [
        (x0 + R_OBS * (1 - math.cos(math.radians(d))),
         y0 + R_OBS * math.sin(math.radians(d)))
        for d in range(5, 181, 5)
    ]


def _bot_arc_obs(x0: float, y0: float) -> list[tuple[float, float]]:
    """R=457.5, -y → +y heading, +915 mm in x (1.5 lanes).
    Entry arc: (610, Y_BOT) → (1525, Y_BOT), arc dips to (1067.5, Y_BOT-457.5).
    """
    return [
        (x0 + R_OBS * (1 - math.cos(math.radians(d))),
         y0 - R_OBS * math.sin(math.radians(d)))
        for d in range(5, 181, 5)
    ]


FULL_PATH = (
    # ── lane 1 up  (x = 0) ───────────────────────────────────────────────────
    [(0.0, 0.0), (0.0, Y_TOP)]
    + _top_arc(0.0, Y_TOP)                  # → (610, 2950), heading -y

    # ── lane 2 down  (x = 610) ───────────────────────────────────────────────
    + [(610.0, Y_BOT)]
    + _bot_arc_obs(610.0, Y_BOT)            # → (1525, 1220), heading +y

    # ── obstacle zone centerline up  (x = 1525) ──────────────────────────────
    + [(X_OBS_MID, Y_TOP)]
    + _top_arc_obs(X_OBS_MID, Y_TOP)        # → (2440, 2950), heading -y

    # ── lane 5 down  (x = 2440) — final lane ─────────────────────────────────
    + [(2440.0, FINISH_Y)]
)


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    print(f"FSM period: {period:.3f} seconds")
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            print("[FSM] INIT (odometry reset)")

            path = densify_polyline(FULL_PATH, spacing=50.0)

            print("[FSM] Full path ready. Entering IDLE state.")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            time.sleep(3)

            if robot.get_button(Button.BTN_2):
                print("BTN_2 pressed. Stopping robot and saving trajectory.")
                robot.shutdown()
                return

            print("[FSM] IDLE - Auto-starting full path.")
            print("[FSM] MOVING")
            state = "MOVING"

        elif state == "MOVING":
            show_moving_leds(robot)
            robot.purepursuit_follow_path(
                waypoints=path,
                velocity=125.0,
                lookahead=150.0,
                tolerance=20.0,
                max_angular_rad_s=1.5,
                advance_radius=80.0,
                blocking=True,
            )
            print("[FSM] Full venue route complete.")
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
