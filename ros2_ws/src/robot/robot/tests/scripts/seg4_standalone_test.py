"""
seg4_standalone_test.py — SEG4 path from (0,0) facing east
==========================================================
Drop-in alternative to running venue_full_course_test.py with
START_SEGMENT=4. Treats the robot's current position as (0,0) and its
current heading as east (0° math convention). Pure odometry — no GPS,
no AprilTag fusion, no checkpoint frame translation.

Path (robot-relative coordinates, mm):
  Leg 1 (forward / "east"):   (0, 0) → (EAST_LEG_MM, 0)
  Leg 2 (right turn / "south"): (EAST_LEG_MM, 0) → (EAST_LEG_MM, -SOUTH_LEG_MM)

Distances match venue_full_course_test.py SEG4:
  EAST_LEG_MM  = 457.5   (CP3 → corner = X_LANE5 - (X_OBS_MID+457.5))
  SOUTH_LEG_MM = 3220    (Y_TOP - FINISH_Y = 3500 - 280)

Stop-sign detection is OPT-IN via ENABLE_STOP_SIGN_DETECTION below; it
runs only on the south leg and triggers a 3-second dwell on the first
close detection (bbox height ≥ STOP_SIGN_MIN_FRAC of the frame).

Usage: physically align the robot with the direction you want to call
"east", uncomment this script's import in main.py, run, then press
BTN_1 (or wait 3 s if AUTO_START=True). BTN_2 anytime stops.
"""
from __future__ import annotations

# ── Run configuration ────────────────────────────────────────────────────────
AUTO_START                 = True   # True: 3-second countdown; False: wait for BTN_1
ENABLE_STOP_SIGN_DETECTION = False  # True: pause 3 s on close stop sign during south leg
# ─────────────────────────────────────────────────────────────────────────────

import math
import time

from robot.hardware_map import (
    Button,
    DCPidLoop,
    DEFAULT_FSM_HZ,
    LED,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    VELOCITY_KD,
    VELOCITY_KI,
    VELOCITY_KP,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline

# ── Path geometry (mm, robot-relative starting from (0,0) facing east) ────────
EAST_LEG_MM  = 457.5
SOUTH_LEG_MM = 3220.0

# ── Drive / nav tuning (mirrors SEG4_CFG in venue_full_course_test.py) ────────
SPEED_MM_S      = 120.0
LOOKAHEAD_MM    = 100.0
PATH_SPACING_MM = 50.0

# ── Stop-sign detection thresholds (mirror venue_full_course_test.py) ─────────
VISION_STALE_S     = 3.0
STOP_SIGN_DWELL_S  = 3.0
STOP_SIGN_MIN_FRAC = 0.18
STOP_SIGN_MIN_CONF = 0.50

# ── Path waypoint definitions ─────────────────────────────────────────────────
EAST_PATH  = [(0.0,         0.0), (EAST_LEG_MM, 0.0)]
SOUTH_PATH = [(EAST_LEG_MM, 0.0), (EAST_LEG_MM, -SOUTH_LEG_MM)]


def _configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    robot.set_pid_gains(LEFT_WHEEL_MOTOR,  DCPidLoop.VELOCITY, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD)
    robot.set_pid_gains(RIGHT_WHEEL_MOTOR, DCPidLoop.VELOCITY, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD)
    robot.enable_lidar()
    if ENABLE_STOP_SIGN_DETECTION:
        robot.enable_vision()
    # Pure odometry — no GPS / no AprilTag fusion.
    robot.set_orientation_fusion_alpha(0.0)
    robot.set_position_fusion_alpha(0.0)


def _init_firmware(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    time.sleep(0.2)
    # 0° (math convention east) == robot's current forward direction.
    robot.set_initial_theta(0.0)
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=3.0):
        print("[WARN] odometry reset not confirmed within 3 s — pose may be stale")
        robot.wait_for_pose_update(timeout=1.0)
    print(f"[CONFIG] pose after reset: {robot.get_odometry_pose()}")


def _load_path(robot: Robot, waypoints: list[tuple[float, float]]) -> None:
    path = densify_polyline(waypoints, spacing=PATH_SPACING_MM)
    # Avoidance OFF — params copied from the `else` branch of
    # venue_full_course_test._load_path to match SEG4's behavior exactly.
    robot._nav_follow_pp_path(
        lookahead_distance=LOOKAHEAD_MM,
        max_linear_speed=SPEED_MM_S,
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
        x_L=0.0,
    )
    robot._set_obstacle_avoidance_path(path)


def _close_stop_sign(robot: Robot) -> bool:
    """True if a stop sign is detected close enough to trigger the dwell."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_S):
        return False
    _, img_h = robot.get_detection_image_size()
    if img_h <= 0:
        return False
    for det in robot.get_detections("stop sign"):
        if float(det.get("confidence", 0.0)) < STOP_SIGN_MIN_CONF:
            continue
        bbox = det.get("bbox") or {}
        h_frac = float(bbox.get("height", 0)) / float(img_h)
        if h_frac >= STOP_SIGN_MIN_FRAC:
            return True
    return False


def _estop(robot: Robot) -> None:
    robot.stop()
    robot.shutdown()


def run(robot: Robot) -> None:  # noqa: C901
    _configure_robot(robot)
    _init_firmware(robot)
    _load_path(robot, EAST_PATH)

    print(f"[FSM] Ready.  AUTO_START={AUTO_START}  "
          f"stop_sign={ENABLE_STOP_SIGN_DETECTION}")
    print(f"[FSM] Path  : east {EAST_LEG_MM:.0f} mm → south {SOUTH_LEG_MM:.0f} mm")

    state: str = "IDLE"
    leg:   str = "east"
    stop_sign_seen:    bool  = False
    stop_sign_pause_t: float = 0.0
    idle_leds_set:     bool  = False

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        # ── IDLE ─────────────────────────────────────────────────────────────
        if state == "IDLE":
            if robot.get_button(Button.BTN_2):
                print("[FSM] BTN_2 — shutting down.")
                _estop(robot)
                return

            if AUTO_START:
                if not idle_leds_set:
                    robot.set_led(LED.ORANGE, 255)
                    robot.set_led(LED.GREEN, 0)
                    robot.set_led(LED.RED, 0)
                    idle_leds_set = True
                robot._draw_lidar_obstacles()
                print("[FSM] Auto-starting in 3 s …")
                time.sleep(3)
                robot.set_led(LED.ORANGE, 0)
                state = "DRIVE"
                idle_leds_set = False
                print("[FSM] → DRIVE (east leg)")
            else:
                if not idle_leds_set:
                    robot.set_led(LED.RED, 255)
                    robot.set_led(LED.GREEN, 0)
                    robot.set_led(LED.ORANGE, 0)
                    idle_leds_set = True
                if robot.get_button(Button.BTN_1):
                    print("[FSM] BTN_1 — starting.")
                    robot.set_led(LED.RED, 0)
                    state = "DRIVE"
                    idle_leds_set = False
                    print("[FSM] → DRIVE (east leg)")

        # ── DRIVE (handles both east and south legs) ─────────────────────────
        elif state == "DRIVE":
            if robot.get_button(Button.BTN_2):
                _estop(robot); return

            if leg == "south" and ENABLE_STOP_SIGN_DETECTION \
                    and stop_sign_seen and stop_sign_pause_t > 0.0:
                # Holding at stop sign
                robot.stop()
                robot.set_led(LED.RED, 255)
                robot.set_led(LED.GREEN, 0)
                if time.monotonic() - stop_sign_pause_t >= STOP_SIGN_DWELL_S:
                    print("[FSM] Stop-sign dwell complete — resuming to finish.")
                    stop_sign_pause_t = 0.0
                    robot.set_led(LED.RED, 0)
                    robot.set_led(LED.GREEN, 255)
                # Stay in DRIVE; next tick resumes nav.

            elif leg == "south" and ENABLE_STOP_SIGN_DETECTION \
                    and not stop_sign_seen and _close_stop_sign(robot):
                robot.stop()
                stop_sign_seen    = True
                stop_sign_pause_t = time.monotonic()
                robot.set_led(LED.RED, 255)
                robot.set_led(LED.GREEN, 0)
                print(f"[FSM] Stop sign close — pausing {STOP_SIGN_DWELL_S:.0f} s.")

            else:
                robot.set_led(LED.GREEN, 255)
                robot.set_led(LED.ORANGE, 0)
                if robot._nav_follow_pp_path_loop() == "IDLE":
                    if leg == "east":
                        px, py, pth = robot.get_odometry_pose()
                        print(f"[FSM] East leg done at ({px:.0f},{py:.0f}) θ={pth:.1f}° "
                              f"— loading south leg.")
                        _load_path(robot, SOUTH_PATH)
                        leg = "south"
                    else:
                        px, py, pth = robot.get_odometry_pose()
                        print(f"[FSM] South leg done at ({px:.0f},{py:.0f}) θ={pth:.1f}° "
                              f"— at finish.")
                        robot.stop()
                        state = "DONE"

        # ── DONE ──────────────────────────────────────────────────────────────
        elif state == "DONE":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            robot.set_led(LED.RED, 0)
            robot.stop()

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
