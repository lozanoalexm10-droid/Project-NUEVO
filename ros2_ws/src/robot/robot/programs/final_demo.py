"""
final_demo.py
=============
Full competition sequence.

State machine:
  INIT
    ↓ always
  IDLE  ─── BTN_1 (green light) ──────────────────────────────────────────────
    ↓                                                                          │
  MOVING_PRE_OBSTACLE   (avoidance OFF)                                        │
    ↓ nav done                                                                 │
  MOVING_OBSTACLE       (avoidance ON)                                         │
    ↓ nav done                                                                 │
  MOVING_POST_OBSTACLE  (avoidance OFF)                                        │
    ↓ nav done                                                                 │
  AT_STOP_SIGN          (stop, STOP_SIGN_DWELL_S)                              │
    ↓                                                                          │
  WAIT_ENCLOSURE        (enable vision, ENCLOSURE_WAIT_S)                      │
    ↓                                                                          │
  ARM_HOME              (home turntable, move to search pose)  BTN_2 ─────────┘
    ↓ success / timeout → RESTOWING                            (shutdown at
  SCANNING              (wait for marshmallow above threshold)  any nav state)
    ↓ confident detection found / timeout → RESTOWING
  APPROACHING           (turntable + shoulder + elbow to pick position)
    ↓
  PICKING               (close gripper)
    ↓
  CARRYING              (retract, rotate turntable, extend to plate)
    ↓ plate IK fails → RESTOWING
  PLACING               (open gripper to GRIPPER_ROAST_DEG)
    ↓
  ROASTING              (heat wire for ROAST_TIME_S)
    ↓
  RESTOWING             (fold arm, home turntable, disable joints)
    ↓
  DONE

Required nodes:  bridge + robot + rplidar + sensors + vision
Launch files:    robot.launch.py + everything_but_robot.launch.py
"""
from __future__ import annotations

import math
import time

from robot.arm_kinematics import OutOfReachError, detection_to_robot_frame, inverse_kinematics
from robot.hardware_map import (
    Button,
    DCMotorMode,
    DCPidLoop,
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LED,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    StepMoveType,
    TAG_ID,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot
from robot.tests.scripts._manipulator_config import (
    ARM_CARRY_ELBOW_DEG,
    ARM_CARRY_SHOULDER_DEG,
    ARM_GEOMETRY,
    ARM_SEARCH_ELBOW_DEG,
    ARM_SEARCH_SHOULDER_DEG,
    ARM_SERVO_STEP_DEG,
    ARM_SERVO_STEP_DWELL,
    CAMERA_HFOV_DEG,
    ELBOW_CHANNEL,
    ELBOW_SAFE_MAX,
    ELBOW_SAFE_MIN,
    ELBOW_STOW_DEG,
    ENCLOSURE_WAIT_S,
    GRIPPER_CHANNEL,
    GRIPPER_CLOSE_DEG,
    GRIPPER_OPEN_DEG,
    GRIPPER_ROAST_DEG,
    HEATING_WIRE_MOTOR_ID,
    HEATING_WIRE_PWM_OFF,
    HEATING_WIRE_PWM_ON,
    MARSHMALLOW_CLASS,
    MARSHMALLOW_DISTANCE_MM,
    MARSHMALLOW_HEIGHT_MM,
    MIN_CONFIDENCE_MARSHMALLOW,
    PLATE_X_MM,
    PLATE_Y_MM,
    PLATE_Z_MM,
    ROAST_TIME_S,
    SCAN_TIMEOUT_S,
    SHOULDER_CHANNEL,
    SHOULDER_SAFE_MAX,
    SHOULDER_SAFE_MIN,
    SHOULDER_STOW_DEG,
    STOP_SIGN_DWELL_S,
    TURNTABLE_ACCELERATION,
    TURNTABLE_HOME_OFFSET_DEG,
    TURNTABLE_MAX_VELOCITY,
    TURNTABLE_SAFE_ARC_DEG,
    TURNTABLE_STEPPER,
    turntable_deg_to_steps,
)
from robot.util import densify_polyline

# ── Drive tuning ──────────────────────────────────────────────────────────────
VELOCITY_KP = 2.28
VELOCITY_KI = 4.45
VELOCITY_KD = 0.0

# ── Waypoints (mm, robot odometry frame) ──────────────────────────────────────
# Copy from full_venue_route.py — update to match competition venue layout.
PRE_OBSTACLE_PATH = [
    (0.0,    0.0),
    (0.0,    3660.0),
    (610.0,  3660.0),
    (610.0,  610.0),
    (1565.0, 610.0),
]

OBSTACLE_FIELD_PATH = [
    (1565.0, 610.0),
    (1565.0, 3660.0),
]
OBSTACLE_FIELD_X_OFFSET = 1565.0  # x_L parameter for lane-switch avoidance

POST_OBSTACLE_PATH = [
    (1565.0, 3660.0),
    (2530.0, 3660.0),
    (2530.0, 610.0),
    (2745.0, 305.0),   # stop sign
]


# ── Helpers ───────────────────────────────────────────────────────────────────

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
    robot.set_pid_gains(LEFT_WHEEL_MOTOR,  DCPidLoop.VELOCITY, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD)
    robot.set_pid_gains(RIGHT_WHEEL_MOTOR, DCPidLoop.VELOCITY, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD)
    robot.enable_lidar()
    robot.enable_gps()
    robot.set_tracked_tag_id(TAG_ID)
    robot.set_orientation_fusion_alpha(0.0)
    robot.set_position_fusion_alpha(0.10)


def _start_path_segment(
    robot: Robot,
    waypoints: list[tuple[float, float]],
    *,
    obstacle_avoidance: bool,
    x_L: float = 0.0,
) -> None:
    path = densify_polyline(waypoints, spacing=50.0)
    robot._nav_follow_pp_path(
        lookahead_distance=100.0,
        max_linear_speed=90.0,
        max_angular_speed=1.5,
        goal_tolerance=20.0,
        obstacles_range=450.0,
        view_angle=math.radians(70.0) if obstacle_avoidance else 0.0,
        safe_dist=250.0,
        avoidance_delay=150,
        alpha_Ld=0.7,
        offset=305.0,
        lane_width=500.0,
        obstacle_avoidance=obstacle_avoidance,
        x_L=x_L,
    )
    robot._set_obstacle_avoidance_path(path)


def _move_servo(robot: Robot, channel: int, current: float, target: float,
                safe_min: float = 10.0, safe_max: float = 170.0) -> float:
    """Incrementally move servo from current to target. Returns final angle."""
    target = max(safe_min, min(safe_max, target))
    direction = 1.0 if target > current else -1.0
    angle = current
    while abs(angle - target) > ARM_SERVO_STEP_DEG:
        angle += direction * ARM_SERVO_STEP_DEG
        robot.set_servo(channel, angle)
        time.sleep(ARM_SERVO_STEP_DWELL)
    robot.set_servo(channel, target)
    time.sleep(ARM_SERVO_STEP_DWELL)
    return target


def _turntable_to_deg(robot: Robot, target_deg: float) -> None:
    """Move turntable to target_deg from robot forward axis (blocking)."""
    steps = turntable_deg_to_steps(target_deg + TURNTABLE_HOME_OFFSET_DEG)
    robot.step_move(TURNTABLE_STEPPER, steps, StepMoveType.ABSOLUTE)


def _best_marshmallow(robot: Robot) -> dict | None:
    """Return highest-confidence marshmallow detection, or None."""
    dets = robot.get_detections(MARSHMALLOW_CLASS)
    confident = [d for d in dets if float(d["confidence"]) >= MIN_CONFIDENCE_MARSHMALLOW]
    if not confident:
        return None
    return max(confident, key=lambda d: float(d["confidence"]))


def _detection_bearing_rad(det: dict, img_w: int) -> float:
    """Convert bounding box centre pixel to horizontal bearing (radians)."""
    bbox = det["bbox"]
    cx = bbox["x"] + bbox["width"] / 2.0
    norm_x = cx / img_w if img_w > 0 else 0.5  # 0=left, 1=right
    return (norm_x - 0.5) * math.radians(CAMERA_HFOV_DEG)


# ── Main FSM ──────────────────────────────────────────────────────────────────

def run(robot: Robot) -> None:  # noqa: C901
    configure_robot(robot)

    state = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()
    state_entry_time = time.monotonic()

    # Tracked servo positions (kept in sync with every _move_servo call)
    shoulder_pos: float = SHOULDER_STOW_DEG
    elbow_pos:    float = ELBOW_STOW_DEG
    gripper_pos:  float = GRIPPER_CLOSE_DEG   # arm is stowed/closed during driving

    # IK solution computed in SCANNING, consumed by APPROACHING
    arm_turntable_deg: float = 0.0
    arm_shoulder_deg:  float = SHOULDER_STOW_DEG
    arm_elbow_deg:     float = ELBOW_STOW_DEG

    while True:

        # ── INIT ─────────────────────────────────────────────────────────────
        if state == "INIT":
            current = robot.get_state()
            if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.reset_odometry()
            if not robot.wait_for_odometry_reset(timeout=2.0):
                print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
                robot.wait_for_pose_update(timeout=0.5)
            _start_path_segment(robot, PRE_OBSTACLE_PATH, obstacle_avoidance=False)
            print("[FSM] INIT complete — entering IDLE, waiting for green light (BTN_1).")
            state = "IDLE"
            state_entry_time = time.monotonic()

        # ── IDLE ─────────────────────────────────────────────────────────────
        elif state == "IDLE":
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 255)
            if robot.get_button(Button.BTN_2):
                robot.shutdown()
                return
            if robot.get_button(Button.BTN_1):
                print("[FSM] Green light — starting pre-obstacle segment.")
                robot.set_led(LED.ORANGE, 0)
                robot.set_led(LED.GREEN, 255)
                state = "MOVING_PRE_OBSTACLE"
                state_entry_time = time.monotonic()

        # ── MOVING_PRE_OBSTACLE ───────────────────────────────────────────────
        elif state == "MOVING_PRE_OBSTACLE":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return
            nav_state = robot._nav_follow_pp_path_loop()
            if nav_state == "IDLE":
                print("[FSM] Pre-obstacle segment done — entering obstacle field.")
                _start_path_segment(
                    robot,
                    OBSTACLE_FIELD_PATH,
                    obstacle_avoidance=True,
                    x_L=OBSTACLE_FIELD_X_OFFSET,
                )
                state = "MOVING_OBSTACLE"
                state_entry_time = time.monotonic()

        # ── MOVING_OBSTACLE ───────────────────────────────────────────────────
        elif state == "MOVING_OBSTACLE":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return
            nav_state = robot._nav_follow_pp_path_loop()
            if nav_state == "IDLE":
                print("[FSM] Obstacle field done — post-obstacle segment.")
                _start_path_segment(robot, POST_OBSTACLE_PATH, obstacle_avoidance=False)
                state = "MOVING_POST_OBSTACLE"
                state_entry_time = time.monotonic()

        # ── MOVING_POST_OBSTACLE ──────────────────────────────────────────────
        elif state == "MOVING_POST_OBSTACLE":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return
            nav_state = robot._nav_follow_pp_path_loop()
            if nav_state == "IDLE":
                print("[FSM] At stop sign — stopping.")
                robot.stop()
                state = "AT_STOP_SIGN"
                state_entry_time = time.monotonic()

        # ── AT_STOP_SIGN ──────────────────────────────────────────────────────
        elif state == "AT_STOP_SIGN":
            robot.stop()
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 255)
            if time.monotonic() - state_entry_time >= STOP_SIGN_DWELL_S:
                print(f"[FSM] Stop sign dwell done — enabling vision, waiting {ENCLOSURE_WAIT_S:.0f}s for enclosure removal.")
                robot.enable_vision()
                state = "WAIT_ENCLOSURE"
                state_entry_time = time.monotonic()

        # ── WAIT_ENCLOSURE ────────────────────────────────────────────────────
        elif state == "WAIT_ENCLOSURE":
            elapsed = time.monotonic() - state_entry_time
            if elapsed >= ENCLOSURE_WAIT_S:
                print("[FSM] Enclosure wait complete — homing arm.")
                state = "ARM_HOME"
                state_entry_time = time.monotonic()

        # ── ARM_HOME ──────────────────────────────────────────────────────────
        elif state == "ARM_HOME":
            # Manual homing: turntable must be aligned to the forward mark before run.
            robot.step_enable(TURNTABLE_STEPPER)
            robot.step_set_config(TURNTABLE_STEPPER, TURNTABLE_MAX_VELOCITY, TURNTABLE_ACCELERATION)
            print("[FSM] ARM_HOME — manual home assumed (align turntable to forward mark before run).")
            if True:
                # Enable servos and move to search pose.
                robot.enable_servo(SHOULDER_CHANNEL)
                robot.enable_servo(ELBOW_CHANNEL)
                robot.enable_servo(GRIPPER_CHANNEL)
                time.sleep(0.2)
                gripper_pos  = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos,  GRIPPER_OPEN_DEG)
                shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, ARM_SEARCH_SHOULDER_DEG,
                                           SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
                elbow_pos    = _move_servo(robot, ELBOW_CHANNEL, elbow_pos,      ARM_SEARCH_ELBOW_DEG,
                                           ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
                print("[FSM] ARM_HOME — arm in search pose. Scanning for marshmallow.")
                state = "SCANNING"
                state_entry_time = time.monotonic()

        # ── SCANNING ──────────────────────────────────────────────────────────
        elif state == "SCANNING":
            if time.monotonic() - state_entry_time > SCAN_TIMEOUT_S:
                print(f"[FSM] SCANNING — {SCAN_TIMEOUT_S:.0f}s timeout, no marshmallow found. Restowing.")
                state = "RESTOWING"
                state_entry_time = time.monotonic()
            else:
                det = _best_marshmallow(robot)
                if det is not None:
                    img_w, _ = robot.get_detection_image_size()
                    bearing_rad = _detection_bearing_rad(det, img_w)

                    # Distance-based position: MARSHMALLOW_DISTANCE_MM is the
                    # assumed horizontal distance. Refine by measuring venue layout.
                    # For depth from bounding-box height:
                    #   dist = (MARSHMALLOW_REAL_HEIGHT_MM * FOCAL_LEN_PX) / bbox["height"]
                    x_mm, y_mm, z_mm = detection_to_robot_frame(
                        MARSHMALLOW_DISTANCE_MM, bearing_rad, MARSHMALLOW_HEIGHT_MM
                    )

                    try:
                        t_deg, sh_srv, el_srv = inverse_kinematics(x_mm, y_mm, z_mm, ARM_GEOMETRY)
                    except OutOfReachError as e:
                        print(f"[FSM] SCANNING — IK out of reach: {e}")
                    else:
                        if abs(t_deg) > TURNTABLE_SAFE_ARC_DEG / 2.0:
                            print(f"[FSM] SCANNING — turntable {t_deg:.1f}° outside safe arc ±{TURNTABLE_SAFE_ARC_DEG/2:.0f}°")
                        else:
                            arm_turntable_deg = t_deg
                            arm_shoulder_deg  = sh_srv
                            arm_elbow_deg     = el_srv
                            print(f"[FSM] SCANNING — target ({x_mm:.0f}, {y_mm:.0f}, {z_mm:.0f}) mm | "
                                  f"conf={det['confidence']:.2f} | "
                                  f"turntable={t_deg:.1f}° shoulder={sh_srv:.1f}° elbow={el_srv:.1f}°")
                            state = "APPROACHING"
                            state_entry_time = time.monotonic()

        # ── APPROACHING ───────────────────────────────────────────────────────
        elif state == "APPROACHING":
            print(f"[FSM] APPROACHING — gripper open, turntable → {arm_turntable_deg:.1f}°")
            # Ensure gripper is open before approaching the target.
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_OPEN_DEG)
            # Rotate turntable first (gripper is open, no collision risk).
            _turntable_to_deg(robot, arm_turntable_deg)
            # Extend shoulder and elbow to pick position.
            shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, arm_shoulder_deg,
                                       SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
            elbow_pos    = _move_servo(robot, ELBOW_CHANNEL, elbow_pos, arm_elbow_deg,
                                       ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
            time.sleep(0.5)  # settle
            print("[FSM] APPROACHING — arm at pick position.")
            state = "PICKING"
            state_entry_time = time.monotonic()

        # ── PICKING ───────────────────────────────────────────────────────────
        elif state == "PICKING":
            print(f"[FSM] PICKING — closing gripper to {GRIPPER_CLOSE_DEG:.0f}°")
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_CLOSE_DEG)
            time.sleep(0.3)
            print("[FSM] PICKING — marshmallow gripped.")
            state = "CARRYING"
            state_entry_time = time.monotonic()

        # ── CARRYING ──────────────────────────────────────────────────────────
        elif state == "CARRYING":
            print("[FSM] CARRYING — moving to plate position.")
            try:
                plate_t_deg, plate_sh, plate_el = inverse_kinematics(
                    PLATE_X_MM, PLATE_Y_MM, PLATE_Z_MM, ARM_GEOMETRY
                )
            except OutOfReachError as e:
                print(f"[FSM] CARRYING — plate IK failed: {e}. Dropping to restow.")
                state = "RESTOWING"
                state_entry_time = time.monotonic()
            else:
                # Retract to safe carry pose before rotating turntable.
                shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, ARM_CARRY_SHOULDER_DEG,
                                           SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
                elbow_pos    = _move_servo(robot, ELBOW_CHANNEL, elbow_pos, ARM_CARRY_ELBOW_DEG,
                                           ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
                # Rotate turntable to plate bearing.
                _turntable_to_deg(robot, plate_t_deg)
                # Extend to plate position.
                shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, plate_sh,
                                           SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
                elbow_pos    = _move_servo(robot, ELBOW_CHANNEL, elbow_pos, plate_el,
                                           ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
                time.sleep(0.5)  # settle
                print("[FSM] CARRYING — arm at plate position.")
                state = "PLACING"
                state_entry_time = time.monotonic()

        # ── PLACING ───────────────────────────────────────────────────────────
        elif state == "PLACING":
            print(f"[FSM] PLACING — opening gripper to roast angle ({GRIPPER_ROAST_DEG:.0f}°)")
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_ROAST_DEG)
            time.sleep(0.3)
            print("[FSM] PLACING — marshmallow released onto plate.")
            robot.enable_motor(HEATING_WIRE_MOTOR_ID, DCMotorMode.PWM)  # entry action for ROASTING
            robot.set_motor_pwm(HEATING_WIRE_MOTOR_ID, HEATING_WIRE_PWM_ON)
            print(f"[FSM] ROASTING — heating wire ON for {ROAST_TIME_S:.1f}s")
            state = "ROASTING"
            state_entry_time = time.monotonic()

        # ── ROASTING ──────────────────────────────────────────────────────────
        elif state == "ROASTING":
            if time.monotonic() - state_entry_time >= ROAST_TIME_S:
                robot.set_motor_pwm(HEATING_WIRE_MOTOR_ID, HEATING_WIRE_PWM_OFF)
                robot.disable_motor(HEATING_WIRE_MOTOR_ID)
                print("[FSM] ROASTING — wire off. Restowing arm.")
                state = "RESTOWING"
                state_entry_time = time.monotonic()

        # ── RESTOWING ─────────────────────────────────────────────────────────
        elif state == "RESTOWING":
            print("[FSM] RESTOWING — folding arm to stow position.")
            # Fold in safe order: elbow first (reduces reach), then shoulder, then turntable home.
            elbow_pos    = _move_servo(robot, ELBOW_CHANNEL, elbow_pos,    ELBOW_STOW_DEG,
                                       ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
            shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, SHOULDER_STOW_DEG,
                                       SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
            _turntable_to_deg(robot, 0.0)
            gripper_pos  = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_CLOSE_DEG)
            # Disable all arm joints.
            robot.disable_servo(ELBOW_CHANNEL)
            robot.disable_servo(SHOULDER_CHANNEL)
            robot.disable_servo(GRIPPER_CHANNEL)
            robot.step_disable(TURNTABLE_STEPPER)
            print("[FSM] RESTOWING — arm stowed. Demo complete.")
            state = "DONE"
            state_entry_time = time.monotonic()

        # ── DONE ──────────────────────────────────────────────────────────────
        elif state == "DONE":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            robot.stop()

        # ── Tick pacing ───────────────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
