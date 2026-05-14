"""
final_demo.py
=============
Full competition sequence.

State machine:
  INIT
    ↓ always (vision enabled, path pre-loaded)
  IDLE  ─── green traffic light OR BTN_1 override ───────────────────────────
    ↓  (red LED while waiting)                                                │
  MOVING_PRE_OBSTACLE   (avoidance OFF)                                       │
    ↓ nav done                                                                │
  MOVING_OBSTACLE       (avoidance ON)                                        │
    ↓ nav done                                                                │
  MOVING_POST_OBSTACLE  (avoidance OFF)                                       │
    ↓ nav done                                                                │
  AT_STOP_SIGN          (red LED; after dwell: stop sign gone OR BTN_1 → go) │
    ↓                                                                         │
  WAIT_ENCLOSURE        (ENCLOSURE_WAIT_S, team removes competition cover)    │
    ↓                                                                         │
  ARM_HOME              (enable turntable + campan, move to search pose)      │
    ↓                                                         BTN_2 ─────────┘
  SCANNING              (3-frame camera pan, pick best hit)  (shutdown at any
    ↓ found / timeout → RESTOWING                             arm state)
  RANGING               (rotate turntable, extend arm, ultrasonic distance)
    ↓ got range + IK / fail → RESTOWING
  APPROACHING           (shoulder + elbow to pick position)
    ↓
  PICKING               (close gripper)
    ↓
  CARRYING              (retract, rotate turntable, extend to plate)
    ↓ plate IK fails → RESTOWING
  PLACING               (open gripper to GRIPPER_ROAST_DEG)
    ↓
  ROASTING              (heat wire for ROAST_TIME_S)
    ↓
  RESTOWING             (fold arm, center campan, home turntable, disable all)
    ↓
  DONE

Required nodes:  bridge + robot + rplidar + sensors + vision
Launch files:    robot.launch.py + everything_but_robot.launch.py
"""
from __future__ import annotations

import math
import time

from robot.arm_kinematics import OutOfReachError, inverse_kinematics
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
    VELOCITY_KD,
    VELOCITY_KI,
    VELOCITY_KP,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot
from robot.tests.scripts._manipulator_config import (
    ARM_CARRY_ELBOW_DEG,
    ARM_CARRY_SHOULDER_DEG,
    ARM_GEOMETRY,
    ARM_L1_MM,
    ARM_SEARCH_ELBOW_DEG,
    ARM_SEARCH_SHOULDER_DEG,
    ARM_SERVO_STEP_DEG,
    ARM_SERVO_STEP_DWELL,
    CAMPAN_ACCELERATION,
    CAMPAN_MAX_VELOCITY,
    CAMPAN_POSITIONS_DEG,
    CAMPAN_SETTLE_S,
    CAMPAN_STEPPER,
    campan_deg_to_steps,
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
    TURNTABLE_MAX_DEG,
    TURNTABLE_MAX_VELOCITY,
    TURNTABLE_MIN_DEG,
    TURNTABLE_SCAN_ARC_DEG,
    TURNTABLE_STEPPER,
    turntable_deg_to_steps,
    ULTRASONIC_FOREARM_OFFSET_MM,
    ULTRASONIC_HEIGHT_OFFSET_MM,
)
from robot.util import densify_polyline

# ── Vision thresholds ─────────────────────────────────────────────────────────
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50
VISION_STALE_S               = 3.0   # seconds before vision is considered lost

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
    (2745.0, 305.0),   # stop sign position
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
    target_deg = max(TURNTABLE_MIN_DEG, min(TURNTABLE_MAX_DEG, target_deg))
    steps = turntable_deg_to_steps(target_deg + TURNTABLE_HOME_OFFSET_DEG)
    robot.step_move(TURNTABLE_STEPPER, steps, StepMoveType.ABSOLUTE)


def _campan_to_deg(robot: Robot, target_deg: float) -> None:
    steps = campan_deg_to_steps(target_deg)
    robot.step_move(CAMPAN_STEPPER, steps, StepMoveType.ABSOLUTE)


def _detection_bearing_deg(det: dict, img_w: int) -> float:
    bbox = det["bbox"]
    cx = bbox["x"] + bbox["width"] / 2.0
    norm_x = cx / img_w if img_w > 0 else 0.5
    return (norm_x - 0.5) * CAMERA_HFOV_DEG


def _get_ultrasonic_mm(robot: Robot) -> float:
    # return robot.get_ultrasonic_mm()   ← uncomment when the API exists
    raise NotImplementedError("Ultrasonic API not yet implemented — wire up _get_ultrasonic_mm()")


def _find_traffic_light_color(robot: Robot) -> str | None:
    """Return 'red' or 'green' from the highest-confidence traffic light detection, or None."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_S):
        return None
    best_color = None
    best_conf = -1.0
    for det in robot.get_detections("traffic light"):
        conf = float(det["confidence"])
        if conf < MIN_TRAFFIC_LIGHT_CONFIDENCE:
            continue
        color = det.get("attributes", {}).get("color", {}).get("value")
        if color in ("red", "green") and conf > best_conf:
            best_conf = conf
            best_color = str(color)
    return best_color


def _restow(robot: Robot, shoulder_pos: float, elbow_pos: float, gripper_pos: float) -> None:
    print("[FSM] RESTOWING — folding arm to stow position.")
    elbow_pos    = _move_servo(robot, ELBOW_CHANNEL,    elbow_pos,    ELBOW_STOW_DEG,    ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
    shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, SHOULDER_STOW_DEG, SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
    _campan_to_deg(robot, 0.0)
    _turntable_to_deg(robot, TURNTABLE_MAX_DEG)
    _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_CLOSE_DEG)
    robot.disable_servo(ELBOW_CHANNEL)
    robot.disable_servo(SHOULDER_CHANNEL)
    robot.disable_servo(GRIPPER_CHANNEL)
    robot.step_disable(CAMPAN_STEPPER)
    robot.step_disable(TURNTABLE_STEPPER)


# ── Main FSM ──────────────────────────────────────────────────────────────────

def run(robot: Robot) -> None:  # noqa: C901
    configure_robot(robot)

    state = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()
    state_entry_time = time.monotonic()

    shoulder_pos: float = SHOULDER_STOW_DEG
    elbow_pos:    float = ELBOW_STOW_DEG
    gripper_pos:  float = GRIPPER_CLOSE_DEG

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
            robot.enable_vision()
            _start_path_segment(robot, PRE_OBSTACLE_PATH, obstacle_avoidance=False)
            print("[FSM] INIT complete — entering IDLE. Waiting for green traffic light (or BTN_1 to override).")
            state = "IDLE"
            state_entry_time = time.monotonic()

        # ── IDLE ─────────────────────────────────────────────────────────────
        # Red LED on = waiting for green. Vision watches for a green traffic light.
        # BTN_1 = manual override to start immediately.
        # BTN_2 = shutdown.
        elif state == "IDLE":
            robot.set_led(LED.RED, 255)
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 0)

            if robot.get_button(Button.BTN_2):
                robot.set_led(LED.RED, 0)
                robot.shutdown()
                return

            traffic_color = _find_traffic_light_color(robot)
            if traffic_color == "green":
                print("[FSM] Green traffic light detected — starting pre-obstacle segment.")
                robot.set_led(LED.RED, 0)
                robot.set_led(LED.GREEN, 255)
                state = "MOVING_PRE_OBSTACLE"
                state_entry_time = time.monotonic()
            elif traffic_color == "red":
                robot.set_led(LED.RED, 255)
                robot.set_led(LED.GREEN, 0)
            elif robot.get_button(Button.BTN_1):
                print("[FSM] BTN_1 override — starting pre-obstacle segment.")
                robot.set_led(LED.RED, 0)
                robot.set_led(LED.GREEN, 255)
                state = "MOVING_PRE_OBSTACLE"
                state_entry_time = time.monotonic()

        # ── MOVING_PRE_OBSTACLE ───────────────────────────────────────────────
        elif state == "MOVING_PRE_OBSTACLE":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
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
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
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
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)

            # Stop sign detected mid-segment: override nav and stop immediately.
            if robot.get_detections("stop sign"):
                robot.stop()
                print("[FSM] Stop sign detected mid-route — stopping early.")
                state = "AT_STOP_SIGN"
                state_entry_time = time.monotonic()
            elif robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return
            else:
                nav_state = robot._nav_follow_pp_path_loop()
                if nav_state == "IDLE":
                    print("[FSM] Post-obstacle nav done — at stop sign position.")
                    robot.stop()
                    state = "AT_STOP_SIGN"
                    state_entry_time = time.monotonic()

        # ── AT_STOP_SIGN ──────────────────────────────────────────────────────
        # Mandatory stop. Hold red LED. After STOP_SIGN_DWELL_S:
        #   - if vision sees stop sign gone → auto-advance
        #   - BTN_1 manual override → advance regardless
        # This handles both "stop sign is removed" and "no vision coverage" cases.
        elif state == "AT_STOP_SIGN":
            robot.stop()
            robot.set_led(LED.RED, 255)
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 0)

            elapsed = time.monotonic() - state_entry_time
            if elapsed >= STOP_SIGN_DWELL_S:
                sign_still_visible = bool(robot.get_detections("stop sign"))
                if not sign_still_visible or robot.get_button(Button.BTN_1):
                    reason = "stop sign gone" if not sign_still_visible else "BTN_1 override"
                    print(f"[FSM] Stop sign clear ({reason}) — waiting {ENCLOSURE_WAIT_S:.0f}s for enclosure removal.")
                    robot.set_led(LED.RED, 0)
                    robot.set_led(LED.ORANGE, 255)
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
            robot.set_led(LED.ORANGE, 255)
            robot.set_led(LED.GREEN, 0)
            robot.step_enable(TURNTABLE_STEPPER)
            robot.step_set_config(TURNTABLE_STEPPER, TURNTABLE_MAX_VELOCITY, TURNTABLE_ACCELERATION)
            robot.step_enable(CAMPAN_STEPPER)
            robot.step_set_config(CAMPAN_STEPPER, CAMPAN_MAX_VELOCITY, CAMPAN_ACCELERATION)
            print("[FSM] ARM_HOME — align turntable and camera pan to forward marks before run.")
            robot.enable_servo(SHOULDER_CHANNEL)
            robot.enable_servo(ELBOW_CHANNEL)
            robot.enable_servo(GRIPPER_CHANNEL)
            time.sleep(0.2)
            gripper_pos  = _move_servo(robot, GRIPPER_CHANNEL,  gripper_pos,  GRIPPER_OPEN_DEG)
            shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, ARM_SEARCH_SHOULDER_DEG,
                                       SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
            elbow_pos    = _move_servo(robot, ELBOW_CHANNEL,    elbow_pos,    ARM_SEARCH_ELBOW_DEG,
                                       ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
            print("[FSM] ARM_HOME — arm in search pose. Scanning for marshmallow.")
            state = "SCANNING"
            state_entry_time = time.monotonic()

        # ── SCANNING ──────────────────────────────────────────────────────────
        # Pan camera through 3 windows (CAMPAN_POSITIONS_DEG) to cover ±90° arc.
        # Collect all confident marshmallow hits across all frames, pick the best
        # (highest confidence). World bearing = cam_pan_deg + pixel_bearing.
        elif state == "SCANNING":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            if robot.get_button(Button.BTN_2):
                _restow(robot, shoulder_pos, elbow_pos, gripper_pos)
                robot.shutdown()
                return
            if time.monotonic() - state_entry_time > SCAN_TIMEOUT_S:
                print(f"[FSM] SCANNING — {SCAN_TIMEOUT_S:.0f}s timeout, no marshmallow. Restowing.")
                state = "RESTOWING"
                state_entry_time = time.monotonic()
            else:
                img_w, _ = robot.get_detection_image_size()
                all_hits: list[dict] = []
                for pan_deg in CAMPAN_POSITIONS_DEG:
                    _campan_to_deg(robot, pan_deg)
                    time.sleep(CAMPAN_SETTLE_S)
                    for det in robot.get_detections(MARSHMALLOW_CLASS):
                        if float(det["confidence"]) < MIN_CONFIDENCE_MARSHMALLOW:
                            continue
                        pixel_bearing_deg = _detection_bearing_deg(det, img_w)
                        world_bearing_deg = pan_deg + pixel_bearing_deg
                        all_hits.append({"det": det, "bearing_deg": world_bearing_deg})

                _campan_to_deg(robot, 0.0)   # return camera to center

                if all_hits:
                    best = max(all_hits, key=lambda h: float(h["det"]["confidence"]))
                    t_deg = best["bearing_deg"]
                    if not (-TURNTABLE_SCAN_ARC_DEG <= t_deg <= TURNTABLE_SCAN_ARC_DEG):
                        print(f"[FSM] SCANNING — best hit at {t_deg:.1f}° outside ±{TURNTABLE_SCAN_ARC_DEG:.0f}° scan zone, ignoring.")
                    else:
                        arm_turntable_deg = t_deg
                        print(f"[FSM] SCANNING — marshmallow at {t_deg:.1f}° "
                              f"(conf={best['det']['confidence']:.2f}, {len(all_hits)} hits across frames)")
                        state = "RANGING"
                        state_entry_time = time.monotonic()

        # ── RANGING ───────────────────────────────────────────────────────────
        # Rotate turntable to marshmallow bearing. Extend arm so the forearm is
        # horizontal with the ultrasonic sensor aimed at marshmallow height.
        # Measure distance, correct for height offset, run IK for pick position.
        elif state == "RANGING":
            if robot.get_button(Button.BTN_2):
                _restow(robot, shoulder_pos, elbow_pos, gripper_pos)
                robot.shutdown()
                return

            print(f"[FSM] RANGING — rotating turntable to {arm_turntable_deg:.1f}°")
            _turntable_to_deg(robot, arm_turntable_deg)

            sensor_target_h = MARSHMALLOW_HEIGHT_MM - ULTRASONIC_HEIGHT_OFFSET_MM
            sin_arg = (sensor_target_h - ARM_GEOMETRY.shoulder_height_mm) / ARM_L1_MM
            sin_arg = max(-1.0, min(1.0, sin_arg))
            ranging_shoulder_geo = math.degrees(math.asin(sin_arg))
            ranging_elbow_geo    = ranging_shoulder_geo + 180.0

            ranging_shoulder_srv = ARM_GEOMETRY.shoulder_geo_to_servo(ranging_shoulder_geo)
            ranging_elbow_srv    = ARM_GEOMETRY.elbow_geo_to_servo(ranging_elbow_geo)
            ranging_shoulder_srv = max(SHOULDER_SAFE_MIN, min(SHOULDER_SAFE_MAX, ranging_shoulder_srv))
            ranging_elbow_srv    = max(ELBOW_SAFE_MIN,    min(ELBOW_SAFE_MAX,    ranging_elbow_srv))

            print(f"[FSM] RANGING — shoulder_geo={ranging_shoulder_geo:.1f}° srv={ranging_shoulder_srv:.1f}°  "
                  f"elbow_geo={ranging_elbow_geo:.1f}° srv={ranging_elbow_srv:.1f}°")
            shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, ranging_shoulder_srv,
                                       SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
            elbow_pos    = _move_servo(robot, ELBOW_CHANNEL,    elbow_pos,    ranging_elbow_srv,
                                       ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
            time.sleep(0.3)

            try:
                d_raw_mm = _get_ultrasonic_mm(robot)
            except NotImplementedError as exc:
                print(f"[FSM] RANGING — {exc}")
                state = "RESTOWING"
                state_entry_time = time.monotonic()
            else:
                sensor_horiz = (
                    ARM_GEOMETRY.shoulder_offset_mm
                    + ARM_L1_MM * math.cos(math.radians(ranging_shoulder_geo))
                    + ULTRASONIC_FOREARM_OFFSET_MM
                )
                actual_sensor_h = (
                    ARM_GEOMETRY.shoulder_height_mm
                    + ARM_L1_MM * math.sin(math.radians(ranging_shoulder_geo))
                    + ULTRASONIC_HEIGHT_OFFSET_MM
                )
                delta_h = MARSHMALLOW_HEIGHT_MM - actual_sensor_h
                d_horizontal = math.sqrt(max(0.0, d_raw_mm ** 2 - delta_h ** 2))
                mallow_reach = sensor_horiz + d_horizontal
                mallow_x = mallow_reach * math.cos(math.radians(arm_turntable_deg))
                mallow_y = mallow_reach * math.sin(math.radians(arm_turntable_deg))
                mallow_z = MARSHMALLOW_HEIGHT_MM
                print(f"[FSM] RANGING — ultrasonic={d_raw_mm:.0f} mm  horizontal={d_horizontal:.0f} mm  "
                      f"reach={mallow_reach:.0f} mm  target=({mallow_x:.0f}, {mallow_y:.0f}, {mallow_z:.0f})")

                try:
                    _, sh_srv, el_srv = inverse_kinematics(mallow_x, mallow_y, mallow_z, ARM_GEOMETRY)
                except OutOfReachError as e:
                    print(f"[FSM] RANGING — IK out of reach: {e}. Restowing.")
                    state = "RESTOWING"
                    state_entry_time = time.monotonic()
                else:
                    arm_shoulder_deg = sh_srv
                    arm_elbow_deg    = el_srv
                    print(f"[FSM] RANGING — pick IK: shoulder={sh_srv:.1f}° elbow={el_srv:.1f}°")
                    state = "APPROACHING"
                    state_entry_time = time.monotonic()

        # ── APPROACHING ───────────────────────────────────────────────────────
        elif state == "APPROACHING":
            print(f"[FSM] APPROACHING — turntable already at {arm_turntable_deg:.1f}°, moving to pick pose")
            gripper_pos  = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_OPEN_DEG)
            shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, arm_shoulder_deg,
                                       SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
            elbow_pos    = _move_servo(robot, ELBOW_CHANNEL, elbow_pos, arm_elbow_deg,
                                       ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
            time.sleep(0.5)
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
                shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, ARM_CARRY_SHOULDER_DEG,
                                           SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
                elbow_pos    = _move_servo(robot, ELBOW_CHANNEL, elbow_pos, ARM_CARRY_ELBOW_DEG,
                                           ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
                _turntable_to_deg(robot, plate_t_deg)
                shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, plate_sh,
                                           SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
                elbow_pos    = _move_servo(robot, ELBOW_CHANNEL, elbow_pos, plate_el,
                                           ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
                time.sleep(0.5)
                print("[FSM] CARRYING — arm at plate position.")
                state = "PLACING"
                state_entry_time = time.monotonic()

        # ── PLACING ───────────────────────────────────────────────────────────
        elif state == "PLACING":
            print(f"[FSM] PLACING — opening gripper to roast angle ({GRIPPER_ROAST_DEG:.0f}°)")
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_ROAST_DEG)
            time.sleep(0.3)
            print("[FSM] PLACING — marshmallow released onto plate.")
            robot.enable_motor(HEATING_WIRE_MOTOR_ID, DCMotorMode.PWM)
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
            _restow(robot, shoulder_pos, elbow_pos, gripper_pos)
            print("[FSM] Demo complete.")
            state = "DONE"
            state_entry_time = time.monotonic()

        # ── DONE ──────────────────────────────────────────────────────────────
        elif state == "DONE":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.RED, 0)
            robot.set_led(LED.ORANGE, 0)
            robot.stop()

        # ── Tick pacing ───────────────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
