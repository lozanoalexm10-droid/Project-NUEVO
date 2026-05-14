"""
manipulator_demo.py
===================
Standalone manipulator sequence — no driving.

Robot must already be parked at the station before running this.
BTN_1 starts the sequence. BTN_2 aborts and restows at any time.

State machine:
  IDLE → ARM_HOME → SCANNING → RANGING → APPROACHING → PICKING
       → CARRYING → PLACING → ROASTING → RESTOWING → DONE

Nodes required:  bridge + robot + vision
Launch file:     robot.launch.py
"""
from __future__ import annotations

import math
import time

from robot.arm_kinematics import OutOfReachError, inverse_kinematics
from robot.hardware_map import Button, DCMotorMode, DEFAULT_FSM_HZ, LED, StepMoveType
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


# ── Helpers (same as final_demo) ──────────────────────────────────────────────

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
    """Move the camera pan stepper to target_deg from its home position (blocking)."""
    steps = campan_deg_to_steps(target_deg)
    robot.step_move(CAMPAN_STEPPER, steps, StepMoveType.ABSOLUTE)


def _detection_bearing_deg(det: dict, img_w: int) -> float:
    """Return the bearing to a detection relative to camera center, in degrees."""
    bbox = det["bbox"]
    cx = bbox["x"] + bbox["width"] / 2.0
    norm_x = cx / img_w if img_w > 0 else 0.5
    return (norm_x - 0.5) * CAMERA_HFOV_DEG


def _get_ultrasonic_mm(robot: Robot) -> float:
    """Return ultrasonic range measurement in mm.

    TODO: replace with the real robot API call once the ultrasonic node is wired up.
    Placeholder returns a large value so ranging fails gracefully during development.
    """
    # return robot.get_ultrasonic_mm()   ← uncomment when the API exists
    raise NotImplementedError("Ultrasonic API not yet implemented — wire up _get_ultrasonic_mm()")


def _restow(robot: Robot, shoulder_pos: float, elbow_pos: float, gripper_pos: float) -> None:
    print("[DEMO] Restowing arm.")
    elbow_pos    = _move_servo(robot, ELBOW_CHANNEL,    elbow_pos,    ELBOW_STOW_DEG,    ELBOW_SAFE_MIN,    ELBOW_SAFE_MAX)
    shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, SHOULDER_STOW_DEG, SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
    _campan_to_deg(robot, 0.0)           # center camera before turntable swings to stow
    _turntable_to_deg(robot, TURNTABLE_MAX_DEG)
    _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_CLOSE_DEG)
    robot.disable_servo(ELBOW_CHANNEL)
    robot.disable_servo(SHOULDER_CHANNEL)
    robot.disable_servo(GRIPPER_CHANNEL)
    robot.step_disable(CAMPAN_STEPPER)
    robot.step_disable(TURNTABLE_STEPPER)


# ── Main FSM ──────────────────────────────────────────────────────────────────

def run(robot: Robot) -> None:  # noqa: C901
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
            robot.enable_vision()
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
                print("[DEMO] BTN_1 — starting manipulator sequence.")
                state = "ARM_HOME"
                state_entry_time = time.monotonic()

        # ── ARM_HOME ──────────────────────────────────────────────────────────
        elif state == "ARM_HOME":
            robot.set_led(LED.ORANGE, 255)
            robot.step_enable(TURNTABLE_STEPPER)
            robot.step_set_config(TURNTABLE_STEPPER, TURNTABLE_MAX_VELOCITY, TURNTABLE_ACCELERATION)
            robot.step_enable(CAMPAN_STEPPER)
            robot.step_set_config(CAMPAN_STEPPER, CAMPAN_MAX_VELOCITY, CAMPAN_ACCELERATION)
            print("[DEMO] ARM_HOME — manual home assumed. Align turntable and camera pan to forward marks before run.")
            robot.enable_servo(SHOULDER_CHANNEL)
            robot.enable_servo(ELBOW_CHANNEL)
            robot.enable_servo(GRIPPER_CHANNEL)
            time.sleep(0.2)
            gripper_pos  = _move_servo(robot, GRIPPER_CHANNEL,  gripper_pos,  GRIPPER_OPEN_DEG)
            shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, ARM_SEARCH_SHOULDER_DEG,
                                       SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
            elbow_pos    = _move_servo(robot, ELBOW_CHANNEL,    elbow_pos,    ARM_SEARCH_ELBOW_DEG,
                                       ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
            print("[DEMO] ARM_HOME — arm in search pose.")
            state = "SCANNING"
            state_entry_time = time.monotonic()

        # ── SCANNING ──────────────────────────────────────────────────────────
        # Pan camera through 3 windows to cover full ±90° arc. Collect every
        # confident marshmallow detection across all frames and pick the single
        # best (highest confidence). The world bearing for each hit is:
        #   world_bearing = cam_pan_position + pixel_bearing_from_cam_center
        elif state == "SCANNING":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            if robot.get_button(Button.BTN_2):
                _restow(robot, shoulder_pos, elbow_pos, gripper_pos)
                robot.shutdown()
                return
            if time.monotonic() - state_entry_time > SCAN_TIMEOUT_S:
                print(f"[DEMO] SCANNING — {SCAN_TIMEOUT_S:.0f}s timeout, no marshmallow. Restowing.")
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

                _campan_to_deg(robot, 0.0)  # return camera to center

                if all_hits:
                    best = max(all_hits, key=lambda h: float(h["det"]["confidence"]))
                    t_deg = best["bearing_deg"]
                    if not (-TURNTABLE_SCAN_ARC_DEG <= t_deg <= TURNTABLE_SCAN_ARC_DEG):
                        print(f"[DEMO] SCANNING — best hit at {t_deg:.1f}° is outside ±{TURNTABLE_SCAN_ARC_DEG:.0f}° scan zone, ignoring.")
                    else:
                        arm_turntable_deg = t_deg
                        print(f"[DEMO] SCANNING — best marshmallow at world bearing {t_deg:.1f}° "
                              f"(conf={best['det']['confidence']:.2f}, {len(all_hits)} total hits across 3 frames)")
                        state = "RANGING"
                        state_entry_time = time.monotonic()

        # ── RANGING ───────────────────────────────────────────────────────────
        # Rotate turntable to marshmallow bearing, extend arm so the ultrasonic
        # sensor is at marshmallow height with the forearm horizontal, measure
        # distance, then compute IK for the actual pick position.
        elif state == "RANGING":
            if robot.get_button(Button.BTN_2):
                _restow(robot, shoulder_pos, elbow_pos, gripper_pos)
                robot.shutdown()
                return

            print(f"[DEMO] RANGING — rotating turntable to {arm_turntable_deg:.1f}°")
            _turntable_to_deg(robot, arm_turntable_deg)

            # Solve for shoulder_geo that places the sensor at marshmallow height
            # with a horizontal forearm: sensor_h = shoulder_h + L1*sin(q1) + z_offset
            sensor_target_h = MARSHMALLOW_HEIGHT_MM - ULTRASONIC_HEIGHT_OFFSET_MM
            sin_arg = (sensor_target_h - ARM_GEOMETRY.shoulder_height_mm) / ARM_L1_MM
            sin_arg = max(-1.0, min(1.0, sin_arg))   # clamp floating-point noise
            ranging_shoulder_geo = math.degrees(math.asin(sin_arg))
            # Horizontal forearm requires elbow_geo = shoulder_geo + 180°
            ranging_elbow_geo = ranging_shoulder_geo + 180.0

            ranging_shoulder_srv = ARM_GEOMETRY.shoulder_geo_to_servo(ranging_shoulder_geo)
            ranging_elbow_srv    = ARM_GEOMETRY.elbow_geo_to_servo(ranging_elbow_geo)

            # Clamp to safe servo limits (ranging pose may not always be feasible)
            ranging_shoulder_srv = max(SHOULDER_SAFE_MIN, min(SHOULDER_SAFE_MAX, ranging_shoulder_srv))
            ranging_elbow_srv    = max(ELBOW_SAFE_MIN,    min(ELBOW_SAFE_MAX,    ranging_elbow_srv))

            print(f"[DEMO] RANGING — arm pose: shoulder_geo={ranging_shoulder_geo:.1f}° "
                  f"shoulder_srv={ranging_shoulder_srv:.1f}° elbow_geo={ranging_elbow_geo:.1f}° "
                  f"elbow_srv={ranging_elbow_srv:.1f}°")

            shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, ranging_shoulder_srv,
                                       SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
            elbow_pos    = _move_servo(robot, ELBOW_CHANNEL,    elbow_pos,    ranging_elbow_srv,
                                       ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
            time.sleep(0.3)   # settle before firing ultrasonic

            try:
                d_raw_mm = _get_ultrasonic_mm(robot)
            except NotImplementedError as exc:
                print(f"[DEMO] RANGING — {exc}")
                state = "RESTOWING"
                state_entry_time = time.monotonic()
            else:
                # Sensor position in world frame (forearm is horizontal)
                sensor_horiz_from_turntable = (
                    ARM_GEOMETRY.shoulder_offset_mm
                    + ARM_L1_MM * math.cos(math.radians(ranging_shoulder_geo))
                    + ULTRASONIC_FOREARM_OFFSET_MM
                )
                actual_sensor_h = (
                    ARM_GEOMETRY.shoulder_height_mm
                    + ARM_L1_MM * math.sin(math.radians(ranging_shoulder_geo))
                    + ULTRASONIC_HEIGHT_OFFSET_MM
                )
                # Correct for any remaining height difference between sensor and mallow
                delta_h = MARSHMALLOW_HEIGHT_MM - actual_sensor_h
                d_horizontal = math.sqrt(max(0.0, d_raw_mm ** 2 - delta_h ** 2))
                mallow_reach = sensor_horiz_from_turntable + d_horizontal
                mallow_x = mallow_reach * math.cos(math.radians(arm_turntable_deg))
                mallow_y = mallow_reach * math.sin(math.radians(arm_turntable_deg))
                mallow_z = MARSHMALLOW_HEIGHT_MM
                print(f"[DEMO] RANGING — ultrasonic={d_raw_mm:.0f} mm  horizontal={d_horizontal:.0f} mm  "
                      f"reach={mallow_reach:.0f} mm  target=({mallow_x:.0f}, {mallow_y:.0f}, {mallow_z:.0f})")

                try:
                    _, sh_srv, el_srv = inverse_kinematics(mallow_x, mallow_y, mallow_z, ARM_GEOMETRY)
                except OutOfReachError as e:
                    print(f"[DEMO] RANGING — IK out of reach: {e}. Restowing.")
                    state = "RESTOWING"
                    state_entry_time = time.monotonic()
                else:
                    arm_shoulder_deg = sh_srv
                    arm_elbow_deg    = el_srv
                    print(f"[DEMO] RANGING — pick IK: shoulder={sh_srv:.1f}° elbow={el_srv:.1f}°")
                    state = "APPROACHING"
                    state_entry_time = time.monotonic()

        # ── APPROACHING ───────────────────────────────────────────────────────
        elif state == "APPROACHING":
            print(f"[DEMO] APPROACHING — turntable already at {arm_turntable_deg:.1f}°, moving to pick pose")
            gripper_pos  = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_OPEN_DEG)
            shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, arm_shoulder_deg,
                                       SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
            elbow_pos    = _move_servo(robot, ELBOW_CHANNEL, elbow_pos, arm_elbow_deg,
                                       ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
            time.sleep(0.5)
            print("[DEMO] APPROACHING — at pick position.")
            state = "PICKING"
            state_entry_time = time.monotonic()

        # ── PICKING ───────────────────────────────────────────────────────────
        elif state == "PICKING":
            print(f"[DEMO] PICKING — closing gripper to {GRIPPER_CLOSE_DEG:.0f}°")
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_CLOSE_DEG)
            time.sleep(0.3)
            print("[DEMO] PICKING — gripped.")
            state = "CARRYING"
            state_entry_time = time.monotonic()

        # ── CARRYING ──────────────────────────────────────────────────────────
        elif state == "CARRYING":
            print("[DEMO] CARRYING — moving to plate.")
            try:
                plate_t_deg, plate_sh, plate_el = inverse_kinematics(
                    PLATE_X_MM, PLATE_Y_MM, PLATE_Z_MM, ARM_GEOMETRY
                )
            except OutOfReachError as e:
                print(f"[DEMO] CARRYING — plate IK failed: {e}. Restowing.")
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
                print("[DEMO] CARRYING — at plate position.")
                state = "PLACING"
                state_entry_time = time.monotonic()

        # ── PLACING ───────────────────────────────────────────────────────────
        elif state == "PLACING":
            print(f"[DEMO] PLACING — opening gripper to {GRIPPER_ROAST_DEG:.0f}°")
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_ROAST_DEG)
            time.sleep(0.3)
            print("[DEMO] PLACING — released.")
            robot.enable_motor(HEATING_WIRE_MOTOR_ID, DCMotorMode.PWM)  # entry action for ROASTING
            robot.set_motor_pwm(HEATING_WIRE_MOTOR_ID, HEATING_WIRE_PWM_ON)
            print(f"[DEMO] ROASTING — heating wire ON for {ROAST_TIME_S:.1f}s")
            state = "ROASTING"
            state_entry_time = time.monotonic()

        # ── ROASTING ──────────────────────────────────────────────────────────
        elif state == "ROASTING":
            if time.monotonic() - state_entry_time >= ROAST_TIME_S:
                robot.set_motor_pwm(HEATING_WIRE_MOTOR_ID, HEATING_WIRE_PWM_OFF)
                robot.disable_motor(HEATING_WIRE_MOTOR_ID)
                print("[DEMO] ROASTING — wire off.")
                state = "RESTOWING"
                state_entry_time = time.monotonic()

        # ── RESTOWING ─────────────────────────────────────────────────────────
        elif state == "RESTOWING":
            _restow(robot, shoulder_pos, elbow_pos, gripper_pos)
            print("[DEMO] Done.")
            state = "DONE"
            state_entry_time = time.monotonic()

        # ── DONE ──────────────────────────────────────────────────────────────
        elif state == "DONE":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
