"""
manipulator_demo.py
===================
Standalone manipulator sequence — no driving.

Robot must already be parked at the station before running this.
BTN_1 starts the sequence. BTN_2 aborts and restows at any time.

State machine:
  IDLE → ARM_HOME → SCANNING → APPROACHING → PICKING
       → CARRYING → PLACING → ROASTING → RESTOWING → DONE

Nodes required:  bridge + robot + vision
Launch file:     robot.launch.py
"""
from __future__ import annotations

import math
import time

from robot.arm_kinematics import OutOfReachError, detection_to_robot_frame, inverse_kinematics
from robot.hardware_map import Button, DCMotorMode, DEFAULT_FSM_HZ, LED, StepMoveType
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
    TURNTABLE_ACCELERATION,
    TURNTABLE_HOME_OFFSET_DEG,
    TURNTABLE_MAX_VELOCITY,
    TURNTABLE_SAFE_ARC_DEG,
    TURNTABLE_STEPPER,
    turntable_deg_to_steps,
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
    steps = turntable_deg_to_steps(target_deg + TURNTABLE_HOME_OFFSET_DEG)
    robot.step_move(TURNTABLE_STEPPER, steps, StepMoveType.ABSOLUTE)


def _best_marshmallow(robot: Robot) -> dict | None:
    dets = robot.get_detections(MARSHMALLOW_CLASS)
    confident = [d for d in dets if float(d["confidence"]) >= MIN_CONFIDENCE_MARSHMALLOW]
    if not confident:
        return None
    return max(confident, key=lambda d: float(d["confidence"]))


def _detection_bearing_rad(det: dict, img_w: int) -> float:
    bbox = det["bbox"]
    cx = bbox["x"] + bbox["width"] / 2.0
    norm_x = cx / img_w if img_w > 0 else 0.5
    return (norm_x - 0.5) * math.radians(CAMERA_HFOV_DEG)


def _restow(robot: Robot, shoulder_pos: float, elbow_pos: float, gripper_pos: float) -> None:
    print("[DEMO] Restowing arm.")
    elbow_pos    = _move_servo(robot, ELBOW_CHANNEL,    elbow_pos,    ELBOW_STOW_DEG,    ELBOW_SAFE_MIN,    ELBOW_SAFE_MAX)
    shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, SHOULDER_STOW_DEG, SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
    _turntable_to_deg(robot, 0.0)
    _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_CLOSE_DEG)
    robot.disable_servo(ELBOW_CHANNEL)
    robot.disable_servo(SHOULDER_CHANNEL)
    robot.disable_servo(GRIPPER_CHANNEL)
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
            print("[DEMO] ARM_HOME — manual home assumed (align turntable to forward mark before run).")
            if True:
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
                det = _best_marshmallow(robot)
                if det is not None:
                    img_w, _ = robot.get_detection_image_size()
                    bearing_rad = _detection_bearing_rad(det, img_w)
                    x_mm, y_mm, z_mm = detection_to_robot_frame(
                        MARSHMALLOW_DISTANCE_MM, bearing_rad, MARSHMALLOW_HEIGHT_MM
                    )
                    try:
                        t_deg, sh_srv, el_srv = inverse_kinematics(x_mm, y_mm, z_mm, ARM_GEOMETRY)
                    except OutOfReachError as e:
                        print(f"[DEMO] SCANNING — IK out of reach: {e}")
                    else:
                        if abs(t_deg) > TURNTABLE_SAFE_ARC_DEG / 2.0:
                            print(f"[DEMO] SCANNING — bearing {t_deg:.1f}° outside safe arc")
                        else:
                            arm_turntable_deg = t_deg
                            arm_shoulder_deg  = sh_srv
                            arm_elbow_deg     = el_srv
                            print(f"[DEMO] SCANNING — marshmallow ({x_mm:.0f}, {y_mm:.0f}, {z_mm:.0f}) mm | "
                                  f"conf={det['confidence']:.2f} | "
                                  f"turntable={t_deg:.1f}° shoulder={sh_srv:.1f}° elbow={el_srv:.1f}°")
                            state = "APPROACHING"
                            state_entry_time = time.monotonic()

        # ── APPROACHING ───────────────────────────────────────────────────────
        elif state == "APPROACHING":
            print(f"[DEMO] APPROACHING — turntable → {arm_turntable_deg:.1f}°")
            gripper_pos  = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_OPEN_DEG)
            _turntable_to_deg(robot, arm_turntable_deg)
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
