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
    CAMERA_HEIGHT_MM,
    CAMERA_HFOV_DEG,
    CUP_CLASS,
    CUP_DIAMETER_MM,
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
    MALLOW_CUP_BEARING_MATCH_DEG,
    MARSHMALLOW_CLASS,
    MARSHMALLOW_DIAMETER_MM,
    MARSHMALLOW_HEIGHT_MM,
    MIN_CONFIDENCE_CUP,
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
    snap_to_cup_tier_mm,
    TURNTABLE_ACCELERATION,
    TURNTABLE_HOME_OFFSET_DEG,
    TURNTABLE_MAX_DEG,
    TURNTABLE_MAX_VELOCITY,
    TURNTABLE_MIN_DEG,
    TURNTABLE_SCAN_ARC_DEG,
    TURNTABLE_STEPPER,
    turntable_deg_to_steps,
    ULTRASONIC_FOREARM_OFFSET_MM,
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


def _turntable_to_deg(robot: Robot, target_deg: float, home_offset_deg: float = TURNTABLE_HOME_OFFSET_DEG) -> None:
    target_deg = max(TURNTABLE_MIN_DEG, min(TURNTABLE_MAX_DEG, target_deg))
    steps = turntable_deg_to_steps(target_deg + home_offset_deg)
    robot.step_move(TURNTABLE_STEPPER, steps, StepMoveType.ABSOLUTE)


def _home_turntable(robot: Robot) -> float:
    """Home turntable CCW against LIM1 (stow position). Returns the home_offset_deg to use
    for all subsequent _turntable_to_deg() calls.

    Requires PIN_ST2_LIMIT = PIN_LIM1 in firmware/arduino/src/config.h.
    After step_home(), firmware step counter 0 = LIM1 trigger point = stow (180°).
    The returned offset (-180.0) maps our angle convention onto that firmware frame.
    """
    print("[HOME] Homing turntable CCW to stow (LIM1)...")
    success = robot.step_home(
        TURNTABLE_STEPPER,
        direction=-1,        # CCW = toward stow/LIM1
        home_velocity=300,   # slow for safe homing
        backoff_steps=50,    # back off ~2.8° after trigger
        timeout=15.0,
    )
    if success:
        print("[HOME] Turntable homed. Stow = firmware step 0.")
    else:
        print("[HOME] WARNING: turntable homing timed out — LIM1 may not be wired. "
              "Continuing with manual alignment (offset=0.0).")
        return 0.0
    # firmware 0 = stow = our 180°  →  offset = -180° so _turntable_to_deg(180°) → step 0
    return -TURNTABLE_MAX_DEG


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


def _detection_dist_mm(det: dict, img_w: int) -> float:
    """Pinhole distance estimate from camera to marshmallow centre (mm)."""
    bbox = det["bbox"]
    px_diam = math.sqrt(max(1.0, bbox["width"] * bbox["height"]))
    focal_px = (img_w / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG / 2.0))
    return (MARSHMALLOW_DIAMETER_MM * focal_px) / px_diam


def _cup_dist_mm(det: dict, img_w: int) -> float:
    """Pinhole distance from camera to cup center using cup diameter (mm).

    Uses bbox width (cup's lateral extent) rather than geometric mean, since
    the cup's known physical diameter is its horizontal dimension from front view.
    """
    bbox = det["bbox"]
    px_width = max(1.0, float(bbox["width"]))
    focal_px = (img_w / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG / 2.0))
    return (CUP_DIAMETER_MM * focal_px) / px_width


def _detection_height_mm(det: dict, img_w: int, img_h: int, dist_mm: float) -> float:
    """Estimate marshmallow centre height above robot base plate (mm).

    Uses bounding-box vertical position, camera VFOV (derived from HFOV and
    image aspect ratio), and the known camera mount height.
    """
    bbox = det["bbox"]
    cy = bbox["y"] + bbox["height"] / 2.0
    hfov_rad = math.radians(CAMERA_HFOV_DEG)
    vfov_rad = 2.0 * math.atan(math.tan(hfov_rad / 2.0) * img_h / img_w)
    elevation_deg = -((cy - img_h / 2.0) / img_h) * math.degrees(vfov_rad)
    h = CAMERA_HEIGHT_MM + dist_mm * math.tan(math.radians(elevation_deg))
    return max(0.0, min(500.0, h))  # clamp to physically plausible range


def _get_ultrasonic_mm(robot: Robot) -> float:
    """Return ultrasonic range measurement in mm, or raise RuntimeError if no reading."""
    value = robot.get_ultrasonic_mm()
    if value is None:
        raise RuntimeError("No ultrasonic reading available — check sensor and enable_ultrasonic() call.")
    return value


def _restow(robot: Robot, shoulder_pos: float, elbow_pos: float, gripper_pos: float,
            home_offset_deg: float = TURNTABLE_HOME_OFFSET_DEG) -> None:
    print("[DEMO] Restowing arm.")
    elbow_pos    = _move_servo(robot, ELBOW_CHANNEL,    elbow_pos,    ELBOW_STOW_DEG,    ELBOW_SAFE_MIN,    ELBOW_SAFE_MAX)
    shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, SHOULDER_STOW_DEG, SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
    _campan_to_deg(robot, 0.0)           # center camera before turntable swings to stow
    _turntable_to_deg(robot, TURNTABLE_MAX_DEG, home_offset_deg)
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

    arm_turntable_deg:   float = 0.0
    arm_shoulder_deg:    float = SHOULDER_STOW_DEG
    arm_elbow_deg:       float = ELBOW_STOW_DEG
    mallow_height_est:   float = MARSHMALLOW_HEIGHT_MM  # updated by SCANNING from vision
    mallow_dist_est:     float = 300.0                  # updated by SCANNING from cup pinhole (mm)
    turntable_home_offset: float = TURNTABLE_HOME_OFFSET_DEG  # set by _home_turntable()

    while True:

        # ── INIT ─────────────────────────────────────────────────────────────
        if state == "INIT":
            current = robot.get_state()
            if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.enable_vision()
            robot.enable_ultrasonic()
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
            turntable_home_offset = _home_turntable(robot)
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
        # Cup-first: pan camera through 3 windows, detect red cups + marshmallows.
        # For each cup, look for a mallow within MALLOW_CUP_BEARING_MATCH_DEG —
        # confirmed pair becomes a candidate. Unmatched cups are skipped (tennis
        # ball or false case). Falls back to direct mallow detection if no pairs.
        elif state == "SCANNING":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            if robot.get_button(Button.BTN_2):
                _restow(robot, shoulder_pos, elbow_pos, gripper_pos, turntable_home_offset)
                robot.shutdown()
                return
            if time.monotonic() - state_entry_time > SCAN_TIMEOUT_S:
                print(f"[DEMO] SCANNING — {SCAN_TIMEOUT_S:.0f}s timeout, no target. Restowing.")
                state = "RESTOWING"
                state_entry_time = time.monotonic()
            else:
                img_w, img_h = robot.get_detection_image_size()
                cup_mallow_hits:      list[dict] = []
                fallback_mallow_hits: list[dict] = []

                for pan_deg in CAMPAN_POSITIONS_DEG:
                    _campan_to_deg(robot, pan_deg)
                    time.sleep(CAMPAN_SETTLE_S)
                    cup_dets    = robot.get_detections(CUP_CLASS)
                    mallow_dets = robot.get_detections(MARSHMALLOW_CLASS)

                    for cup in cup_dets:
                        if float(cup["confidence"]) < MIN_CONFIDENCE_CUP:
                            continue
                        cup_pixel_bearing = _detection_bearing_deg(cup, img_w)
                        cup_world_bearing = pan_deg + cup_pixel_bearing
                        if not (-TURNTABLE_SCAN_ARC_DEG <= cup_world_bearing <= TURNTABLE_SCAN_ARC_DEG):
                            continue
                        cup_dist = _cup_dist_mm(cup, img_w)

                        best_m_conf   = 0.0
                        best_m_height = None
                        for mallow in mallow_dets:
                            if float(mallow["confidence"]) < MIN_CONFIDENCE_MARSHMALLOW:
                                continue
                            if abs(_detection_bearing_deg(mallow, img_w) - cup_pixel_bearing) <= MALLOW_CUP_BEARING_MATCH_DEG:
                                m_conf = float(mallow["confidence"])
                                if m_conf > best_m_conf:
                                    best_m_conf   = m_conf
                                    best_m_height = snap_to_cup_tier_mm(
                                        _detection_height_mm(mallow, img_w, img_h, cup_dist)
                                    )

                        if best_m_height is not None:
                            cup_mallow_hits.append({
                                "bearing_deg": cup_world_bearing,
                                "dist_mm":     cup_dist,
                                "height_mm":   best_m_height,
                                "conf":        best_m_conf,
                            })

                    for mallow in mallow_dets:
                        if float(mallow["confidence"]) < MIN_CONFIDENCE_MARSHMALLOW:
                            continue
                        m_pixel_bearing = _detection_bearing_deg(mallow, img_w)
                        m_world_bearing = pan_deg + m_pixel_bearing
                        if not (-TURNTABLE_SCAN_ARC_DEG <= m_world_bearing <= TURNTABLE_SCAN_ARC_DEG):
                            continue
                        m_dist   = _detection_dist_mm(mallow, img_w)
                        m_height = snap_to_cup_tier_mm(_detection_height_mm(mallow, img_w, img_h, m_dist))
                        fallback_mallow_hits.append({
                            "bearing_deg": m_world_bearing,
                            "dist_mm":     m_dist,
                            "height_mm":   m_height,
                            "conf":        float(mallow["confidence"]),
                        })

                _campan_to_deg(robot, 0.0)

                hits   = cup_mallow_hits if cup_mallow_hits else fallback_mallow_hits
                source = "cup+mallow" if cup_mallow_hits else "mallow-only"
                if hits:
                    best = max(hits, key=lambda h: h["conf"])
                    arm_turntable_deg = best["bearing_deg"]
                    mallow_dist_est   = best["dist_mm"]
                    mallow_height_est = best["height_mm"]
                    print(
                        f"[DEMO] SCANNING ({source}) — target at {arm_turntable_deg:.1f}°  "
                        f"dist={mallow_dist_est:.0f}mm  height={mallow_height_est:.0f}mm  "
                        f"conf={best['conf']:.2f}"
                    )
                    state = "RANGING"
                    state_entry_time = time.monotonic()

        # ── RANGING ───────────────────────────────────────────────────────────
        # Rotate turntable to bearing. Arm stays in search pose — no movement.
        # US reading is FK-corrected for the forearm's elevation angle to get
        # accurate horizontal reach. mallow_height_est (snapped tier) is always
        # used for z since it is more reliable than the US vertical component.
        # Falls back to camera distance if US is unavailable or out of range.
        elif state == "RANGING":
            if robot.get_button(Button.BTN_2):
                _restow(robot, shoulder_pos, elbow_pos, gripper_pos, turntable_home_offset)
                robot.shutdown()
                return

            print(f"[DEMO] RANGING — turntable to {arm_turntable_deg:.1f}°, "
                  f"camera dist={mallow_dist_est:.0f}mm height={mallow_height_est:.0f}mm")
            _turntable_to_deg(robot, arm_turntable_deg, turntable_home_offset)
            time.sleep(0.2)

            # Camera-based estimate is the default; US overwrites x/y if plausible.
            bearing_rad = math.radians(arm_turntable_deg)
            mallow_x = ARM_GEOMETRY.camera_forward_offset_mm + mallow_dist_est * math.cos(bearing_rad)
            mallow_y = mallow_dist_est * math.sin(bearing_rad)
            mallow_z = mallow_height_est

            try:
                d_raw = _get_ultrasonic_mm(robot)
                if 20.0 < d_raw < 800.0:
                    # Convert current servo angles to forearm elevation angle.
                    sh_geo      = ARM_GEOMETRY.shoulder_servo_to_geo(shoulder_pos)
                    el_geo      = ARM_GEOMETRY.elbow_servo_to_geo(elbow_pos)
                    sh_rad      = math.radians(sh_geo)
                    el_rad      = math.radians(el_geo)
                    forearm_rad = sh_rad + (math.pi - el_rad)

                    # Horizontal position of the US sensor from the turntable axis.
                    elbow_horiz  = ARM_GEOMETRY.L1 * math.cos(sh_rad)
                    sensor_horiz = (ARM_GEOMETRY.shoulder_offset_mm + elbow_horiz
                                    + ULTRASONIC_FOREARM_OFFSET_MM * math.cos(forearm_rad))

                    # Project US reading onto horizontal axis.
                    mallow_reach = sensor_horiz + d_raw * math.cos(forearm_rad)
                    mallow_x = mallow_reach * math.cos(bearing_rad)
                    mallow_y = mallow_reach * math.sin(bearing_rad)
                    print(f"[DEMO] RANGING — US={d_raw:.0f}mm forearm={math.degrees(forearm_rad):.1f}° "
                          f"→ reach={mallow_reach:.0f}mm")
                else:
                    print(f"[DEMO] RANGING — US={d_raw:.0f}mm out of plausible range, using camera estimate.")
            except RuntimeError as exc:
                print(f"[DEMO] RANGING — US unavailable ({exc}), using camera estimate.")

            try:
                _, sh_srv, el_srv = inverse_kinematics(mallow_x, mallow_y, mallow_z, ARM_GEOMETRY)
            except OutOfReachError as e:
                print(f"[DEMO] RANGING — IK out of reach: {e}. Restowing.")
                state = "RESTOWING"
                state_entry_time = time.monotonic()
            else:
                arm_shoulder_deg = sh_srv
                arm_elbow_deg    = el_srv
                print(f"[DEMO] RANGING — target=({mallow_x:.0f},{mallow_y:.0f},{mallow_z:.0f}) → "
                      f"shoulder={sh_srv:.1f}° elbow={el_srv:.1f}°")
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
                _turntable_to_deg(robot, plate_t_deg, turntable_home_offset)
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
            _restow(robot, shoulder_pos, elbow_pos, gripper_pos, turntable_home_offset)
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
