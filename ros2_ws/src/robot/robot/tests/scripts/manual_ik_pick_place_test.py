"""
manual_ik_pick_place_test.py
============================
Manual IK verification test: provide the marshmallow position directly (no
camera scan, no sensor input) and execute the full pick-and-place task.

Use this to verify that inverse_kinematics() can grab a mallow at a known
position and place it on the plate, without any vision or sensing involved.

HOW TO MEASURE THE THREE INPUTS
---------------------------------
  MALLOW_BEARING_DEG
      Horizontal angle of the mallow in robot frame.
      0° = straight forward, positive = left, negative = right.
      Keep at 0.0 while the turntable is at the forward (0°) position.

  MALLOW_DIST_MM
      Horizontal distance from the camera/campan lens to the center of the
      marshmallow (mm). Measure with a ruler from the front of the camera
      housing to the mallow center.

      NOTE: this is distance from the camera axis, not from the turntable
      axis. The camera sits ~209 mm ahead of the turntable axis. If you
      measure from the turntable axis instead, subtract CAMERA_FORWARD_OFFSET_MM
      and set DIST_IS_FROM_TURNTABLE_AXIS = True below.

  MALLOW_HEIGHT_MM
      Height of the marshmallow center above the robot base plate (mm).
      The base plate is z = 0. The competition floor is z ≈ -229 mm.
      Typical values:
        mallow on floor alone:  ≈ -204 mm  (floor + marshmallow radius 12.5 mm)
        mallow on 1 Solo cup:   ≈ -116 mm
        mallow on 2 Solo cups:  ≈  -22 mm
        mallow on 3 Solo cups:  ≈  +72 mm

COORDINATE MATH (for verification)
-------------------------------------
  Robot frame: origin at turntable axis, +x forward, +y left, +z up.

  Bearing → robot frame:
    bearing_rad = radians(MALLOW_BEARING_DEG)
    x = CAMERA_FORWARD_OFFSET_MM + MALLOW_DIST_MM * cos(bearing_rad)
    y = MALLOW_DIST_MM * sin(bearing_rad)
    z = MALLOW_HEIGHT_MM

  IK step 1 — turntable azimuth:
    turntable_deg = atan2(y, x)
    (for bearing=0 and y=0 this is 0° — straight forward)

  IK step 2 — arm plane coordinates:
    reach  = hypot(x, y) - shoulder_offset_mm    (≈14 mm forward offset)
    height = z - shoulder_height_mm              (≈95 mm shoulder height)
    d      = sqrt(reach² + height²)              (straight-line dist from shoulder to target)

  IK step 3 — 2-link planar IK (law of cosines):
    cos_elbow_geo = (d² - L1² - L2²) / (2·L1·L2)   L1=156mm, L2=315mm
    elbow_geo     = 180° - acos(cos_elbow_geo)       180=straight, 0=folded
    alpha = atan2(height, reach)                      angle to target from horizontal
    beta  = atan2(L2·sin(elbow_geo), L1-L2·cos(elbow_geo))  elbow contribution
    shoulder_geo  = alpha - beta                      degrees above horizontal

  IK step 4 — servo conversion:
    shoulder_servo = shoulder_servo_offset + shoulder_servo_sign * shoulder_geo
    elbow_servo    = elbow_servo_offset   + elbow_servo_sign   * (elbow_geo - 180°)

State machine
-------------
  INIT → IDLE → SAFE_RAISE → ARM_HOME → IK_COMPUTE
       → APPROACHING → PICKING → CARRY_TO_PLACE → PLACING → RESTOW → DONE
"""
from __future__ import annotations

import math
import time

from robot.arm_kinematics import OutOfReachError, inverse_kinematics
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Limit, StepMoveType
from robot.robot import FirmwareState, Robot
from robot.tests.scripts._manipulator_config import (
    ARM_GEOMETRY,
    ARM_SERVO_STEP_DEG,
    ARM_SERVO_STEP_DWELL,
    CAMPAN_ACCELERATION,
    CAMPAN_MAX_VELOCITY,
    CAMPAN_STEPPER,
    CAMERA_FORWARD_OFFSET_MM,
    ELBOW_CHANNEL,
    ELBOW_SAFE_MAX,
    ELBOW_SAFE_MIN,
    ELBOW_STOW_DEG,
    GRIPPER_CHANNEL,
    GRIPPER_CLOSE_DEG,
    GRIPPER_GRAB_DEG,
    GRIPPER_OPEN_DEG,
    PLATE_Z_MM,
    SHOULDER_CHANNEL,
    SHOULDER_SAFE_MAX,
    SHOULDER_SAFE_MIN,
    SHOULDER_STOW_DEG,
    TURNTABLE_ACCELERATION,
    TURNTABLE_HOME_OFFSET_DEG,
    TURNTABLE_MAX_DEG,
    TURNTABLE_MAX_VELOCITY,
    TURNTABLE_MIN_DEG,
    TURNTABLE_STEPPER,
    campan_deg_to_steps,
    turntable_deg_to_steps,
)

# ┌─────────────────────────────────────────────────────────────────────────────┐
# │  EDIT THESE TO MATCH YOUR MEASURED SETUP                                   │
# └─────────────────────────────────────────────────────────────────────────────┘

# Bearing of the marshmallow in robot frame.
# 0° = straight forward.  Keep at 0.0 for a head-on test.
MALLOW_BEARING_DEG: float = 0.0

# Horizontal distance from the camera lens to the marshmallow center (mm).
# Measure with a ruler from the camera housing face to the mallow center.
# 7.628 in × 25.4 = 193.8 mm
MALLOW_DIST_MM: float = 193.8

# Height of the marshmallow center in robot frame (mm).
# Base plate = 0 mm.  Floor measured at -244.2 mm below base plate.
# Stack: 2.33 mm cardboard + 6 Solo cups (121 + 5×5.3 = 147.5 mm) + mallow half-height 13 mm
# Mallow center = -244.2 + 2.33 + 147.5 + 13 = -81.37 mm
MALLOW_HEIGHT_MM: float = -81.37

# Set True if you measured MALLOW_DIST_MM from the turntable axis instead of
# from the camera. In that case the camera forward offset is NOT added.
DIST_IS_FROM_TURNTABLE_AXIS: bool = False

# ── Place target (same defaults as turntable_pick_place_test.py) ──────────────
PLACE_TURNTABLE_DEG: float = -45.0        # bearing to place at
PLACE_REACH_MM:      float = CAMERA_FORWARD_OFFSET_MM   # reach from turntable axis
PLACE_Z_MM:          float = PLATE_Z_MM                 # z of plate surface

# ── Safe carry pose ───────────────────────────────────────────────────────────
SAFE_SHOULDER_DEG = 105.0
SAFE_ELBOW_DEG    = 90.0

# ── Turntable homing ──────────────────────────────────────────────────────────
HOME_NUDGE_DEG = 5.0
HOME_TIMEOUT_S = 30.0
HOME_BACKOFF   = 200
# Limit switch fires before the full mechanical stow position. Measured on
# this robot: LIM1 trips at ~171° instead of 180°, so commanding 0° using a
# home_offset of 180° overshoots forward by ~9°. Use the measured value
# (matches TURNTABLE_HOME_OFFSET_DEG in _manipulator_config.py).
TURNTABLE_HOME_OFFSET_MEASURED_DEG = TURNTABLE_HOME_OFFSET_DEG


# ── Helpers (mirrors turntable_pick_place_test.py exactly) ────────────────────

def _move_servo(
    robot: Robot,
    channel: int,
    current: float,
    target: float,
    safe_min: float = 10.0,
    safe_max: float = 170.0,
) -> float:
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


def _safe_arm_retract(
    robot: Robot,
    shoulder_pos: float,
    elbow_pos: float,
    gripper_pos: float,
) -> tuple[float, float, float]:
    robot.enable_servo(SHOULDER_CHANNEL)
    robot.enable_servo(ELBOW_CHANNEL)
    robot.enable_servo(GRIPPER_CHANNEL)
    gripper_pos  = _move_servo(robot, GRIPPER_CHANNEL,  gripper_pos,  GRIPPER_CLOSE_DEG)
    shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, SAFE_SHOULDER_DEG,
                               SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
    elbow_pos    = _move_servo(robot, ELBOW_CHANNEL,    elbow_pos,    SAFE_ELBOW_DEG,
                               ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
    return shoulder_pos, elbow_pos, gripper_pos


def _turntable_to_deg(robot: Robot, target_deg: float, home_offset: float) -> None:
    target_deg = max(TURNTABLE_MIN_DEG, min(TURNTABLE_MAX_DEG, target_deg))
    delta_deg  = abs(home_offset - target_deg)
    steps      = turntable_deg_to_steps(home_offset - target_deg)
    # Fire-and-forget — _wait_stepper_idle consistently misses the brief
    # IDLE→active→IDLE transition on the turntable so blocking just stalls the
    # FSM for the full timeout while the move itself completes correctly.
    # Sleep for the worst-case travel time at TURNTABLE_MAX_VELOCITY plus a
    # safety margin to cover the trapezoidal velocity profile and settling.
    robot.step_move(TURNTABLE_STEPPER, steps, StepMoveType.ABSOLUTE, blocking=False)
    deg_per_sec = TURNTABLE_MAX_VELOCITY * (360.0 / 3200.0)  # 2800 steps/s ≈ 315°/s
    travel_s    = delta_deg / deg_per_sec + 0.5
    time.sleep(travel_s)


def _home_turntable(robot: Robot) -> float:
    """Nudge CW to clear LIM1, then home CCW.  Returns home_offset_deg."""
    lim1_state = "TRIGGERED" if robot.get_limit(Limit.LIM_1) else "open"
    print(f"[HOME] LIM1 at startup: {lim1_state}")

    nudge_steps = turntable_deg_to_steps(HOME_NUDGE_DEG)
    print(f"[HOME] Nudging {nudge_steps} steps CW to clear switch...")
    ok = robot.step_move(TURNTABLE_STEPPER, nudge_steps, StepMoveType.RELATIVE, timeout=5.0)
    if not ok:
        print("[HOME] WARNING: CW nudge timed out — proceeding anyway.")
    time.sleep(0.3)

    lim1_state = "TRIGGERED" if robot.get_limit(Limit.LIM_1) else "open"
    print(f"[HOME] LIM1 after nudge: {lim1_state}")
    print(f"[HOME] Homing CCW to LIM1 (timeout={HOME_TIMEOUT_S:.0f}s)...")

    ok = robot.step_home(
        TURNTABLE_STEPPER,
        direction=-1,
        home_velocity=2000,
        backoff_steps=HOME_BACKOFF,
        timeout=HOME_TIMEOUT_S,
    )

    lim1_state = "TRIGGERED" if robot.get_limit(Limit.LIM_1) else "open"
    print(f"[HOME] LIM1 after home attempt: {lim1_state}")

    if ok:
        print(f"[HOME] Turntable homed.  Firmware step 0 = stow "
              f"({TURNTABLE_HOME_OFFSET_MEASURED_DEG:.1f}°, measured).")
        return TURNTABLE_HOME_OFFSET_MEASURED_DEG
    else:
        print("[HOME] WARNING: homing timed out — LIM1 may not be wired. "
              "Falling back to manual alignment (offset=0°).")
        return 0.0


def _campan_to_deg(robot: Robot, target_deg: float) -> None:
    # Fire-and-forget — see comment in _turntable_to_deg. Campan range is small
    # so a fixed 1 s settle covers the worst-case sweep.
    robot.step_move(CAMPAN_STEPPER, campan_deg_to_steps(target_deg),
                    StepMoveType.ABSOLUTE, blocking=False)
    time.sleep(1.0)


def _bearing_dist_to_robot_frame(
    bearing_deg: float,
    dist_mm: float,
    height_mm: float,
) -> tuple[float, float, float]:
    """
    Convert (bearing, distance, height) to robot-frame (x, y, z) for IK.

    bearing_deg: horizontal angle in robot frame (0=forward, + =left, - =right)
    dist_mm:     horizontal distance — from camera if DIST_IS_FROM_TURNTABLE_AXIS=False,
                 from turntable axis if True
    height_mm:   z in robot frame (base plate = 0)

    Returns (x_mm, y_mm, z_mm) with turntable axis as origin, ready for IK.
    """
    bearing_rad = math.radians(bearing_deg)
    if DIST_IS_FROM_TURNTABLE_AXIS:
        x_mm = dist_mm * math.cos(bearing_rad)
        y_mm = dist_mm * math.sin(bearing_rad)
    else:
        # Camera axis sits CAMERA_FORWARD_OFFSET_MM ahead of the turntable axis.
        # A distance measured from the camera must be offset by this amount.
        x_mm = CAMERA_FORWARD_OFFSET_MM + dist_mm * math.cos(bearing_rad)
        y_mm = dist_mm * math.sin(bearing_rad)
    return x_mm, y_mm, height_mm


# ── Main FSM ──────────────────────────────────────────────────────────────────

def run(robot: Robot) -> None:  # noqa: C901
    state       = "INIT"
    period      = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick   = time.monotonic()
    state_entry = time.monotonic()

    shoulder_pos: float = SHOULDER_STOW_DEG
    elbow_pos:    float = ELBOW_STOW_DEG
    gripper_pos:  float = GRIPPER_CLOSE_DEG

    arm_shoulder_deg: float = SHOULDER_STOW_DEG
    arm_elbow_deg:    float = ELBOW_STOW_DEG
    turntable_home_offset: float = TURNTABLE_HOME_OFFSET_DEG

    # Pre-compute pick coordinates from manual inputs.
    pick_x, pick_y, pick_z = _bearing_dist_to_robot_frame(
        MALLOW_BEARING_DEG, MALLOW_DIST_MM, MALLOW_HEIGHT_MM
    )

    # Pre-compute place coordinates.
    place_rad = math.radians(PLACE_TURNTABLE_DEG)
    place_x   = PLACE_REACH_MM * math.cos(place_rad)
    place_y   = PLACE_REACH_MM * math.sin(place_rad)
    place_z   = PLACE_Z_MM

    print(f"[TEST] ── Manual IK Pick-Place ─────────────────────────────────")
    print(f"[TEST] Mallow input: bearing={MALLOW_BEARING_DEG:.1f}°  "
          f"dist={MALLOW_DIST_MM:.0f} mm  height={MALLOW_HEIGHT_MM:.0f} mm")
    dist_ref = "turntable axis" if DIST_IS_FROM_TURNTABLE_AXIS else "camera lens"
    print(f"[TEST]   (distance measured from {dist_ref})")
    print(f"[TEST] Pick robot-frame:  x={pick_x:.0f}  y={pick_y:.0f}  z={pick_z:.0f} mm")
    print(f"[TEST] Place robot-frame: x={place_x:.0f}  y={place_y:.0f}  z={place_z:.0f} mm")
    print(f"[TEST] Shoulder height={ARM_GEOMETRY.shoulder_height_mm:.0f} mm  "
          f"L1={ARM_GEOMETRY.L1:.0f} mm  L2={ARM_GEOMETRY.L2:.0f} mm")
    print(f"[TEST] ───────────────────────────────────────────────────────────")

    while True:

        # ── INIT ─────────────────────────────────────────────────────────────
        if state == "INIT":
            if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 255)
            state = "IDLE"
            state_entry = time.monotonic()

        # ── IDLE ─────────────────────────────────────────────────────────────
        elif state == "IDLE":
            if robot.get_button(Button.BTN_2):
                robot.shutdown()
                return
            for n in (3, 2, 1):
                print(f"[TEST] Starting in {n}...")
                time.sleep(1.0)
            print("[TEST] GO")
            state = "SAFE_RAISE"
            state_entry = time.monotonic()

        # ── SAFE_RAISE ───────────────────────────────────────────────────────
        elif state == "SAFE_RAISE":
            print("[TEST] SAFE_RAISE — clearing arm before any motor motion.")
            robot.enable_servo(SHOULDER_CHANNEL)
            robot.enable_servo(ELBOW_CHANNEL)
            robot.enable_servo(GRIPPER_CHANNEL)
            time.sleep(0.2)

            shoulder_pos = _move_servo(
                robot, SHOULDER_CHANNEL, shoulder_pos, SAFE_SHOULDER_DEG,
                SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX,
            )
            elbow_pos = _move_servo(
                robot, ELBOW_CHANNEL, elbow_pos, SAFE_ELBOW_DEG,
                ELBOW_SAFE_MIN, ELBOW_SAFE_MAX,
            )
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_OPEN_DEG)
            print(f"[TEST] SAFE_RAISE — shoulder={shoulder_pos:.1f}°  "
                  f"elbow={elbow_pos:.1f}°  gripper={gripper_pos:.1f}°")
            state = "ARM_HOME"
            state_entry = time.monotonic()

        # ── ARM_HOME ──────────────────────────────────────────────────────────
        elif state == "ARM_HOME":
            print("[TEST] ARM_HOME — enabling steppers, homing turntable.")
            robot.step_set_config(TURNTABLE_STEPPER, TURNTABLE_MAX_VELOCITY, TURNTABLE_ACCELERATION)
            robot.step_enable(TURNTABLE_STEPPER)
            robot.step_set_config(CAMPAN_STEPPER, CAMPAN_MAX_VELOCITY, CAMPAN_ACCELERATION)
            robot.step_enable(CAMPAN_STEPPER)

            turntable_home_offset = _home_turntable(robot)
            print(f"[TEST] ARM_HOME — home_offset={turntable_home_offset:.1f}°")

            print("[TEST] ARM_HOME — centering campan and moving turntable to forward (0°).")
            _campan_to_deg(robot, 0.0)
            _turntable_to_deg(robot, 0.0, turntable_home_offset)
            time.sleep(0.5)

            state = "IK_COMPUTE"
            state_entry = time.monotonic()

        # ── IK_COMPUTE ────────────────────────────────────────────────────────
        # Compute pick IK from manual inputs, rotate turntable to pick bearing.
        elif state == "IK_COMPUTE":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print(f"[TEST] IK_COMPUTE — rotating turntable to {MALLOW_BEARING_DEG:.1f}°")
            shoulder_pos, elbow_pos, gripper_pos = _safe_arm_retract(
                robot, shoulder_pos, elbow_pos, gripper_pos
            )
            _turntable_to_deg(robot, MALLOW_BEARING_DEG, turntable_home_offset)
            time.sleep(0.3)

            print(f"[TEST] IK_COMPUTE — target robot-frame: "
                  f"x={pick_x:.0f}  y={pick_y:.0f}  z={pick_z:.0f} mm")

            # ── Inline IK walkthrough (same math as inverse_kinematics()) ───
            # Step 1: turntable azimuth
            ik_turntable_deg = math.degrees(math.atan2(pick_y, pick_x))

            # Step 2: effective reach and height from shoulder pivot
            horizontal_dist = math.hypot(pick_x, pick_y)
            reach  = horizontal_dist - ARM_GEOMETRY.shoulder_offset_mm
            height = pick_z - ARM_GEOMETRY.shoulder_height_mm
            d      = math.sqrt(reach ** 2 + height ** 2)

            print(f"[TEST] IK_COMPUTE — ik_turntable={ik_turntable_deg:.1f}°  "
                  f"reach={reach:.0f} mm  height_from_shoulder={height:.0f} mm  "
                  f"d={d:.0f} mm  (max={ARM_GEOMETRY.L1+ARM_GEOMETRY.L2:.0f})")

            try:
                _, pick_sh, pick_el = inverse_kinematics(pick_x, pick_y, pick_z, ARM_GEOMETRY)
            except OutOfReachError as exc:
                print(f"[TEST] IK_COMPUTE — UNREACHABLE: {exc}")
                print("[TEST] IK_COMPUTE — check MALLOW_DIST_MM / MALLOW_HEIGHT_MM "
                      "and arm geometry constants.")
                state = "RESTOW"
                state_entry = time.monotonic()
            else:
                arm_shoulder_deg = pick_sh
                arm_elbow_deg    = pick_el
                print(f"[TEST] IK_COMPUTE — pick IK: "
                      f"shoulder_servo={pick_sh:.1f}°  elbow_servo={pick_el:.1f}°")
                state = "APPROACHING"
                state_entry = time.monotonic()

        # ── APPROACHING ───────────────────────────────────────────────────────
        elif state == "APPROACHING":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print(f"[TEST] APPROACHING — shoulder={arm_shoulder_deg:.1f}°  "
                  f"elbow={arm_elbow_deg:.1f}°")
            gripper_pos  = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_OPEN_DEG)
            shoulder_pos = _move_servo(
                robot, SHOULDER_CHANNEL, shoulder_pos, arm_shoulder_deg,
                SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX,
            )
            elbow_pos = _move_servo(
                robot, ELBOW_CHANNEL, elbow_pos, arm_elbow_deg,
                ELBOW_SAFE_MIN, ELBOW_SAFE_MAX,
            )
            time.sleep(0.5)
            print("[TEST] APPROACHING — at pick position.")
            state = "PICKING"
            state_entry = time.monotonic()

        # ── PICKING ───────────────────────────────────────────────────────────
        elif state == "PICKING":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print(f"[TEST] PICKING — closing gripper to {GRIPPER_GRAB_DEG:.0f}°")
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_GRAB_DEG)
            time.sleep(0.4)
            print("[TEST] PICKING — gripped.")
            state = "CARRY_TO_PLACE"
            state_entry = time.monotonic()

        # ── CARRY_TO_PLACE ────────────────────────────────────────────────────
        elif state == "CARRY_TO_PLACE":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print(f"[TEST] CARRY_TO_PLACE — retracting and rotating to "
                  f"place bearing ({PLACE_TURNTABLE_DEG:.1f}°)")
            shoulder_pos, elbow_pos, gripper_pos = _safe_arm_retract(
                robot, shoulder_pos, elbow_pos, gripper_pos
            )
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_GRAB_DEG)
            _turntable_to_deg(robot, PLACE_TURNTABLE_DEG, turntable_home_offset)

            print(f"[TEST] CARRY_TO_PLACE — place robot-frame: "
                  f"x={place_x:.0f}  y={place_y:.0f}  z={place_z:.0f} mm")
            try:
                _, place_sh, place_el = inverse_kinematics(place_x, place_y, place_z, ARM_GEOMETRY)
            except OutOfReachError as exc:
                print(f"[TEST] CARRY_TO_PLACE — place IK failed: {exc}")
                state = "RESTOW"
                state_entry = time.monotonic()
            else:
                arm_shoulder_deg = place_sh
                arm_elbow_deg    = place_el
                print(f"[TEST] CARRY_TO_PLACE — place IK: "
                      f"shoulder_servo={place_sh:.1f}°  elbow_servo={place_el:.1f}°")
                state = "PLACING"
                state_entry = time.monotonic()

        # ── PLACING ───────────────────────────────────────────────────────────
        elif state == "PLACING":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print(f"[TEST] PLACING — extending to place position (z={place_z:.0f} mm)")
            shoulder_pos = _move_servo(
                robot, SHOULDER_CHANNEL, shoulder_pos, arm_shoulder_deg,
                SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX,
            )
            elbow_pos = _move_servo(
                robot, ELBOW_CHANNEL, elbow_pos, arm_elbow_deg,
                ELBOW_SAFE_MIN, ELBOW_SAFE_MAX,
            )
            time.sleep(0.4)
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_OPEN_DEG)
            time.sleep(0.3)
            print("[TEST] PLACING — released.  PASS")
            state = "RESTOW"
            state_entry = time.monotonic()

        # ── RESTOW ────────────────────────────────────────────────────────────
        elif state == "RESTOW":
            print("[TEST] RESTOW — retracting arm and returning turntable to forward.")
            shoulder_pos, elbow_pos, gripper_pos = _safe_arm_retract(
                robot, shoulder_pos, elbow_pos, gripper_pos
            )
            _turntable_to_deg(robot, 0.0, turntable_home_offset)
            time.sleep(0.5)
            _campan_to_deg(robot, 0.0)
            time.sleep(0.3)
            robot.step_disable(TURNTABLE_STEPPER)
            robot.step_disable(CAMPAN_STEPPER)
            print("[TEST] RESTOW — done.")
            state = "DONE"
            state_entry = time.monotonic()

        # ── DONE ──────────────────────────────────────────────────────────────
        elif state == "DONE":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            robot.stop()

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
