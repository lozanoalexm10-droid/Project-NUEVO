"""
turntable_pick_place_test.py
============================
Scan robot-left side → detect red-cup/marshmallow stack → pick → place at
-45° turntable, z ≈ base-plate level.

Hardware constraint (temporary)
--------------------------------
The turntable currently cannot travel below -5° (CW direction is cable-limited).
  TURNTABLE_HW_LIMIT_DEG = -5.0
Bearings below this limit are rejected during scanning.
Place angle is clamped to this limit with a logged warning until the hardware
is fixed.

Scan strategy
-------------
Camera pan is restricted to the robot-left arc (left and centre positions only).
The competition cup+mallow stack is expected to the LEFT of the robot's forward
direction (+y side, positive turntable angle).  The right campan position (+60°)
is skipped.

Safe-arm convention (applied BEFORE every turntable move)
----------------------------------------------------------
1. Elbow  → SAFE_ELBOW_DEG  (100°) first — swings forearm clear of chassis
2. Shoulder → SAFE_SHOULDER_DEG (70°) — lifts upper arm clear

This order is mandatory.  Reversing it risks the forearm striking the chassis.

Constants requiring physical calibration
-----------------------------------------
  ARM_SHOULDER_HEIGHT_MM  = 95.5 mm  (measured)
  GROUND_Z_MM             = -229.0 mm (floor is 229 mm below base plate — measured)
  PLATE_Z_MM is now derived as GROUND_Z_MM + 15 = -214 mm.
  Cup tier heights are all negative (floor-relative in robot frame).

  SAFE_ELBOW_DEG / SAFE_SHOULDER_DEG
      Tune if the arm still contacts the chassis or overshoots in the opposite
      direction.

  PLACE_REACH_MM
      Distance from the turntable axis to the intended drop point, measured
      along the arm's horizontal plane.  Currently inherits PLATE_X_MM
      (177.8 mm).  Adjust to match your physical placement target.

State machine
-------------
  IDLE → SAFE_RAISE → ARM_HOME → SCANNING → RANGING
       → APPROACHING → PICKING → CARRY_TO_PLACE → PLACING → DONE

Nodes required:  bridge (auto) + vision + robot
Launch:          robot.launch.py  (with vision node running separately)
"""
from __future__ import annotations

import math
import time

from robot.arm_kinematics import OutOfReachError, inverse_kinematics
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, StepMoveType
from robot.robot import FirmwareState, Robot
from robot.tests.scripts._manipulator_config import (
    ARM_GEOMETRY,
    ARM_SERVO_STEP_DEG,
    ARM_SERVO_STEP_DWELL,
    CAMPAN_ACCELERATION,
    CAMPAN_MAX_VELOCITY,
    CAMPAN_SETTLE_S,
    CAMPAN_STEPPER,
    campan_deg_to_steps,
    CAMERA_HEIGHT_MM,
    CAMERA_HFOV_DEG,
    CUP_CLASS,
    CUP_DIAMETER_MM,
    CAMERA_FORWARD_OFFSET_MM,
    ELBOW_CHANNEL,
    ELBOW_SAFE_MAX,
    ELBOW_SAFE_MIN,
    GRIPPER_CHANNEL,
    GRIPPER_CLOSE_DEG,
    GRIPPER_GRAB_DEG,
    GRIPPER_OPEN_DEG,
    GROUND_Z_MM,
    MALLOW_CUP_BEARING_MATCH_DEG,
    MARSHMALLOW_CLASS,
    MARSHMALLOW_DIAMETER_MM,
    MIN_CONFIDENCE_CUP,
    MIN_CONFIDENCE_MARSHMALLOW,
    PLATE_Z_MM,
    SCAN_TIMEOUT_S,
    SHOULDER_CHANNEL,
    SHOULDER_SAFE_MAX,
    SHOULDER_SAFE_MIN,
    SHOULDER_STOW_DEG,
    ELBOW_STOW_DEG,
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


# ── Test-specific constants ───────────────────────────────────────────────────

# Turntable hardware floor for this test (cable-limited, plan to fix)
TURNTABLE_HW_LIMIT_DEG = -5.0

# Safe arm positions used before EVERY turntable move.
# Shoulder moves FIRST to 150° to lift the upper arm clear of hardware,
# then elbow moves to 90°.  Order is mandatory — reversing it caused hardware contact.
SAFE_SHOULDER_DEG = 150.0   # upper arm lifted clear of hardware first
SAFE_ELBOW_DEG    = 90.0    # forearm swung clear after shoulder is up

# Full-arc campan sweep: start at left, work through center into negative (right).
# Ordered 60° → 0° → -60° so detection at 0° (primary bearing) is hit early
# and the sweep continues into negative degrees where the hardware-limited
# place target lives.
SCAN_CAMPAN_DEG = [60.0, 30.0, 0.0, -30.0, -60.0]

# Place target — -45° is the intended final bearing once hardware is fixed.
# For now it is clamped to TURNTABLE_HW_LIMIT_DEG (-5°) at runtime.
PLACE_TURNTABLE_DEG_TARGET = -45.0
# Reach measured from the camera position (CAMERA_FORWARD_OFFSET_MM = 197.6 mm),
# not the chassis front.  This shifts the drop point ~20 mm further forward vs
# the old PLATE_X_MM (177.8 mm) reference.
PLACE_REACH_MM             = CAMERA_FORWARD_OFFSET_MM   # 197.6 mm — tune to your drop point
# z = PLATE_Z_MM = GROUND_Z_MM + 15 = -214 mm (floor + graham cracker clearance).


# ── Helpers ───────────────────────────────────────────────────────────────────

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
    """Retract arm to safe carry position before any turntable rotation.

    Order is mandatory: shoulder first (lifts arm clear of hardware), then elbow.
    Gripper closes to avoid snagging.
    """
    robot.enable_servo(ELBOW_CHANNEL)
    robot.enable_servo(SHOULDER_CHANNEL)
    robot.enable_servo(GRIPPER_CHANNEL)
    gripper_pos  = _move_servo(robot, GRIPPER_CHANNEL,  gripper_pos,  GRIPPER_CLOSE_DEG)
    shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, SAFE_SHOULDER_DEG,
                               SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
    elbow_pos    = _move_servo(robot, ELBOW_CHANNEL,    elbow_pos,    SAFE_ELBOW_DEG,
                               ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
    return shoulder_pos, elbow_pos, gripper_pos


def _rotate_turntable_safe(
    robot: Robot,
    target_deg: float,
    home_offset_deg: float,
    shoulder_pos: float,
    elbow_pos: float,
    gripper_pos: float,
) -> tuple[float, float, float]:
    """Retract arm to safe angles, then rotate turntable."""
    shoulder_pos, elbow_pos, gripper_pos = _safe_arm_retract(
        robot, shoulder_pos, elbow_pos, gripper_pos
    )
    _turntable_to_deg(robot, target_deg, home_offset_deg)
    return shoulder_pos, elbow_pos, gripper_pos


def _turntable_to_deg(
    robot: Robot,
    target_deg: float,
    home_offset_deg: float = TURNTABLE_HOME_OFFSET_DEG,
) -> None:
    target_deg = max(TURNTABLE_MIN_DEG, min(TURNTABLE_MAX_DEG, target_deg))
    steps = turntable_deg_to_steps(home_offset_deg - target_deg)
    robot.step_move(TURNTABLE_STEPPER, steps, StepMoveType.ABSOLUTE)


def _home_turntable(robot: Robot) -> float:
    print("[HOME] Homing turntable CCW to stow (LIM1)...")
    success = robot.step_home(
        TURNTABLE_STEPPER,
        direction=-1,
        home_velocity=2000,
        backoff_steps=50,
        timeout=25.0,
    )
    if success:
        print("[HOME] Turntable homed. Stow = firmware step 0.")
        return TURNTABLE_MAX_DEG   # -180° offset: maps firmware 0 → stow (180°)
    else:
        print("[HOME] WARNING: homing timed out — LIM1 may not be wired. "
              "Using manual alignment (offset=0.0).")
        return 0.0


def _campan_to_deg(robot: Robot, target_deg: float) -> None:
    steps = campan_deg_to_steps(target_deg)
    robot.step_move(CAMPAN_STEPPER, steps, StepMoveType.ABSOLUTE)


def _detection_bearing_deg(det: dict, img_w: int) -> float:
    bbox = det["bbox"]
    cx = bbox["x"] + bbox["width"] / 2.0
    norm_x = cx / img_w if img_w > 0 else 0.5
    return (norm_x - 0.5) * CAMERA_HFOV_DEG


def _detection_dist_mm(det: dict, img_w: int) -> float:
    bbox = det["bbox"]
    px_diam = math.sqrt(max(1.0, bbox["width"] * bbox["height"]))
    focal_px = (img_w / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG / 2.0))
    return (MARSHMALLOW_DIAMETER_MM * focal_px) / px_diam


def _cup_dist_mm(det: dict, img_w: int) -> float:
    bbox = det["bbox"]
    px_width = max(1.0, float(bbox["width"]))
    focal_px = (img_w / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG / 2.0))
    return (CUP_DIAMETER_MM * focal_px) / px_width


def _detection_height_mm(det: dict, img_w: int, img_h: int, dist_mm: float) -> float:
    bbox = det["bbox"]
    cy = bbox["y"] + bbox["height"] / 2.0
    hfov_rad = math.radians(CAMERA_HFOV_DEG)
    vfov_rad = 2.0 * math.atan(math.tan(hfov_rad / 2.0) * img_h / img_w)
    elevation_deg = -((cy - img_h / 2.0) / img_h) * math.degrees(vfov_rad)
    h = CAMERA_HEIGHT_MM + dist_mm * math.tan(math.radians(elevation_deg))
    return max(0.0, min(500.0, h))



# ── Main FSM ──────────────────────────────────────────────────────────────────

def run(robot: Robot) -> None:  # noqa: C901
    state = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()
    state_entry_time = time.monotonic()

    # Track servo positions for incremental moves
    shoulder_pos: float = SHOULDER_STOW_DEG
    elbow_pos:    float = ELBOW_STOW_DEG
    gripper_pos:  float = GRIPPER_CLOSE_DEG

    # Populated by SCANNING / RANGING
    arm_turntable_deg: float = 0.0
    arm_shoulder_deg:  float = SHOULDER_STOW_DEG
    arm_elbow_deg:     float = ELBOW_STOW_DEG
    mallow_height_est: float = 100.0
    mallow_dist_est:   float = 300.0

    turntable_home_offset: float = TURNTABLE_HOME_OFFSET_DEG

    # Pre-compute place coordinates at clamped turntable angle
    place_t_deg = max(TURNTABLE_HW_LIMIT_DEG, PLACE_TURNTABLE_DEG_TARGET)
    if place_t_deg != PLACE_TURNTABLE_DEG_TARGET:
        print(f"[TEST] ⚠  Place turntable {PLACE_TURNTABLE_DEG_TARGET:.0f}° clamped to "
              f"{place_t_deg:.0f}° (hardware limit).  Fix axle then remove clamp.")
    place_rad = math.radians(place_t_deg)
    place_x   = PLACE_REACH_MM * math.cos(place_rad)
    place_y   = PLACE_REACH_MM * math.sin(place_rad)
    place_z   = PLATE_Z_MM
    print(f"[TEST] Place position: turntable={place_t_deg:.1f}°  "
          f"(x={place_x:.0f}, y={place_y:.0f}, z={place_z:.0f}) mm")
    print(f"[TEST] Shoulder height={ARM_GEOMETRY.shoulder_height_mm:.1f} mm above base plate  |  "
          f"floor at z={GROUND_Z_MM:.0f} mm  |  place z={place_z:.0f} mm")

    while True:

        # ── INIT ─────────────────────────────────────────────────────────────
        if state == "INIT":
            current = robot.get_state()
            if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.enable_vision()
            robot.enable_ultrasonic()
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 255)
            state = "IDLE"
            state_entry_time = time.monotonic()

        # ── IDLE ─────────────────────────────────────────────────────────────
        elif state == "IDLE":
            if robot.get_button(Button.BTN_2):
                robot.shutdown()
                return
            print("[TEST] Starting in 3...")
            time.sleep(1)
            print("[TEST] Starting in 2...")
            time.sleep(1)
            print("[TEST] Starting in 1...")
            time.sleep(1)
            print("[TEST] GO — starting pick-and-place sequence.")
            state = "SAFE_RAISE"
            state_entry_time = time.monotonic()

        # ── SAFE_RAISE ───────────────────────────────────────────────────────
        # Must execute before ANY turntable or other servo motion.
        # Elbow first (clears chassis), then shoulder (lifts arm).
        elif state == "SAFE_RAISE":
            print("[TEST] SAFE_RAISE — clearing arm from chassis before any motor motion.")
            robot.enable_servo(ELBOW_CHANNEL)
            robot.enable_servo(SHOULDER_CHANNEL)
            robot.enable_servo(GRIPPER_CHANNEL)
            time.sleep(0.2)

            # 1. Shoulder first — lift upper arm clear of hardware
            shoulder_pos = _move_servo(
                robot, SHOULDER_CHANNEL, shoulder_pos, SAFE_SHOULDER_DEG,
                SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX,
            )
            print(f"[TEST] SAFE_RAISE — shoulder at {shoulder_pos:.1f}°")

            # 2. Elbow — swing forearm clear
            elbow_pos = _move_servo(
                robot, ELBOW_CHANNEL, elbow_pos, SAFE_ELBOW_DEG,
                ELBOW_SAFE_MIN, ELBOW_SAFE_MAX,
            )
            print(f"[TEST] SAFE_RAISE — elbow at {elbow_pos:.1f}°  arm clear.")

            # Open gripper for upcoming approach
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_OPEN_DEG)
            print(f"[TEST] SAFE_RAISE — gripper open ({gripper_pos:.1f}°)")

            state = "ARM_HOME"
            state_entry_time = time.monotonic()

        # ── ARM_HOME ──────────────────────────────────────────────────────────
        # Arm is already raised from SAFE_RAISE — now enable steppers and home.
        elif state == "ARM_HOME":
            print("[TEST] ARM_HOME — enabling steppers and homing turntable.")
            robot.step_enable(TURNTABLE_STEPPER)
            robot.step_set_config(TURNTABLE_STEPPER, TURNTABLE_MAX_VELOCITY, TURNTABLE_ACCELERATION)
            robot.step_enable(CAMPAN_STEPPER)
            robot.step_set_config(CAMPAN_STEPPER, CAMPAN_MAX_VELOCITY, CAMPAN_ACCELERATION)

            # Arm is already in safe position from SAFE_RAISE — home turntable now
            turntable_home_offset = _home_turntable(robot)

            print("[TEST] ARM_HOME — ready.  Moving to SCANNING.")
            state = "SCANNING"
            state_entry_time = time.monotonic()

        # ── SCANNING ──────────────────────────────────────────────────────────
        # Camera pans over left-side positions only.  Looks for cup+mallow pairs
        # whose world bearing satisfies the turntable hardware limit.
        elif state == "SCANNING":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)

            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            if time.monotonic() - state_entry_time > SCAN_TIMEOUT_S:
                print(f"[TEST] SCANNING — {SCAN_TIMEOUT_S:.0f}s timeout, no valid target. "
                      "Stopping.")
                state = "DONE"
                state_entry_time = time.monotonic()
            else:
                img_w, img_h = robot.get_detection_image_size()
                cup_mallow_hits:      list[dict] = []
                fallback_mallow_hits: list[dict] = []

                for pan_deg in SCAN_CAMPAN_DEG:
                    _campan_to_deg(robot, pan_deg)
                    time.sleep(CAMPAN_SETTLE_S)
                    cup_dets    = robot.get_detections(CUP_CLASS)
                    mallow_dets = robot.get_detections(MARSHMALLOW_CLASS)

                    # ── Cup-first pairing ─────────────────────────────────────
                    for cup in cup_dets:
                        if float(cup["confidence"]) < MIN_CONFIDENCE_CUP:
                            continue
                        cup_pixel_bearing = _detection_bearing_deg(cup, img_w)
                        cup_world_bearing = pan_deg + cup_pixel_bearing

                        # Reject targets outside reachable turntable arc
                        if cup_world_bearing < TURNTABLE_HW_LIMIT_DEG:
                            print(f"[TEST] SCANNING — cup at {cup_world_bearing:.1f}° "
                                  "below HW limit, skipping.")
                            continue
                        if not (-TURNTABLE_SCAN_ARC_DEG <= cup_world_bearing <= TURNTABLE_SCAN_ARC_DEG):
                            continue

                        cup_dist = _cup_dist_mm(cup, img_w)

                        best_m_conf   = 0.0
                        best_m_height = None
                        for mallow in mallow_dets:
                            if float(mallow["confidence"]) < MIN_CONFIDENCE_MARSHMALLOW:
                                continue
                            if abs(_detection_bearing_deg(mallow, img_w) - cup_pixel_bearing) \
                                    <= MALLOW_CUP_BEARING_MATCH_DEG:
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

                    # ── Mallow-only fallback ──────────────────────────────────
                    for mallow in mallow_dets:
                        if float(mallow["confidence"]) < MIN_CONFIDENCE_MARSHMALLOW:
                            continue
                        m_pixel_bearing = _detection_bearing_deg(mallow, img_w)
                        m_world_bearing = pan_deg + m_pixel_bearing

                        if m_world_bearing < TURNTABLE_HW_LIMIT_DEG:
                            continue
                        if not (-TURNTABLE_SCAN_ARC_DEG <= m_world_bearing <= TURNTABLE_SCAN_ARC_DEG):
                            continue

                        m_dist   = _detection_dist_mm(mallow, img_w)
                        m_height = snap_to_cup_tier_mm(
                            _detection_height_mm(mallow, img_w, img_h, m_dist)
                        )
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
                        f"[TEST] SCANNING ({source}) — target at "
                        f"{arm_turntable_deg:.1f}°  "
                        f"dist={mallow_dist_est:.0f} mm  "
                        f"height={mallow_height_est:.0f} mm  "
                        f"conf={best['conf']:.2f}"
                    )
                    state = "RANGING"
                    state_entry_time = time.monotonic()
                # else: keep looping in SCANNING state until timeout

        # ── RANGING ───────────────────────────────────────────────────────────
        # Rotate turntable to bearing (safe retract first), re-extend arm to
        # search pose, fire ultrasonic to refine x/y, compute IK for pick.
        elif state == "RANGING":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print(f"[TEST] RANGING — rotating turntable to {arm_turntable_deg:.1f}°")
            # _rotate_turntable_safe calls _safe_arm_retract internally
            shoulder_pos, elbow_pos, gripper_pos = _rotate_turntable_safe(
                robot, arm_turntable_deg, turntable_home_offset,
                shoulder_pos, elbow_pos, gripper_pos,
            )

            # Re-extend to search pose so US sensor is aimed at the target
            shoulder_pos = _move_servo(
                robot, SHOULDER_CHANNEL, shoulder_pos, SAFE_SHOULDER_DEG,
                SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX,
            )
            elbow_pos = _move_servo(
                robot, ELBOW_CHANNEL, elbow_pos, SAFE_ELBOW_DEG,
                ELBOW_SAFE_MIN, ELBOW_SAFE_MAX,
            )
            time.sleep(0.3)

            # Camera bearing estimate as default
            bearing_rad = math.radians(arm_turntable_deg)
            mallow_x = ARM_GEOMETRY.camera_forward_offset_mm + mallow_dist_est * math.cos(bearing_rad)
            mallow_y = mallow_dist_est * math.sin(bearing_rad)
            mallow_z = mallow_height_est

            # Refine with ultrasonic if available
            try:
                d_raw = robot.get_ultrasonic_mm()
                if d_raw is not None and 20.0 < d_raw < 800.0:
                    sh_geo      = ARM_GEOMETRY.shoulder_servo_to_geo(shoulder_pos)
                    el_geo      = ARM_GEOMETRY.elbow_servo_to_geo(elbow_pos)
                    sh_rad      = math.radians(sh_geo)
                    el_rad      = math.radians(el_geo)
                    forearm_rad = sh_rad + (math.pi - el_rad)

                    elbow_horiz  = ARM_GEOMETRY.L1 * math.cos(sh_rad)
                    sensor_horiz = (ARM_GEOMETRY.shoulder_offset_mm + elbow_horiz
                                    + ULTRASONIC_FOREARM_OFFSET_MM * math.cos(forearm_rad))
                    mallow_reach = sensor_horiz + d_raw * math.cos(forearm_rad)
                    mallow_x = mallow_reach * math.cos(bearing_rad)
                    mallow_y = mallow_reach * math.sin(bearing_rad)
                    print(f"[TEST] RANGING — US={d_raw:.0f} mm  "
                          f"forearm={math.degrees(forearm_rad):.1f}°  "
                          f"reach={mallow_reach:.0f} mm")
                else:
                    print(f"[TEST] RANGING — US reading "
                          f"{'None' if d_raw is None else f'{d_raw:.0f} mm'} "
                          f"out of range — using camera estimate.")
            except Exception as exc:
                print(f"[TEST] RANGING — US error ({exc}) — using camera estimate.")

            print(f"[TEST] RANGING — target (x={mallow_x:.0f}, y={mallow_y:.0f}, "
                  f"z={mallow_z:.0f}) mm")

            try:
                _, pick_sh, pick_el = inverse_kinematics(mallow_x, mallow_y, mallow_z, ARM_GEOMETRY)
            except OutOfReachError as e:
                print(f"[TEST] RANGING — IK out of reach: {e}.  Stopping.")
                state = "DONE"
                state_entry_time = time.monotonic()
            else:
                arm_shoulder_deg = pick_sh
                arm_elbow_deg    = pick_el
                print(f"[TEST] RANGING — IK: shoulder={pick_sh:.1f}°  elbow={pick_el:.1f}°")
                state = "APPROACHING"
                state_entry_time = time.monotonic()

        # ── APPROACHING ───────────────────────────────────────────────────────
        elif state == "APPROACHING":
            print(f"[TEST] APPROACHING — turntable={arm_turntable_deg:.1f}°  "
                  f"shoulder={arm_shoulder_deg:.1f}°  elbow={arm_elbow_deg:.1f}°")
            # Gripper must be open before advancing on the target
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_OPEN_DEG)
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
            state_entry_time = time.monotonic()

        # ── PICKING ───────────────────────────────────────────────────────────
        elif state == "PICKING":
            print(f"[TEST] PICKING — closing gripper to grab angle ({GRIPPER_GRAB_DEG:.0f}°)")
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_GRAB_DEG)
            time.sleep(0.4)
            print("[TEST] PICKING — marshmallow gripped.")
            state = "CARRY_TO_PLACE"
            state_entry_time = time.monotonic()

        # ── CARRY_TO_PLACE ────────────────────────────────────────────────────
        # Retract safely, rotate to place bearing, compute place IK.
        elif state == "CARRY_TO_PLACE":
            print(f"[TEST] CARRY_TO_PLACE — rotating to place angle "
                  f"({place_t_deg:.1f}°)")
            # Safe retract + rotate (gripper stays closed around marshmallow)
            shoulder_pos, elbow_pos, gripper_pos = _safe_arm_retract(
                robot, shoulder_pos, elbow_pos, gripper_pos
            )
            # Keep gripper closed (override the close from _safe_arm_retract)
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_GRAB_DEG)

            _turntable_to_deg(robot, place_t_deg, turntable_home_offset)

            # Compute place IK
            try:
                _, place_sh, place_el = inverse_kinematics(
                    place_x, place_y, place_z, ARM_GEOMETRY
                )
            except OutOfReachError as e:
                print(f"[TEST] CARRY_TO_PLACE — place IK failed: {e}.  Stopping.")
                state = "DONE"
                state_entry_time = time.monotonic()
            else:
                print(f"[TEST] CARRY_TO_PLACE — place IK: "
                      f"shoulder={place_sh:.1f}°  elbow={place_el:.1f}°")
                state = "PLACING"
                # Store IK angles for PLACING state
                arm_shoulder_deg = place_sh
                arm_elbow_deg    = place_el
                state_entry_time = time.monotonic()

        # ── PLACING ───────────────────────────────────────────────────────────
        elif state == "PLACING":
            print(f"[TEST] PLACING — moving arm to place position "
                  f"(z={place_z:.0f} mm above base plate)")
            shoulder_pos = _move_servo(
                robot, SHOULDER_CHANNEL, shoulder_pos, arm_shoulder_deg,
                SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX,
            )
            elbow_pos = _move_servo(
                robot, ELBOW_CHANNEL, elbow_pos, arm_elbow_deg,
                ELBOW_SAFE_MIN, ELBOW_SAFE_MAX,
            )
            time.sleep(0.4)
            # Release marshmallow
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_OPEN_DEG)
            time.sleep(0.3)
            print("[TEST] PLACING — marshmallow released.  PASS — sequence complete.")
            state = "DONE"
            state_entry_time = time.monotonic()

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
