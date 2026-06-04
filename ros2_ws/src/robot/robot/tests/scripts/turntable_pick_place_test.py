"""
turntable_pick_place_test.py
============================
Home the turntable, scan the full ±90° arc for a red-cup/marshmallow stack,
pick the marshmallow, carry it to the place bearing (-45°), and release.

Safe-arm convention (applied BEFORE every turntable move)
---------------------------------------------------------
1. Shoulder → SAFE_SHOULDER_DEG (140°) FIRST — lifts upper arm well above
   horizontal, clearing the chassis before the forearm can swing anywhere
   dangerous.
2. Elbow    → SAFE_ELBOW_DEG   (90°)  — keeps forearm collinear with the
   raised upper arm.

This order is mandatory.  Reversing it (elbow first while shoulder is low)
swings the forearm into the chassis.

Ranging point-down pose
-----------------------
For US/cup ranging the arm drops to RANGING_SHOULDER_DEG (135°) / RANGING_ELBOW_DEG
(30°) so the forearm tilts down toward the cup. With the cup centered in the
campan camera, the cup's bbox + known CUP_DIAMETER_MM give a precise distance.
Marshmallow centre is taken to be the cup centre; height comes from the cup
tier already snapped during SCANNING.

Homing sequence (mirrors turntable_home_test.py)
------------------------------------------------
1. Nudge CW a few degrees — clears the switch if the turntable is already
   sitting on LIM1 at startup.  Also confirms the motor moves.
2. Home CCW to LIM1 with generous backoff so the firmware sees a clean edge.

Coordinate convention
---------------------
  After homing:  firmware step 0 = stow = 180°
  Formula:       abs_steps = turntable_deg_to_steps(home_offset - target_deg)
  Full range:    TURNTABLE_MIN_DEG (-90°) → TURNTABLE_MAX_DEG (+180°)
  Scan range:    ±TURNTABLE_SCAN_ARC_DEG  (±90°)

State machine
-------------
  INIT → IDLE → SAFE_RAISE → ARM_HOME → SCANNING
       → RANGING → APPROACHING → PICKING
       → CARRY_TO_PLACE → PLACING → RESTOW → DONE

Nodes required:  bridge (auto) + vision + robot
"""
from __future__ import annotations

import math
import shutil
import time
from datetime import datetime
from pathlib import Path

from robot.arm_kinematics import OutOfReachError, inverse_kinematics
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Limit, StepMoveType
from robot.robot import FirmwareState, Robot
from robot.tests.scripts._manipulator_config import (
    ARM_GEOMETRY,
    ARM_SERVO_STEP_DEG,
    ARM_SERVO_STEP_DWELL,
    CAMPAN_ACCELERATION,
    CAMPAN_MAX_VELOCITY,
    CAMPAN_SETTLE_S,
    CAMPAN_STEPPER,
    CAMERA_FORWARD_OFFSET_MM,
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
    TURNTABLE_ACCELERATION,
    TURNTABLE_HOME_OFFSET_DEG,
    TURNTABLE_MAX_DEG,
    TURNTABLE_MAX_VELOCITY,
    TURNTABLE_MIN_DEG,
    TURNTABLE_SCAN_ARC_DEG,
    TURNTABLE_STEPPER,
    ULTRASONIC_FOREARM_OFFSET_MM,
    campan_deg_to_steps,
    snap_to_cup_tier_mm,
    turntable_deg_to_steps,
)

# ── Test-specific constants ────────────────────────────────────────────────────

# Safe carry angles — used before EVERY turntable rotation.
# Shoulder moves FIRST to lift the arm, THEN elbow swings clear.
SAFE_SHOULDER_DEG = 140.0
SAFE_ELBOW_DEG    = 90.0

# Ranging point-down pose — used in RANGING so the forearm (and the ultrasonic
# mounted on it) tilts down toward the cup. With the cup centered in the
# camera frame, its bbox width plus the known CUP_DIAMETER_MM gives the
# horizontal distance the marshmallow sits at.
RANGING_SHOULDER_DEG = 135.0
RANGING_ELBOW_DEG    = 30.0

# Campan limit used when panning to centre a chosen target during RANGING.
# Targets beyond this in world bearing can't be centred in the camera; the
# clamp keeps the move legal and the existing cup detection still has a chance
# if the target falls inside the frame after clamping.
RANGING_CAMPAN_MAX_DEG = 66.0

# Settle dwell after the campan pans before sampling the cup detection.
# Slightly longer than CAMPAN_SETTLE_S so the detection stream catches up.
RANGING_CUP_SETTLE_S = CAMPAN_SETTLE_S + 0.3

# Full-arc campan sweep across the entire ±90° field.
SCAN_CAMPAN_DEG = [60.0, 30.0, 0.0, -30.0, -60.0]

# Place target — right side of robot (-45°).  No hardware clamp anymore.
PLACE_TURNTABLE_DEG_TARGET = -45.0
PLACE_REACH_MM             = CAMERA_FORWARD_OFFSET_MM   # 197.6 mm from turntable axis

# Homing nudge — moves the turntable off LIM1 if it starts on the switch.
HOME_NUDGE_DEG  = 5.0
HOME_TIMEOUT_S  = 30.0
HOME_BACKOFF    = 200   # steps

# Scan-decision image saving — copies the annotated frame the vision node writes
# to ``VISION_LATEST_JPG`` after each campan pan, so the user can see what each
# pan's marshmallow decision was based on. Requires the vision node to be
# launched with ``debug_save_enabled:=true`` (otherwise latest.jpg is absent and
# snapshots silently no-op).
VISION_LATEST_JPG  = Path("/runtime_output/vision/latest.jpg")
SCAN_SNAPSHOT_DIR  = Path("/runtime_output/turntable_pick_place")


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
    """Move arm to safe carry pose before any turntable rotation.

    Order: shoulder FIRST (lifts upper arm clear), then elbow (swings forearm).
    Gripper closes to prevent snagging during rotation.
    """
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
    steps = turntable_deg_to_steps(home_offset - target_deg)
    robot.step_move(TURNTABLE_STEPPER, steps, StepMoveType.ABSOLUTE)


def _rotate_turntable_safe(
    robot: Robot,
    target_deg: float,
    home_offset: float,
    shoulder_pos: float,
    elbow_pos: float,
    gripper_pos: float,
) -> tuple[float, float, float]:
    shoulder_pos, elbow_pos, gripper_pos = _safe_arm_retract(
        robot, shoulder_pos, elbow_pos, gripper_pos
    )
    _turntable_to_deg(robot, target_deg, home_offset)
    return shoulder_pos, elbow_pos, gripper_pos


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
        print("[HOME] Turntable homed.  Firmware step 0 = stow (180°).")
        return TURNTABLE_MAX_DEG
    else:
        print("[HOME] WARNING: homing timed out — LIM1 may not be wired. "
              "Falling back to manual alignment (offset=0°).")
        return 0.0


def _campan_to_deg(robot: Robot, target_deg: float) -> None:
    robot.step_move(CAMPAN_STEPPER, campan_deg_to_steps(target_deg), StepMoveType.ABSOLUTE)


def _detection_bearing_deg(det: dict, img_w: int) -> float:
    bbox = det["bbox"]
    cx = bbox["x"] + bbox["width"] / 2.0
    return ((cx / img_w) - 0.5) * CAMERA_HFOV_DEG if img_w > 0 else 0.0


def _mallow_dist_mm(det: dict, img_w: int) -> float:
    bbox = det["bbox"]
    px_diam  = math.sqrt(max(1.0, bbox["width"] * bbox["height"]))
    focal_px = (img_w / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG / 2.0))
    return (MARSHMALLOW_DIAMETER_MM * focal_px) / px_diam


def _cup_dist_mm(det: dict, img_w: int) -> float:
    bbox     = det["bbox"]
    px_width = max(1.0, float(bbox["width"]))
    focal_px = (img_w / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG / 2.0))
    return (CUP_DIAMETER_MM * focal_px) / px_width


def _detection_height_mm(det: dict, img_w: int, img_h: int, dist_mm: float) -> float:
    bbox = det["bbox"]
    cy   = bbox["y"] + bbox["height"] / 2.0
    hfov_rad = math.radians(CAMERA_HFOV_DEG)
    vfov_rad = 2.0 * math.atan(math.tan(hfov_rad / 2.0) * img_h / img_w)
    elev_deg = -((cy - img_h / 2.0) / img_h) * math.degrees(vfov_rad)
    return max(0.0, min(500.0, CAMERA_HEIGHT_MM + dist_mm * math.tan(math.radians(elev_deg))))


def _save_scan_snapshot(run_id: str, label: str) -> Path | None:
    """Copy the vision node's latest annotated frame to a labeled snapshot.

    Returns the destination path, or None if the source frame is missing
    (vision node likely launched without debug_save_enabled=true) or the
    copy fails. Does not raise — the test should keep running either way.
    """
    if not VISION_LATEST_JPG.is_file():
        return None
    try:
        SCAN_SNAPSHOT_DIR.mkdir(parents=True, exist_ok=True)
        dest = SCAN_SNAPSHOT_DIR / f"scan_{run_id}_{label}.jpg"
        shutil.copyfile(VISION_LATEST_JPG, dest)
        return dest
    except OSError as exc:
        print(f"[TEST] WARNING: failed to save snapshot {label}: {exc}")
        return None


# ── Main FSM ──────────────────────────────────────────────────────────────────

def run(robot: Robot) -> None:  # noqa: C901
    state          = "INIT"
    period         = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick      = time.monotonic()
    state_entry    = time.monotonic()

    shoulder_pos: float = SHOULDER_STOW_DEG
    elbow_pos:    float = ELBOW_STOW_DEG
    gripper_pos:  float = GRIPPER_CLOSE_DEG

    arm_turntable_deg: float = 0.0
    arm_shoulder_deg:  float = SHOULDER_STOW_DEG
    arm_elbow_deg:     float = ELBOW_STOW_DEG
    mallow_height_est: float = 100.0
    mallow_dist_est:   float = 300.0
    turntable_home_offset: float = TURNTABLE_HOME_OFFSET_DEG

    # Pre-compute place coordinates
    place_t_deg = PLACE_TURNTABLE_DEG_TARGET
    place_rad   = math.radians(place_t_deg)
    place_x     = PLACE_REACH_MM * math.cos(place_rad)
    place_y     = PLACE_REACH_MM * math.sin(place_rad)
    place_z     = PLATE_Z_MM
    print(f"[TEST] Place target: turntable={place_t_deg:.1f}°  "
          f"x={place_x:.0f} y={place_y:.0f} z={place_z:.0f} mm")
    print(f"[TEST] Shoulder height={ARM_GEOMETRY.shoulder_height_mm:.1f} mm  "
          f"floor z={GROUND_Z_MM:.0f} mm  scan arc=±{TURNTABLE_SCAN_ARC_DEG:.0f}°")

    while True:

        # ── INIT ─────────────────────────────────────────────────────────────
        if state == "INIT":
            if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.enable_vision()
            robot.enable_ultrasonic()
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
        # Runs BEFORE any motor motion.
        # Shoulder goes up first (arm straight up, clears chassis),
        # then elbow swings to horizontal ranging position.
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
            print(f"[TEST] SAFE_RAISE — shoulder at {shoulder_pos:.1f}° (arm up)")

            elbow_pos = _move_servo(
                robot, ELBOW_CHANNEL, elbow_pos, SAFE_ELBOW_DEG,
                ELBOW_SAFE_MIN, ELBOW_SAFE_MAX,
            )
            print(f"[TEST] SAFE_RAISE — elbow at {elbow_pos:.1f}° (forearm horizontal, arm clear)")

            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_OPEN_DEG)
            print(f"[TEST] SAFE_RAISE — gripper open ({gripper_pos:.1f}°)")

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
            print(f"[TEST] ARM_HOME — home_offset={turntable_home_offset:.1f}°  "
                  f"forward(0°) = {turntable_deg_to_steps(turntable_home_offset):.0f} steps")

            # Move turntable to forward position after homing
            print("[TEST] ARM_HOME — moving to forward (0°) position.")
            _turntable_to_deg(robot, 0.0, turntable_home_offset)
            time.sleep(0.5)

            state = "SCANNING"
            state_entry = time.monotonic()

        # ── SCANNING ──────────────────────────────────────────────────────────
        # Full ±90° arc scan.  Camera pans through all five positions.
        # Prefers cup+mallow pairs; falls back to mallow-only.
        elif state == "SCANNING":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)

            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            if time.monotonic() - state_entry > SCAN_TIMEOUT_S:
                print(f"[TEST] SCANNING — {SCAN_TIMEOUT_S:.0f}s timeout, no target found.")
                state = "RESTOW"
                state_entry = time.monotonic()
            else:
                img_w, img_h = robot.get_detection_image_size()
                cup_mallow_hits:      list[dict] = []
                fallback_mallow_hits: list[dict] = []
                run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
                pan_snapshots: dict[float, Path] = {}

                for pan_deg in SCAN_CAMPAN_DEG:
                    _campan_to_deg(robot, pan_deg)
                    time.sleep(CAMPAN_SETTLE_S)
                    cup_dets    = robot.get_detections(CUP_CLASS)
                    mallow_dets = robot.get_detections(MARSHMALLOW_CLASS)

                    # Save the annotated frame the vision node just published, so
                    # we have a per-pan record of what the model called a
                    # marshmallow. Tagged with the pan angle for later lookup.
                    snap = _save_scan_snapshot(run_id, f"pan{int(round(pan_deg)):+03d}")
                    if snap is not None:
                        pan_snapshots[pan_deg] = snap

                    # Cup+mallow pairing
                    for cup in cup_dets:
                        if float(cup["confidence"]) < MIN_CONFIDENCE_CUP:
                            continue
                        cup_bearing  = _detection_bearing_deg(cup, img_w)
                        world_bearing = pan_deg + cup_bearing
                        if not (-TURNTABLE_SCAN_ARC_DEG <= world_bearing <= TURNTABLE_SCAN_ARC_DEG):
                            continue
                        cup_dist = _cup_dist_mm(cup, img_w)

                        best_conf   = 0.0
                        best_height = None
                        for mallow in mallow_dets:
                            if float(mallow["confidence"]) < MIN_CONFIDENCE_MARSHMALLOW:
                                continue
                            if abs(_detection_bearing_deg(mallow, img_w) - cup_bearing) \
                                    <= MALLOW_CUP_BEARING_MATCH_DEG:
                                mc = float(mallow["confidence"])
                                if mc > best_conf:
                                    best_conf   = mc
                                    best_height = snap_to_cup_tier_mm(
                                        _detection_height_mm(mallow, img_w, img_h, cup_dist)
                                    )

                        if best_height is not None:
                            cup_mallow_hits.append({
                                "bearing_deg": world_bearing,
                                "dist_mm":     cup_dist,
                                "height_mm":   best_height,
                                "conf":        best_conf,
                                "pan_deg":     pan_deg,
                            })

                    # Mallow-only fallback
                    for mallow in mallow_dets:
                        if float(mallow["confidence"]) < MIN_CONFIDENCE_MARSHMALLOW:
                            continue
                        m_bearing = _detection_bearing_deg(mallow, img_w)
                        world_bearing = pan_deg + m_bearing
                        if not (-TURNTABLE_SCAN_ARC_DEG <= world_bearing <= TURNTABLE_SCAN_ARC_DEG):
                            continue
                        m_dist = _mallow_dist_mm(mallow, img_w)
                        fallback_mallow_hits.append({
                            "bearing_deg": world_bearing,
                            "dist_mm":     m_dist,
                            "height_mm":   snap_to_cup_tier_mm(
                                _detection_height_mm(mallow, img_w, img_h, m_dist)
                            ),
                            "conf":    float(mallow["confidence"]),
                            "pan_deg": pan_deg,
                        })

                _campan_to_deg(robot, 0.0)

                hits   = cup_mallow_hits if cup_mallow_hits else fallback_mallow_hits
                source = "cup+mallow" if cup_mallow_hits else "mallow-only"

                if hits:
                    best = max(hits, key=lambda h: h["conf"])
                    arm_turntable_deg = best["bearing_deg"]
                    mallow_dist_est   = best["dist_mm"]
                    mallow_height_est = best["height_mm"]
                    print(f"[TEST] SCANNING ({source}) — target "
                          f"bearing={arm_turntable_deg:.1f}°  "
                          f"dist={mallow_dist_est:.0f} mm  "
                          f"height={mallow_height_est:.0f} mm  "
                          f"conf={best['conf']:.2f}")

                    # Mark which pan's frame was the decider so it's easy to
                    # find later when reviewing what the camera actually saw.
                    decider = pan_snapshots.get(best["pan_deg"])
                    if decider is not None:
                        try:
                            SCAN_SNAPSHOT_DIR.mkdir(parents=True, exist_ok=True)
                            decider_link = SCAN_SNAPSHOT_DIR / f"scan_{run_id}_decider.jpg"
                            shutil.copyfile(decider, decider_link)
                            print(f"[TEST] SCANNING — decider image: {decider_link}")
                        except OSError as exc:
                            print(f"[TEST] SCANNING — decider image at {decider} (copy failed: {exc})")
                    else:
                        print("[TEST] SCANNING — decider image unavailable "
                              "(launch vision with debug_save_enabled:=true to capture it)")
                    state = "RANGING"
                    state_entry = time.monotonic()

        # ── RANGING ───────────────────────────────────────────────────────────
        # Safe-retract + rotate, drop to point-down pose, refine via cup
        # detection (cup centre = marshmallow centre), fall back to US, IK.
        elif state == "RANGING":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print(f"[TEST] RANGING — rotating to {arm_turntable_deg:.1f}°")
            shoulder_pos, elbow_pos, gripper_pos = _rotate_turntable_safe(
                robot, arm_turntable_deg, turntable_home_offset,
                shoulder_pos, elbow_pos, gripper_pos,
            )

            # Drop to the point-down pose so the forearm (and the ultrasonic)
            # tilts toward the cup. This pose also keeps the camera view of
            # the cup unobstructed by the arm.
            shoulder_pos = _move_servo(
                robot, SHOULDER_CHANNEL, shoulder_pos, RANGING_SHOULDER_DEG,
                SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX,
            )
            elbow_pos = _move_servo(
                robot, ELBOW_CHANNEL, elbow_pos, RANGING_ELBOW_DEG,
                ELBOW_SAFE_MIN, ELBOW_SAFE_MAX,
            )

            # Centre the campan on the chosen target so the cup occupies the
            # middle of the frame — cleanest bbox for the diameter→distance
            # calculation. Clamp to the campan's mechanical range.
            camera_pan_deg = max(-RANGING_CAMPAN_MAX_DEG,
                                 min(RANGING_CAMPAN_MAX_DEG, arm_turntable_deg))
            _campan_to_deg(robot, camera_pan_deg)
            time.sleep(RANGING_CUP_SETTLE_S)

            # Camera-model fallback in case neither cup detection nor US gives a
            # usable refinement below.
            bearing_rad = math.radians(arm_turntable_deg)
            mallow_x = ARM_GEOMETRY.camera_forward_offset_mm + mallow_dist_est * math.cos(bearing_rad)
            mallow_y = mallow_dist_est * math.sin(bearing_rad)
            mallow_z = mallow_height_est

            # Preferred refinement: cup detection. The marshmallow is always
            # centred on the cup, so converting the cup bbox + known
            # CUP_DIAMETER_MM into a distance also gives the marshmallow's
            # horizontal position. z is the tier height already snapped during
            # SCANNING.
            img_w, img_h = robot.get_detection_image_size()
            best_cup = None
            best_cup_conf = MIN_CONFIDENCE_CUP
            for cup in robot.get_detections(CUP_CLASS):
                cc = float(cup["confidence"])
                if cc < best_cup_conf:
                    continue
                cup_world_bearing = camera_pan_deg + _detection_bearing_deg(cup, img_w)
                if abs(cup_world_bearing - arm_turntable_deg) > MALLOW_CUP_BEARING_MATCH_DEG:
                    continue
                best_cup = cup
                best_cup_conf = cc

            refined = False
            if best_cup is not None:
                cup_dist     = _cup_dist_mm(best_cup, img_w)
                cup_bearing  = camera_pan_deg + _detection_bearing_deg(best_cup, img_w)
                refined_rad  = math.radians(cup_bearing)
                mallow_x = ARM_GEOMETRY.camera_forward_offset_mm + cup_dist * math.cos(refined_rad)
                mallow_y = cup_dist * math.sin(refined_rad)
                refined = True
                snap = _save_scan_snapshot(
                    datetime.now().strftime("%Y%m%d_%H%M%S"), "ranging_cup")
                snap_msg = f"  snapshot={snap}" if snap else ""
                print(f"[TEST] RANGING — cup refine: dist={cup_dist:.0f} mm  "
                      f"bearing={cup_bearing:.1f}°  conf={best_cup_conf:.2f}{snap_msg}")

            # Ultrasonic fallback — only if no cup was matched. Keeps the old
            # behaviour available so a missing detection isn't a hard failure.
            if not refined:
                try:
                    d_raw = robot.get_ultrasonic_mm()
                    if d_raw is not None and 20.0 < d_raw < 800.0:
                        sh_geo      = ARM_GEOMETRY.shoulder_servo_to_geo(shoulder_pos)
                        el_geo      = ARM_GEOMETRY.elbow_servo_to_geo(elbow_pos)
                        sh_rad      = math.radians(sh_geo)
                        el_rad      = math.radians(el_geo)
                        forearm_rad = sh_rad + (math.pi - el_rad)
                        sensor_horiz = (ARM_GEOMETRY.shoulder_offset_mm
                                        + ARM_GEOMETRY.L1 * math.cos(sh_rad)
                                        + ULTRASONIC_FOREARM_OFFSET_MM * math.cos(forearm_rad))
                        mallow_reach = sensor_horiz + d_raw * math.cos(forearm_rad)
                        mallow_x = mallow_reach * math.cos(bearing_rad)
                        mallow_y = mallow_reach * math.sin(bearing_rad)
                        print(f"[TEST] RANGING — US fallback: US={d_raw:.0f} mm  "
                              f"reach={mallow_reach:.0f} mm  (no cup matched at "
                              f"{arm_turntable_deg:.1f}°)")
                    else:
                        print(f"[TEST] RANGING — no cup at {arm_turntable_deg:.1f}° and "
                              f"US {'None' if d_raw is None else f'{d_raw:.0f} mm'} "
                              f"out of range — using SCANNING estimate.")
                except Exception as exc:
                    print(f"[TEST] RANGING — no cup matched and US error ({exc}) — "
                          f"using SCANNING estimate.")

            print(f"[TEST] RANGING — IK target: x={mallow_x:.0f} y={mallow_y:.0f} z={mallow_z:.0f} mm")

            try:
                _, pick_sh, pick_el = inverse_kinematics(mallow_x, mallow_y, mallow_z, ARM_GEOMETRY)
            except OutOfReachError as exc:
                print(f"[TEST] RANGING — out of reach: {exc}")
                state = "RESTOW"
                state_entry = time.monotonic()
            else:
                arm_shoulder_deg = pick_sh
                arm_elbow_deg    = pick_el
                print(f"[TEST] RANGING — IK: shoulder={pick_sh:.1f}°  elbow={pick_el:.1f}°")
                state = "APPROACHING"
                state_entry = time.monotonic()

        # ── APPROACHING ───────────────────────────────────────────────────────
        elif state == "APPROACHING":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print(f"[TEST] APPROACHING — shoulder={arm_shoulder_deg:.1f}°  elbow={arm_elbow_deg:.1f}°")
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

            print(f"[TEST] CARRY_TO_PLACE — rotating to place bearing ({place_t_deg:.1f}°)")
            # Safe retract (shoulder first, then elbow), then rotate.
            shoulder_pos, elbow_pos, gripper_pos = _safe_arm_retract(
                robot, shoulder_pos, elbow_pos, gripper_pos
            )
            # Re-close gripper after _safe_arm_retract opened it
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_GRAB_DEG)
            _turntable_to_deg(robot, place_t_deg, turntable_home_offset)

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
                      f"shoulder={place_sh:.1f}°  elbow={place_el:.1f}°")
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
        # Always runs before DONE — brings arm back to safe carry pose,
        # rotates turntable to forward (0°), then disables servos/steppers.
        elif state == "RESTOW":
            print("[TEST] RESTOW — retracting arm and returning to forward.")
            shoulder_pos, elbow_pos, gripper_pos = _safe_arm_retract(
                robot, shoulder_pos, elbow_pos, gripper_pos
            )
            _turntable_to_deg(robot, 0.0, turntable_home_offset)
            time.sleep(0.5)
            _campan_to_deg(robot, 0.0)
            time.sleep(0.3)
            robot.step_disable(TURNTABLE_STEPPER)
            robot.step_disable(CAMPAN_STEPPER)
            print("[TEST] RESTOW — arm retracted, turntable at 0°, steppers disabled.")
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
