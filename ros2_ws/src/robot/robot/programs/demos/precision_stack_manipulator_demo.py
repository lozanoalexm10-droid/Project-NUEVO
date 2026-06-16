"""
precision_stack_manipulator_demo.py
====================================
Vision-guided (or hardcoded) 4-stack IK pick-and-place. Selects the
marshmallow-bearing stack via camera detection (DETECT_STACK=True) or a
hardcoded index (PICK_STACK_INDEX). Runs the full pick → carry → place →
restow sequence using inverse kinematics computed from calibrated STACK_TABLE
coordinates.

State machine:
  INIT → IDLE → SAFE_RAISE → ARM_HOME → [DETECT_STACK_STATE] → IK_COMPUTE
       → APPROACHING → PICKING → CARRY_TO_PLACE → PLACING → RESTOW → DONE
"""
from __future__ import annotations

import math
import os
import shutil
import time

from robot.arm_kinematics import OutOfReachError, forward_kinematics, inverse_kinematics
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Limit, StepMoveType
from robot.robot import FirmwareState, Robot
from robot._manipulator_config import (
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
    MALLOW_Z_STACK_9_MM,
    MALLOW_Z_STACK_11_MM,
    MALLOW_Z_STACK_14_MM,
    MALLOW_Z_STACK_16_MM,
    PLACE_X_MM,
    PLACE_Y_MM,
    PLACE_Z_MM as CONFIG_PLACE_Z_MM,
    PLATE_Z_MM,
    SHOULDER_CHANNEL,
    SHOULDER_SAFE_MAX,
    SHOULDER_SAFE_MIN,
    SHOULDER_STOW_DEG,
    STEPS_PER_CAMPAN_DEG,
    TURNTABLE_ACCELERATION,
    TURNTABLE_HOME_OFFSET_DEG,
    TURNTABLE_MAX_DEG,
    TURNTABLE_MAX_VELOCITY,
    TURNTABLE_MIN_DEG,
    TURNTABLE_STEPPER,
    campan_deg_to_steps,
    turntable_deg_to_steps,
)

# 1 = leftmost (robot's view, +y), 4 = rightmost (-y).
# DETECT_STACK=True → vision selects the marshmallow stack; falls back to PICK_STACK_INDEX.
DETECT_STACK:     bool = True
PICK_STACK_INDEX: int  = 4

# SCAN_ONLY: run ARM_HOME → detect → RESTOW without picking. Saves per-stack images
# to /runtime_output/vision/ for debugging detection misses.
SCAN_ONLY: bool = False

# Stack table — index 0 unused (1-based). Each entry: (front_mm, side_mm, mallow_z_mm, label)
STACK_TABLE: list[tuple[float, float, float, str]] = [
    (  0.0,    0.0,  0.0, ""),                                    # 0 — unused
    (123.0+25, -192.0-25, MALLOW_Z_STACK_16_MM, "stack#1 LEFTMOST 16-cup"),  # 1
    (217.0,  -6.0, MALLOW_Z_STACK_11_MM, "stack#2 11-cup"),           # 2
    (185.0,  135.0, MALLOW_Z_STACK_9_MM,  "stack#3 9-cup"),            # 3
    ( 97.0,  212.0+35, MALLOW_Z_STACK_14_MM, "stack#4 RIGHTMOST 14-cup"), # 4
]

# Per-stack campan pan angles. +campan_deg = camera looks LEFT (verified on hardware).
# Negate atan2(side, front) to point the camera at each stack.
STACK_CAMPAN_DEG: list[float] = [
    0.0,
    -math.degrees(math.atan2(STACK_TABLE[1][1], STACK_TABLE[1][0])),
    -math.degrees(math.atan2(STACK_TABLE[2][1], STACK_TABLE[2][0])),
    -math.degrees(math.atan2(STACK_TABLE[3][1], STACK_TABLE[3][0])),
    -math.degrees(math.atan2(STACK_TABLE[4][1], STACK_TABLE[4][0])),
]

DETECT_CAMPAN_SETTLE_S:     float = 1.0
DETECT_MIN_CONFIDENCE:      float = 0.35
DETECT_VISION_WAIT_S:       float = 5.0
MARSHMALLOW_CLASS:          str   = "marshmallow"

# Arm sags under load; lift pick z slightly above the known marshmallow center.
PICK_Z_LIFT_MM: float = 12.0

# Forward distance from the turntable axis to the campan axis (calibrated in-situ).
PICK_FORWARD_OFFSET_MM: float = 205.0

def _pick_coords_for_stack(
    index: int,
) -> tuple[float, float, float, float, float, str]:
    front, side, mallow_z, label = STACK_TABLE[index]
    pick_x      = front + PICK_FORWARD_OFFSET_MM
    pick_y      = -side
    pick_z      = mallow_z + PICK_Z_LIFT_MM
    pick_tt_deg = math.degrees(math.atan2(pick_y, pick_x))
    return pick_x, pick_y, pick_z, pick_tt_deg, mallow_z, label


(
    PICK_X_MM,
    PICK_Y_MM,
    PICK_Z_MM,
    PICK_TURNTABLE_DEG,
    _stack_mallow_z,
    _stack_label,
) = _pick_coords_for_stack(PICK_STACK_INDEX)
_stack_front, _stack_side = STACK_TABLE[PICK_STACK_INDEX][0], STACK_TABLE[PICK_STACK_INDEX][1]

# Place target — 80° right of forward. Lift z above config to avoid forearm clipping.
PLACE_TURNTABLE_DEG: float = -80.0
PLACE_REACH_MM:      float = math.hypot(PLACE_X_MM, PLACE_Y_MM)
PLACE_Z_LIFT_MM:     float = 40.0
PLACE_Z_MM:          float = CONFIG_PLACE_Z_MM + PLACE_Z_LIFT_MM

SQUEEZE_PERCENT: float = 80.0
SQUEEZE_DEG:     float = GRIPPER_OPEN_DEG + (SQUEEZE_PERCENT / 100.0) * (GRIPPER_CLOSE_DEG - GRIPPER_OPEN_DEG)
PHASE_HOLD_S: float = 1.5

RUN_VISION_SCAN: bool = False
SCAN_PAN_ANGLES   = [-60.0, -30.0, 0.0, 30.0, 60.0]
SCAN_SETTLE_S     = 0.6
VISION_OUT_DIR    = "/runtime_output/vision"

SAFE_SHOULDER_DEG = 110.0
SAFE_ELBOW_DEG    = 70.0

HOME_NUDGE_DEG = 5.0
HOME_TIMEOUT_S = 30.0
HOME_BACKOFF   = 200
# LIM1 trips at ~171° (not 180°); use the measured offset so 0° doesn't overshoot.
TURNTABLE_HOME_OFFSET_MEASURED_DEG = TURNTABLE_HOME_OFFSET_DEG


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


def _hold_grip(robot: Robot, angle: float, seconds: float, label: str = "hold") -> None:
    """Re-assert gripper at `angle` every 0.4 s — firmware relaxes actuators when the host goes quiet."""
    print(f"[TEST]   {label}: holding gripper at {angle:.0f}° for {seconds:.0f}s")
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        robot.enable_servo(GRIPPER_CHANNEL)
        robot.set_servo(GRIPPER_CHANNEL, angle)
        time.sleep(0.4)


def _safe_arm_retract(
    robot: Robot,
    shoulder_pos: float,
    elbow_pos: float,
    gripper_pos: float,
) -> tuple[float, float, float]:
    robot.enable_servo(SHOULDER_CHANNEL)
    robot.enable_servo(ELBOW_CHANNEL)
    robot.enable_servo(GRIPPER_CHANNEL)
    # Raise shoulder first without lowering it — tall stacks can leave the pick
    # shoulder already above 110°, and lowering it would push the gripper into the cups.
    shoulder_target = max(shoulder_pos, SAFE_SHOULDER_DEG)
    shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, shoulder_target,
                               SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
    elbow_pos    = _move_servo(robot, ELBOW_CHANNEL,    elbow_pos,    SAFE_ELBOW_DEG,
                               ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
    return shoulder_pos, elbow_pos, gripper_pos


def _turntable_to_deg(robot: Robot, target_deg: float, home_offset: float) -> None:
    target_deg = max(TURNTABLE_MIN_DEG, min(TURNTABLE_MAX_DEG, target_deg))
    delta_deg  = abs(home_offset - target_deg)
    steps      = turntable_deg_to_steps(home_offset - target_deg)
    # Fire-and-forget: blocking mode misses the brief idle→active→idle transition.
    # Sleep for worst-case travel time instead.
    robot.step_move(TURNTABLE_STEPPER, steps, StepMoveType.ABSOLUTE, blocking=False)
    deg_per_sec = TURNTABLE_MAX_VELOCITY * (360.0 / 3200.0)
    time.sleep(delta_deg / deg_per_sec + 0.5)


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
    robot.step_move(CAMPAN_STEPPER, campan_deg_to_steps(target_deg),
                    StepMoveType.ABSOLUTE, blocking=False)
    time.sleep(1.0)  # fixed settle covers worst-case campan sweep


def _campan_to_deg_blocking(robot: Robot, target_deg: float) -> bool:
    """Blocking campan move for detection scans — fire-and-forget misses far stacks mid-motion."""
    steps = campan_deg_to_steps(target_deg)
    deg_per_sec = CAMPAN_MAX_VELOCITY / STEPS_PER_CAMPAN_DEG
    travel_s    = abs(target_deg) / deg_per_sec
    timeout_s   = max(travel_s * 2.0 + 1.0, 4.0)
    ok = robot.step_move(
        CAMPAN_STEPPER, steps, StepMoveType.ABSOLUTE, timeout=timeout_s,
    )
    if not ok:
        print(f"[TEST] DETECT_STACK — campan blocking move to {target_deg:+.1f}° "
              f"timed out after {timeout_s:.1f}s.")
    return ok


def _detect_stack_index(robot: Robot) -> int | None:
    """Pan to each of the 4 stacks and return the 1-based index with the highest-confidence
    marshmallow hit, or None if nothing meeting DETECT_MIN_CONFIDENCE was seen."""
    print("[TEST] DETECT_STACK — enabling vision and scanning all 4 stacks.")
    try:
        robot.enable_vision()
    except Exception as exc:
        print(f"[TEST] DETECT_STACK — enable_vision failed: {exc}")
        return None

    deadline = time.monotonic() + DETECT_VISION_WAIT_S
    vision_up = False
    while time.monotonic() < deadline:
        try:
            if robot.is_vision_active(timeout_s=1.0):
                vision_up = True
                break
        except Exception:
            pass
        time.sleep(0.3)
    if not vision_up:
        print(f"[TEST] DETECT_STACK — vision node not active after "
              f"{DETECT_VISION_WAIT_S:.1f}s; cannot detect.")
        return None

    try:
        os.makedirs(VISION_OUT_DIR, exist_ok=True)
    except Exception:
        pass

    best_idx:  int   = -1
    best_conf: float = -1.0
    for idx in (1, 2, 3, 4):
        campan_deg = STACK_CAMPAN_DEG[idx]
        _campan_to_deg_blocking(robot, campan_deg)
        time.sleep(DETECT_CAMPAN_SETTLE_S)
        try:
            src = os.path.join(VISION_OUT_DIR, "latest.jpg")
            dst = os.path.join(
                VISION_OUT_DIR,
                f"manual_stack{idx}_pan{campan_deg:+04.0f}.jpg",
            )
            shutil.copyfile(src, dst)
            img_note = f"  img={dst}"
        except Exception as exc:
            img_note = f"  img=FAILED ({exc})"
        try:
            dets = robot.get_detections(MARSHMALLOW_CLASS)
        except Exception as exc:
            print(f"[TEST] DETECT_STACK — get_detections failed at stack#{idx}: {exc}")
            dets = []
        for d in dets:
            print(f"[TEST] DETECT_STACK —   raw det: class={d.get('class_name')}  "
                  f"conf={float(d.get('confidence', 0.0)):.2f}  bbox={d.get('bbox')}")
        confs = [float(d.get("confidence", 0.0)) for d in dets]
        good  = [c for c in confs if c >= DETECT_MIN_CONFIDENCE]
        top   = max(good) if good else (max(confs) if confs else 0.0)
        print(f"[TEST] DETECT_STACK — stack#{idx} campan={campan_deg:+5.1f}°  "
              f"detections={len(dets)}  good(≥{DETECT_MIN_CONFIDENCE:.2f})={len(good)}  "
              f"top_conf={top:.2f}  ({STACK_TABLE[idx][3]}){img_note}")
        if good and top > best_conf:
            best_conf = top
            best_idx  = idx

    _campan_to_deg(robot, 0.0)

    if best_idx < 0:
        print("[TEST] DETECT_STACK — no marshmallow detected on any stack.")
        return None
    print(f"[TEST] DETECT_STACK — selected stack#{best_idx}  conf={best_conf:.2f}  "
          f"({STACK_TABLE[best_idx][3]})")
    return best_idx


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

    selected_stack_index: int   = PICK_STACK_INDEX
    pick_x:               float = PICK_X_MM
    pick_y:               float = PICK_Y_MM
    pick_z:               float = PICK_Z_MM
    pick_turntable_deg:   float = PICK_TURNTABLE_DEG
    pick_label:           str   = _stack_label
    pick_mallow_z:        float = _stack_mallow_z

    # Pre-compute place coordinates.
    place_rad = math.radians(PLACE_TURNTABLE_DEG)
    place_x   = PLACE_REACH_MM * math.cos(place_rad)
    place_y   = PLACE_REACH_MM * math.sin(place_rad)
    place_z   = PLACE_Z_MM

    print(f"[TEST] ── Manual IK Pick-Place ─────────────────────────────────")
    print(f"[TEST] SCAN_ONLY={SCAN_ONLY}  (True = scan + save images, no pick/place)")
    print(f"[TEST] Stack-selection mode: "
          f"{'DETECT (vision)' if DETECT_STACK else f'MANUAL (PICK_STACK_INDEX={PICK_STACK_INDEX})'}")
    print(f"[TEST] Default/manual stack: #{PICK_STACK_INDEX}  ({pick_label})  "
          f"mallow_z={pick_mallow_z:+.0f} + lift {PICK_Z_LIFT_MM:+.0f} = {pick_z:+.0f} mm")
    print(f"[TEST] Stack (front, side) from campan axis: "
          f"({_stack_front:+.0f}, {_stack_side:+.0f}) mm  → "
          f"pick robot-frame  x={pick_x:.0f}  y={pick_y:.0f}  z={pick_z:.0f} mm  "
          f"turntable_deg={pick_turntable_deg:+.1f}°")
    print(f"[TEST] Place robot-frame: x={place_x:.0f}  y={place_y:.0f}  z={place_z:.0f} mm  "
          f"turntable_deg={PLACE_TURNTABLE_DEG:+.1f}°")
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
            time.sleep(0.4)

            # Fold the elbow ALL THE WAY IN first so the forearm sits roughly
            # above the upper arm — that puts the arm's COM directly over the
            # shoulder pivot and minimizes the gravity moment the shoulder has
            # to overcome on the lift. Then we lift the shoulder, then return
            # the elbow to the desired safe pose with shoulder already up.
            TIGHT_ELBOW_DEG = 25.0  # near ELBOW_SAFE_MIN — max fold inward
            print(f"[TEST] SAFE_RAISE — tight-folding elbow to {TIGHT_ELBOW_DEG:.1f}°.")
            elbow_pos = _move_servo(
                robot, ELBOW_CHANNEL, elbow_pos, TIGHT_ELBOW_DEG,
                ELBOW_SAFE_MIN, ELBOW_SAFE_MAX,
            )
            time.sleep(0.4)

            print(f"[TEST] SAFE_RAISE — raising shoulder to {SAFE_SHOULDER_DEG:.1f}°.")
            for i in range(5):
                robot.set_servo(SHOULDER_CHANNEL, SAFE_SHOULDER_DEG)
                time.sleep(0.3 if i == 0 else 0.15)
            shoulder_pos = SAFE_SHOULDER_DEG

            print(f"[TEST] SAFE_RAISE — unfolding elbow to {SAFE_ELBOW_DEG:.1f}°.")
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

            # SCAN_ONLY forces the detection pass even if DETECT_STACK is
            # off — the whole point of scan-only is to capture images.
            if DETECT_STACK or SCAN_ONLY:
                state = "DETECT_STACK_STATE"
            elif RUN_VISION_SCAN:
                state = "VISION_SCAN"
            else:
                state = "IK_COMPUTE"
            state_entry = time.monotonic()

        # ── DETECT_STACK_STATE ───────────────────────────────────────────────
        elif state == "DETECT_STACK_STATE":
            if robot.get_button(Button.BTN_2):
                robot.stop(); robot.shutdown(); return
            detected = _detect_stack_index(robot)
            if detected is None:
                print(f"[TEST] DETECT_STACK_STATE — falling back to "
                      f"PICK_STACK_INDEX={PICK_STACK_INDEX} ({_stack_label}).")
                selected_stack_index = PICK_STACK_INDEX
            else:
                selected_stack_index = detected
            (
                pick_x,
                pick_y,
                pick_z,
                pick_turntable_deg,
                pick_mallow_z,
                pick_label,
            ) = _pick_coords_for_stack(selected_stack_index)
            print(f"[TEST] DETECT_STACK_STATE — using stack#{selected_stack_index}  "
                  f"({pick_label})  pick=({pick_x:.0f},{pick_y:.0f},{pick_z:.0f}) mm  "
                  f"turntable={pick_turntable_deg:+.1f}°")
            if SCAN_ONLY:
                print(f"[TEST] DETECT_STACK_STATE — SCAN_ONLY=True, skipping "
                      f"pick/place. Captured images in {VISION_OUT_DIR}.")
                state = "RESTOW"
            elif RUN_VISION_SCAN:
                state = "VISION_SCAN"
            else:
                state = "IK_COMPUTE"
            state_entry = time.monotonic()

        # ── VISION_SCAN ───────────────────────────────────────────────────────
        elif state == "VISION_SCAN":
            if robot.get_button(Button.BTN_2):
                robot.stop(); robot.shutdown(); return
            print("[TEST] VISION_SCAN — panning camera across cups.")
            try:
                robot.enable_vision()
            except Exception as exc:
                print(f"[TEST] VISION_SCAN — enable_vision failed (continuing): {exc}")
            for ang in SCAN_PAN_ANGLES:
                _campan_to_deg(robot, ang)
                time.sleep(SCAN_SETTLE_S)
                n_mallow = n_cup = 0
                try:
                    n_mallow = len(robot.get_detections("marshmallow"))
                    n_cup    = len(robot.get_detections("red_cup"))
                except Exception:
                    pass
                try:
                    dst = os.path.join(VISION_OUT_DIR, f"manual_scan_pan{ang:+04.0f}.jpg")
                    shutil.copyfile(os.path.join(VISION_OUT_DIR, "latest.jpg"), dst)
                except Exception:
                    dst = "—"
                print(f"[TEST] VISION_SCAN — pan={ang:+.0f}°  marshmallow={n_mallow}  "
                      f"red_cup={n_cup}  img={dst}")
            _campan_to_deg(robot, 0.0)
            time.sleep(0.3)
            print("[TEST] VISION_SCAN — done; proceeding to hardcoded pick.")
            state = "IK_COMPUTE"
            state_entry = time.monotonic()

        # ── IK_COMPUTE ────────────────────────────────────────────────────────
        elif state == "IK_COMPUTE":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print(f"[TEST] IK_COMPUTE — rotating turntable to {pick_turntable_deg:.1f}°")
            shoulder_pos, elbow_pos, gripper_pos = _safe_arm_retract(
                robot, shoulder_pos, elbow_pos, gripper_pos
            )
            _turntable_to_deg(robot, pick_turntable_deg, turntable_home_offset)
            time.sleep(0.3)

            print(f"[TEST] IK_COMPUTE — target: x={pick_x:.0f}  y={pick_y:.0f}  z={pick_z:.0f} mm")

            ik_turntable_deg = math.degrees(math.atan2(pick_y, pick_x))
            horizontal_dist = math.hypot(pick_x, pick_y)
            reach  = horizontal_dist - ARM_GEOMETRY.shoulder_offset_mm
            height = pick_z - ARM_GEOMETRY.shoulder_height_mm
            d      = math.sqrt(reach ** 2 + height ** 2)

            print(f"[TEST] IK_COMPUTE — ik_turntable={ik_turntable_deg:.1f}°  "
                  f"reach={reach:.0f} mm  height_from_shoulder={height:.0f} mm  "
                  f"d={d:.0f} mm  (max={ARM_GEOMETRY.L1+ARM_GEOMETRY.L2:.0f})")

            try:
                _, pick_sh, pick_el = inverse_kinematics(
                    pick_x, pick_y, pick_z, ARM_GEOMETRY,
                )
            except OutOfReachError as exc:
                print(f"[TEST] IK_COMPUTE — UNREACHABLE: {exc}")
                print("[TEST] IK_COMPUTE — check the STACK_TABLE (front, side, z) "
                      "for the selected stack and arm geometry constants.")
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
            fk_x, fk_y, fk_z = forward_kinematics(
                pick_turntable_deg, shoulder_pos, elbow_pos, ARM_GEOMETRY,
            )
            print(f"[TEST] APPROACHING — commanded servos: "
                  f"shoulder={shoulder_pos:.1f}° (target {arm_shoulder_deg:.1f}°)  "
                  f"elbow={elbow_pos:.1f}° (target {arm_elbow_deg:.1f}°)")
            print(f"[TEST] APPROACHING — FK gripper position: "
                  f"x={fk_x:.0f}  y={fk_y:.0f}  z={fk_z:.0f} mm")
            print(f"[TEST] APPROACHING — pick target was:      "
                  f"x={pick_x:.0f}  y={pick_y:.0f}  z={pick_z:.0f} mm")
            print(f"[TEST] APPROACHING — FK − target offset:   "
                  f"dx={fk_x - pick_x:+.0f}  dy={fk_y - pick_y:+.0f}  dz={fk_z - pick_z:+.0f} mm")
            # Pause with the claw open around the marshmallow before squeezing.
            _hold_grip(robot, GRIPPER_OPEN_DEG, PHASE_HOLD_S, "settle (claw around marshmallow)")
            print("[TEST] APPROACHING — at pick position, grabbing.")
            state = "PICKING"
            state_entry = time.monotonic()

        # ── PICKING ───────────────────────────────────────────────────────────
        elif state == "PICKING":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print(f"[TEST] PICKING — squeezing gripper to {SQUEEZE_DEG:.0f}° ({SQUEEZE_PERCENT:.0f}%)")
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, SQUEEZE_DEG)
            time.sleep(0.4)
            _hold_grip(robot, SQUEEZE_DEG, PHASE_HOLD_S, "hold squeeze")
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
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, SQUEEZE_DEG)
            _turntable_to_deg(robot, PLACE_TURNTABLE_DEG, turntable_home_offset)

            print(f"[TEST] CARRY_TO_PLACE — place robot-frame: "
                  f"x={place_x:.0f}  y={place_y:.0f}  z={place_z:.0f} mm")
            try:
                _, place_sh, place_el = inverse_kinematics(
                    place_x, place_y, place_z, ARM_GEOMETRY,
                )
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
            _hold_grip(robot, SQUEEZE_DEG, PHASE_HOLD_S, "dwell at place location")
            # Lift shoulder ~10° to tip the gripper vertical before opening —
            # at the place pose the arm reaches sideways and gravity biases the jaw closed.
            UNLOAD_LIFT_DEG = 10.0
            unload_target = shoulder_pos + UNLOAD_LIFT_DEG
            print(f"[TEST] PLACING — unloading jaw: shoulder → {unload_target:.1f}°")
            shoulder_pos = _move_servo(
                robot, SHOULDER_CHANNEL, shoulder_pos, unload_target,
                SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX,
            )
            time.sleep(0.3)
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_OPEN_DEG)
            _hold_grip(robot, GRIPPER_OPEN_DEG, 1.2, "release (hammer open)")
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
            print("[TEST] DONE — pick-and-place complete.")
            return

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()


def main() -> None:
    """Standalone entry point: build a Robot like robot_node does, then run()."""
    import threading

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("precision_stack_manipulator")
    robot = Robot(node)

    def _spin() -> None:
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()
    try:
        run(robot)
    finally:
        try:
            robot.shutdown()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
