"""
manual_ik_pick_place_test.py
============================
Hardcoded 4-stack IK pick-and-place test, no vision/detection. Pick one of
the four cup stacks via PICK_STACK_INDEX (1=leftmost..4=rightmost), the
script computes the pick coordinates from the STACK_TABLE entry and runs
the full pick → carry → place → restow sequence.

STACK_TABLE entries are (front_mm, side_mm, mallow_z_mm, label):
  front  — mm forward of the campan turntable axis (= center of workspace
           circle), measured at camera pan = 0°.
  side   — lateral offset from the campan axis. +ve = RIGHT of robot.
  mallow_z — z of the marshmallow center (robot frame, base plate = 0),
             pulled from MALLOW_Z_STACK_{9,11,14,16}_MM in _manipulator_config.py.

Conversion to turntable-axis frame (arm origin):
  pick_x = front + CAMPAN_FORWARD_OFFSET_MM   (forward of arm base)
  pick_y = -side                              (robot +y = LEFT)
  pick_z = mallow_z + PICK_Z_LIFT_MM          (arm sags ~12 mm under load)
  pick_turntable_deg = atan2(pick_y, pick_x)  (FROM TURNTABLE AXIS, not camera)

The turntable rotation MUST use the from-axis bearing — using the
camera/campan bearing made the arm miss by ~180 mm in the previous run.

State machine
-------------
  INIT → IDLE → SAFE_RAISE → ARM_HOME → IK_COMPUTE
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

# ┌─────────────────────────────────────────────────────────────────────────────┐
# │  PICK ONE OF THE 4 HARDCODED STACKS (one at a time)                         │
# └─────────────────────────────────────────────────────────────────────────────┘
# 1 = leftmost (robot's view, +y), 4 = rightmost (-y).
# Height order LEFT → RIGHT for this run: 16, 11, 9, 14 cup stacks.
#
# Cup measurements are (front_mm, side_mm) FROM THE CAMPAN TURNTABLE AXIS
# (= center of the workspace circle, directly below the camera-pan stepper).
# NOT from the camera lens — the lens sits CAMPAN_OFFSET_FROM_CAMERA_MM forward
# of the campan axis, so using the camera lens as reference would bias every
# pick by that offset. Camera-lens convention is in hardcoded_cup_pick_place_test.py
# and is slightly wrong; this file is the corrected version.
#   +front = forward of campan axis
#   +side  = right of campan axis  (robot +y = LEFT, so y = -side)
#
# Stack heights are HARDCODED (no longer randomized) — the only thing that
# varies between runs is which stack carries the marshmallow. Flip
# DETECT_STACK to choose how that stack is selected:
#   True  → pan the campan to each of the 4 stacks, run vision detection on
#           each, pick whichever stack returns the highest-confidence
#           marshmallow hit. Falls back to PICK_STACK_INDEX if nothing is
#           detected (e.g. vision node not up).
#   False → use PICK_STACK_INDEX as-is.
DETECT_STACK:     bool = True
PICK_STACK_INDEX: int  = 4

# When SCAN_ONLY is True the script does ARM_HOME → DETECT_STACK_STATE → RESTOW
# and never runs the IK pick/place. Use this to collect per-stack camera frames
# (saved into /runtime_output/vision/manual_stack{N}_pan{deg}.jpg) when the
# detector misses a marshmallow so you can eyeball the image, exposure, and
# framing before retuning the rule-based detector. Set False to do the full
# pick → carry → place once detection is happy.
SCAN_ONLY: bool = False

# Stack table — index 0 unused so PICK_STACK_INDEX is 1-based.
# Each entry: (front_mm, side_mm, mallow_z_mm, label)
# Values reused from hardcoded_cup_pick_place_test.py — pending remeasure
# with a fresh caliper read.
STACK_TABLE: list[tuple[float, float, float, str]] = [
    (  0.0,    0.0,  0.0, ""),                                    # 0 — unused
    (123.0+25, -192.0-25, MALLOW_Z_STACK_16_MM, "stack#1 LEFTMOST 16-cup"),  # 1
    (217.0,  -6.0, MALLOW_Z_STACK_11_MM, "stack#2 11-cup"),           # 2
    (185.0,  135.0, MALLOW_Z_STACK_9_MM,  "stack#3 9-cup"),            # 3
    ( 97.0,  212.0+35, MALLOW_Z_STACK_14_MM, "stack#4 RIGHTMOST 14-cup"), # 4
]

# Per-stack campan pan angle for the DETECT_STACK scan, computed from each
# stack's (front, side) bearing from the campan axis. The actual hardware
# behavior is +campan_deg → camera looks LEFT (see venue_full_course_test.py
# GREEN_LIGHT_CAMPAN_DEG = +30°, which is the LEFT-side green light). The
# comments in _manipulator_config.py and arm_kinematics.py that say
# "positive = right" are wrong/aspirational — trust the hardware behavior.
# Since +side = right of robot, the campan_deg that POINTS the camera at a
# stack is the NEGATION of atan2(side, front):
#   stack on the RIGHT  (side > 0) → need camera to look right → campan < 0
#   stack on the LEFT   (side < 0) → need camera to look left  → campan > 0
# Index 0 unused (1-based stack index).
STACK_CAMPAN_DEG: list[float] = [
    0.0,
    -math.degrees(math.atan2(STACK_TABLE[1][1], STACK_TABLE[1][0])),
    -math.degrees(math.atan2(STACK_TABLE[2][1], STACK_TABLE[2][0])),
    -math.degrees(math.atan2(STACK_TABLE[3][1], STACK_TABLE[3][0])),
    -math.degrees(math.atan2(STACK_TABLE[4][1], STACK_TABLE[4][0])),
]

# Detection-mode tuning. SETTLE = pause after each campan move so the vision
# pipeline has a fresh frame before we read detections. MIN_CONFIDENCE filters
# out spurious low-confidence hits. VISION_WAIT_S is how long we wait at the
# start for the vision node to come up before giving up and falling back.
DETECT_CAMPAN_SETTLE_S:     float = 1.0
DETECT_MIN_CONFIDENCE:      float = 0.35
DETECT_VISION_WAIT_S:       float = 5.0
MARSHMALLOW_CLASS:          str   = "marshmallow"

# Lift the commanded pick z by this much above the known mallow center — the
# arm sags under load and IK predicts a slightly higher tip than reality.
PICK_Z_LIFT_MM: float = 12.0

# Forward distance from the turntable axis to the workspace-circle dot,
# which sits directly below the campan stepper axis. Measured 2026-06-11
# turntable→campan = 180 mm; bumped to 205 after stack #2 (head-on) still
# landed 25 mm short with FK matching target (dx≈0) — implies either the
# dot is actually 25 mm further forward than measured, or the (front, side)
# measurements were taken from a slightly different reference than expected.
PICK_FORWARD_OFFSET_MM: float = 205.0

# Build the chosen stack's pick coordinates directly in TURNTABLE-AXIS frame
# (turntable axis = origin, +x = forward, +y = LEFT, +z = up). PICK_TURNTABLE_DEG
# is the bearing from the TURNTABLE AXIS — feeding a camera/campan-frame bearing
# into _turntable_to_deg is what made the off-axis pick miss by ~180 mm earlier
# (the original head-on test masked the bug by always picking at bearing=0).
def _pick_coords_for_stack(
    index: int,
) -> tuple[float, float, float, float, float, str]:
    """Return (pick_x, pick_y, pick_z, pick_turntable_deg, mallow_z, label)
    in the turntable-axis frame for STACK_TABLE[index]."""
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

# ── Place target ──────────────────────────────────────────────────────────────
# Reach + base z from _manipulator_config.py (PLACE_X_MM, PLACE_Y_MM = 128, -226
# → reach ≈ 260 mm; PLACE_Z_MM = -201 mm). Turntable angle = -80° (80° to the
# RIGHT of forward).
# PLACE_Z_LIFT_MM raises the drop height above the config z so the shoulder
# servo doesn't dip below 90° when the arm is placing to the SIDE — at the low
# config z the arm dives steeply and the forearm clips the rover body. Drop
# from higher; mallow still lands on the plate.
PLACE_TURNTABLE_DEG: float = -80.0
PLACE_REACH_MM:      float = math.hypot(PLACE_X_MM, PLACE_Y_MM)
PLACE_Z_LIFT_MM:     float = 40.0
PLACE_Z_MM:          float = CONFIG_PLACE_Z_MM + PLACE_Z_LIFT_MM

# ── Grip strength ─────────────────────────────────────────────────────────────
# The gripper is angle-commanded; "strength" = how far past contact it closes.
# 0% = GRIPPER_OPEN_DEG (29°), 100% = GRIPPER_CLOSE_DEG (90°, hard stop).
# Dropped from 90 → 80 % so the mallow is held firmly but isn't squeezed deep
# into the jaws — easier to release on placement.
SQUEEZE_PERCENT: float = 80.0
SQUEEZE_DEG:     float = GRIPPER_OPEN_DEG + (SQUEEZE_PERCENT / 100.0) * (GRIPPER_CLOSE_DEG - GRIPPER_OPEN_DEG)

# Duration of each gripper hold phase (settle / hold squeeze / dwell at place).
# Dropped from 3.0 → 1.5 s so the full pick-place cycle takes ~6 s less; still
# enough for the servos to settle before the next action.
PHASE_HOLD_S: float = 1.5

# ── Cosmetic camera scan (currently disabled) ─────────────────────────────────
# Pan the camera across the cups for show before the (hardcoded) pick. Disabled
# while we're testing pure IK without vision; flip RUN_VISION_SCAN to re-enable.
RUN_VISION_SCAN: bool = False
SCAN_PAN_ANGLES   = [-60.0, -30.0, 0.0, 30.0, 60.0]  # campan sweep (deg)
SCAN_SETTLE_S     = 0.6      # settle after each pan before snapshot
VISION_OUT_DIR    = "/runtime_output/vision"

# ── Safe carry pose ───────────────────────────────────────────────────────────
SAFE_SHOULDER_DEG = 110.0   # upper arm angled slightly up (geo ≈ +14°)
SAFE_ELBOW_DEG    = 70.0    # forearm tucked — works empirically (grab succeeded).

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


def _hold_grip(robot: Robot, angle: float, seconds: float, label: str = "hold") -> None:
    """Wait `seconds` while actively re-asserting the gripper at `angle`.

    The firmware relaxes actuators when the host goes quiet, so a plain sleep
    would let the grip drift. Re-commanding (and re-enabling) every ~0.4 s keeps
    the jaw exactly where we want it through each timed pause.
    """
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
    # NOTE: do NOT change the gripper position here. After PICKING the gripper
    # is at GRAB squeezing the mallow; closing it tighter and then having the
    # caller relax it back to GRAB causes the mallow to drop.
    robot.enable_servo(SHOULDER_CHANNEL)
    robot.enable_servo(ELBOW_CHANNEL)
    robot.enable_servo(GRIPPER_CHANNEL)
    # Raise shoulder FIRST so the gripper lifts before the elbow geometry
    # changes. Target = max(current, SAFE_SHOULDER_DEG) so we never *lower*
    # the shoulder here — picking from a tall stack (16/14-cup) can leave
    # the pick shoulder already above 110°, and forcing it down to 110°
    # would shove the gripper sideways through the cups before the elbow
    # has had a chance to retract.
    shoulder_target = max(shoulder_pos, SAFE_SHOULDER_DEG)
    shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, shoulder_target,
                               SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
    # Elbow second — gripper is now clear of the cup stack above the pick
    # site, so the forearm can swing inward to the safe carry pose.
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


def _campan_to_deg_blocking(robot: Robot, target_deg: float) -> bool:
    """Blocking campan move used by the detection scan. Earlier the scan used
    _campan_to_deg (fire-and-forget + 1.0 s sleep) and the camera grabbed
    frames while the stepper was still moving — stack #4 at ~+65° literally
    never showed up because at 200 steps/sec ≈ 35°/sec the move takes ~1.9 s,
    longer than the fixed sleep. Block until the firmware reports the move
    finished, with a timeout computed from the full angular travel."""
    steps = campan_deg_to_steps(target_deg)
    # Worst case: a full sweep from one end of the campan range to the other.
    # At 200 steps/sec ≈ 35°/sec a 140° sweep is 4.0 s; add headroom for accel.
    deg_per_sec = CAMPAN_MAX_VELOCITY / STEPS_PER_CAMPAN_DEG
    travel_s    = abs(target_deg) / deg_per_sec  # campan home = 0°, so |target|
    timeout_s   = max(travel_s * 2.0 + 1.0, 4.0)
    ok = robot.step_move(
        CAMPAN_STEPPER, steps, StepMoveType.ABSOLUTE, timeout=timeout_s,
    )
    if not ok:
        print(f"[TEST] DETECT_STACK — campan blocking move to {target_deg:+.1f}° "
              f"timed out after {timeout_s:.1f}s.")
    return ok


def _detect_stack_index(robot: Robot) -> int | None:
    """Pan the campan to each of the 4 stacks and run vision detection at
    each position. Returns the 1-based index of the stack with the highest-
    confidence marshmallow hit, or None if nothing meeting
    DETECT_MIN_CONFIDENCE was seen on any stack.

    Caller is responsible for falling back to PICK_STACK_INDEX on None.
    """
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
        # Block until the stepper actually reaches the target — earlier this
        # used the fire-and-forget _campan_to_deg, which let the camera grab
        # frames mid-motion and meant stack #4 (~+65°, ~1.9 s travel) was
        # never on-screen by the time we snapped.
        _campan_to_deg_blocking(robot, campan_deg)
        # Settle: camera + detection pipeline has frame latency. Wait long
        # enough that the next "latest.jpg" the vision node writes is from
        # AFTER the motor finished moving, not the last frame captured while
        # it was still rotating.
        time.sleep(DETECT_CAMPAN_SETTLE_S)
        # Snapshot the camera frame so failed detections can be debugged
        # after the fact. The vision pipeline writes latest.jpg on every
        # frame; copy it to a per-stack name before the next pan moves on.
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
        # Fetch detections AFTER the snapshot/settle so the detections and
        # the saved frame come from roughly the same moment in time.
        try:
            dets = robot.get_detections(MARSHMALLOW_CLASS)
        except Exception as exc:
            print(f"[TEST] DETECT_STACK — get_detections failed at stack#{idx}: {exc}")
            dets = []
        # Log every detection (including ones below MIN_CONFIDENCE) so we
        # can tell "rule-based detector saw nothing" from "saw it but at
        # 0.20 confidence, raise tolerance".
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

    # Pick coordinates come directly from the module-level STACK_TABLE → PICK_*
    # derivation (turntable-axis frame). No bearing/distance round-trip. When
    # DETECT_STACK is on these are overwritten in DETECT_STACK_STATE once
    # vision picks the marshmallow-bearing stack.
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
            print(f"[TEST] SAFE_RAISE — tight-folding elbow {elbow_pos:.1f}° → "
                  f"{TIGHT_ELBOW_DEG:.1f}° (COM over shoulder).")
            elbow_pos = _move_servo(
                robot, ELBOW_CHANNEL, elbow_pos, TIGHT_ELBOW_DEG,
                ELBOW_SAFE_MIN, ELBOW_SAFE_MAX,
            )
            time.sleep(0.4)

            # Direct shoulder lift with the elbow tight-folded over the pivot.
            # Hammer the target value with hold writes so even if the first
            # command is dropped, the next one lands.
            print(f"[TEST] SAFE_RAISE — raising shoulder {shoulder_pos:.1f}° → "
                  f"{SAFE_SHOULDER_DEG:.1f}° (direct set, no ramp).")
            for i in range(5):
                robot.set_servo(SHOULDER_CHANNEL, SAFE_SHOULDER_DEG)
                time.sleep(0.3 if i == 0 else 0.15)
            shoulder_pos = SAFE_SHOULDER_DEG

            # Now that the shoulder is up, unfold the elbow to the desired
            # safe pose so the arm is ready for IK.
            print(f"[TEST] SAFE_RAISE — unfolding elbow {elbow_pos:.1f}° → "
                  f"{SAFE_ELBOW_DEG:.1f}° (shoulder is up, gripper clear).")
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
        # Pan campan to each stack's bearing, run vision detection at each
        # position, pick the stack with the highest-confidence marshmallow
        # hit. Pick coordinates (pick_x/y/z + pick_turntable_deg) are then
        # recomputed for that stack so IK_COMPUTE picks up the new target.
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

        # ── VISION_SCAN (cosmetic) ────────────────────────────────────────────
        # Pan the camera across the cups for show + log/save what it sees. The
        # pick still targets the hardcoded marshmallow position regardless.
        elif state == "VISION_SCAN":
            if robot.get_button(Button.BTN_2):
                robot.stop(); robot.shutdown(); return
            print("[TEST] VISION_SCAN — panning camera across cups (cosmetic; "
                  "pick uses the hardcoded position).")
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
        # Compute pick IK from manual inputs, rotate turntable to pick bearing.
        elif state == "IK_COMPUTE":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print(f"[TEST] IK_COMPUTE — rotating turntable to {pick_turntable_deg:.1f}° "
                  f"(bearing from turntable axis)")
            shoulder_pos, elbow_pos, gripper_pos = _safe_arm_retract(
                robot, shoulder_pos, elbow_pos, gripper_pos
            )
            _turntable_to_deg(robot, pick_turntable_deg, turntable_home_offset)
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
                # elbow_up=True (default) — combined with ELBOW_SERVO_SIGN=-1
                # this produces the elbow-at-top, gripper-from-above pose per
                # MANIPULATOR.md's sign=-1 convention (low elbow servo = forearm
                # angled up).
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
            # Forward-kinematics check — where does the script *think* the gripper
            # tip is, given the (possibly clipped) commanded servo angles? Compare
            # with the actual measured position to isolate calibration error vs
            # safe-limit clipping vs geometry constants.
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

            print(f"[TEST] PICKING — squeezing gripper to {SQUEEZE_DEG:.0f}° ({SQUEEZE_PERCENT:.0f}% grip)")
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, SQUEEZE_DEG)
            time.sleep(0.4)
            # Hold the squeeze for 5 s.
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
            # Leave the marshmallow in place (still gripped) before dropping.
            _hold_grip(robot, SQUEEZE_DEG, PHASE_HOLD_S, "dwell at place location")
            # Unload the jaw before opening. At the place pose the arm reaches
            # out to the side and the gripper hangs almost horizontal; the
            # mallow's weight + the moving-jaw's own weight under gravity bias
            # the jaw toward CLOSED, and a worn / under-volted MG996R (see
            # MG996R.pdf — torque sags ~20 % from 6 V → 4.8 V, stall current
            # 2.5 A) can't overcome that to swing OPEN. Lifting the shoulder
            # ~10° tips the gripper back toward vertical and takes that
            # gravity assist off CLOSE, so the open command actually moves
            # the jaw instead of holding it shut until the arm retracts.
            UNLOAD_LIFT_DEG = 10.0
            unload_target = shoulder_pos + UNLOAD_LIFT_DEG
            print(f"[TEST] PLACING — unloading jaw: shoulder {shoulder_pos:.1f}° → "
                  f"{unload_target:.1f}° before opening gripper.")
            shoulder_pos = _move_servo(
                robot, SHOULDER_CHANNEL, shoulder_pos, unload_target,
                SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX,
            )
            time.sleep(0.3)
            # Release — ramp to OPEN, then hammer the open command for ~1.2 s
            # so a low-battery servo or a mallow stuck in the jaws can't keep
            # the gripper half-closed. _hold_grip re-asserts the angle every
            # ~0.4 s with a re-enable so a single dropped frame won't strand
            # the jaw mid-release.
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
    node = Node("manual_ik_pick_place")
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
