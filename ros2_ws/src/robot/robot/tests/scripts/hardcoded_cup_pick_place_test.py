"""
hardcoded_cup_pick_place_test.py
================================
Pick-and-place with HARDCODED cup positions + a camera "which cup / how tall"
classification pass.  No LiDAR.

The idea
--------
The four cup-stack positions (x, y) are fixed and known (measured by hand).
The four stack HEIGHTS are also a known, fixed set — but which physical
position holds which height is RANDOM each run.  So the camera does two jobs
during its opening pan across the cups:

  1. Ranks the cups by apparent height (tallest → shortest) and maps that
     ordering onto the sorted known mallow-centre heights, giving each cup
     position its correct z.  Relative ranking is robust to the camera's
     downward tilt bias — every cup is biased the same way, so the *order*
     survives even when the absolute height estimate is off.
  2. Finds which cup has the (white) marshmallow on top.

It then feeds the chosen cup's (x, y, rank-assigned z) into the exact same
manual-IK pick-and-place sequence used by manual_ik_pick_place_test.py — the
careful COM-over-shoulder SAFE_RAISE, the squeeze-and-hold grip, and the
fixed place target.

State machine
-------------
  INIT → IDLE → SAFE_RAISE → ARM_HOME → VISION_FIND_CUP
       → IK_COMPUTE → APPROACHING → PICKING → CARRY_TO_PLACE
       → PLACING → RESTOW → DONE

Nodes required: bridge (auto) + vision + robot.  No lidar node needed.
"""
from __future__ import annotations

import math
import os
import shutil
import time

from robot.arm_kinematics import OutOfReachError, forward_kinematics, inverse_kinematics
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Limit, StepMoveType
from robot.robot import FirmwareState, Robot
from robot.tests.scripts._cup_scan_logic import (
    assign_heights_by_rank,
    bbox_center_height_mm,
    camera_bearing_to_cup,
    cup_top_height_mm,
    detection_center_bearing_deg,
)
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
    ELBOW_CHANNEL,
    ELBOW_SAFE_MAX,
    ELBOW_SAFE_MIN,
    ELBOW_STOW_DEG,
    GRIPPER_CHANNEL,
    GRIPPER_CLOSE_DEG,
    GRIPPER_OPEN_DEG,
    MALLOW_CUP_BEARING_MATCH_DEG,
    MALLOW_Z_STACK_9_MM,
    MALLOW_Z_STACK_11_MM,
    MALLOW_Z_STACK_14_MM,
    MALLOW_Z_STACK_16_MM,
    MARSHMALLOW_CLASS,
    MIN_CONFIDENCE_CUP,
    MIN_CONFIDENCE_MARSHMALLOW,
    PLACE_X_MM,
    PLACE_Y_MM,
    PLACE_Z_MM as CONFIG_PLACE_Z_MM,
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
    snap_to_venue_tier_mm,
    turntable_deg_to_steps,
)

# ┌─────────────────────────────────────────────────────────────────────────────┐
# │  HARDCODED CUP POSITIONS — measured by hand                                 │
# └─────────────────────────────────────────────────────────────────────────────┘
# Each entry is (front_mm, side_mm), measured in the CAMERA frame:
#   front = mm forward of the camera lens
#   side  = lateral offset, POSITIVE = to the RIGHT of the camera
# Converted to turntable frame at scan time:  x = front + CAMERA_FORWARD_OFFSET_MM,
#                                             y = -side  (robot +y = left)
HARDCODED_CUPS_CAMERA_MM: list[tuple[float, float]] = [
    (90.0,  204.0),   # cup 1
    (178.0, 103.0),   # cup 2
    (201.0, -15.6),   # cup 3
    (122.0, -183.0),  # cup 4
]

# The four known mallow-centre heights (robot frame z), sorted SHORTEST → TALLEST.
# These come from _manipulator_config.py (9/11/14/16-cup stacks ≈ -62/-50/-30/-16).
# Which physical cup has which height is decided at runtime by the camera ranking.
KNOWN_MALLOW_Z_MM_SORTED: list[float] = sorted([
    MALLOW_Z_STACK_9_MM,
    MALLOW_Z_STACK_11_MM,
    MALLOW_Z_STACK_14_MM,
    MALLOW_Z_STACK_16_MM,
])

# Command the pick this much ABOVE the rank-assigned mallow centre — the arm
# sags under load and grabs low.  manual_ik_pick_place_test uses ~+11 mm.
PICK_Z_LIFT_MM: float = 12.0

# Where the vision node writes its annotated frame, and where we copy per-angle
# scan snapshots for diagnosis.
VISION_OUT_DIR: str = "/runtime_output/vision"
# Set env PICK_SCAN_ONLY=1 to pan + save the 4 scan images and exit WITHOUT
# picking — used to diagnose which cups are detected and at what heights.
SCAN_ONLY: bool = bool(os.environ.get("PICK_SCAN_ONLY"))

# ── Vision scan tunables ──────────────────────────────────────────────────────
PICK_CAMPAN_SETTLE_S = CAMPAN_SETTLE_S + 0.3   # settle after panning before reading
PICK_CAMPAN_MAX_DEG  = 66.0                    # campan physical pan limit
VISION_WARMUP_TIMEOUT_S = 5.0                  # wait for first inference before scanning
SCAN_SAMPLES   = 6     # detection reads per cup (defeats the stale-cache first read)
SCAN_SAMPLE_DT = 0.2   # seconds between samples (~1.2 s total at the slow ~4 Hz node)

# ── Place target (same defaults as manual_ik_pick_place_test.py) ──────────────
PLACE_TURNTABLE_DEG: float = -80.0
PLACE_REACH_MM:      float = math.hypot(PLACE_X_MM, PLACE_Y_MM)
PLACE_Z_MM:          float = CONFIG_PLACE_Z_MM

# ── Safe carry pose ───────────────────────────────────────────────────────────
SAFE_SHOULDER_DEG = 110.0
SAFE_ELBOW_DEG    = 70.0

# ── Turntable homing ──────────────────────────────────────────────────────────
HOME_NUDGE_DEG = 5.0
HOME_TIMEOUT_S = 30.0
HOME_BACKOFF   = 200
TURNTABLE_HOME_OFFSET_MEASURED_DEG = TURNTABLE_HOME_OFFSET_DEG

# Camera-rig geometry (measured): the campan stepper axis sits 178.5 mm in front
# of the turntable axis, and the focal lens is another 30 mm in front of the
# campan axis (so the lens is 208.5 mm forward). The CAMERA pans about the
# CAMPAN axis, so the per-stack pointing angle must be measured from the campan
# axis — using the lens offset here would bias every aim angle.
CAMPAN_OFFSET_FROM_CAMERA_MM = 30.0     # lens is 30 mm ahead of the campan pivot
CAMPAN_FORWARD_OFFSET_MM     = CAMERA_FORWARD_OFFSET_MM - CAMPAN_OFFSET_FROM_CAMERA_MM  # ≈ 178.5 mm


# ── Servo / motion helpers (mirror manual_ik_pick_place_test.py exactly) ──────

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
    would let the grip drift. Re-commanding every ~0.4 s keeps the jaw put.
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
    # NOTE: do NOT change the gripper position here — after PICKING the gripper
    # is squeezing the mallow and re-closing it would drop it.
    robot.enable_servo(SHOULDER_CHANNEL)
    robot.enable_servo(ELBOW_CHANNEL)
    robot.enable_servo(GRIPPER_CHANNEL)
    shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, SAFE_SHOULDER_DEG,
                               SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
    elbow_pos    = _move_servo(robot, ELBOW_CHANNEL,    elbow_pos,    SAFE_ELBOW_DEG,
                               ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
    return shoulder_pos, elbow_pos, gripper_pos


def _turntable_to_deg(robot: Robot, target_deg: float, home_offset: float) -> None:
    target_deg = max(TURNTABLE_MIN_DEG, min(TURNTABLE_MAX_DEG, target_deg))
    delta_deg  = abs(home_offset - target_deg)
    steps      = turntable_deg_to_steps(home_offset - target_deg)
    # Fire-and-forget — blocking stalls the FSM on a missed IDLE transition.
    robot.step_move(TURNTABLE_STEPPER, steps, StepMoveType.ABSOLUTE, blocking=False)
    deg_per_sec = TURNTABLE_MAX_VELOCITY * (360.0 / 3200.0)
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

    print(f"[HOME] Homing CCW to LIM1 (timeout={HOME_TIMEOUT_S:.0f}s)...")
    ok = robot.step_home(
        TURNTABLE_STEPPER,
        direction=-1,
        home_velocity=2000,
        backoff_steps=HOME_BACKOFF,
        timeout=HOME_TIMEOUT_S,
    )

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
    time.sleep(1.0)


# ── Vision helpers ────────────────────────────────────────────────────────────

def _build_cups() -> list[dict[str, float]]:
    """Convert the hardcoded camera-frame measurements to turntable-frame dicts."""
    cups: list[dict[str, float]] = []
    for i, (front, side) in enumerate(HARDCODED_CUPS_CAMERA_MM):
        cups.append({
            "id":   float(i + 1),
            "x_mm": front + CAMERA_FORWARD_OFFSET_MM,
            "y_mm": -side,
        })
    return cups


def _detection_center_bearing_deg(det: dict, img_w: int) -> float:
    """Bearing of a detection's bbox centre from frame centre (config-wired)."""
    return detection_center_bearing_deg(det, img_w, CAMERA_HFOV_DEG)


def _cup_top_height_mm(det: dict, img_w: int, img_h: int, dist_mm: float) -> float:
    """Robot-frame z of the TOP edge of a red-cup bbox (config-wired).

    Uses the bbox top edge so the estimate tracks stack height; only the
    relative ORDER across cups is used downstream, so the camera-tilt bias
    (MANIPULATOR_TASKS.md §0) cancels out.
    """
    return cup_top_height_mm(det, img_w, img_h, dist_mm, CAMERA_HFOV_DEG, CAMERA_HEIGHT_MM)


def _camera_bearing_to_cup(x_mm: float, y_mm: float) -> float:
    """Campan-frame bearing to a turntable-frame point (turntable parked at 0°)."""
    return camera_bearing_to_cup(x_mm, y_mm, CAMPAN_FORWARD_OFFSET_MM)


def _wait_vision_ready(robot: Robot, timeout_s: float) -> tuple[int, int]:
    """Poll get_detection_image_size() until the vision node publishes a frame.

    Avoids racing the node's first inference (see MANIPULATOR_TASKS.md §0).
    """
    deadline = time.monotonic() + timeout_s
    img_w, img_h = robot.get_detection_image_size()
    while (img_w <= 0 or img_h <= 0) and time.monotonic() < deadline:
        time.sleep(0.1)
        img_w, img_h = robot.get_detection_image_size()
    return img_w, img_h


def _scan_cups(robot: Robot, cups: list[dict[str, float]]) -> list[dict[str, float]]:
    """Pan the camera to each hardcoded cup and record its measured top height
    and any marshmallow detection sitting on it.

    Augments each cup dict in place with:
      top_mm       — estimated top-of-stack z, or None if no cup detected
      mallow_conf  — best marshmallow confidence on this cup (0.0 if none)
      mallow_det   — the marshmallow detection dict (for fallback height), or None
    """
    img_w, img_h = _wait_vision_ready(robot, VISION_WARMUP_TIMEOUT_S)
    if img_w <= 0 or img_h <= 0:
        print("[VISION] WARNING: no vision frame after warm-up; scan may be empty.")

    for cup in cups:
        cam_bearing = _camera_bearing_to_cup(cup["x_mm"], cup["y_mm"])
        clamped     = max(-PICK_CAMPAN_MAX_DEG, min(PICK_CAMPAN_MAX_DEG, cam_bearing))
        cup["cam_bearing_deg"] = cam_bearing
        cup["pan_deg"]         = clamped
        # Where the cup should appear in-frame after the (possibly clamped) pan.
        expected_in_frame = cam_bearing - clamped

        _campan_to_deg(robot, clamped)
        time.sleep(PICK_CAMPAN_SETTLE_S)

        # Save the annotated camera frame at this pan angle for diagnosis.
        try:
            time.sleep(0.3)  # let one more annotated frame land after settle
            dst = os.path.join(VISION_OUT_DIR, f"scan_cup{int(cup['id'])}_pan{clamped:+04.0f}.jpg")
            shutil.copyfile(os.path.join(VISION_OUT_DIR, "latest.jpg"), dst)
            cup["scan_img"] = dst
            print(f"[VISION]   saved scan image -> {dst}")
        except Exception as exc:
            print(f"[VISION]   (could not save scan image: {exc})")

        dist_mm = math.hypot(cup["x_mm"] - CAMERA_FORWARD_OFFSET_MM, cup["y_mm"])

        # Multi-sample over a short window: one get_detections() right after a pan
        # frequently returns the previous angle's stale (or empty) cache before a
        # fresh inference lands, which is what dropped far/edge cups before. Read
        # several times and keep the best per class.
        cup["top_mm"]      = None
        cup["mallow_conf"] = 0.0
        cup["mallow_det"]  = None
        best_cup_offset    = 1e9   # keep the cup detection nearest frame centre
        for _sample in range(SCAN_SAMPLES):
            # --- cup top height (for ranking): best-CENTERED cup, not best-conf ---
            for det in robot.get_detections(CUP_CLASS):
                if float(det["confidence"]) < MIN_CONFIDENCE_CUP:
                    continue
                b = _detection_center_bearing_deg(det, img_w)
                off = abs(b - expected_in_frame)
                if off > MALLOW_CUP_BEARING_MATCH_DEG:
                    continue
                if off < best_cup_offset:
                    best_cup_offset = off
                    cup["top_mm"] = _cup_top_height_mm(det, img_w, img_h, dist_mm)
            # --- marshmallow presence: highest confidence among centered ---
            for det in robot.get_detections(MARSHMALLOW_CLASS):
                conf = float(det["confidence"])
                if conf < MIN_CONFIDENCE_MARSHMALLOW:
                    continue
                b = _detection_center_bearing_deg(det, img_w)
                if abs(b - expected_in_frame) > MALLOW_CUP_BEARING_MATCH_DEG:
                    continue
                if conf > cup["mallow_conf"]:
                    cup["mallow_conf"] = conf
                    cup["mallow_det"]  = det
            time.sleep(SCAN_SAMPLE_DT)

        top_str = f"{cup['top_mm']:+.0f} mm" if cup["top_mm"] is not None else "—"
        print(f"[VISION] cup#{cup['id']:.0f} pan={clamped:+.1f}°  "
              f"top≈{top_str}  mallow_conf={cup['mallow_conf']:.2f}")

    _campan_to_deg(robot, 0.0)
    return cups


def _assign_heights_by_rank(cups: list[dict[str, float]]) -> bool:
    """Rank cups by measured top height and map onto the sorted known heights.

    Returns True if every cup got a rank-assigned z_mm (all four measured),
    False if the height scan was incomplete (caller must fall back).
    """
    if not assign_heights_by_rank(cups, KNOWN_MALLOW_Z_MM_SORTED):
        return False
    for rank, cup in enumerate(sorted(cups, key=lambda c: c["top_mm"])):
        print(f"[VISION] rank {rank} (top {cup['top_mm']:+.0f} mm) → cup#{cup['id']:.0f}"
              f"  z={cup['z_mm']:+.0f} mm")
    return True


def _mallow_fallback_z_mm(cup: dict, img_w: int, img_h: int) -> float:
    """Last-resort z for the target cup when rank-assignment was not possible.

    Snap the target's own measured height (cup top, else marshmallow bbox) to
    the nearest known tier so we still command a sane height.
    """
    if cup.get("top_mm") is not None:
        return snap_to_venue_tier_mm(cup["top_mm"])
    det = cup.get("mallow_det")
    if det is not None and img_w > 0 and img_h > 0:
        dist_mm = math.hypot(cup["x_mm"] - CAMERA_FORWARD_OFFSET_MM, cup["y_mm"])
        raw_z   = bbox_center_height_mm(
            det, img_w, img_h, dist_mm, CAMERA_HFOV_DEG, CAMERA_HEIGHT_MM)
        return snap_to_venue_tier_mm(raw_z)
    # Nothing to go on — middle of the known set.
    return KNOWN_MALLOW_Z_MM_SORTED[len(KNOWN_MALLOW_Z_MM_SORTED) // 2]


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

    # Pick coordinates — filled in by VISION_FIND_CUP.
    pick_x = pick_y = pick_z = 0.0
    pick_bearing_deg = 0.0

    # Pre-compute place coordinates.
    place_rad = math.radians(PLACE_TURNTABLE_DEG)
    place_x   = PLACE_REACH_MM * math.cos(place_rad)
    place_y   = PLACE_REACH_MM * math.sin(place_rad)
    place_z   = PLACE_Z_MM

    print("[TEST] ── Hardcoded Cup Pick-Place ─────────────────────────────────")
    print(f"[TEST] {len(HARDCODED_CUPS_CAMERA_MM)} hardcoded cups; "
          f"known heights (sorted) = "
          f"{[round(z) for z in KNOWN_MALLOW_Z_MM_SORTED]} mm")
    print(f"[TEST] Place robot-frame: x={place_x:.0f}  y={place_y:.0f}  z={place_z:.0f} mm")
    print(f"[TEST] Shoulder height={ARM_GEOMETRY.shoulder_height_mm:.0f} mm  "
          f"L1={ARM_GEOMETRY.L1:.0f} mm  L2={ARM_GEOMETRY.L2:.0f} mm")
    print("[TEST] ───────────────────────────────────────────────────────────")

    while True:

        # ── INIT ─────────────────────────────────────────────────────────────
        if state == "INIT":
            if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.enable_vision()
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

            # Tight-fold the elbow first so the arm COM sits over the shoulder
            # pivot, minimising the gravity moment during the lift.
            TIGHT_ELBOW_DEG = 25.0
            print(f"[TEST] SAFE_RAISE — tight-folding elbow {elbow_pos:.1f}° → "
                  f"{TIGHT_ELBOW_DEG:.1f}° (COM over shoulder).")
            elbow_pos = _move_servo(
                robot, ELBOW_CHANNEL, elbow_pos, TIGHT_ELBOW_DEG,
                ELBOW_SAFE_MIN, ELBOW_SAFE_MAX,
            )
            time.sleep(0.4)

            print(f"[TEST] SAFE_RAISE — raising shoulder {shoulder_pos:.1f}° → "
                  f"{SAFE_SHOULDER_DEG:.1f}° (direct set, no ramp).")
            for i in range(5):
                robot.set_servo(SHOULDER_CHANNEL, SAFE_SHOULDER_DEG)
                time.sleep(0.3 if i == 0 else 0.15)
            shoulder_pos = SAFE_SHOULDER_DEG

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

            state = "VISION_FIND_CUP"
            state_entry = time.monotonic()

        # ── VISION_FIND_CUP ───────────────────────────────────────────────────
        # Pan across the hardcoded cups: rank them by height (→ assign known z)
        # and find which one holds the marshmallow.
        elif state == "VISION_FIND_CUP":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print("[VISION] Scanning hardcoded cups for height + marshmallow.")
            cups = _scan_cups(robot, _build_cups())

            # One retry if no marshmallow showed up on the first pass.
            if not any(c["mallow_conf"] > 0.0 for c in cups):
                print("[VISION] No marshmallow on first pass — retrying once.")
                time.sleep(0.5)
                cups = _scan_cups(robot, _build_cups())

            ranked_ok = _assign_heights_by_rank(cups)
            if not ranked_ok:
                print("[VISION] WARNING: incomplete height scan — could not rank all "
                      "cups; will snap the target's own height instead.")

            # Per-cup scan summary (pan / detected top height / marshmallow conf).
            print("[VISION] ── scan summary ──")
            for c in cups:
                top_str = f"{c['top_mm']:+.0f}mm" if c.get("top_mm") is not None else "no cup"
                print(f"[VISION]   cup#{c['id']:.0f} pan={c['pan_deg']:+.1f}°  "
                      f"top={top_str}  mallow_conf={c['mallow_conf']:.2f}  "
                      f"img={c.get('scan_img', '—')}")

            if SCAN_ONLY:
                print("[VISION] SCAN_ONLY — 4 angle images saved to "
                      f"{VISION_OUT_DIR}/scan_cup*.jpg. Exiting without pick.")
                return

            target = max(cups, key=lambda c: c["mallow_conf"])
            if target["mallow_conf"] <= 0.0:
                print("[VISION] No marshmallow detected on any cup — aborting.")
                state = "RESTOW"
                state_entry = time.monotonic()
                continue

            img_w, img_h = robot.get_detection_image_size()
            assigned_z = target["z_mm"] if ranked_ok else _mallow_fallback_z_mm(
                target, img_w, img_h)

            pick_x = target["x_mm"]
            pick_y = target["y_mm"]
            pick_z = assigned_z + PICK_Z_LIFT_MM
            pick_bearing_deg = math.degrees(math.atan2(pick_y, pick_x))

            print(f"[VISION] TARGET cup#{target['id']:.0f}  conf={target['mallow_conf']:.2f}  "
                  f"x={pick_x:.0f} y={pick_y:.0f}  "
                  f"z={assigned_z:+.0f}{'+%.0f lift' % PICK_Z_LIFT_MM} = {pick_z:+.0f} mm  "
                  f"bearing={pick_bearing_deg:+.1f}°")
            # Save the chosen cup's annotated frame as the "correct detection" image.
            try:
                if target.get("scan_img"):
                    shutil.copyfile(target["scan_img"], os.path.join(VISION_OUT_DIR, "scan_chosen.jpg"))
                    print(f"[VISION] chosen-detection image -> {VISION_OUT_DIR}/scan_chosen.jpg")
            except Exception as exc:
                print(f"[VISION] (could not save chosen image: {exc})")
            state = "IK_COMPUTE"
            state_entry = time.monotonic()

        # ── IK_COMPUTE ────────────────────────────────────────────────────────
        elif state == "IK_COMPUTE":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print(f"[TEST] IK_COMPUTE — rotating turntable to {pick_bearing_deg:.1f}°")
            shoulder_pos, elbow_pos, gripper_pos = _safe_arm_retract(
                robot, shoulder_pos, elbow_pos, gripper_pos
            )
            _turntable_to_deg(robot, pick_bearing_deg, turntable_home_offset)
            time.sleep(0.3)

            print(f"[TEST] IK_COMPUTE — target robot-frame: "
                  f"x={pick_x:.0f}  y={pick_y:.0f}  z={pick_z:.0f} mm")

            horizontal_dist = math.hypot(pick_x, pick_y)
            reach  = horizontal_dist - ARM_GEOMETRY.shoulder_offset_mm
            height = pick_z - ARM_GEOMETRY.shoulder_height_mm
            d      = math.sqrt(reach ** 2 + height ** 2)
            print(f"[TEST] IK_COMPUTE — reach={reach:.0f} mm  "
                  f"height_from_shoulder={height:.0f} mm  d={d:.0f} mm  "
                  f"(max={ARM_GEOMETRY.L1 + ARM_GEOMETRY.L2:.0f})")

            try:
                _, pick_sh, pick_el = inverse_kinematics(
                    pick_x, pick_y, pick_z, ARM_GEOMETRY,
                )
            except OutOfReachError as exc:
                print(f"[TEST] IK_COMPUTE — UNREACHABLE: {exc}")
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
                pick_bearing_deg, shoulder_pos, elbow_pos, ARM_GEOMETRY,
            )
            print(f"[TEST] APPROACHING — commanded servos: "
                  f"shoulder={shoulder_pos:.1f}° (target {arm_shoulder_deg:.1f}°)  "
                  f"elbow={elbow_pos:.1f}° (target {arm_elbow_deg:.1f}°)")
            print(f"[TEST] APPROACHING — FK gripper position: "
                  f"x={fk_x:.0f}  y={fk_y:.0f}  z={fk_z:.0f} mm")
            print(f"[TEST] APPROACHING — FK − target offset:   "
                  f"dx={fk_x - pick_x:+.0f}  dy={fk_y - pick_y:+.0f}  dz={fk_z - pick_z:+.0f} mm")
            _hold_grip(robot, GRIPPER_OPEN_DEG, 5.0, "settle (claw around marshmallow)")
            print("[TEST] APPROACHING — at pick position, grabbing.")
            state = "PICKING"
            state_entry = time.monotonic()

        # ── PICKING ───────────────────────────────────────────────────────────
        elif state == "PICKING":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print(f"[TEST] PICKING — squeezing gripper to {GRIPPER_CLOSE_DEG:.0f}°")
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_CLOSE_DEG)
            time.sleep(0.4)
            _hold_grip(robot, GRIPPER_CLOSE_DEG, 5.0, "hold squeeze")
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
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_CLOSE_DEG)
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
            _hold_grip(robot, GRIPPER_CLOSE_DEG, 5.0, "dwell at place location")
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
    node = Node("hardcoded_cup_pick_place")
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
