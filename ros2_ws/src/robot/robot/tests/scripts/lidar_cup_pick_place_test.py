"""
lidar_cup_pick_place_test.py
=============================
Autonomous pick-place test for the competition venue. Combines:

  * the safe startup / homing flow from ``manual_ik_pick_place_test.py``
    (deterministic E-STOP reset, shoulder-first SAFE_RAISE, turntable home),
  * a new LiDAR-driven cup-stack mapper that gets bearing + radial distance
    for each of the three known cup stacks (8, 18, 24 cups), and
  * a vision pass that pans the camera across the cup bearings to pick out
    which stack has the marshmallow on top (the team will color the mallow
    purple with a Sharpie for higher detection contrast).

The ultrasonic sensor is intentionally NOT used. The forearm-mounted US
sits behind/above the cup centre line for low-tier targets and never gets
a clean line of sight — LiDAR is the right primary distance sensor for
this task. See ``MANIPULATOR_TASKS.md`` for the full coordinate-source
breakdown.

State machine
-------------
  INIT → IDLE → SAFE_RAISE → ARM_HOME → LIDAR_MAP
       → VISION_PICK_STACK → APPROACHING → PICKING
       → CARRY_TO_PLACE → PLACING → RESTOW → DONE

Nodes required: bridge (auto) + vision + robot, lidar (rplidar_c1_node).
"""
from __future__ import annotations

import math
import time

from robot.arm_kinematics import OutOfReachError, inverse_kinematics
from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    Limit,
    StepMoveType,
)
from robot.robot import FirmwareState, Robot
from robot.tests.scripts._manipulator_config import (
    ARM_GEOMETRY,
    ARM_SERVO_STEP_DEG,
    ARM_SERVO_STEP_DWELL,
    CAMERA_FORWARD_OFFSET_MM,
    CAMERA_HEIGHT_MM,
    CAMERA_HFOV_DEG,
    CAMPAN_ACCELERATION,
    CAMPAN_MAX_VELOCITY,
    CAMPAN_SETTLE_S,
    CAMPAN_STEPPER,
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
    TURNTABLE_X_IN_BODY_MM,
    campan_deg_to_steps,
    snap_to_venue_tier_mm,
    turntable_deg_to_steps,
)

# ── Safe poses (shared with manual / turntable tests) ─────────────────────────
SAFE_SHOULDER_DEG = 105.0
SAFE_ELBOW_DEG    = 90.0

# ── Place target (mirrors turntable_pick_place_test) ──────────────────────────
PLACE_TURNTABLE_DEG = -45.0
PLACE_REACH_MM      = CAMERA_FORWARD_OFFSET_MM
PLACE_Z_MM          = PLATE_Z_MM

# ── Turntable homing ──────────────────────────────────────────────────────────
HOME_NUDGE_DEG = 5.0
HOME_TIMEOUT_S = 30.0
HOME_BACKOFF   = 200

# ── LiDAR cup-stack mapper ────────────────────────────────────────────────────
# Cup stacks live on a semicircle around the camera. The mallow zone is the
# annulus 100–600 mm from the turntable axis, ±90° bearing. Filtering here
# rejects walls, the robot body, and far-field clutter.
CUP_ZONE_MIN_RADIUS_MM   = 100.0
CUP_ZONE_MAX_RADIUS_MM   = 600.0
CUP_ZONE_BEARING_LIMIT   = TURNTABLE_SCAN_ARC_DEG   # ±90°

# Greedy single-pass clustering. Any two points within this radius are merged
# into the same cluster. A solo cup is ~65 mm OD so 70 mm catches the front
# arc of one cup while keeping adjacent stacks separated.
CUP_CLUSTER_RADIUS_MM    = 70.0
CUP_MIN_POINTS_PER_STACK = 3        # reject stray returns / noise
LIDAR_SETTLE_S           = 0.5      # wait this long after entering LIDAR_MAP for fresh data

# Number of independent scans to accumulate before clustering — averages out
# the per-scan jitter on which points get returned in which sweep.
LIDAR_ACCUMULATE_SCANS   = 3
LIDAR_ACCUMULATE_DELAY_S = 0.12     # > 100 ms scan period at 10 Hz

# ── Vision pick-stack pass ────────────────────────────────────────────────────
# Settle time after panning the campan before reading detections.
PICK_CAMPAN_SETTLE_S = CAMPAN_SETTLE_S + 0.3

# Campan can't pan arbitrarily; clamp the requested bearing to its physical
# range. Cups beyond this can still be reached by the turntable later — they
# just won't be centred during this scan.
PICK_CAMPAN_MAX_DEG = 66.0


# ── Servo motion helpers (mirror manual_ik_pick_place_test) ───────────────────

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
    keep_grip: bool = False,
) -> tuple[float, float, float]:
    robot.enable_servo(SHOULDER_CHANNEL)
    robot.enable_servo(ELBOW_CHANNEL)
    robot.enable_servo(GRIPPER_CHANNEL)
    grip_target = GRIPPER_GRAB_DEG if keep_grip else GRIPPER_CLOSE_DEG
    gripper_pos  = _move_servo(robot, GRIPPER_CHANNEL,  gripper_pos,  grip_target)
    shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, SAFE_SHOULDER_DEG,
                               SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
    elbow_pos    = _move_servo(robot, ELBOW_CHANNEL,    elbow_pos,    SAFE_ELBOW_DEG,
                               ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
    return shoulder_pos, elbow_pos, gripper_pos


def _turntable_to_deg(robot: Robot, target_deg: float, home_offset: float) -> None:
    target_deg = max(TURNTABLE_MIN_DEG, min(TURNTABLE_MAX_DEG, target_deg))
    delta_deg  = abs(home_offset - target_deg)
    steps      = turntable_deg_to_steps(home_offset - target_deg)
    # Fire-and-forget — see manual_ik_pick_place_test for the rationale.
    robot.step_move(TURNTABLE_STEPPER, steps, StepMoveType.ABSOLUTE, blocking=False)
    deg_per_sec = TURNTABLE_MAX_VELOCITY * (360.0 / 3200.0)
    travel_s    = delta_deg / deg_per_sec + 0.5
    time.sleep(travel_s)


def _campan_to_deg(robot: Robot, target_deg: float) -> None:
    robot.step_move(CAMPAN_STEPPER, campan_deg_to_steps(target_deg),
                    StepMoveType.ABSOLUTE, blocking=False)
    time.sleep(1.0)


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
              f"({TURNTABLE_HOME_OFFSET_DEG:.1f}°, measured).")
        return TURNTABLE_HOME_OFFSET_DEG
    else:
        print("[HOME] WARNING: homing timed out — falling back to manual alignment (offset=0°).")
        return 0.0


# ── LiDAR cup-stack mapping ───────────────────────────────────────────────────

def _body_to_turntable(bx: float, by: float) -> tuple[float, float]:
    """Convert a body-frame point (origin = wheel midpoint) to turntable-frame
    (origin = turntable axis)."""
    return bx - TURNTABLE_X_IN_BODY_MM, by


def _filter_to_cup_zone(
    points_turntable: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    """Keep only points inside the mallow zone annulus + ±90° bearing arc."""
    kept: list[tuple[float, float]] = []
    for x, y in points_turntable:
        r = math.hypot(x, y)
        if not (CUP_ZONE_MIN_RADIUS_MM <= r <= CUP_ZONE_MAX_RADIUS_MM):
            continue
        bearing = math.degrees(math.atan2(y, x))
        if abs(bearing) > CUP_ZONE_BEARING_LIMIT:
            continue
        kept.append((x, y))
    return kept


def _cluster_points(
    points: list[tuple[float, float]],
    radius_mm: float,
) -> list[list[tuple[float, float]]]:
    """Greedy single-pass spatial clustering.

    For each point, attach it to the first existing cluster whose centroid is
    within ``radius_mm``; otherwise start a new cluster. Centroids are updated
    incrementally so later points "drift" the cluster toward its true centre.

    Good enough for the cup-stack problem because cups are ≥120 mm apart and
    the cluster radius is 70 mm — clusters never touch.
    """
    clusters: list[list[tuple[float, float]]] = []
    centroids: list[tuple[float, float]] = []
    r2 = radius_mm * radius_mm
    for p in points:
        attached = False
        for i, c in enumerate(centroids):
            dx = p[0] - c[0]
            dy = p[1] - c[1]
            if dx * dx + dy * dy <= r2:
                clusters[i].append(p)
                n = len(clusters[i])
                centroids[i] = (
                    (c[0] * (n - 1) + p[0]) / n,
                    (c[1] * (n - 1) + p[1]) / n,
                )
                attached = True
                break
        if not attached:
            clusters.append([p])
            centroids.append(p)
    return clusters


def _map_cup_stacks(robot: Robot) -> list[dict[str, float]]:
    """Accumulate a few LiDAR snapshots, cluster, return stack list.

    Each returned dict has:
      x_mm, y_mm       — centroid in turntable frame
      bearing_deg      — atan2(y, x), 0 = forward
      dist_mm          — distance from turntable axis (= hypot(x, y))
      hits             — number of lidar points in the cluster

    The list is sorted by bearing (left → right, i.e. + to -).
    """
    raw_body: list[tuple[float, float]] = []
    for _ in range(LIDAR_ACCUMULATE_SCANS):
        raw_body.extend(robot.get_lidar_obstacles_robot())
        time.sleep(LIDAR_ACCUMULATE_DELAY_S)

    raw_turntable = [_body_to_turntable(bx, by) for bx, by in raw_body]
    filtered = _filter_to_cup_zone(raw_turntable)
    clusters = _cluster_points(filtered, CUP_CLUSTER_RADIUS_MM)

    stacks: list[dict[str, float]] = []
    for cluster in clusters:
        if len(cluster) < CUP_MIN_POINTS_PER_STACK:
            continue
        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)
        # Cup OUTER face — the lidar sees the near arc of the cup. Project
        # to the cup CENTRE by adding one cup radius along the radial.
        r_arc = math.hypot(cx, cy)
        if r_arc > 1e-6:
            scale = (r_arc + CUP_DIAMETER_MM / 2.0) / r_arc
            cx *= scale
            cy *= scale
        stacks.append({
            "x_mm":        cx,
            "y_mm":        cy,
            "bearing_deg": math.degrees(math.atan2(cy, cx)),
            "dist_mm":     math.hypot(cx, cy),
            "hits":        float(len(cluster)),
        })

    stacks.sort(key=lambda s: -s["bearing_deg"])   # left first (positive bearing)
    return stacks


# ── Vision helpers ────────────────────────────────────────────────────────────

def _detection_bearing_deg(det: dict, img_w: int) -> float:
    bbox = det["bbox"]
    cx = bbox["x"] + bbox["width"] / 2.0
    return ((cx / img_w) - 0.5) * CAMERA_HFOV_DEG if img_w > 0 else 0.0


def _detection_height_mm(det: dict, img_w: int, img_h: int, dist_mm: float) -> float:
    bbox = det["bbox"]
    cy   = bbox["y"] + bbox["height"] / 2.0
    hfov_rad = math.radians(CAMERA_HFOV_DEG)
    vfov_rad = 2.0 * math.atan(math.tan(hfov_rad / 2.0) * img_h / img_w)
    elev_deg = -((cy - img_h / 2.0) / img_h) * math.degrees(vfov_rad)
    return CAMERA_HEIGHT_MM + dist_mm * math.tan(math.radians(elev_deg))


def _mallow_dist_mm(det: dict, img_w: int) -> float:
    bbox = det["bbox"]
    px_diam  = math.sqrt(max(1.0, bbox["width"] * bbox["height"]))
    focal_px = (img_w / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG / 2.0))
    return (MARSHMALLOW_DIAMETER_MM * focal_px) / px_diam


def _camera_bearing_to_stack(stack: dict) -> float:
    """Bearing from the camera lens to the stack, in camera-frame degrees.

    Camera sits at (CAMERA_FORWARD_OFFSET_MM, 0) in turntable frame. The
    turntable is parked at 0° during scanning, so the camera +x is also +x
    forward in the world. A point at (x, y) in turntable frame is therefore
    at (x − CAMERA_FORWARD_OFFSET_MM, y) relative to the camera.
    """
    dx = stack["x_mm"] - CAMERA_FORWARD_OFFSET_MM
    dy = stack["y_mm"]
    return math.degrees(math.atan2(dy, dx))


def _identify_target_stack(
    robot: Robot,
    stacks: list[dict[str, float]],
) -> dict[str, float] | None:
    """Pan the camera through each stack bearing and pick the one with the
    highest-confidence marshmallow detection. Estimate the mallow tier from
    the bbox y position and snap to the nearest measured tier height.

    Returns the chosen stack dict augmented with ``mallow_z_mm`` and
    ``mallow_conf``, or None if nothing met the confidence threshold.
    """
    best: dict[str, float] | None = None
    img_w, img_h = robot.get_detection_image_size()

    for stack in stacks:
        cam_bearing = _camera_bearing_to_stack(stack)
        clamped     = max(-PICK_CAMPAN_MAX_DEG, min(PICK_CAMPAN_MAX_DEG, cam_bearing))
        print(f"[VISION] Stack at turntable bearing {stack['bearing_deg']:+.1f}°  "
              f"→ camera bearing {cam_bearing:+.1f}°  (pan to {clamped:+.1f}°)")

        _campan_to_deg(robot, clamped)
        time.sleep(PICK_CAMPAN_SETTLE_S)

        mallow_dets = robot.get_detections(MARSHMALLOW_CLASS)
        for mallow in mallow_dets:
            conf = float(mallow["confidence"])
            if conf < MIN_CONFIDENCE_MARSHMALLOW:
                continue
            m_bearing = _detection_bearing_deg(mallow, img_w)
            # The mallow must sit on a cup we know about: within ±MALLOW_CUP_BEARING_MATCH_DEG
            # of the centre of the camera frame after the pan.
            if abs(m_bearing - (cam_bearing - clamped)) > MALLOW_CUP_BEARING_MATCH_DEG:
                continue

            # Height estimate uses the LIDAR-derived stack distance from camera,
            # then snaps to the nearest competition tier.
            cam_dist = math.hypot(
                stack["x_mm"] - CAMERA_FORWARD_OFFSET_MM,
                stack["y_mm"],
            )
            raw_z   = _detection_height_mm(mallow, img_w, img_h, cam_dist)
            snap_z  = snap_to_venue_tier_mm(raw_z)

            print(f"[VISION]   mallow conf={conf:.2f}  "
                  f"raw_z={raw_z:.0f} mm → tier z={snap_z:+.0f} mm")

            if best is None or conf > best.get("mallow_conf", 0.0):
                best = dict(stack)
                best["mallow_z_mm"] = snap_z
                best["mallow_conf"] = conf

    _campan_to_deg(robot, 0.0)
    return best


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

    pick_x = pick_y = pick_z = 0.0
    target_stack: dict[str, float] | None = None
    lidar_stacks: list[dict[str, float]] = []

    place_rad = math.radians(PLACE_TURNTABLE_DEG)
    place_x   = PLACE_REACH_MM * math.cos(place_rad)
    place_y   = PLACE_REACH_MM * math.sin(place_rad)
    place_z   = PLACE_Z_MM

    print("[TEST] ── LiDAR Cup Pick-Place ────────────────────────────────")
    print(f"[TEST] Lidar mount in body frame: x={LIDAR_MOUNT_X_MM:.0f} mm, "
          f"y={LIDAR_MOUNT_Y_MM:.0f} mm")
    print(f"[TEST] Turntable axis in body frame: x={TURNTABLE_X_IN_BODY_MM:.0f} mm "
          f"(verify on robot — see _manipulator_config.py)")
    print(f"[TEST] Place target: turntable={PLACE_TURNTABLE_DEG:.1f}°  "
          f"x={place_x:.0f} y={place_y:.0f} z={place_z:.0f} mm")
    print(f"[TEST] Floor z={GROUND_Z_MM:.0f} mm  scan arc=±{TURNTABLE_SCAN_ARC_DEG:.0f}°")
    print( "[TEST] ───────────────────────────────────────────────────────────")

    while True:

        # ── INIT ─────────────────────────────────────────────────────────────
        if state == "INIT":
            if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.enable_vision()
            robot.enable_lidar()
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
            _campan_to_deg(robot, 0.0)
            _turntable_to_deg(robot, 0.0, turntable_home_offset)
            time.sleep(0.5)

            state = "LIDAR_MAP"
            state_entry = time.monotonic()

        # ── LIDAR_MAP ─────────────────────────────────────────────────────────
        # Read the lidar, cluster cup stacks, store the list.
        elif state == "LIDAR_MAP":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print(f"[LIDAR] Settling for {LIDAR_SETTLE_S:.1f}s, then mapping cup stacks.")
            time.sleep(LIDAR_SETTLE_S)
            stacks = _map_cup_stacks(robot)
            print(f"[LIDAR] Found {len(stacks)} candidate stack(s):")
            for i, s in enumerate(stacks):
                print(f"[LIDAR]   #{i}: bearing={s['bearing_deg']:+6.1f}°  "
                      f"dist={s['dist_mm']:6.0f} mm  "
                      f"({s['hits']:.0f} lidar pts)")

            if not stacks:
                print("[LIDAR] No cup stacks found — aborting.")
                state = "RESTOW"
                state_entry = time.monotonic()
            else:
                lidar_stacks = stacks
                state = "VISION_PICK_STACK"
                state_entry = time.monotonic()

        # ── VISION_PICK_STACK ─────────────────────────────────────────────────
        # Pan the camera through each LIDAR-detected stack, look for the
        # purple marshmallow, pick the highest-confidence one.
        elif state == "VISION_PICK_STACK":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print("[VISION] Scanning each LIDAR stack for the marshmallow.")
            scan_deadline = time.monotonic() + SCAN_TIMEOUT_S
            target_stack  = _identify_target_stack(robot, lidar_stacks)
            if target_stack is None and time.monotonic() < scan_deadline:
                # Single retry — vision detections can be empty for one frame.
                print("[VISION] First pass empty; retrying once.")
                time.sleep(0.5)
                target_stack = _identify_target_stack(robot, lidar_stacks)

            if target_stack is None:
                print("[VISION] No marshmallow detected on any stack — aborting.")
                state = "RESTOW"
                state_entry = time.monotonic()
            else:
                pick_x = target_stack["x_mm"]
                pick_y = target_stack["y_mm"]
                pick_z = target_stack["mallow_z_mm"]
                print(f"[VISION] Target stack bearing={target_stack['bearing_deg']:+.1f}°  "
                      f"x={pick_x:.0f} y={pick_y:.0f} z={pick_z:+.0f} mm  "
                      f"conf={target_stack['mallow_conf']:.2f}")
                state = "APPROACHING"
                state_entry = time.monotonic()

        # ── APPROACHING ───────────────────────────────────────────────────────
        elif state == "APPROACHING":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print(f"[TEST] APPROACHING — IK to ({pick_x:.0f}, {pick_y:.0f}, {pick_z:+.0f}) mm")
            try:
                tt_deg, sh_servo, el_servo = inverse_kinematics(
                    pick_x, pick_y, pick_z, ARM_GEOMETRY,
                )
            except OutOfReachError as exc:
                print(f"[TEST] APPROACHING — pick UNREACHABLE: {exc}")
                state = "RESTOW"
                state_entry = time.monotonic()
                continue

            arm_shoulder_deg = sh_servo
            arm_elbow_deg    = el_servo
            print(f"[TEST] APPROACHING — turntable={tt_deg:+.1f}°  "
                  f"shoulder={sh_servo:.1f}°  elbow={el_servo:.1f}°")

            # Safe-retract before rotating turntable to the target bearing.
            shoulder_pos, elbow_pos, gripper_pos = _safe_arm_retract(
                robot, shoulder_pos, elbow_pos, gripper_pos
            )
            _turntable_to_deg(robot, tt_deg, turntable_home_offset)
            time.sleep(0.3)

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
            state = "CARRY_TO_PLACE"
            state_entry = time.monotonic()

        # ── CARRY_TO_PLACE ────────────────────────────────────────────────────
        elif state == "CARRY_TO_PLACE":
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return

            print(f"[TEST] CARRY_TO_PLACE — retracting and rotating to "
                  f"{PLACE_TURNTABLE_DEG:.1f}°")
            shoulder_pos, elbow_pos, gripper_pos = _safe_arm_retract(
                robot, shoulder_pos, elbow_pos, gripper_pos, keep_grip=True,
            )
            _turntable_to_deg(robot, PLACE_TURNTABLE_DEG, turntable_home_offset)

            try:
                _, place_sh, place_el = inverse_kinematics(place_x, place_y, place_z, ARM_GEOMETRY)
            except OutOfReachError as exc:
                print(f"[TEST] CARRY_TO_PLACE — place IK failed: {exc}")
                state = "RESTOW"
                state_entry = time.monotonic()
                continue

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
