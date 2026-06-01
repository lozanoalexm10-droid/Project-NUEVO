"""
marshmallow_pick.py — actually try to pick up a marshmallow.

Pipeline:
  1. enable_vision + enable_ultrasonic, wait for both active
  2. ensure firmware RUNNING (already true on this system, but checked anyway)
  3. enable stepper / servos, move arm to safe stow
  4. find best marshmallow detection paired with a red_cup within
     MALLOW_CUP_BEARING_MATCH_DEG (10°) — suppresses the wall false positive
  5. rotate turntable to point the arm at the marshmallow (atan2 of the
     vision bearing — geometry-agnostic, no IK needed)
  6. move shoulder/elbow to ARM_SEARCH_* preset poses
  7. open gripper
  8. descend by stepping elbow toward ELBOW_EXTEND_DEG one ARM_SERVO_STEP_DEG
     at a time, polling the ultrasonic. Stop when distance < PICK_THRESHOLD_MM
     OR safe-limit reached OR APPROACH_TIMEOUT_S elapsed.
  9. close gripper (grab)
 10. lift to ARM_CARRY_* pose

Cleanup runs in finally: retract (elbow-then-shoulder), return turntable to 0°,
leave gripper closed (holds marshmallow), leave servos enabled (do not drop).

CRITICAL: the arm geometry constants in _manipulator_config.py are still
``TODO: measure`` placeholders, so this script does NOT use inverse_kinematics()
to derive servo angles. Only the turntable angle is derived from the vision
bearing — that is geometry-agnostic. Arm motion uses preset poses + ultrasonic
feedback.

Prerequisites the runtime checks for:
  - bridge node running (firmware service /set_firmware_state available)
  - vision node publishing /vision/detections
  - qwiic_ultrasonic publishing /ultrasonic_range
  - turntable previously homed (uses TURNTABLE_HOME_OFFSET_DEG from config)

Run via main.py dispatch OR directly:
    python3 /ros2_ws/src/robot/robot/tests/scripts/marshmallow_pick.py
"""
from __future__ import annotations

import time

from robot.arm_kinematics import OutOfReachError  # noqa: F401  (only here to surface ImportError early)
from robot.hardware_map import StepMoveType
from robot.robot import FirmwareState, Robot

try:
    from robot.tests.scripts._manipulator_config import (
        ARM_CARRY_ELBOW_DEG,
        ARM_CARRY_SHOULDER_DEG,
        ARM_SEARCH_ELBOW_DEG,
        ARM_SEARCH_SHOULDER_DEG,
        ARM_SERVO_STEP_DEG,
        ARM_SERVO_STEP_DWELL,
        CAMERA_HFOV_DEG,
        CUP_CLASS,
        ELBOW_CHANNEL,
        ELBOW_EXTEND_DEG,
        ELBOW_SAFE_MAX,
        ELBOW_SAFE_MIN,
        ELBOW_STOW_DEG,
        GRIPPER_CHANNEL,
        GRIPPER_CLOSE_DEG,
        GRIPPER_GRAB_DEG,
        GRIPPER_OPEN_DEG,
        MALLOW_CUP_BEARING_MATCH_DEG,
        MARSHMALLOW_CLASS,
        MIN_CONFIDENCE_CUP,
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
        turntable_deg_to_steps,
    )
except ImportError:  # pragma: no cover — direct-script fallback
    from _manipulator_config import (  # type: ignore
        ARM_CARRY_ELBOW_DEG,
        ARM_CARRY_SHOULDER_DEG,
        ARM_SEARCH_ELBOW_DEG,
        ARM_SEARCH_SHOULDER_DEG,
        ARM_SERVO_STEP_DEG,
        ARM_SERVO_STEP_DWELL,
        CAMERA_HFOV_DEG,
        CUP_CLASS,
        ELBOW_CHANNEL,
        ELBOW_EXTEND_DEG,
        ELBOW_SAFE_MAX,
        ELBOW_SAFE_MIN,
        ELBOW_STOW_DEG,
        GRIPPER_CHANNEL,
        GRIPPER_CLOSE_DEG,
        GRIPPER_GRAB_DEG,
        GRIPPER_OPEN_DEG,
        MALLOW_CUP_BEARING_MATCH_DEG,
        MARSHMALLOW_CLASS,
        MIN_CONFIDENCE_CUP,
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
        turntable_deg_to_steps,
    )

# ── Tunables ──────────────────────────────────────────────────────────────────
CAMPAN_DEG = 0.0                 # not driven by this script
MALLOW_MIN_CONFIDENCE = 0.80     # higher than config's 0.60 to reject false positives
PICK_THRESHOLD_MM = 50.0         # close gripper when ultrasonic reads under this
APPROACH_TIMEOUT_S = 10.0        # wall-clock cap on the descent loop
VISION_WAIT_S = 10.0
ULTRASONIC_WAIT_S = 10.0
STEPPER_MOVE_TIMEOUT_S = 15.0

# Runtime-overridable settings. main() fills these from argparse so you can
# iterate on the arm pose from the CLI while watching the live stream, e.g.:
#   python3 marshmallow_pick.py --probe --shoulder 80 --elbow 60 --turntable 0
from types import SimpleNamespace  # noqa: E402

S = SimpleNamespace(
    shoulder=ARM_SEARCH_SHOULDER_DEG,   # search-pose shoulder angle (deg)
    elbow=ARM_SEARCH_ELBOW_DEG,         # search-pose (start) elbow angle (deg)
    elbow_min=ELBOW_SAFE_MIN,           # how far down the descent may drive the elbow
    pick_threshold=PICK_THRESHOLD_MM,   # ultrasonic mm to trigger the grab
    approach_timeout=APPROACH_TIMEOUT_S,
    turntable=None,    # None → aim via vision; a float → use this angle directly
    probe=False,       # probe mode: reach search pose, stream ultrasonic, no descent/grab
    probe_s=10.0,      # seconds to stream ultrasonic in probe mode
    no_grab=False,     # run the full descent but do NOT close the gripper (dry run)
    sweep=None,        # 'shoulder' or 'elbow': sweep that joint and stream ultrasonic
    sweep_from=None,   # sweep start angle (deg); default = joint safe min
    sweep_to=None,     # sweep end angle (deg);   default = joint safe max
    grab=False,        # go straight to the search pose and grab (no ultrasonic descent)
    grip=GRIPPER_GRAB_DEG,  # gripper close angle for the grab (higher = tighter)
)


# ── Helpers (copies of the manipulator_demo idioms) ───────────────────────────
def _move_servo(robot: Robot, channel: int, current: float, target: float,
                safe_min: float = 10.0, safe_max: float = 170.0) -> float:
    """Smooth-step a servo from current to target with ARM_SERVO_STEP_DEG / dwell."""
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


def _bbox_bearing_deg(bbox: dict, img_w: int) -> float:
    """Horizontal bearing (deg, + = right) of a detection's bbox centre."""
    cx = float(bbox["x"]) + float(bbox["width"]) / 2.0
    return ((cx - img_w / 2.0) / img_w) * CAMERA_HFOV_DEG


def _turntable_to_deg(robot: Robot, target_deg: float) -> bool:
    """Drive the turntable to ``target_deg`` (user frame), blocking until idle.

    Mirrors manipulator_demo._turntable_to_deg formula exactly.
    """
    target_deg = max(TURNTABLE_MIN_DEG, min(TURNTABLE_MAX_DEG, target_deg))
    steps = turntable_deg_to_steps(TURNTABLE_HOME_OFFSET_DEG - target_deg)
    return robot.step_move(
        TURNTABLE_STEPPER, steps, StepMoveType.ABSOLUTE,
        blocking=True, timeout=STEPPER_MOVE_TIMEOUT_S,
    )


def run(robot: Robot) -> None:  # noqa: C901 — the pipeline is intentionally linear
    print("[PICK] marshmallow_pick — WILL physically move the turntable and arm.")

    # Track our commanded servo positions so smooth-stepping works.
    # Initial values are an assumption; if the arm is not at stow before this
    # script runs, the first _move_servo() will issue an immediate set_servo()
    # to the stow target and the servo will move to it at its own rate.
    shoulder_pos = SHOULDER_STOW_DEG
    elbow_pos = ELBOW_STOW_DEG
    gripper_pos = GRIPPER_CLOSE_DEG

    def _retract_and_recenter() -> None:
        """Cleanup: get the arm to a safe carry pose and recenter the turntable."""
        nonlocal shoulder_pos, elbow_pos, gripper_pos
        try:
            print("[PICK] Cleanup: retract → carry pose, turntable → 0°")
            elbow_pos = _move_servo(
                robot, ELBOW_CHANNEL, elbow_pos, ARM_CARRY_ELBOW_DEG,
                ELBOW_SAFE_MIN, ELBOW_SAFE_MAX,
            )
            shoulder_pos = _move_servo(
                robot, SHOULDER_CHANNEL, shoulder_pos, ARM_CARRY_SHOULDER_DEG,
                SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX,
            )
            _turntable_to_deg(robot, 0.0)
        except Exception as exc:
            print(f"[PICK] Cleanup error (continuing): {exc}")

    try:
        # ── 1. Firmware state ─────────────────────────────────────────────────
        current_state = robot.get_state()
        if current_state in (int(FirmwareState.ESTOP), int(FirmwareState.ERROR)):
            print("[PICK] Firmware in ESTOP/ERROR — calling reset_estop()...")
            try:
                robot.reset_estop()
            except Exception as exc:
                print(f"[PICK] reset_estop failed: {exc}")
        if current_state != int(FirmwareState.RUNNING):
            if not robot.set_state(FirmwareState.RUNNING, timeout=15.0):
                # Bridge's inner firmware-handshake timeout is ~2s, which can
                # fire under load even though the firmware then transitions a
                # moment later. Re-check the actual state before giving up.
                time.sleep(1.0)
                if robot.get_state() == int(FirmwareState.RUNNING):
                    print("[PICK] set_state returned False but firmware reached RUNNING — continuing.")
                else:
                    print("[PICK] FAIL: set_state(RUNNING) returned False and firmware did not transition.")
                    return
        else:
            print("[PICK] Firmware already RUNNING — skipping set_state.")

        # ── 2. Sensors ────────────────────────────────────────────────────────
        robot.enable_vision()
        robot.enable_ultrasonic()

        # Vision is only needed when aiming via the camera. With a manual
        # --turntable override we skip the vision requirement entirely.
        if S.turntable is None:
            vision_deadline = time.monotonic() + VISION_WAIT_S
            while time.monotonic() < vision_deadline:
                if robot.is_vision_active(timeout_s=1.0):
                    break
                time.sleep(0.2)
            else:
                print("[PICK] FAIL: vision not active within timeout. Is vision_node publishing /vision/detections?")
                return

        # The ultrasonic descent needs the sensor; --grab goes to a known pose and
        # closes without it, so skip the requirement in that mode.
        if not S.grab:
            us_deadline = time.monotonic() + ULTRASONIC_WAIT_S
            while time.monotonic() < us_deadline:
                if robot.is_ultrasonic_active(timeout_s=0.5):
                    break
                time.sleep(0.2)
            else:
                print("[PICK] FAIL: ultrasonic not active within timeout. Is qwiic_ultrasonic publishing /ultrasonic_range?")
                return

        print("[PICK] vision + ultrasonic active.")

        # ── 3. Actuators ──────────────────────────────────────────────────────
        robot.step_enable(TURNTABLE_STEPPER)
        try:
            robot.step_set_config(TURNTABLE_STEPPER, TURNTABLE_MAX_VELOCITY, TURNTABLE_ACCELERATION)
        except Exception as exc:
            print(f"[PICK] step_set_config warning (using firmware defaults): {exc}")
        robot.enable_servo(SHOULDER_CHANNEL)
        robot.enable_servo(ELBOW_CHANNEL)
        robot.enable_servo(GRIPPER_CHANNEL)
        time.sleep(0.2)

        # ── 4. Safe stow (elbow then shoulder; gripper closed → opened next) ──
        print("[PICK] Settling to stow pose...")
        elbow_pos = _move_servo(robot, ELBOW_CHANNEL, elbow_pos, ELBOW_STOW_DEG,
                                ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
        shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, SHOULDER_STOW_DEG,
                                   SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
        # Make sure the servos actually reach their commanded positions.
        time.sleep(0.6)

        # ── 5/6. Targeting ────────────────────────────────────────────────────
        if S.turntable is not None:
            bearing_deg = None  # not used; manual angle below
            turntable_target_deg = S.turntable
            print(f"[PICK] Manual turntable override → {turntable_target_deg:+.1f}° (skipping vision targeting).")
            # Skip detection; jump straight to the rotate step.
            _do_targeting = False
        else:
            _do_targeting = True

        if _do_targeting:
            img_w, img_h = robot.get_detection_image_size()
            if img_w <= 0 or img_h <= 0:
                print(f"[PICK] FAIL: invalid image size {img_w}x{img_h}")
                return

            cup_dets_raw = robot.get_detections(class_name=CUP_CLASS)
            mallow_dets_raw = robot.get_detections(class_name=MARSHMALLOW_CLASS)
            cups = [c for c in cup_dets_raw if float(c["confidence"]) >= MIN_CONFIDENCE_CUP]
            mallows = [m for m in mallow_dets_raw if float(m["confidence"]) >= MALLOW_MIN_CONFIDENCE]
            print(f"[PICK] {len(cups)} red_cup(s) ≥{MIN_CONFIDENCE_CUP:.2f}, "
                  f"{len(mallows)} marshmallow(s) ≥{MALLOW_MIN_CONFIDENCE:.2f} in view.")

            candidates: list[tuple[float, float, dict]] = []
            for m in mallows:
                mb = _bbox_bearing_deg(m["bbox"], img_w)
                paired = any(
                    abs(_bbox_bearing_deg(c["bbox"], img_w) - mb) <= MALLOW_CUP_BEARING_MATCH_DEG
                    for c in cups
                )
                if paired:
                    candidates.append((float(m["confidence"]), mb, m))

            if candidates:
                best_conf, bearing_deg, _best = max(candidates, key=lambda t: t[0])
                print(f"[PICK] Target marshmallow conf={best_conf:.2f} bearing={bearing_deg:+.1f}°")
            else:
                # Marshmallow detection is unreliable (it tends to misfire on light
                # backgrounds and miss the real one when it's centred on a cup).
                # Fall back to targeting the highest-confidence red_cup: the
                # ultrasonic will decide when to close the gripper regardless of
                # what's actually on top of it.
                if not cups:
                    print("[PICK] No marshmallow paired with a cup AND no cup either — aborting.")
                    return
                best_cup = max(cups, key=lambda c: float(c["confidence"]))
                bearing_deg = _bbox_bearing_deg(best_cup["bbox"], img_w)
                print(f"[PICK] No paired marshmallow; falling back to red_cup "
                      f"conf={float(best_cup['confidence']):.2f} bearing={bearing_deg:+.1f}° "
                      f"— ultrasonic will trigger the grab.")

            # Geometry-agnostic turntable target from the vision bearing (no IK).
            turntable_target_deg = -(CAMPAN_DEG + bearing_deg)

        # ── Rotate turntable to the chosen target ─────────────────────────────
        turntable_target_deg = max(TURNTABLE_MIN_DEG, min(TURNTABLE_MAX_DEG, turntable_target_deg))
        print(f"[PICK] Rotating turntable → {turntable_target_deg:+.1f}°...")
        if _turntable_to_deg(robot, turntable_target_deg) is False:
            # blocking step_move waits for motion to START; a zero-step / already-there
            # move never starts and "times out". That is not a stall — continue.
            print("[PICK] (turntable move reported no motion — likely already at target; continuing.)")

        # ── 7. Search pose ────────────────────────────────────────────────────
        print(f"[PICK] Search pose: shoulder={S.shoulder}° elbow={S.elbow}°")
        shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, S.shoulder,
                                   SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
        elbow_pos = _move_servo(robot, ELBOW_CHANNEL, elbow_pos, S.elbow,
                                ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
        time.sleep(0.3)

        # ── SWEEP MODE: step one joint across a range, stream ultrasonic ──────
        if S.sweep in ("shoulder", "elbow"):
            if S.sweep == "shoulder":
                channel, lo_hard, hi_hard, cur = SHOULDER_CHANNEL, SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX, shoulder_pos
            else:
                channel, lo_hard, hi_hard, cur = ELBOW_CHANNEL, ELBOW_SAFE_MIN, ELBOW_SAFE_MAX, elbow_pos
            a = S.sweep_from if S.sweep_from is not None else lo_hard
            b = S.sweep_to if S.sweep_to is not None else hi_hard
            a = max(lo_hard, min(hi_hard, a))
            b = max(lo_hard, min(hi_hard, b))
            step = ARM_SERVO_STEP_DEG if b >= a else -ARM_SERVO_STEP_DEG
            print(f"[PICK] SWEEP {S.sweep} from {a:.0f}° to {b:.0f}° (step {step:+.0f}°), "
                  f"streaming ultrasonic. Lowest reading = best aim.")
            # Move to the sweep start first.
            cur = _move_servo(robot, channel, cur, a, lo_hard, hi_hard)
            best = (None, None)  # (distance_mm, angle)
            angle = a
            while (step > 0 and angle <= b) or (step < 0 and angle >= b):
                robot.set_servo(channel, angle)
                if S.sweep == "shoulder":
                    shoulder_pos = angle
                else:
                    elbow_pos = angle
                time.sleep(0.25)  # settle before reading
                d_mm = robot.get_ultrasonic_mm(timeout_s=0.5)
                d_str = f"{d_mm:.0f}mm" if d_mm is not None else "no reading"
                print(f"[SWEEP]   {S.sweep}={angle:5.1f}°  ultrasonic={d_str}")
                if d_mm is not None and (best[0] is None or d_mm < best[0]):
                    best = (d_mm, angle)
                angle += step
            if best[0] is not None:
                print(f"[PICK] SWEEP best: {S.sweep}={best[1]:.1f}° at {best[0]:.0f}mm. "
                      f"Re-run with --{S.sweep} {best[1]:.0f} (no --sweep) to use it.")
            else:
                print("[PICK] SWEEP saw no valid readings — the sensor never pointed at anything in range.")
            return

        # ── PROBE MODE: stream ultrasonic, no descent/grab (for aiming) ───────
        if S.probe:
            print(f"[PICK] PROBE: holding search pose, streaming ultrasonic for {S.probe_s:.0f}s "
                  f"(no descent). Adjust --shoulder/--elbow/--turntable until the reading is the "
                  f"distance to the cup top.")
            probe_deadline = time.monotonic() + S.probe_s
            while time.monotonic() < probe_deadline:
                d_mm = robot.get_ultrasonic_mm(timeout_s=0.5)
                d_str = f"{d_mm:.0f}mm" if d_mm is not None else "no reading (out of range)"
                print(f"[PROBE]   ultrasonic={d_str}")
                time.sleep(0.5)
            print("[PICK] PROBE done — retracting.")
            return

        # ── 8. Open gripper ───────────────────────────────────────────────────
        print(f"[PICK] Opening gripper → {GRIPPER_OPEN_DEG}°")
        gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, GRIPPER_OPEN_DEG, 0.0, 180.0)

        # ── GRAB MODE: jaws are already around the target at this pose; close ──
        if S.grab:
            time.sleep(0.4)
            print(f"[PICK] GRAB: closing gripper → {S.grip}° (no ultrasonic descent).")
            gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, S.grip, 0.0, 180.0)
            time.sleep(0.4)
            print(f"[PICK] Lifting to carry: elbow={ARM_CARRY_ELBOW_DEG}°, shoulder={ARM_CARRY_SHOULDER_DEG}°")
            elbow_pos = _move_servo(robot, ELBOW_CHANNEL, elbow_pos, ARM_CARRY_ELBOW_DEG,
                                    ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
            shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, ARM_CARRY_SHOULDER_DEG,
                                       SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
            print("[PICK] GRAB complete — marshmallow gripped, arm at carry pose.")
            return

        # ── 9. Approach with ultrasonic feedback ──────────────────────────────
        # "Extending"/descending means driving the elbow toward S.elbow_min.
        # Derive the step sign robustly so this works regardless of pose direction.
        elbow_floor = max(ELBOW_SAFE_MIN, min(ELBOW_SAFE_MAX, S.elbow_min))
        extend_step = -ARM_SERVO_STEP_DEG if elbow_floor < S.elbow else +ARM_SERVO_STEP_DEG
        # Bound the descent so the elbow only travels from the search angle toward the floor.
        descent_min = elbow_floor if extend_step < 0 else ELBOW_SAFE_MIN
        descent_max = S.elbow if extend_step < 0 else elbow_floor
        print(f"[PICK] Approach loop: elbow step={extend_step:+.1f}°/iter, floor={elbow_floor:.0f}°, "
              f"pick threshold={S.pick_threshold:.0f}mm, timeout={S.approach_timeout:.0f}s")
        approach_deadline = time.monotonic() + S.approach_timeout
        picked = False
        while time.monotonic() < approach_deadline:
            d_mm = robot.get_ultrasonic_mm(timeout_s=0.5)
            if d_mm is not None and d_mm < S.pick_threshold:
                print(f"[PICK]   ultrasonic={d_mm:.0f}mm < {S.pick_threshold:.0f}mm — stopping descent.")
                picked = True
                break
            next_elbow = elbow_pos + extend_step
            clamped = max(descent_min, min(descent_max, next_elbow))
            if abs(clamped - elbow_pos) < 1e-6:
                d_str = f"{d_mm:.0f}mm" if d_mm is not None else "no reading"
                print(f"[PICK]   elbow at descent limit ({elbow_pos:.1f}°); ultrasonic={d_str}. Aborting approach.")
                break
            robot.set_servo(ELBOW_CHANNEL, clamped)
            elbow_pos = clamped
            d_str = f"{d_mm:.0f}mm" if d_mm is not None else "no reading"
            print(f"[PICK]   elbow={elbow_pos:.1f}°  ultrasonic={d_str}")
            time.sleep(ARM_SERVO_STEP_DWELL)
        else:
            print("[PICK] Approach loop hit approach timeout without trigger.")

        if not picked:
            print("[PICK] Skipping grab — descent did not detect target.")
            return

        if S.no_grab:
            print("[PICK] --no-grab set: target detected but NOT closing gripper (dry run).")
            return

        # ── 10. Close gripper ─────────────────────────────────────────────────
        print(f"[PICK] Closing gripper → {S.grip}°")
        gripper_pos = _move_servo(robot, GRIPPER_CHANNEL, gripper_pos, S.grip, 0.0, 180.0)
        time.sleep(0.3)

        # ── 11. Lift to carry pose (elbow first, then shoulder) ───────────────
        print(f"[PICK] Lifting to carry pose: elbow={ARM_CARRY_ELBOW_DEG}°, shoulder={ARM_CARRY_SHOULDER_DEG}°")
        elbow_pos = _move_servo(robot, ELBOW_CHANNEL, elbow_pos, ARM_CARRY_ELBOW_DEG,
                                ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
        shoulder_pos = _move_servo(robot, SHOULDER_CHANNEL, shoulder_pos, ARM_CARRY_SHOULDER_DEG,
                                   SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
        time.sleep(0.3)

        print("[PICK] Pick sequence complete — marshmallow gripped, arm at carry pose.")
    finally:
        _retract_and_recenter()
        # Leave the firmware in RUNNING state (it was RUNNING before we started)
        # and the servos enabled — we do NOT want to drop the marshmallow.


def _parse_args(argv=None):
    import argparse

    p = argparse.ArgumentParser(
        description="Pick a marshmallow off a red cup. Tune the arm pose from the CLI "
                    "while watching the live stream at http://<pi>:8091/",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--shoulder", type=float, default=S.shoulder,
                   help="Search-pose shoulder servo angle (deg).")
    p.add_argument("--elbow", type=float, default=S.elbow,
                   help="Search-pose (start) elbow servo angle (deg).")
    p.add_argument("--elbow-min", type=float, default=S.elbow_min,
                   help="Lowest elbow angle the descent may reach (deg). Clamped to ELBOW_SAFE_MIN.")
    p.add_argument("--pick-threshold", type=float, default=S.pick_threshold,
                   help="Ultrasonic distance (mm) that triggers the grab.")
    p.add_argument("--approach-timeout", type=float, default=S.approach_timeout,
                   help="Wall-clock cap (s) on the descent loop.")
    p.add_argument("--turntable", type=float, default=None,
                   help="Manual turntable angle (deg). If set, skips vision targeting.")
    p.add_argument("--probe", action="store_true",
                   help="Aim mode: rotate + move to search pose, then stream ultrasonic "
                        "without descending or grabbing. Use to point the sensor at the cup.")
    p.add_argument("--probe-s", type=float, default=S.probe_s,
                   help="Seconds to stream ultrasonic in --probe mode.")
    p.add_argument("--no-grab", action="store_true",
                   help="Run the full descent but do NOT close the gripper (dry run).")
    p.add_argument("--sweep", choices=("shoulder", "elbow"), default=None,
                   help="Sweep this joint across a range while streaming ultrasonic; "
                        "reports the angle with the closest reading (best aim).")
    p.add_argument("--sweep-from", type=float, default=None,
                   help="Sweep start angle (deg). Default: joint safe minimum.")
    p.add_argument("--sweep-to", type=float, default=None,
                   help="Sweep end angle (deg). Default: joint safe maximum.")
    p.add_argument("--grab", action="store_true",
                   help="Go straight to the --shoulder/--elbow pose and close the gripper "
                        "(open→grab→lift), with NO ultrasonic descent. Use once the pose is known.")
    p.add_argument("--grip", type=float, default=GRIPPER_GRAB_DEG,
                   help="Gripper close angle for the grab (deg; higher = tighter hold).")
    return p.parse_args(argv)


def main() -> None:
    """Standalone entry point: build a Robot the way robot_node does, then run()."""
    import threading

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    args = _parse_args()
    S.shoulder = args.shoulder
    S.elbow = args.elbow
    S.elbow_min = args.elbow_min
    S.pick_threshold = args.pick_threshold
    S.approach_timeout = args.approach_timeout
    S.turntable = args.turntable
    S.probe = args.probe
    S.probe_s = args.probe_s
    S.no_grab = args.no_grab
    S.sweep = args.sweep
    S.sweep_from = args.sweep_from
    S.sweep_to = args.sweep_to
    S.grab = args.grab
    S.grip = args.grip

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("marshmallow_pick")
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
        # robot.shutdown() transitions the firmware to IDLE, which cuts power to
        # the servos and RELEASES the gripper. In --grab mode we want to keep
        # holding the marshmallow, so we deliberately skip shutdown and leave the
        # firmware RUNNING with the gripper enabled. The bridge keeps the firmware
        # alive after this script exits, so the grip is maintained.
        if not S.grab:
            try:
                robot.shutdown()
            except Exception:
                pass
        else:
            print("[PICK] --grab: leaving firmware RUNNING and gripper enabled to hold the marshmallow.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
