"""
elbow_calibration_test.py
=========================
Steps the elbow servo through a fixed set of commanded angles and pauses at
each, so you can compare the commanded value against the physical angle with
a protractor / digital level. Use to verify or re-derive ELBOW_SERVO_OFFSET
and ELBOW_SERVO_SIGN in _manipulator_config.py.

Pose during the test:
  - Shoulder is held at SHOULDER_HOLD_DEG (default 105°) so the upper arm is
    nearly horizontal and the elbow joint is unloaded.
  - Gripper is closed so it doesn't snag on anything during the sweep.

What to expect with the current calibration
-------------------------------------------
elbow_geo = 270° - elbow_servo  (because ELBOW_SERVO_SIGN = -1, OFFSET = 90)
where elbow_geo = 180° means the forearm is straight (in line with upper arm),
and smaller elbow_geo means more folded toward the upper arm.

  servo  →  expected forearm geo angle
   30°   →  240°   (forearm bent ~60° "past straight" — up/back of upper arm)
   60°   →  210°   (forearm bent ~30° past straight)
   90°   →  180°   (forearm in line with upper arm — STRAIGHT)
  120°   →  150°   (forearm folded 30° toward upper arm)
  150°   →  120°   (forearm folded 60° toward upper arm)

If the physical "straight" pose lands at a servo other than 90°, the offset
is wrong by that delta. If the sweep direction is reversed, ELBOW_SERVO_SIGN
needs to flip.
"""
from __future__ import annotations

import time

from robot.robot import FirmwareState, Robot

try:
    from robot._manipulator_config import (
        ELBOW_CHANNEL,
        ELBOW_SERVO_OFFSET,
        ELBOW_SERVO_SIGN,
        GRIPPER_CHANNEL,
        GRIPPER_CLOSE_DEG,
        SHOULDER_CHANNEL,
    )
except ImportError:
    from _manipulator_config import (
        ELBOW_CHANNEL,
        ELBOW_SERVO_OFFSET,
        ELBOW_SERVO_SIGN,
        GRIPPER_CHANNEL,
        GRIPPER_CLOSE_DEG,
        SHOULDER_CHANNEL,
    )

# ── Tunable ──────────────────────────────────────────────────────────────────
# Park the shoulder pointing nearly straight up so the forearm hangs free —
# bending in either direction will not run the gripper into the cup stack /
# base plate.
SHOULDER_HOLD_DEG = 150.0
# Start at the calibrated "straight" position (servo 90 ⇒ geo 180° with the
# current ELBOW_SERVO_OFFSET=90 / SIGN=-1) and sweep a narrow band in each
# direction. If the physical "straight" lands somewhere other than servo 90°,
# only the first waypoint reveals the offset error.
WAYPOINTS_DEG     = [90.0, 75.0, 105.0, 60.0, 120.0]
DWELL_AT_WAYPOINT = 5.0     # seconds to hold each angle for measurement
STEP_DEG          = 5.0     # step size while moving between waypoints
STEP_DWELL        = 0.10    # delay between steps
# ─────────────────────────────────────────────────────────────────────────────


def _ramp_to(robot: Robot, channel, current: float, target: float) -> float:
    direction = 1.0 if target > current else -1.0
    angle = current
    while abs(angle - target) > STEP_DEG:
        angle += direction * STEP_DEG
        robot.set_servo(channel, angle)
        time.sleep(STEP_DWELL)
    robot.set_servo(channel, target)
    time.sleep(STEP_DWELL)
    return target


def _expected_geo(servo_deg: float) -> float:
    """Apply the current calibration to predict the forearm geo angle."""
    return 180.0 + ELBOW_SERVO_SIGN * (servo_deg - ELBOW_SERVO_OFFSET)


def run(robot: Robot) -> None:
    print("[ELBOW-CAL] Elbow servo calibration sweep")
    print(f"[ELBOW-CAL] current calibration: OFFSET={ELBOW_SERVO_OFFSET}  "
          f"SIGN={ELBOW_SERVO_SIGN}")
    print(f"[ELBOW-CAL] formula: elbow_geo = 180 + SIGN * (servo - OFFSET)")
    print("[ELBOW-CAL] Hold a protractor against the upper arm / forearm at each "
          "waypoint and record the measured geo angle.")

    # Firmware boots into ESTOP after launch / Ctrl+C; without clearing it,
    # set_state(RUNNING) is a silent no-op and every set_servo below is ignored.
    current_state = robot.get_state()
    if current_state in (FirmwareState.ESTOP, FirmwareState.ERROR):
        print(f"[ELBOW-CAL] Firmware in state {int(current_state)}; resetting E-STOP.")
        robot.reset_estop()
        time.sleep(0.3)
    robot.set_state(FirmwareState.RUNNING)
    time.sleep(0.3)
    print(f"[ELBOW-CAL] Firmware state now: {int(robot.get_state())}")

    robot.enable_servo(SHOULDER_CHANNEL)
    robot.enable_servo(ELBOW_CHANNEL)
    robot.enable_servo(GRIPPER_CHANNEL)
    time.sleep(0.5)

    # Park the shoulder and gripper first so the elbow swings unloaded.
    print(f"[ELBOW-CAL] Parking shoulder at {SHOULDER_HOLD_DEG:.0f}°")
    robot.set_servo(SHOULDER_CHANNEL, SHOULDER_HOLD_DEG)
    robot.set_servo(GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG)
    time.sleep(1.5)

    # Snap elbow to the first waypoint so the ramp starts from a known place.
    current = WAYPOINTS_DEG[0]
    print(f"[ELBOW-CAL] Snapping elbow to {current:.0f}°")
    robot.set_servo(ELBOW_CHANNEL, current)
    time.sleep(1.5)
    print(f"[ELBOW-CAL]   servo={current:.1f}°  expected geo={_expected_geo(current):.1f}°  "
          f"— hold {DWELL_AT_WAYPOINT:.0f} s, measure now.")
    time.sleep(DWELL_AT_WAYPOINT)

    for target in WAYPOINTS_DEG[1:]:
        print(f"[ELBOW-CAL] Ramping {current:.0f}° → {target:.0f}°")
        current = _ramp_to(robot, ELBOW_CHANNEL, current, target)
        print(f"[ELBOW-CAL]   servo={current:.1f}°  expected geo={_expected_geo(current):.1f}°  "
              f"— hold {DWELL_AT_WAYPOINT:.0f} s, measure now.")
        time.sleep(DWELL_AT_WAYPOINT)

    # Return to a neutral pose.
    print("[ELBOW-CAL] Returning to elbow 90° (geo 180° = straight forearm).")
    _ramp_to(robot, ELBOW_CHANNEL, current, 90.0)
    time.sleep(1.0)

    robot.disable_servo(ELBOW_CHANNEL)
    robot.disable_servo(SHOULDER_CHANNEL)
    robot.disable_servo(GRIPPER_CHANNEL)
    print("[ELBOW-CAL] Done. Compare your measured geo angles against the "
          "expected geo column to detect offset or sign errors.")
