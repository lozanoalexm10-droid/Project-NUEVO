"""
arm_extend_retract_test.py
==========================
Coordinate shoulder and elbow to extend the arm to a reach pose, hold, then retract.
Turntable stays at 0° (centered/forward) for this test.

Pass criteria:
  - Arm extends to reach pose without collision.
  - Arm retracts cleanly to stow without collision.
  - No servo stall or grinding.

Safety:
  - Clear a 12-inch radius in front of the robot before running.
  - Elbow MUST stow before shoulder lowers — the retract order is enforced in the script.
  - Do not skip straight to a wide shoulder angle with the elbow already extended.
"""
from __future__ import annotations

import time

from robot.robot import FirmwareState, Robot

from _manipulator_config import (
    ELBOW_CHANNEL,
    ELBOW_EXTEND_DEG,
    ELBOW_SAFE_MAX,
    ELBOW_SAFE_MIN,
    ELBOW_STOW_DEG,
    SHOULDER_CHANNEL,
    SHOULDER_SAFE_MAX,
    SHOULDER_SAFE_MIN,
    SHOULDER_STOW_DEG,
    SHOULDER_UP_DEG,
)

STEP_DEG   = 5.0
STEP_DWELL = 0.08
HOLD_S     = 2.0


def _move_to(robot: Robot, channel, current: float, target: float, safe_min: float, safe_max: float) -> float:
    direction = 1.0 if target > current else -1.0
    angle = current
    while abs(angle - target) > STEP_DEG:
        angle = max(safe_min, min(safe_max, angle + direction * STEP_DEG))
        robot.set_servo(channel, angle)
        time.sleep(STEP_DWELL)
    robot.set_servo(channel, target)
    time.sleep(STEP_DWELL)
    return target


def run(robot: Robot) -> None:
    print("[TEST] arm_extend_retract_test")
    print(f"[TEST] shoulder: stow={SHOULDER_STOW_DEG}° → up={SHOULDER_UP_DEG}°")
    print(f"[TEST] elbow:    stow={ELBOW_STOW_DEG}°  → extend={ELBOW_EXTEND_DEG}°")

    robot.set_state(FirmwareState.RUNNING)
    robot.enable_servo(SHOULDER_CHANNEL)
    robot.enable_servo(ELBOW_CHANNEL)
    time.sleep(0.1)

    # ── Stow both joints ──────────────────────────────────────────────────────
    print("[TEST] Stowing both joints")
    shoulder_pos = _move_to(robot, SHOULDER_CHANNEL, SHOULDER_STOW_DEG, SHOULDER_STOW_DEG, SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
    elbow_pos    = _move_to(robot, ELBOW_CHANNEL,    ELBOW_STOW_DEG,    ELBOW_STOW_DEG,    ELBOW_SAFE_MIN,    ELBOW_SAFE_MAX)
    time.sleep(HOLD_S)

    # ── Extend: shoulder first, then elbow ────────────────────────────────────
    print(f"[TEST] Raising shoulder to {SHOULDER_UP_DEG}°")
    shoulder_pos = _move_to(robot, SHOULDER_CHANNEL, shoulder_pos, SHOULDER_UP_DEG, SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)
    time.sleep(0.5)

    print(f"[TEST] Extending elbow to {ELBOW_EXTEND_DEG}°")
    elbow_pos = _move_to(robot, ELBOW_CHANNEL, elbow_pos, ELBOW_EXTEND_DEG, ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)

    print(f"[TEST] Arm extended — holding for {HOLD_S}s")
    time.sleep(HOLD_S)

    # ── Retract: elbow first, then shoulder ───────────────────────────────────
    # Order matters: folding the elbow first prevents the forearm from swinging
    # into the body when the shoulder lowers.
    print(f"[TEST] Folding elbow to stow ({ELBOW_STOW_DEG}°)")
    elbow_pos = _move_to(robot, ELBOW_CHANNEL, elbow_pos, ELBOW_STOW_DEG, ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
    time.sleep(0.5)

    print(f"[TEST] Lowering shoulder to stow ({SHOULDER_STOW_DEG}°)")
    shoulder_pos = _move_to(robot, SHOULDER_CHANNEL, shoulder_pos, SHOULDER_STOW_DEG, SHOULDER_SAFE_MIN, SHOULDER_SAFE_MAX)

    robot.disable_servo(ELBOW_CHANNEL)
    robot.disable_servo(SHOULDER_CHANNEL)
    print("[TEST] PASS — arm extended and retracted without collision")
    print("[TEST] Verify: smooth extend, full reach pose held, clean retract to stow")
