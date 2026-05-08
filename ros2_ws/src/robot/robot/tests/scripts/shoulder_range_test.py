"""
shoulder_range_test.py
======================
Sweep the shoulder servo through its safe operating range.

Pass criteria:
  - Servo reaches stow, up, and all waypoints without binding or grinding.
  - No mechanical interference anywhere in the safe range.

Safety:
  - Keep elbow stowed (folded) throughout this test — do not run with elbow extended.
  - Do not command angles outside SHOULDER_SAFE_MIN / SHOULDER_SAFE_MAX.
  - Have a clear overhead path for the arm.
"""
from __future__ import annotations

import time

from robot.robot import FirmwareState, Robot

from _manipulator_config import (
    SHOULDER_CHANNEL,
    SHOULDER_SAFE_MAX,
    SHOULDER_SAFE_MIN,
    SHOULDER_STOW_DEG,
    SHOULDER_UP_DEG,
)

STEP_DEG    = 5.0
STEP_DWELL  = 0.08
HOLD_S      = 1.0


def _move_to(robot: Robot, current: float, target: float) -> float:
    direction = 1.0 if target > current else -1.0
    angle = current
    while abs(angle - target) > STEP_DEG:
        angle = max(SHOULDER_SAFE_MIN, min(SHOULDER_SAFE_MAX, angle + direction * STEP_DEG))
        robot.set_servo(SHOULDER_CHANNEL, angle)
        time.sleep(STEP_DWELL)
    robot.set_servo(SHOULDER_CHANNEL, target)
    time.sleep(STEP_DWELL)
    return target


def run(robot: Robot) -> None:
    print(f"[TEST] shoulder_range_test  channel={SHOULDER_CHANNEL.name}")
    print(f"[TEST] safe range [{SHOULDER_SAFE_MIN}°, {SHOULDER_SAFE_MAX}°]")
    print(f"[TEST] stow={SHOULDER_STOW_DEG}°  up={SHOULDER_UP_DEG}°")

    robot.set_state(FirmwareState.RUNNING)
    robot.enable_servo(SHOULDER_CHANNEL)
    time.sleep(0.1)

    # Start from stow
    print(f"[TEST] Moving to stow position ({SHOULDER_STOW_DEG}°)")
    pos = _move_to(robot, SHOULDER_STOW_DEG, SHOULDER_STOW_DEG)
    time.sleep(HOLD_S)

    # Move to raised/reach position
    print(f"[TEST] Moving to up position ({SHOULDER_UP_DEG}°)")
    pos = _move_to(robot, pos, SHOULDER_UP_DEG)
    time.sleep(HOLD_S)

    # Sweep to safe min
    print(f"[TEST] Sweeping to safe min ({SHOULDER_SAFE_MIN}°)")
    pos = _move_to(robot, pos, SHOULDER_SAFE_MIN)
    time.sleep(HOLD_S)

    # Sweep to safe max
    print(f"[TEST] Sweeping to safe max ({SHOULDER_SAFE_MAX}°)")
    pos = _move_to(robot, pos, SHOULDER_SAFE_MAX)
    time.sleep(HOLD_S)

    # Return to stow
    print(f"[TEST] Returning to stow ({SHOULDER_STOW_DEG}°)")
    _move_to(robot, pos, SHOULDER_STOW_DEG)

    robot.disable_servo(SHOULDER_CHANNEL)
    print("[TEST] PASS — shoulder swept full safe range and returned to stow")
    print("[TEST] Verify: no binding, no cable snag, smooth motion throughout")
