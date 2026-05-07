"""
elbow_range_test.py
===================
Sweep the elbow servo through its safe operating range.

Pass criteria:
  - Servo reaches stow, extend, and all waypoints without binding.
  - No forearm collision with the robot body or cables.

Safety:
  - Hold the shoulder at SHOULDER_STOW_DEG (90°) during this test.
    A raised shoulder combined with a swept elbow may cause the forearm to strike the frame.
  - Do not command angles outside ELBOW_SAFE_MIN / ELBOW_SAFE_MAX.
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
    SHOULDER_STOW_DEG,
)

STEP_DEG   = 5.0
STEP_DWELL = 0.08
HOLD_S     = 1.0


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
    print(f"[TEST] elbow_range_test  channel={ELBOW_CHANNEL.name}")
    print(f"[TEST] safe range [{ELBOW_SAFE_MIN}°, {ELBOW_SAFE_MAX}°]")

    robot.set_state(FirmwareState.RUNNING)

    # Pin shoulder to stow first
    robot.enable_servo(SHOULDER_CHANNEL)
    robot.set_servo(SHOULDER_CHANNEL, SHOULDER_STOW_DEG)
    time.sleep(0.5)

    robot.enable_servo(ELBOW_CHANNEL)
    time.sleep(0.1)

    print(f"[TEST] Moving to stow ({ELBOW_STOW_DEG}°)")
    pos = _move_to(robot, ELBOW_CHANNEL, ELBOW_STOW_DEG, ELBOW_STOW_DEG, ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
    time.sleep(HOLD_S)

    print(f"[TEST] Extending to {ELBOW_EXTEND_DEG}°")
    pos = _move_to(robot, ELBOW_CHANNEL, pos, ELBOW_EXTEND_DEG, ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
    time.sleep(HOLD_S)

    print(f"[TEST] Sweeping to safe min ({ELBOW_SAFE_MIN}°)")
    pos = _move_to(robot, ELBOW_CHANNEL, pos, ELBOW_SAFE_MIN, ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
    time.sleep(HOLD_S)

    print(f"[TEST] Sweeping to safe max ({ELBOW_SAFE_MAX}°)")
    pos = _move_to(robot, ELBOW_CHANNEL, pos, ELBOW_SAFE_MAX, ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)
    time.sleep(HOLD_S)

    print(f"[TEST] Returning to stow ({ELBOW_STOW_DEG}°)")
    _move_to(robot, ELBOW_CHANNEL, pos, ELBOW_STOW_DEG, ELBOW_SAFE_MIN, ELBOW_SAFE_MAX)

    robot.disable_servo(ELBOW_CHANNEL)
    robot.disable_servo(SHOULDER_CHANNEL)
    print("[TEST] PASS — elbow swept full safe range and returned to stow")
    print("[TEST] Verify: no collision with body or cables throughout sweep")
