from __future__ import annotations

import time

from robot.robot import FirmwareState, Robot

try:
    from robot.tests.scripts._manipulator_config import GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG, GRIPPER_OPEN_DEG
except ImportError:
    from _manipulator_config import GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG, GRIPPER_OPEN_DEG

OPEN_DEG  = GRIPPER_OPEN_DEG
CLOSE_DEG = GRIPPER_CLOSE_DEG

NEUTRAL_DEG = (OPEN_DEG + CLOSE_DEG) / 2.0

STEP_DEG = 5.0       # degrees per increment during slow approach
STEP_DWELL_S = 0.08  # seconds between increments
HOLD_S = 1.2         # seconds to hold open/closed before moving back
CYCLES = 3


def _move_to(robot: Robot, current: float, target: float) -> float:
    """Incrementally drive the servo from current to target, returning target."""
    direction = 1.0 if target > current else -1.0
    angle = current
    while abs(angle - target) > STEP_DEG:
        angle += direction * STEP_DEG
        robot.set_servo(GRIPPER_CHANNEL, angle)
        time.sleep(STEP_DWELL_S)
    robot.set_servo(GRIPPER_CHANNEL, target)
    time.sleep(STEP_DWELL_S)
    return target


def run(robot: Robot) -> None:
    print(f"[TEST] gripper_open_close_test  channel={GRIPPER_CHANNEL.name}")
    print(f"[TEST] open={OPEN_DEG}°  close={CLOSE_DEG}°  cycles={CYCLES}")
    print(f"[TEST] If gripper does not move, check OPEN_DEG/CLOSE_DEG in _manipulator_config and confirm CH_14 is wired.")

    robot.set_state(FirmwareState.RUNNING)
    robot.enable_servo(GRIPPER_CHANNEL)
    time.sleep(0.1)

    print(f"[TEST] Moving to neutral ({NEUTRAL_DEG:.0f}°)")
    _move_to(robot, NEUTRAL_DEG, NEUTRAL_DEG)
    time.sleep(HOLD_S)

    for i in range(1, CYCLES + 1):
        print(f"[TEST] Cycle {i}/{CYCLES} — opening to {OPEN_DEG}°")
        pos = _move_to(robot, NEUTRAL_DEG, OPEN_DEG)
        time.sleep(HOLD_S)

        print(f"[TEST] Cycle {i}/{CYCLES} — closing to {CLOSE_DEG}°")
        pos = _move_to(robot, pos, CLOSE_DEG)
        time.sleep(HOLD_S)

        print(f"[TEST] Cycle {i}/{CYCLES} — returning to neutral ({NEUTRAL_DEG:.0f}°)")
        pos = _move_to(robot, pos, NEUTRAL_DEG)
        time.sleep(HOLD_S)

    robot.disable_servo(GRIPPER_CHANNEL)

    print("[TEST] PASS — all cycles complete")
    print("[TEST] Verify visually:")
    print(f"[TEST]   jaw opened fully at {OPEN_DEG}° with no binding")
    print(f"[TEST]   jaw closed fully at {CLOSE_DEG}° with no slip")
    print("[TEST] If jaw barely moved, increase the gap between OPEN_DEG and CLOSE_DEG by 10° at a time.")
