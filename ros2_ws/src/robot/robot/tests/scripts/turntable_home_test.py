"""
turntable_home_test.py
======================
Verify turntable stepper movement from manual home position.

Setup:
  - Rotate the turntable to the forward mark before running.
  - The step counter is zeroed at startup; this test moves to a few
    known angles and returns to 0 to confirm motion is working.

Pass criteria:
  - Each move completes before the timeout.
  - The stepper returns to 0 without stalling or grinding.
"""
from __future__ import annotations

import time

from robot.hardware_map import StepMoveType
from robot.robot import FirmwareState, Robot

from _manipulator_config import (
    TURNTABLE_ACCELERATION,
    TURNTABLE_MAX_VELOCITY,
    TURNTABLE_STEPPER,
    turntable_deg_to_steps,
)

MOVE_TIMEOUT_S = 10.0
TEST_ANGLES    = [45.0, 90.0, 135.0, 0.0]   # degrees, last entry returns to start


def run(robot: Robot) -> None:
    print(f"[TEST] turntable_home_test  stepper={TURNTABLE_STEPPER.name}  mode=manual")
    print("[TEST] Ensure turntable is aligned to the forward mark before continuing.")

    robot.set_state(FirmwareState.RUNNING)
    robot.step_set_config(TURNTABLE_STEPPER, max_velocity=TURNTABLE_MAX_VELOCITY, acceleration=TURNTABLE_ACCELERATION)
    robot.step_enable(TURNTABLE_STEPPER)
    time.sleep(0.1)

    for angle in TEST_ANGLES:
        steps = turntable_deg_to_steps(angle)
        print(f"[TEST] Moving to {angle:.1f}° ({steps} steps absolute)...")
        ok = robot.step_move(TURNTABLE_STEPPER, steps, StepMoveType.ABSOLUTE, timeout=MOVE_TIMEOUT_S)
        if not ok:
            print(f"[TEST] FAIL: move to {angle:.1f}° timed out")
            robot.step_disable(TURNTABLE_STEPPER)
            return
        print(f"[TEST]   reached {angle:.1f}° OK")
        time.sleep(0.3)

    robot.step_disable(TURNTABLE_STEPPER)
    print("[TEST] PASS — all moves completed, turntable returned to 0°")
