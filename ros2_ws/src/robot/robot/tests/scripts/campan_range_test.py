"""
campan_range_test.py
====================
Sweep the camera pan stepper (Stepper 2) through all three scan positions
and return to center.  Mirrors turntable_range_test.py in structure.

Pass criteria:
  - All three position moves complete before timeout.
  - Stepper returns to 0° (center) without drift.
  - No missed steps or binding at any position.
  - Camera image visually shifts between frames (confirm on-screen if connected).

Safety:
  - Ensure camera cable has enough slack to cover the full ±60° travel.
  - Run before any full-scan demo so you know the stepper motion is reliable.

Setup:
  - Manually center the camera pan mechanism to the forward mark before running.
  - Position 0 = wherever the stepper sits at startup (open-loop).
"""
from __future__ import annotations

import time

from robot.hardware_map import StepMoveType
from robot.robot import FirmwareState, Robot

from robot.tests.scripts._manipulator_config import (
    CAMPAN_ACCELERATION,
    CAMPAN_MAX_VELOCITY,
    CAMPAN_POSITIONS_DEG,
    CAMPAN_SETTLE_S,
    CAMPAN_STEPPER,
    campan_deg_to_steps,
)

MOVE_TIMEOUT_S = 10.0


def run(robot: Robot) -> None:
    print(f"[TEST] campan_range_test  stepper={CAMPAN_STEPPER.name}")
    print(f"[TEST] positions: {CAMPAN_POSITIONS_DEG} deg")

    robot.set_state(FirmwareState.RUNNING)
    robot.step_set_config(CAMPAN_STEPPER, max_velocity=CAMPAN_MAX_VELOCITY, acceleration=CAMPAN_ACCELERATION)
    robot.step_enable(CAMPAN_STEPPER)
    time.sleep(0.1)

    # Sweep through all configured scan positions in order.
    for target_deg in CAMPAN_POSITIONS_DEG:
        steps = campan_deg_to_steps(target_deg)
        print(f"[TEST] Moving to {target_deg:.1f}° ({steps} steps absolute)")
        ok = robot.step_move(CAMPAN_STEPPER, steps, StepMoveType.ABSOLUTE, timeout=MOVE_TIMEOUT_S)
        if not ok:
            print(f"[TEST] FAIL: move to {target_deg:.1f}° timed out")
            robot.step_disable(CAMPAN_STEPPER)
            return
        time.sleep(CAMPAN_SETTLE_S)
        print(f"[TEST]   settled at {target_deg:.1f}° — verify camera image has shifted")

    # Return to center (0°).
    print("[TEST] Returning to center (0°)")
    ok = robot.step_move(CAMPAN_STEPPER, 0, StepMoveType.ABSOLUTE, timeout=MOVE_TIMEOUT_S)
    if not ok:
        print("[TEST] FAIL: return to center timed out")
        robot.step_disable(CAMPAN_STEPPER)
        return
    time.sleep(CAMPAN_SETTLE_S)

    robot.step_disable(CAMPAN_STEPPER)
    print("[TEST] PASS — camera pan swept all positions and returned to center")
    print("[TEST] Verify: camera is back at forward-facing position, no cable strain")
