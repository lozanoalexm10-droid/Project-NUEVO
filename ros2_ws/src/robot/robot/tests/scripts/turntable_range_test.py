"""
turntable_range_test.py
=======================
Sweep the turntable stepper left and right through a defined arc from home.

Pass criteria:
  - Both sweeps complete before timeout.
  - Stepper returns to 0° (home) position without drift.
  - No missed steps audible (no irregular clicking at cruise speed).

Safety:
  - Keep the arm in stowed position (shoulder/elbow folded) during this test.
  - Confirm the sweep arc is clear of cables and structure before running.
"""
from __future__ import annotations

import time

from robot.hardware_map import StepMoveType
from robot.robot import FirmwareState, Robot

from robot.tests.scripts._manipulator_config import (
    TURNTABLE_ACCELERATION,
    TURNTABLE_MAX_VELOCITY,
    TURNTABLE_MIN_DEG,
    TURNTABLE_SCAN_ARC_DEG,
    TURNTABLE_STEPPER,
    turntable_deg_to_steps,
)

SWEEP_DEG      = 90.0   # degrees to sweep each direction; keep ≤ TURNTABLE_SCAN_ARC_DEG
MOVE_TIMEOUT_S = 15.0


def run(robot: Robot) -> None:
    print(f"[TEST] turntable_range_test  stepper={TURNTABLE_STEPPER.name}")
    print(f"[TEST] sweep ±{SWEEP_DEG}°  ({turntable_deg_to_steps(SWEEP_DEG)} steps each direction)")
    print(f"[TEST] Safe scan zone: ±{TURNTABLE_SCAN_ARC_DEG:.0f}°  min limit: {TURNTABLE_MIN_DEG:.0f}°  — do not change SWEEP_DEG beyond TURNTABLE_SCAN_ARC_DEG.")

    robot.set_state(FirmwareState.RUNNING)
    robot.step_set_config(TURNTABLE_STEPPER, max_velocity=TURNTABLE_MAX_VELOCITY, acceleration=TURNTABLE_ACCELERATION)
    robot.step_enable(TURNTABLE_STEPPER)
    time.sleep(0.1)

    steps = turntable_deg_to_steps(SWEEP_DEG)

    # Sweep right (+)
    print(f"[TEST] Sweeping right +{SWEEP_DEG}° ({steps} steps)")
    ok = robot.step_move(TURNTABLE_STEPPER, steps, StepMoveType.RELATIVE, timeout=MOVE_TIMEOUT_S)
    if not ok:
        print("[TEST] FAIL: right sweep timed out")
        robot.step_disable(TURNTABLE_STEPPER)
        return
    time.sleep(0.5)

    # Return to center
    print(f"[TEST] Returning to center")
    ok = robot.step_move(TURNTABLE_STEPPER, -steps, StepMoveType.RELATIVE, timeout=MOVE_TIMEOUT_S)
    if not ok:
        print("[TEST] FAIL: return from right timed out")
        robot.step_disable(TURNTABLE_STEPPER)
        return
    time.sleep(0.5)

    # Sweep left (-)
    print(f"[TEST] Sweeping left -{SWEEP_DEG}° ({steps} steps)")
    ok = robot.step_move(TURNTABLE_STEPPER, -steps, StepMoveType.RELATIVE, timeout=MOVE_TIMEOUT_S)
    if not ok:
        print("[TEST] FAIL: left sweep timed out")
        robot.step_disable(TURNTABLE_STEPPER)
        return
    time.sleep(0.5)

    # Return to center
    print(f"[TEST] Returning to center")
    ok = robot.step_move(TURNTABLE_STEPPER, steps, StepMoveType.RELATIVE, timeout=MOVE_TIMEOUT_S)
    if not ok:
        print("[TEST] FAIL: return from left timed out")
        robot.step_disable(TURNTABLE_STEPPER)
        return

    robot.step_disable(TURNTABLE_STEPPER)
    print("[TEST] PASS — turntable swept ±90° and returned to start")
    print("[TEST] Verify visually: turntable back at starting angle, no binding or skipping")
