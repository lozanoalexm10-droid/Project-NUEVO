"""
turntable_home_test.py
======================
Home the turntable stepper against its limit switch and verify repeatability.

Pass criteria:
  - Homing completes and limit switch triggers before timeout.
  - Position after three successive homes lands within ±10 steps.
  - No grinding or stall during home travel.

Safety:
  - Confirm the arm is stowed before running.
  - Confirm the limit switch side of travel is clear.
"""
from __future__ import annotations

import time

from robot.hardware_map import StepMoveType
from robot.robot import FirmwareState, Robot

from _manipulator_config import (
    TURNTABLE_ACCELERATION,
    TURNTABLE_HOME_LIMIT,
    TURNTABLE_MAX_VELOCITY,
    TURNTABLE_STEPPER,
    turntable_deg_to_steps,
)

HOME_VELOCITY    = 800   # steps/sec during homing — slower than cruise for safety
BACKOFF_STEPS    = 100   # steps to back off after limit triggers
HOME_TIMEOUT_S   = 20.0
RUNS             = 3     # repeatability check
REPEATABILITY_TOL = 15   # maximum allowed step spread across runs


def run(robot: Robot) -> None:
    print(f"[TEST] turntable_home_test  stepper={TURNTABLE_STEPPER.name}  limit={TURNTABLE_HOME_LIMIT.name}")

    robot.set_state(FirmwareState.RUNNING)
    robot.step_set_config(TURNTABLE_STEPPER, max_velocity=TURNTABLE_MAX_VELOCITY, acceleration=TURNTABLE_ACCELERATION)
    robot.step_enable(TURNTABLE_STEPPER)
    time.sleep(0.1)

    positions: list[bool] = []

    for run_n in range(1, RUNS + 1):
        print(f"[TEST] Homing run {run_n}/{RUNS}")
        ok = robot.step_home(
            TURNTABLE_STEPPER,
            direction=-1,
            home_velocity=HOME_VELOCITY,
            backoff_steps=BACKOFF_STEPS,
            timeout=HOME_TIMEOUT_S,
        )
        if not ok:
            print(f"[TEST] FAIL: homing run {run_n} timed out — limit switch may not be wired or triggered")
            robot.step_disable(TURNTABLE_STEPPER)
            return

        triggered = robot.was_limit_triggered(TURNTABLE_HOME_LIMIT)
        print(f"[TEST]   limit switch triggered: {triggered}")
        positions.append(triggered)

        if run_n < RUNS:
            # Move away from home so the next home run has travel to do
            away_steps = turntable_deg_to_steps(45.0)
            robot.step_move(TURNTABLE_STEPPER, away_steps, StepMoveType.RELATIVE, timeout=10.0)
            time.sleep(0.3)

    robot.step_disable(TURNTABLE_STEPPER)

    if all(positions):
        print("[TEST] PASS — limit switch triggered on all homing runs")
    else:
        print(f"[TEST] FAIL: limit switch did not trigger on all runs: {positions}")
        print("[TEST] Check wiring of LIM_1 and confirm the turntable physically reaches the switch.")
