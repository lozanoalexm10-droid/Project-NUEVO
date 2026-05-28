"""
turntable_home_test.py
======================
Home the turntable against LIM1 (stow position) via limit switch, then verify
CW movement to forward and a short sweep.

Setup:
  - No manual pre-positioning needed — the limit switch handles homing.
  - Ensure the full sweep arc is clear of cables and structure.

Homing direction:
  - Always CCW (direction=-1) toward the stow limit switch (LIM1 / PIN_ST2_LIMIT).
  - After step_home(), firmware step counter 0 = stow = our +180°.
  - CW = positive firmware steps.  Forward (0°) = +3200 steps from stow.

Pass criteria:
  - Sanity nudge moves CW (confirms motor is live before homing).
  - LIM1 triggers before HOME_TIMEOUT_S (homing completes).
  - Each subsequent move completes before MOVE_TIMEOUT_S.
  - Turntable ends at forward (0°) without stalling or grinding.
"""
from __future__ import annotations

import time

from robot.hardware_map import Limit, StepMoveType
from robot.robot import FirmwareState, Robot

from robot.tests.scripts._manipulator_config import (
    TURNTABLE_ACCELERATION,
    TURNTABLE_MAX_DEG,
    TURNTABLE_MAX_VELOCITY,
    TURNTABLE_STEPPER,
    turntable_deg_to_steps,
)

HOME_TIMEOUT_S = 25.0   # firmware uses home_velocity/4 internally; 180° at ~500 sps ≈ 6s
MOVE_TIMEOUT_S = 10.0

# Angles in our convention (0° = forward, +180° = stow).
# Sequence visits left, right, then returns to forward.
TEST_ANGLES_DEG = [0.0, 45.0, -45.0, 0.0]

LIM1 = Limit.LIM_1


def _steps_for(target_deg: float) -> int:
    """Convert our angle convention to firmware absolute steps post-homing.

    After homing: firmware 0 = stow = our 180°.  CW = positive steps.
    Formula: turntable_deg_to_steps(180 - target) so forward(0°) = +3200 CW.
    """
    return turntable_deg_to_steps(TURNTABLE_MAX_DEG - target_deg)


def run(robot: Robot) -> None:
    print(f"[TEST] turntable_home_test  stepper={TURNTABLE_STEPPER.name}  mode=limit-switch")

    robot.set_state(FirmwareState.RUNNING)
    robot.step_set_config(
        TURNTABLE_STEPPER,
        max_velocity=TURNTABLE_MAX_VELOCITY,
        acceleration=TURNTABLE_ACCELERATION,
    )
    robot.step_enable(TURNTABLE_STEPPER)
    time.sleep(0.2)

    # ── Stepper sanity check (CW nudge away from stow) ───────────────────────
    # Positive steps = CW = away from the stow/limit switch.
    # If this doesn't move, the issue is upstream: battery not detected or
    # firmware not in RUNNING state — not a limit switch problem.
    sanity_steps = turntable_deg_to_steps(5.0)
    print(f"[TEST] Sanity check: nudging +{sanity_steps} steps CW to confirm motor is live...")
    ok = robot.step_move(TURNTABLE_STEPPER, sanity_steps, StepMoveType.RELATIVE, timeout=5.0)
    if not ok:
        print("[TEST] FAIL: sanity move timed out — check firmware state and battery connection")
        robot.step_disable(TURNTABLE_STEPPER)
        return
    print("[TEST]   motor responded OK")
    time.sleep(0.3)

    # ── Pre-homing diagnostics ────────────────────────────────────────────────
    lim_state = robot.get_limit(LIM1)
    print(f"[TEST] LIM1 state before homing: {'TRIGGERED' if lim_state else 'not triggered'}")
    if lim_state:
        print("[TEST] WARNING: LIM1 already triggered — check wiring if turntable is not at stow.")

    # ── Homing: always CCW to stow (LIM1) ────────────────────────────────────
    print("[TEST] Homing turntable CCW to stow (LIM1)...")
    ok = robot.step_home(
        TURNTABLE_STEPPER,
        direction=-1,         # CCW — toward stow/LIM1; never change this
        home_velocity=2000,   # firmware divides by 4 → ~500 sps actual; 180° ≈ 6s
        backoff_steps=50,
        timeout=HOME_TIMEOUT_S,
    )
    if not ok:
        print("[TEST] FAIL: homing timed out — check LIM1 wiring and firmware PIN_ST2_LIMIT")
        robot.step_disable(TURNTABLE_STEPPER)
        return

    lim_after = robot.get_limit(LIM1)
    print(f"[TEST] Homed OK. LIM1 now: {'TRIGGERED' if lim_after else 'not triggered'}")
    print(f"[TEST] Stow = firmware step 0.  Forward (0°) = +{_steps_for(0.0)} steps CW.")
    time.sleep(0.5)

    # ── Sweep through test angles (CW to forward first, then left/right) ─────
    for angle in TEST_ANGLES_DEG:
        steps = _steps_for(angle)
        print(f"[TEST] Moving to {angle:+.1f}° ({steps:+d} steps absolute)...")
        ok = robot.step_move(
            TURNTABLE_STEPPER, steps, StepMoveType.ABSOLUTE, timeout=MOVE_TIMEOUT_S
        )
        if not ok:
            print(f"[TEST] FAIL: move to {angle:+.1f}° timed out")
            robot.step_disable(TURNTABLE_STEPPER)
            return
        print(f"[TEST]   reached {angle:+.1f}° OK")
        time.sleep(0.3)

    robot.step_disable(TURNTABLE_STEPPER)
    print("[TEST] PASS — homed CCW, swept CW to forward, returned to 0°")
