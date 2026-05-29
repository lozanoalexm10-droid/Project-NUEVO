"""
turntable_home_test.py
======================
Diagnose and exercise the turntable limit-switch homing sequence.

Handles the case where the turntable is already sitting on LIM1 at startup:
  - If LIM1 is already triggered, backs off CW first so the firmware can
    detect the falling edge before issuing the home command.
  - Prints LIM1 state before and after every significant move so you can
    see exactly what the switch is doing.

After successful homing the test holds position and waits — it does NOT
move to 0° or any other angle.  Ctrl-C to exit.
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

HOME_TIMEOUT_S  = 30.0   # generous — full 180° at ~500 sps ≈ 6 s normally
BACKOFF_DEG     = 10.0   # CW backoff if switch is already triggered at startup
MOVE_TIMEOUT_S  = 8.0

LIM1 = Limit.LIM_1


def _lim(robot: Robot) -> str:
    return "TRIGGERED" if robot.get_limit(LIM1) else "open"


def run(robot: Robot) -> None:
    print("[HOME_TEST] turntable_home_test  stepper=%s" % TURNTABLE_STEPPER.name)

    robot.set_state(FirmwareState.RUNNING)
    robot.step_set_config(
        TURNTABLE_STEPPER,
        max_velocity=TURNTABLE_MAX_VELOCITY,
        acceleration=TURNTABLE_ACCELERATION,
    )
    robot.step_enable(TURNTABLE_STEPPER)
    time.sleep(0.2)

    # ── 1. Always nudge CW first ──────────────────────────────────────────────
    # The turntable may be sitting on LIM1 at startup.  Nudging CW physically
    # moves it off the switch so step_home() sees a clean falling edge.
    print(f"[HOME_TEST] LIM1 on startup: {_lim(robot)}")
    sanity_steps = turntable_deg_to_steps(5.0)
    print(f"[HOME_TEST] Nudging {sanity_steps} steps CW to clear switch and confirm motor...")
    ok = robot.step_move(
        TURNTABLE_STEPPER, sanity_steps, StepMoveType.RELATIVE, timeout=5.0
    )
    if not ok:
        print("[HOME_TEST] FAIL: CW nudge timed out — check firmware state and battery")
        robot.step_disable(TURNTABLE_STEPPER)
        return
    time.sleep(0.3)
    print(f"[HOME_TEST] After CW nudge — LIM1: {_lim(robot)}")

    # ── 2. Home CCW to LIM1 ───────────────────────────────────────────────────
    print(f"[HOME_TEST] Starting CCW home to LIM1 (timeout={HOME_TIMEOUT_S:.0f}s)...")
    ok = robot.step_home(
        TURNTABLE_STEPPER,
        direction=-1,
        home_velocity=2000,
        backoff_steps=200,
        timeout=HOME_TIMEOUT_S,
    )

    print(f"[HOME_TEST] LIM1 after home attempt: {_lim(robot)}")

    if not ok:
        print("[HOME_TEST] FAIL: homing timed out — LIM1 never triggered within "
              f"{HOME_TIMEOUT_S:.0f}s.  Check LIM1 wiring and PIN_ST2_LIMIT in firmware.")
        robot.step_disable(TURNTABLE_STEPPER)
        return

    # ── 4. Success — report and hold position ─────────────────────────────────
    forward_steps = turntable_deg_to_steps(TURNTABLE_MAX_DEG)
    print("[HOME_TEST] PASS — turntable homed.  Firmware step 0 = stow (+180°).")
    print(f"[HOME_TEST] Forward (0°) would be +{forward_steps} steps CW from here.")
    print("[HOME_TEST] Holding position — Ctrl-C to exit.")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    robot.step_disable(TURNTABLE_STEPPER)
    print("[HOME_TEST] Done.")
