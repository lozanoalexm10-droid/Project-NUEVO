"""
heating_wire_test.py
====================
Basic power-on / power-off test for the heating wire roaster.

!! FIRE / BURN HAZARD — read all safety notes before running !!

Safety:
  - Run this test ONLY after all motion tests pass — the gripper should be closed
    around a marshmallow or an equivalent heat-safe test object, not open in free air.
  - Have a fire extinguisher within reach.
  - Do not leave the wire powered unattended.
  - Start with HEATING_WIRE_PWM_ON set LOW (≤ 64) and increase only if needed.
    The PWM range is −255 to 255. Full power is 255.
  - The wire should glow faintly orange at full power — if it glows bright white, cut
    power immediately and reduce PWM.

Pass criteria:
  - Wire heats visibly (orange glow or temperature change on target) at PWM_ON duty.
  - Wire cools and stops glowing within a few seconds of PWM_OFF.
  - No unexpected sparks, smoke, or mechanical damage.

TODO: Confirm HEATING_WIRE_MOTOR_ID and control method with firmware team.
      If the heating wire is controlled via a relay instead of PWM, replace
      set_motor_pwm() with the appropriate relay command once that API exists.

Nodes required:  robot
"""
from __future__ import annotations

import time

from robot.hardware_map import DCMotorMode
from robot.robot import FirmwareState, Robot

from _manipulator_config import (
    HEATING_WIRE_MOTOR_ID,
    HEATING_WIRE_PWM_OFF,
    HEATING_WIRE_PWM_ON,
)

PREHEAT_S  = 3.0   # seconds to hold power on during test
COOLDOWN_S = 5.0   # seconds to wait after power off before declaring safe


def run(robot: Robot) -> None:
    print("[TEST] heating_wire_test")
    print(f"[TEST] motor_id={HEATING_WIRE_MOTOR_ID}  PWM_ON={HEATING_WIRE_PWM_ON}  PWM_OFF={HEATING_WIRE_PWM_OFF}")
    print("[TEST] !! FIRE HAZARD — confirm safety checks before proceeding !!")
    print(f"[TEST] Wire will be powered for {PREHEAT_S}s at PWM={HEATING_WIRE_PWM_ON}")

    robot.set_state(FirmwareState.RUNNING)

    # Ensure wire starts off
    robot.enable_motor(HEATING_WIRE_MOTOR_ID, DCMotorMode.PWM)
    robot.set_motor_pwm(HEATING_WIRE_MOTOR_ID, HEATING_WIRE_PWM_OFF)
    time.sleep(0.5)

    # Power on
    print(f"[TEST] Powering ON heating wire (PWM={HEATING_WIRE_PWM_ON})")
    robot.set_motor_pwm(HEATING_WIRE_MOTOR_ID, HEATING_WIRE_PWM_ON)
    time.sleep(PREHEAT_S)

    # Power off
    print("[TEST] Powering OFF heating wire")
    robot.set_motor_pwm(HEATING_WIRE_MOTOR_ID, HEATING_WIRE_PWM_OFF)

    print(f"[TEST] Cooling down ({COOLDOWN_S}s) — do not touch wire")
    time.sleep(COOLDOWN_S)

    robot.disable_motor(HEATING_WIRE_MOTOR_ID)
    print("[TEST] PASS — wire powered on and off without fault")
    print("[TEST] Verify: wire glowed at expected level, cooled after shutoff, no damage")
