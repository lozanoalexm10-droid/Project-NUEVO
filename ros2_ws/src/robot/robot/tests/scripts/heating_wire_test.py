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
  - The wire should glow faintly orange — if it glows bright white, cut power immediately.

Hardware: 1-channel 5V optical relay module (active LOW) on RPi GPIO.
  Wiring: JD-VCC→5V, VCC→3.3V, GND→GND, IN1→BCM HEATING_WIRE_GPIO_PIN.
  Relay output: heating wire between COM and NO terminals.

Pass criteria:
  - Wire heats visibly (orange glow or temperature change on target).
  - Wire cools and stops glowing within a few seconds of relay open.
  - No unexpected sparks, smoke, or mechanical damage.

Nodes required:  robot
"""
from __future__ import annotations

import time

from robot.robot import FirmwareState, Robot

from robot.tests.scripts._manipulator_config import HEATING_WIRE_GPIO_PIN

PREHEAT_S  = 3.0   # seconds to hold relay closed during test
COOLDOWN_S = 5.0   # seconds to wait after relay opens before declaring safe


def run(robot: Robot) -> None:
    print("[TEST] heating_wire_test")
    print(f"[TEST] relay GPIO pin (BCM)={HEATING_WIRE_GPIO_PIN}")
    print("[TEST] !! FIRE HAZARD — confirm safety checks before proceeding !!")
    print(f"[TEST] Wire will be powered for {PREHEAT_S}s")

    robot.set_state(FirmwareState.RUNNING)

    robot.enable_relay(HEATING_WIRE_GPIO_PIN)
    robot.set_relay(False)   # ensure off
    time.sleep(0.5)

    print("[TEST] Closing relay — heating wire ON")
    robot.set_relay(True)
    time.sleep(PREHEAT_S)

    print("[TEST] Opening relay — heating wire OFF")
    robot.set_relay(False)

    print(f"[TEST] Cooling down ({COOLDOWN_S}s) — do not touch wire")
    time.sleep(COOLDOWN_S)

    robot.disable_relay()
    print("[TEST] PASS — relay opened and closed without fault")
    print("[TEST] Verify: wire glowed at expected level, cooled after relay opened, no damage")
