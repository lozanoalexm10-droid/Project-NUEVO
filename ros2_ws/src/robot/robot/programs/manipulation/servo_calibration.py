"""
servo_calibration.py
====================
Drives all arm servos to raw 90° and holds position so arms can be attached.
Run this before physical assembly to ensure servos are centered.

Usage: set main.py to import run from this file, then launch the robot node.
Press Ctrl+C when arms are attached.
"""
from __future__ import annotations

import time

from robot.robot import FirmwareState, Robot

try:
    from robot._manipulator_config import (
        ELBOW_CHANNEL,
        GRIPPER_CHANNEL,
        SHOULDER_CHANNEL,
    )
except ImportError:
    from _manipulator_config import (
        ELBOW_CHANNEL,
        GRIPPER_CHANNEL,
        SHOULDER_CHANNEL,
    )

CALIBRATION_ANGLE = 90.0


def run(robot: Robot) -> None:
    print("[CALIBRATION] Setting all arm servos to 90° for arm attachment")

    # Firmware typically boots into ESTOP — clear it before driving.
    current_state = robot.get_state()
    if current_state in (FirmwareState.ESTOP, FirmwareState.ERROR):
        print(f"[CALIBRATION] Firmware in state {int(current_state)}; resetting E-STOP.")
        robot.reset_estop()
        time.sleep(0.3)
    robot.set_state(FirmwareState.RUNNING)
    time.sleep(0.3)
    print(f"[CALIBRATION] Firmware state now: {int(robot.get_state())}")

    for channel in [SHOULDER_CHANNEL, ELBOW_CHANNEL, GRIPPER_CHANNEL]:
        robot.enable_servo(channel)
    time.sleep(0.5)

    for channel in [SHOULDER_CHANNEL, ELBOW_CHANNEL, GRIPPER_CHANNEL]:
        robot.set_servo(channel, CALIBRATION_ANGLE)
        print(f"[CALIBRATION]   {channel.name} → {CALIBRATION_ANGLE}°")

    print("[CALIBRATION] All servos at 90° — attach arms now. Press Ctrl+C when done.")
    while True:
        time.sleep(1.0)
