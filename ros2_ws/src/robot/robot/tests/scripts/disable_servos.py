from __future__ import annotations

import time

from robot.robot import FirmwareState, Robot

try:
    from robot.tests.scripts._manipulator_config import ELBOW_CHANNEL, GRIPPER_CHANNEL, SHOULDER_CHANNEL
except ImportError:
    from _manipulator_config import ELBOW_CHANNEL, GRIPPER_CHANNEL, SHOULDER_CHANNEL


def run(robot: Robot) -> None:
    print("[TEST] disable_servos — releasing all arm servos")
    robot.set_state(FirmwareState.RUNNING)
    for channel in [SHOULDER_CHANNEL, ELBOW_CHANNEL, GRIPPER_CHANNEL]:
        robot.disable_servo(channel)
        print(f"[TEST]   {channel.name} disabled")
    time.sleep(1.0)
    print("[TEST] all servos released — arm is now limp")
