from __future__ import annotations

import time

from robot.hardware_map import ServoChannel
from robot.robot import FirmwareState, Robot

# Channels to sweep. Add or remove channels based on which are physically connected.
CHANNELS_TO_TEST = [
    ServoChannel.CH_1,
    ServoChannel.CH_2,
]

SWEEP_ANGLES = [0.0, 90.0, 180.0, 90.0]
DWELL_S = 1.0  # seconds to hold each angle


def run(robot: Robot) -> None:
    print("[TEST] servo_range_test starting")
    robot.set_state(FirmwareState.RUNNING)

    for channel in CHANNELS_TO_TEST:
        print(f"[TEST] Testing channel {channel.name}")
        robot.enable_servo(channel)

        for angle in SWEEP_ANGLES:
            print(f"[TEST]   -> {angle:.0f} deg")
            robot.set_servo(channel, angle)
            time.sleep(DWELL_S)

        robot.disable_servo(channel)
        print(f"[TEST] Channel {channel.name} sweep complete")

    print("[TEST] PASS — all channels swept without error")
    print("[TEST] Verify visually: each servo moved to 0, 90, 180, 90 deg without binding")
