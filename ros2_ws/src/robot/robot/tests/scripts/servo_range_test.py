from __future__ import annotations

import time

from robot.hardware_map import ServoChannel
from robot.robot import FirmwareState, Robot

# Channels to test. Add or remove based on which are physically connected.
CHANNELS_TO_TEST = [
    ServoChannel.CH_1,
    ServoChannel.CH_2,
]

# Phase 1: just center both servos. Observe where the arm ends up before
# uncommenting Phase 2. Never run Phase 2 until you have confirmed Phase 1
# positions are safe and know which direction increasing angle moves each joint.
PHASE_1_ONLY = True

# Phase 2 sweep — only used when PHASE_1_ONLY = False.
# Moves in 5-degree increments so motion is slow and stoppable.
STEP_DEG   = 5.0
STEP_DWELL = 0.15   # seconds between steps — increase if motion feels too fast
HOLD_DWELL = 2.0    # seconds to hold each waypoint before moving on
SWEEP_WAYPOINTS = [45.0, 90.0, 135.0, 90.0]  # never go to 0 or 180 until arm is calibrated


def _move_to(robot: Robot, channel: ServoChannel, current: float, target: float) -> float:
    """Step incrementally from current to target angle. Returns final angle."""
    step = STEP_DEG if target > current else -STEP_DEG
    angle = current
    while abs(target - angle) > STEP_DEG / 2:
        angle += step
        angle = max(0.0, min(180.0, angle))
        robot.set_servo(channel, angle)
        time.sleep(STEP_DWELL)
    robot.set_servo(channel, target)
    time.sleep(STEP_DWELL)
    return target


def run(robot: Robot) -> None:
    print("[TEST] servo_range_test starting")
    robot.set_state(FirmwareState.RUNNING)

    if PHASE_1_ONLY:
        print("[TEST] PHASE 1 — centering both servos to 90 deg")
        print("[TEST] Watch where each joint moves. Ctrl+C immediately if anything binds.")
        for channel in CHANNELS_TO_TEST:
            print(f"[TEST]   Enabling {channel.name}")
            robot.enable_servo(channel)
            print(f"[TEST]   -> 90 deg")
            robot.set_servo(channel, 90.0)
            time.sleep(3.0)
            robot.disable_servo(channel)
            print(f"[TEST]   {channel.name} done — note physical position before continuing")
            time.sleep(1.0)
        print("[TEST] PHASE 1 complete.")
        print("[TEST] If 90 deg positions look safe, set PHASE_1_ONLY = False to run sweep.")
        return

    print("[TEST] PHASE 2 — incremental sweep")
    for channel in CHANNELS_TO_TEST:
        print(f"[TEST] Testing {channel.name}")
        robot.enable_servo(channel)
        current = 90.0
        robot.set_servo(channel, current)
        time.sleep(HOLD_DWELL)

        for waypoint in SWEEP_WAYPOINTS:
            print(f"[TEST]   -> {waypoint:.0f} deg")
            current = _move_to(robot, channel, current, waypoint)
            time.sleep(HOLD_DWELL)

        robot.disable_servo(channel)
        print(f"[TEST] {channel.name} sweep complete")

    print("[TEST] PASS — all channels swept incrementally without error")
