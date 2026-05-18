from __future__ import annotations

import time

from robot.robot import FirmwareState, Robot

try:
    from robot.tests.scripts._manipulator_config import ELBOW_CHANNEL, GRIPPER_CHANNEL, SHOULDER_CHANNEL
except ImportError:
    from _manipulator_config import ELBOW_CHANNEL, GRIPPER_CHANNEL, SHOULDER_CHANNEL

# ── Edit these ────────────────────────────────────────────────────────────────
CHANNEL     = GRIPPER_CHANNEL  # swap to SHOULDER_CHANNEL or ELBOW_CHANNEL as needed

# GRIPPER direction (confirmed): LOWER angle = more open, HIGHER angle = more closed.
# 70° was past fully open. Start closed and sweep toward open to find the right position.
# Ctrl+C immediately if it binds while closing.
SWEEP_FROM  = 120   # deg — assumed closed side; snap here on enable
SWEEP_TO    = 45  # deg — sweep toward open; stop when gripper is at the right gap

STEP_DEG    = 15.0    # degrees per step — keep small near a hard stop
STEP_DWELL  = 0.2    # seconds between steps — slow enough to Ctrl+C if arm binds
# ─────────────────────────────────────────────────────────────────────────────


def _move_to(robot: Robot, channel, current: float, target: float) -> float:
    step = STEP_DEG if target > current else -STEP_DEG
    angle = current
    while abs(target - angle) > STEP_DEG / 2:
        angle += step
        angle = max(0.0, min(180.0, angle))
        robot.set_servo(channel, angle)
        print(f"[TEST]   {angle:.1f} deg")
        time.sleep(STEP_DWELL)
    robot.set_servo(channel, target)
    print(f"[TEST]   {target:.1f} deg  <-- waypoint reached")
    time.sleep(STEP_DWELL)
    return target


def run(robot: Robot) -> None:
    print(f"[TEST] servo_range_test — {CHANNEL.name}")
    robot.set_state(FirmwareState.RUNNING)

    robot.enable_servo(CHANNEL)

    print(f"[TEST] Snapping to SWEEP_FROM = {SWEEP_FROM:.0f} deg")
    robot.set_servo(CHANNEL, SWEEP_FROM)
    time.sleep(2.0)

    print(f"[TEST] Sweeping {SWEEP_FROM:.0f} -> {SWEEP_TO:.0f} deg  (Ctrl+C immediately if arm binds)")
    _move_to(robot, CHANNEL, SWEEP_FROM, SWEEP_TO)

    robot.disable_servo(CHANNEL)
    print("[TEST] Done")
