"""
arm_ik_calibration_test.py
==========================
Step-by-step calibration procedure to measure the arm geometry and determine
the servo offset values needed by ArmGeometry in _manipulator_config.py.

Run this BEFORE using inverse_kinematics() in any program.
All motion is slow and manually supervised — do not leave robot unattended.

Pass criteria:
  - Physical measurements recorded for ARM_L1_MM, ARM_L2_MM,
    ARM_SHOULDER_HEIGHT_MM, ARM_SHOULDER_OFFSET_MM.
  - Servo offset values confirmed for shoulder and elbow.
  - FK round-trip error < 5 mm at two known positions.

Nodes required:  robot
"""
from __future__ import annotations

import math
import time

from robot.arm_kinematics import ArmGeometry, forward_kinematics, inverse_kinematics, OutOfReachError
from robot.robot import FirmwareState, Robot

from robot.tests.scripts._manipulator_config import (
    ARM_GEOMETRY,
    ELBOW_CHANNEL,
    SHOULDER_CHANNEL,
)

STEP_DEG   = 3.0
STEP_DWELL = 0.06


def _move_to(robot: Robot, channel, current: float, target: float) -> float:
    direction = 1.0 if target > current else -1.0
    angle = current
    while abs(angle - target) > STEP_DEG:
        angle += direction * STEP_DEG
        robot.set_servo(channel, angle)
        time.sleep(STEP_DWELL)
    robot.set_servo(channel, target)
    time.sleep(STEP_DWELL)
    return target


def run(robot: Robot) -> None:
    print("[CAL] arm_ik_calibration_test")
    print("[CAL] Follow the prompts. Record values in _manipulator_config.py when done.")

    robot.set_state(FirmwareState.RUNNING)
    robot.enable_servo(SHOULDER_CHANNEL)
    robot.enable_servo(ELBOW_CHANNEL)
    time.sleep(0.2)

    # ── Phase 1: shoulder offset calibration ──────────────────────────────────
    print("\n[CAL] === PHASE 1: Shoulder servo offset ===")
    print("[CAL] Moving shoulder to 90° (servo angle).")
    _move_to(robot, SHOULDER_CHANNEL, 90.0, 90.0)
    time.sleep(1.0)
    print("[CAL] MEASURE: Use a level or protractor to measure the angle of the")
    print("[CAL]          upper arm above horizontal. Write it down.")
    print("[CAL]          shoulder_servo_offset = 90 - your_measured_angle")
    print("[CAL]          Example: arm is 10° above horizontal → offset = 90 - 10 = 80")

    print("\n[CAL] Moving shoulder to 100° (+10 from 90).")
    _move_to(robot, SHOULDER_CHANNEL, 90.0, 100.0)
    time.sleep(1.0)
    print("[CAL] OBSERVE: Did the arm rise or fall?")
    print("[CAL]          Rose  → shoulder_servo_sign = +1")
    print("[CAL]          Fell  → shoulder_servo_sign = -1")

    print("\n[CAL] Returning shoulder to 90°.")
    _move_to(robot, SHOULDER_CHANNEL, 100.0, 90.0)

    # ── Phase 2: elbow offset calibration ─────────────────────────────────────
    print("\n[CAL] === PHASE 2: Elbow servo offset ===")
    print("[CAL] Moving elbow to 90° (servo angle).")
    _move_to(robot, ELBOW_CHANNEL, 90.0, 90.0)
    time.sleep(1.0)
    print("[CAL] MEASURE: Measure the total arm opening angle at the elbow joint.")
    print("[CAL]          (angle between upper arm and forearm, measured on the outside)")
    print("[CAL]          elbow_servo_offset = 90 - (your_measured_angle - 180)")
    print("[CAL]          Example: arm is straight (180°) → offset = 90 - 0 = 90")
    print("[CAL]          Example: arm is bent to 120°   → offset = 90 - (120-180) = 150")

    print("\n[CAL] Moving elbow to 100° (+10 from 90).")
    _move_to(robot, ELBOW_CHANNEL, 90.0, 100.0)
    time.sleep(1.0)
    print("[CAL] OBSERVE: Did the elbow open (straighten) or close (fold)?")
    print("[CAL]          Opened → elbow_servo_sign = +1")
    print("[CAL]          Closed → elbow_servo_sign = -1")

    print("\n[CAL] Returning elbow to 90°.")
    _move_to(robot, ELBOW_CHANNEL, 100.0, 90.0)

    # ── Phase 3: FK round-trip verification ───────────────────────────────────
    print("\n[CAL] === PHASE 3: FK verification (uses current ARM_GEOMETRY values) ===")
    print("[CAL] Update _manipulator_config.py with your measurements first,")
    print("[CAL] then re-run this script to reach this phase with correct values.")

    test_positions = [
        (90.0, 90.0, "stow position"),
        (ARM_GEOMETRY.shoulder_geo_to_servo(20.0),
         ARM_GEOMETRY.elbow_geo_to_servo(150.0),
         "partial extend"),
    ]

    for shoulder_servo, elbow_servo, label in test_positions:
        print(f"\n[CAL] Moving to {label}: shoulder={shoulder_servo:.1f}°  elbow={elbow_servo:.1f}°")
        _move_to(robot, SHOULDER_CHANNEL, 90.0, shoulder_servo)
        _move_to(robot, ELBOW_CHANNEL, 90.0, elbow_servo)
        time.sleep(1.0)

        x, y, z = forward_kinematics(0.0, shoulder_servo, elbow_servo, ARM_GEOMETRY)
        print(f"[CAL] FK predicts gripper at: x={x:.1f}mm  y={y:.1f}mm  z={z:.1f}mm")
        print("[CAL] MEASURE: physically measure gripper tip position and compare.")
        print("[CAL] Error < 10 mm is acceptable. Larger error → re-check L1, L2, offsets.")

    # ── Phase 4: IK spot check ─────────────────────────────────────────────────
    print("\n[CAL] === PHASE 4: IK spot check ===")
    test_target_mm = (200.0, 0.0, ARM_GEOMETRY.shoulder_height_mm + 50.0)
    x, y, z = test_target_mm
    print(f"[CAL] Commanding IK to reach ({x:.0f}, {y:.0f}, {z:.0f}) mm (forward, centre, 50mm above shoulder)")
    try:
        _, shoulder_servo, elbow_servo = inverse_kinematics(x, y, z, ARM_GEOMETRY)
        print(f"[CAL] IK solution: shoulder={shoulder_servo:.1f}°  elbow={elbow_servo:.1f}°")
        _move_to(robot, SHOULDER_CHANNEL, 90.0, shoulder_servo)
        _move_to(robot, ELBOW_CHANNEL, 90.0, elbow_servo)
        time.sleep(1.5)
        print("[CAL] MEASURE: Is the gripper near the target position?")
        print("[CAL] If not, re-check geometry constants and servo offsets.")
    except OutOfReachError as e:
        print(f"[CAL] FAIL: target unreachable with current geometry: {e}")
        print("[CAL] Check that ARM_L1_MM + ARM_L2_MM is large enough to reach this point.")

    # Return to stow
    _move_to(robot, ELBOW_CHANNEL, elbow_servo if 'elbow_servo' in dir() else 90.0, 90.0)
    _move_to(robot, SHOULDER_CHANNEL, shoulder_servo if 'shoulder_servo' in dir() else 90.0, 90.0)

    robot.disable_servo(ELBOW_CHANNEL)
    robot.disable_servo(SHOULDER_CHANNEL)

    print("\n[CAL] Calibration procedure complete.")
    print("[CAL] Update _manipulator_config.py with measured values and re-run to verify.")
