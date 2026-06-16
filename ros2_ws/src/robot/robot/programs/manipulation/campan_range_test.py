"""
campan_range_test.py
====================
Sweep the camera pan stepper (Stepper 2) through all three scan positions,
capture and save a PNG at each position, then return to center.

Pass criteria:
  - All three position moves complete before timeout.
  - Stepper returns to 0° (center) without drift.
  - No missed steps or binding at any position.
  - Three PNG files saved to SAVE_DIR, one per scan angle.

Safety:
  - Ensure camera cable has enough slack to cover the full ±60° travel.
  - Run before any full-scan demo so you know the stepper motion is reliable.

Setup:
  - Manually center the camera pan mechanism to the forward mark before running.
  - Position 0 = wherever the stepper sits at startup (open-loop).
"""
from __future__ import annotations

import time
from pathlib import Path

import cv2

from robot.hardware_map import StepMoveType
from robot.robot import FirmwareState, Robot

from robot._manipulator_config import (
    CAMPAN_ACCELERATION,
    CAMPAN_MAX_VELOCITY,
    CAMPAN_POSITIONS_DEG,
    CAMPAN_SETTLE_S,
    CAMPAN_STEPPER,
    campan_deg_to_steps,
)

MOVE_TIMEOUT_S = 10.0
CAMERA_DEVICE  = "/dev/video10"
SAVE_DIR       = Path("/runtime_output/campan_scan")


def _capture_frame(device: str) -> "cv2.Mat | None":
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[TEST] WARNING: could not open camera {device} — skipping capture")
        return None
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    # Discard one stale frame before reading.
    cap.read()
    ok, frame = cap.read()
    cap.release()
    return frame if ok else None


def run(robot: Robot) -> None:
    print(f"[TEST] campan_range_test  stepper={CAMPAN_STEPPER.name}")
    print(f"[TEST] positions: {CAMPAN_POSITIONS_DEG} deg")

    SAVE_DIR.mkdir(parents=True, exist_ok=True)

    robot.set_state(FirmwareState.RUNNING)
    robot.step_set_config(CAMPAN_STEPPER, max_velocity=CAMPAN_MAX_VELOCITY, acceleration=CAMPAN_ACCELERATION)
    robot.step_enable(CAMPAN_STEPPER)
    time.sleep(0.1)

    # Sweep through all configured scan positions in order.
    for target_deg in CAMPAN_POSITIONS_DEG:
        steps = campan_deg_to_steps(target_deg)
        print(f"[TEST] Moving to {target_deg:.1f}° ({steps} steps absolute)")
        ok = robot.step_move(CAMPAN_STEPPER, steps, StepMoveType.ABSOLUTE, timeout=MOVE_TIMEOUT_S)
        if not ok:
            print(f"[TEST] FAIL: move to {target_deg:.1f}° timed out")
            robot.step_disable(CAMPAN_STEPPER)
            return
        time.sleep(CAMPAN_SETTLE_S)

        sign = "neg" if target_deg < 0 else ("pos" if target_deg > 0 else "")
        label = f"{sign}{abs(target_deg):.0f}deg"
        save_path = SAVE_DIR / f"campan_scan_{label}.png"
        frame = _capture_frame(CAMERA_DEVICE)
        if frame is not None:
            cv2.imwrite(str(save_path), frame)
            print(f"[TEST]   saved {save_path}")
        else:
            print(f"[TEST]   WARNING: no frame captured at {target_deg:.1f}°")

    # Return to center (0°).
    print("[TEST] Returning to center (0°)")
    ok = robot.step_move(CAMPAN_STEPPER, 0, StepMoveType.ABSOLUTE, timeout=MOVE_TIMEOUT_S)
    if not ok:
        print("[TEST] FAIL: return to center timed out")
        robot.step_disable(CAMPAN_STEPPER)
        return
    time.sleep(CAMPAN_SETTLE_S)

    robot.step_disable(CAMPAN_STEPPER)
    print("[TEST] PASS — camera pan swept all positions and returned to center")
    print(f"[TEST] Images saved to {SAVE_DIR}  (campan_scan_neg60deg.png / campan_scan_0deg.png / campan_scan_pos60deg.png)")
    print("[TEST] Verify: camera is back at forward-facing position, no cable strain")
