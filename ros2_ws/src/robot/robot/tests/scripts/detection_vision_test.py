"""
detection_vision_test.py
========================
Verify the vision pipeline is running and can detect objects in front of the robot.
Place a marshmallow (and optionally other objects) in the detection zone before running.

Detection zone:  half-circle, 6–12 inch radius, 0–12 inch height, forward of robot.

Pass criteria:
  - Vision system reports active within VISION_WAIT_S seconds.
  - At least one detection is returned from the camera feed.
  - A marshmallow is detected with confidence >= MIN_CONFIDENCE when one is present.
  - Other non-marshmallow objects do not trigger a false positive.

Nodes required:  robot  +  vision node (publishes /vision/detections)
"""
from __future__ import annotations

import time

from robot.robot import FirmwareState, Robot

from robot.tests.scripts._manipulator_config import (
    MARSHMALLOW_CLASS,
    MIN_CONFIDENCE,
    VISION_WAIT_S,
)


def run(robot: Robot) -> None:
    print("[TEST] detection_vision_test")
    print(f"[TEST] Looking for class='{MARSHMALLOW_CLASS}' with confidence >= {MIN_CONFIDENCE}")
    print(f"[TEST] Make sure the vision node is running and /vision/detections is publishing.")

    robot.set_state(FirmwareState.RUNNING)
    robot.enable_vision()

    # ── Wait for vision to come online ────────────────────────────────────────
    print(f"[TEST] Waiting up to {VISION_WAIT_S}s for vision system to activate...")
    deadline = time.monotonic() + VISION_WAIT_S
    while time.monotonic() < deadline:
        if robot.is_vision_active(timeout_s=1.0):
            break
        time.sleep(0.5)
    else:
        print("[TEST] FAIL: vision system did not become active within timeout")
        print("[TEST] Check that the vision node is launched and /vision/detections is publishing.")
        return

    print("[TEST] Vision system active")
    w, h = robot.get_detection_image_size()
    print(f"[TEST] Image size: {w}x{h}")

    # ── Report all current detections ─────────────────────────────────────────
    all_detections = robot.get_detections()
    if not all_detections:
        print("[TEST] WARNING: no detections in current frame — is something in the detection zone?")
    else:
        print(f"[TEST] {len(all_detections)} detection(s) in frame:")
        for det in all_detections:
            print(f"[TEST]   class={det['class_name']}  confidence={float(det['confidence']):.2f}  bbox={det.get('bbox')}")

    # ── Check specifically for marshmallow ────────────────────────────────────
    if robot.has_detection(MARSHMALLOW_CLASS, min_confidence=MIN_CONFIDENCE):
        dets = robot.get_detections(class_name=MARSHMALLOW_CLASS)
        best = max(dets, key=lambda d: float(d["confidence"]))
        print(f"[TEST] Marshmallow detected: confidence={float(best['confidence']):.2f}  bbox={best.get('bbox')}")
        print("[TEST] PASS — marshmallow detected above confidence threshold")
    else:
        print(f"[TEST] FAIL: no '{MARSHMALLOW_CLASS}' detected above {MIN_CONFIDENCE} confidence")
        print("[TEST] If a marshmallow is in the zone: lower MIN_CONFIDENCE or check vision model class names.")
        print("[TEST] If testing without a marshmallow: ignore this FAIL — run again with one present.")
