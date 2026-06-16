"""
detection_vision_test.py
========================
Verify the vision pipeline detects a marshmallow and report its estimated
bearing, elevation, and distance so the arm can eventually use the result.

Detection zone (competition spec):
  half-circle, 6–12 inch radius (152–305 mm), 0–12 inch height, forward of robot.

Pass criteria:
  - Vision system reports active within VISION_WAIT_S seconds.
  - At least one marshmallow is detected with confidence >= MIN_CONFIDENCE_MARSHMALLOW.
  - Estimated distance falls within the 6–12 inch detection zone.

Positional output (for each marshmallow candidate):
  - bearing_deg  : horizontal angle from camera centre (+right, −left)
  - elevation_deg: vertical angle from camera centre  (+up, −down)
  - dist_est_mm  : pinhole-model distance estimate using MARSHMALLOW_DIAMETER_MM

Robot-frame bearing (once campan angle is known):
  robot_bearing_deg = campan_angle_deg + bearing_deg

Nodes required:  robot  +  vision node (publishes /vision/detections)
"""
from __future__ import annotations

import math
import time

from robot.arm_kinematics import detection_to_robot_frame
from robot.robot import FirmwareState, Robot

from robot._manipulator_config import (
    ARM_GEOMETRY,
    CAMERA_HFOV_DEG,
    MARSHMALLOW_CLASS,
    MARSHMALLOW_DIAMETER_MM,
    MARSHMALLOW_HEIGHT_MM,
    MIN_CONFIDENCE_MARSHMALLOW,
    VISION_WAIT_S,
)

# Detection zone bounds in mm (6–12 inches).
ZONE_NEAR_MM = 152.0
ZONE_FAR_MM  = 305.0


def _bbox_position(bbox: dict, img_w: int, img_h: int) -> tuple[float, float, float, float, float]:
    """Return (bearing_deg, elevation_deg, dist_mm, bbox_cx_px, bbox_cy_px).

    bearing_deg  : positive = right of camera centre
    elevation_deg: positive = above camera centre
    dist_mm      : pinhole estimate using known physical marshmallow diameter
    """
    bbox_cx = bbox["x"] + bbox["width"]  / 2.0
    bbox_cy = bbox["y"] + bbox["height"] / 2.0

    hfov_rad = math.radians(CAMERA_HFOV_DEG)
    vfov_rad = 2.0 * math.atan(math.tan(hfov_rad / 2.0) * img_h / img_w)

    bearing_deg   =  ((bbox_cx - img_w / 2.0) / img_w) * CAMERA_HFOV_DEG
    elevation_deg = -((bbox_cy - img_h / 2.0) / img_h) * math.degrees(vfov_rad)

    # Pinhole distance: dist = (real_diam * focal_px) / px_diam
    focal_px   = (img_w / 2.0) / math.tan(hfov_rad / 2.0)
    px_diameter = math.sqrt(max(1.0, bbox["width"] * bbox["height"]))
    dist_mm    = (MARSHMALLOW_DIAMETER_MM * focal_px) / px_diameter

    return bearing_deg, elevation_deg, dist_mm, bbox_cx, bbox_cy


def run(robot: Robot) -> None:
    print("[TEST] detection_vision_test")
    print(f"[TEST] Class='{MARSHMALLOW_CLASS}'  confidence>={MIN_CONFIDENCE_MARSHMALLOW}  zone={ZONE_NEAR_MM:.0f}–{ZONE_FAR_MM:.0f} mm")
    print("[TEST] Make sure the vision node is running and /vision/detections is publishing.")

    robot.set_state(FirmwareState.RUNNING)
    robot.enable_vision()

    # ── Wait for vision to come online ────────────────────────────────────────
    print(f"[TEST] Waiting up to {VISION_WAIT_S}s for vision system...")
    deadline = time.monotonic() + VISION_WAIT_S
    while time.monotonic() < deadline:
        if robot.is_vision_active(timeout_s=1.0):
            break
        time.sleep(0.5)
    else:
        print("[TEST] FAIL: vision system did not become active within timeout")
        print("[TEST] Check that the vision node is launched and /vision/detections is publishing.")
        return

    img_w, img_h = robot.get_detection_image_size()
    print(f"[TEST] Vision active — frame {img_w}x{img_h}")

    # ── Report all detections in current frame ────────────────────────────────
    all_detections = robot.get_detections()
    if not all_detections:
        print("[TEST] WARNING: no detections in frame — is the marshmallow in the camera's view?")
    else:
        print(f"[TEST] {len(all_detections)} detection(s) in frame:")
        for det in all_detections:
            print(f"[TEST]   class={det['class_name']}  conf={float(det['confidence']):.2f}  bbox={det.get('bbox')}")

    # ── Marshmallow candidates ────────────────────────────────────────────────
    candidates = [
        d for d in robot.get_detections(class_name=MARSHMALLOW_CLASS)
        if float(d["confidence"]) >= MIN_CONFIDENCE_MARSHMALLOW
    ]

    if not candidates:
        print(f"[TEST] FAIL: no '{MARSHMALLOW_CLASS}' above confidence {MIN_CONFIDENCE_MARSHMALLOW}")
        print("[TEST] If a marshmallow is present: check HSV lighting or lower MIN_CONFIDENCE_MARSHMALLOW.")
        print("[TEST] If testing without a marshmallow: ignore — run again with one in view.")
        return

    # ── Positional report for each candidate ─────────────────────────────────
    print(f"[TEST] {len(candidates)} marshmallow candidate(s):")
    best_candidate = None
    best_conf = -1.0

    for det in candidates:
        conf  = float(det["confidence"])
        bbox  = det["bbox"]
        bearing_deg, elevation_deg, dist_mm, cx, cy = _bbox_position(bbox, img_w, img_h)

        in_zone = ZONE_NEAR_MM <= dist_mm <= ZONE_FAR_MM
        zone_tag = "IN ZONE" if in_zone else f"out of zone ({ZONE_NEAR_MM:.0f}–{ZONE_FAR_MM:.0f} mm)"

        # Translate to turntable-axis robot frame (campan=0 for static test).
        ik_x, ik_y, ik_z = detection_to_robot_frame(
            distance_mm=dist_mm,
            bearing_deg=bearing_deg,
            height_mm=MARSHMALLOW_HEIGHT_MM,
            geom=ARM_GEOMETRY,
            campan_deg=0.0,  # update when running with campan sweep
        )
        print(
            f"[TEST]   conf={conf:.2f}"
            f"  bearing={bearing_deg:+.1f}°  elevation={elevation_deg:+.1f}°"
            f"  dist_from_camera={dist_mm:.0f}mm ({dist_mm/25.4:.1f}in)"
            f"  [{zone_tag}]"
        )
        print(
            f"[TEST]     IK target (turntable frame): x={ik_x:.0f}mm  y={ik_y:.0f}mm  z={ik_z:.0f}mm"
        )
        print(
            f"[TEST]     Note: robot_bearing_deg = -(campan_deg + bearing_deg)"
            f" = -({0.0:.1f} + {bearing_deg:.1f}) = {-(0.0+bearing_deg):.1f}°"
        )

        if conf > best_conf:
            best_conf = conf
            best_candidate = (bearing_deg, elevation_deg, dist_mm)

    # ── Summary ───────────────────────────────────────────────────────────────
    bearing_deg, elevation_deg, dist_mm = best_candidate
    in_zone = ZONE_NEAR_MM <= dist_mm <= ZONE_FAR_MM
    print()
    print(f"[TEST] Best candidate: bearing={bearing_deg:+.1f}°  elevation={elevation_deg:+.1f}°  dist_est={dist_mm:.0f}mm")
    if in_zone:
        print("[TEST] PASS — marshmallow detected within 6–12 inch zone")
        print("[TEST] Next: use bearing + dist to set turntable angle and arm IK target.")
        print("[TEST] Ultrasonic sensor will confirm/correct distance once arm is near.")
    else:
        print("[TEST] WARN: marshmallow detected but outside expected 6–12 inch zone")
        print("[TEST] Check physical placement or update MARSHMALLOW_DIAMETER_MM in _manipulator_config.py")
