"""
marshmallow_pick_solver_test.py
===============================
Report-only solver: for every marshmallow the vision node currently sees,
print the three things needed to pick it up — WITHOUT moving the arm.

  1. position (x, y, z) in the turntable robot frame  [mm]
  2. turntable angle to point the claw at it           [deg + stepper steps]
  3. claw pose: shoulder / elbow servo angles + gripper open->grab, with reach check

Depth caveat: the Pi camera is 2-D. `distance` is a pinhole estimate from the
bounding box and MARSHMALLOW_DIAMETER_MM; `z` is estimated from the bbox
elevation + distance + CAMERA_HEIGHT_MM, then snapped to the nearest known
cup-tier height. Geometry/servo constants in _manipulator_config.py are mostly
placeholders ("TODO: measure") — the numbers are only as good as that calibration.

Run (inside the container, with the vision node publishing /vision/detections):
    python3 /ros2_ws/src/robot/robot/tests/scripts/marshmallow_pick_solver_test.py

Or dispatch through main.py:
    from robot.tests.scripts.marshmallow_pick_solver_test import run
"""
from __future__ import annotations

import math
import time

from robot.arm_kinematics import OutOfReachError, detection_to_robot_frame, inverse_kinematics
from robot.robot import Robot

try:
    from robot.tests.scripts._manipulator_config import (
        ARM_GEOMETRY,
        CAMERA_HEIGHT_MM,
        CAMERA_HFOV_DEG,
        GRIPPER_GRAB_DEG,
        GRIPPER_OPEN_DEG,
        MARSHMALLOW_CLASS,
        MARSHMALLOW_DIAMETER_MM,
        VISION_WAIT_S,
        snap_to_cup_tier_mm,
        turntable_deg_to_steps,
    )
except ImportError:  # pragma: no cover - direct-script fallback
    from _manipulator_config import (  # type: ignore
        ARM_GEOMETRY,
        CAMERA_HEIGHT_MM,
        CAMERA_HFOV_DEG,
        GRIPPER_GRAB_DEG,
        GRIPPER_OPEN_DEG,
        MARSHMALLOW_CLASS,
        MARSHMALLOW_DIAMETER_MM,
        VISION_WAIT_S,
        snap_to_cup_tier_mm,
        turntable_deg_to_steps,
    )

# Match the vision-node display filter we set earlier.
MIN_CONFIDENCE = 0.70
# Camera pan angle (positive = right). 0 for a static camera; pass the campan
# stepper angle here if the camera has been panned off-centre.
CAMPAN_DEG = 0.0


def _bbox_geometry(bbox: dict, img_w: int, img_h: int) -> tuple[float, float, float, float, float]:
    """Pinhole model. Returns (bearing_deg, elevation_deg, dist_mm, cx_px, cy_px).

    bearing_deg  : + = right of image centre
    elevation_deg: + = above image centre
    dist_mm      : range estimate from known marshmallow diameter
    """
    cx = bbox["x"] + bbox["width"] / 2.0
    cy = bbox["y"] + bbox["height"] / 2.0

    hfov = math.radians(CAMERA_HFOV_DEG)
    vfov = 2.0 * math.atan(math.tan(hfov / 2.0) * img_h / img_w)

    bearing_deg = ((cx - img_w / 2.0) / img_w) * CAMERA_HFOV_DEG
    elevation_deg = -((cy - img_h / 2.0) / img_h) * math.degrees(vfov)

    focal_px = (img_w / 2.0) / math.tan(hfov / 2.0)
    px_diameter = math.sqrt(max(1.0, bbox["width"] * bbox["height"]))
    dist_mm = (MARSHMALLOW_DIAMETER_MM * focal_px) / px_diameter

    return bearing_deg, elevation_deg, dist_mm, cx, cy


def _solve_one(det: dict, img_w: int, img_h: int) -> None:
    conf = float(det["confidence"])
    bbox = det["bbox"]
    bearing, elevation, dist_mm, cx, cy = _bbox_geometry(bbox, img_w, img_h)

    # Height (z): rise above the camera centreline over the measured distance,
    # added to camera height, then snapped to the nearest known cup-tier height.
    z_raw = CAMERA_HEIGHT_MM + dist_mm * math.tan(math.radians(elevation))
    z_snapped = snap_to_cup_tier_mm(z_raw)

    x_mm, y_mm, z_mm = detection_to_robot_frame(
        distance_mm=dist_mm,
        bearing_deg=bearing,
        height_mm=z_snapped,
        geom=ARM_GEOMETRY,
        campan_deg=CAMPAN_DEG,
    )

    print(f"[SOLVE]   conf={conf:.2f}  pixel=({cx:.0f},{cy:.0f})  "
          f"bearing={bearing:+.1f}deg  elevation={elevation:+.1f}deg  "
          f"cam_dist={dist_mm:.0f}mm ({dist_mm / 25.4:.1f}in)")
    print(f"[SOLVE]   1) position (turntable frame): "
          f"x={x_mm:.0f}mm  y={y_mm:.0f}mm  z={z_mm:.0f}mm  (z snapped from {z_raw:.0f}mm)")

    try:
        turntable_deg, shoulder_servo, elbow_servo = inverse_kinematics(x_mm, y_mm, z_mm, ARM_GEOMETRY)
    except OutOfReachError as exc:
        bearing_only = math.degrees(math.atan2(y_mm, x_mm))
        print(f"[SOLVE]   2) turntable angle: {bearing_only:+.1f}deg")
        print(f"[SOLVE]   3) claw pose: UNREACHABLE — {exc}")
        return

    steps = turntable_deg_to_steps(turntable_deg)
    print(f"[SOLVE]   2) turntable angle: {turntable_deg:+.1f}deg  ({steps} steps)")
    print(f"[SOLVE]   3) claw pose: shoulder={shoulder_servo:.1f}deg  elbow={elbow_servo:.1f}deg  "
          f"gripper open={GRIPPER_OPEN_DEG:.0f}deg -> grab={GRIPPER_GRAB_DEG:.0f}deg")


def run(robot: Robot) -> None:
    print("[SOLVE] marshmallow_pick_solver — REPORT ONLY (the arm will NOT move)")
    robot.enable_vision()

    deadline = time.monotonic() + VISION_WAIT_S
    while time.monotonic() < deadline:
        if robot.is_vision_active(timeout_s=1.0):
            break
        time.sleep(0.5)
    else:
        print("[SOLVE] FAIL: vision not active — is the vision node publishing /vision/detections?")
        return

    img_w, img_h = robot.get_detection_image_size()
    print(f"[SOLVE] vision active — frame {img_w}x{img_h}")

    candidates = [
        d for d in robot.get_detections(class_name=MARSHMALLOW_CLASS)
        if float(d["confidence"]) >= MIN_CONFIDENCE
    ]
    if not candidates:
        print(f"[SOLVE] no '{MARSHMALLOW_CLASS}' >= {MIN_CONFIDENCE:.2f} in view")
        return

    print(f"[SOLVE] {len(candidates)} marshmallow(s) >= {MIN_CONFIDENCE:.2f}:")
    for i, det in enumerate(candidates, 1):
        print(f"[SOLVE] --- marshmallow #{i} ---")
        _solve_one(det, img_w, img_h)
    print("[SOLVE] done — report only, no motion commanded.")


def main() -> None:
    """Standalone entry point: build a Robot like robot_node does, then run()."""
    import threading

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("marshmallow_pick_solver")
    robot = Robot(node)

    def _spin() -> None:
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()
    try:
        run(robot)
    finally:
        try:
            robot.shutdown()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
