"""
campan_detect_ik_test.py
========================
Pan the camera stepper through the three scan positions (-60°, 0°, +60°),
capture and save an annotated PNG at each position, then report the full
detection chain for every marshmallow found.

Per-marshmallow output:
  · Camera bearing and elevation (deg)
  · Pinhole distance estimate from camera (mm / in)
  · Height estimate snapped to the nearest cup-tier (mm)
  · Robot-frame position (x, y, z) in the turntable coordinate system (mm)
  · IK joint angles: turntable_deg, shoulder_servo_deg, elbow_servo_deg
    — or UNREACHABLE with the workspace-violation reason

Three PNG files are saved to /runtime_output/campan_detect_ik/:
  scan_neg60deg.png  /  scan_0deg.png  /  scan_pos60deg.png
Each image has bounding boxes for all detected objects (marshmallows in green,
cups in orange, everything else in blue).

Physical motion: campan stepper only.
All other actuators (turntable, shoulder, elbow, gripper, wheels) are idle.

Nodes required: robot + vision (must publish /vision/detections).
Launch everything yourself, then run robot — this script is called from main.py.
"""
from __future__ import annotations

import math
import time
from pathlib import Path

import cv2

from robot.arm_kinematics import OutOfReachError, detection_to_robot_frame, inverse_kinematics
from robot.hardware_map import StepMoveType
from robot.robot import FirmwareState, Robot

from robot.tests.scripts._manipulator_config import (
    ARM_GEOMETRY,
    CAMERA_HEIGHT_MM,
    CAMERA_HFOV_DEG,
    CAMPAN_ACCELERATION,
    CAMPAN_MAX_VELOCITY,
    CAMPAN_POSITIONS_DEG,
    CAMPAN_SETTLE_S,
    CAMPAN_STEPPER,
    CUP_CLASS,
    MARSHMALLOW_CLASS,
    MARSHMALLOW_DIAMETER_MM,
    MIN_CONFIDENCE_CUP,
    MIN_CONFIDENCE_MARSHMALLOW,
    VISION_WAIT_S,
    campan_deg_to_steps,
    snap_to_cup_tier_mm,
    turntable_deg_to_steps,
)

MOVE_TIMEOUT_S = 10.0
CAMERA_DEVICE  = "/dev/video10"
SAVE_DIR       = Path("/runtime_output/campan_detect_ik")

# Extra wait after settle so the vision node processes a fresh frame at each position.
VISION_FRAME_LAG_S = 0.2


# ── Helpers ───────────────────────────────────────────────────────────────────

def _capture_frame(device: str) -> "cv2.Mat | None":
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[TEST] WARNING: could not open {device}")
        return None
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.read()  # discard stale buffered frame
    ok, frame = cap.read()
    cap.release()
    if not ok:
        return None
    return cv2.flip(frame, -1)  # camera is mounted upside-down


def _angle_label(deg: float) -> str:
    if deg < 0:
        return f"neg{abs(deg):.0f}"
    if deg > 0:
        return f"pos{deg:.0f}"
    return "0"


def _pinhole_geometry(
    bbox: dict,
    img_w: int,
    img_h: int,
    real_diam_mm: float,
) -> tuple[float, float, float]:
    """Return (bearing_deg, elevation_deg, dist_mm) for a single bounding box."""
    cx = bbox["x"] + bbox["width"]  / 2.0
    cy = bbox["y"] + bbox["height"] / 2.0

    hfov_rad = math.radians(CAMERA_HFOV_DEG)
    vfov_rad = 2.0 * math.atan(math.tan(hfov_rad / 2.0) * img_h / img_w)

    bearing_deg   =  ((cx - img_w / 2.0) / img_w)  * CAMERA_HFOV_DEG
    elevation_deg = -((cy - img_h / 2.0) / img_h)  * math.degrees(vfov_rad)

    focal_px    = (img_w / 2.0) / math.tan(hfov_rad / 2.0)
    px_diameter = math.sqrt(max(1.0, bbox["width"] * bbox["height"]))
    dist_mm     = (real_diam_mm * focal_px) / px_diameter

    return bearing_deg, elevation_deg, dist_mm


def _draw_detections(frame: "cv2.Mat", detections: list[dict]) -> "cv2.Mat":
    """Overlay bounding boxes and labels on a copy of the frame."""
    out = frame.copy()
    for det in detections:
        bbox = det.get("bbox")
        if not bbox:
            continue
        x, y, w, h = int(bbox["x"]), int(bbox["y"]), int(bbox["width"]), int(bbox["height"])
        cls  = det.get("class_name", "?")
        conf = float(det.get("confidence", 0.0))

        if cls == MARSHMALLOW_CLASS:
            color = (0, 220, 0)       # green
        elif cls == CUP_CLASS:
            color = (0, 140, 255)     # orange
        else:
            color = (255, 120, 0)     # blue

        cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)
        label = f"{cls} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(out, (x, y - th - 6), (x + tw + 4, y), color, cv2.FILLED)
        cv2.putText(out, label, (x + 2, y - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
    return out


# ── Per-position processing ───────────────────────────────────────────────────

def _process_position(
    robot: Robot,
    campan_deg: float,
    frame: "cv2.Mat | None",
    img_w: int,
    img_h: int,
) -> list[dict]:
    """
    Fetch detections, annotate and save the frame, then log + return all
    marshmallow results (position + IK) for this campan position.
    """
    all_dets = robot.get_detections()
    print(f"[TEST]   {len(all_dets)} detection(s) at campan={campan_deg:+.1f}°:")
    for d in all_dets:
        print(f"[TEST]     class={d['class_name']}  conf={float(d['confidence']):.2f}"
              f"  bbox={d.get('bbox')}")

    # Save annotated image
    if frame is not None:
        annotated = _draw_detections(frame, all_dets)
        save_path = SAVE_DIR / f"scan_{_angle_label(campan_deg)}deg.png"
        cv2.imwrite(str(save_path), annotated)
        print(f"[TEST]   image saved → {save_path}")
    else:
        print("[TEST]   WARNING: no frame captured — image not saved")

    # Process marshmallows
    mallows = [d for d in all_dets
               if d["class_name"] == MARSHMALLOW_CLASS
               and float(d["confidence"]) >= MIN_CONFIDENCE_MARSHMALLOW]

    results: list[dict] = []
    for i, det in enumerate(mallows, 1):
        bbox     = det["bbox"]
        conf     = float(det["confidence"])
        bearing, elevation, dist_mm = _pinhole_geometry(bbox, img_w, img_h, MARSHMALLOW_DIAMETER_MM)

        # Z: camera-model raw height, then snapped to the nearest cup-tier
        z_raw     = CAMERA_HEIGHT_MM + dist_mm * math.tan(math.radians(elevation))
        z_snapped = snap_to_cup_tier_mm(z_raw)

        # Convert to turntable-frame robot coordinates
        x_mm, y_mm, z_mm = detection_to_robot_frame(
            distance_mm=dist_mm,
            bearing_deg=bearing,
            height_mm=z_snapped,
            geom=ARM_GEOMETRY,
            campan_deg=campan_deg,
        )

        print(f"[TEST]   --- marshmallow #{i}  conf={conf:.2f} ---")
        print(f"[TEST]     camera:       bearing={bearing:+.1f}°  elevation={elevation:+.1f}°"
              f"  dist={dist_mm:.0f} mm ({dist_mm / 25.4:.1f} in)")
        print(f"[TEST]     height:       raw={z_raw:.0f} mm  →  snapped={z_snapped:.0f} mm")
        print(f"[TEST]     robot frame:  x={x_mm:.0f} mm  y={y_mm:.0f} mm  z={z_mm:.0f} mm")

        try:
            tt_deg, sh_deg, el_deg = inverse_kinematics(x_mm, y_mm, z_mm, ARM_GEOMETRY)
            tt_steps = turntable_deg_to_steps(tt_deg)
            print(f"[TEST]     IK angles:    turntable={tt_deg:+.1f}° ({tt_steps} steps)"
                  f"  shoulder={sh_deg:.1f}°  elbow={el_deg:.1f}°")
            results.append({
                "campan_deg":    campan_deg,
                "bearing":       bearing,
                "elevation":     elevation,
                "dist_mm":       dist_mm,
                "x_mm":          x_mm,
                "y_mm":          y_mm,
                "z_mm":          z_mm,
                "turntable_deg": tt_deg,
                "tt_steps":      tt_steps,
                "shoulder_deg":  sh_deg,
                "elbow_deg":     el_deg,
                "conf":          conf,
            })
        except OutOfReachError as exc:
            print(f"[TEST]     IK angles:    UNREACHABLE — {exc}")

    return results


# ── Main entry point ──────────────────────────────────────────────────────────

def run(robot: Robot) -> None:
    print("[TEST] campan_detect_ik_test")
    print(f"[TEST] Scan positions: {CAMPAN_POSITIONS_DEG} deg")
    print("[TEST] Motion: campan stepper only — arm, turntable, and wheels stay idle")

    SAVE_DIR.mkdir(parents=True, exist_ok=True)

    # Enable vision
    robot.set_state(FirmwareState.RUNNING)
    robot.enable_vision()

    print(f"[TEST] Waiting up to {VISION_WAIT_S}s for vision node...")
    deadline = time.monotonic() + VISION_WAIT_S
    while time.monotonic() < deadline:
        if robot.is_vision_active(timeout_s=1.0):
            break
        time.sleep(0.5)
    else:
        print("[TEST] FAIL: vision node not active — launch it before running robot")
        return

    img_w, img_h = robot.get_detection_image_size()
    print(f"[TEST] Vision active — frame {img_w}×{img_h}")

    # Configure and enable campan stepper
    robot.step_set_config(
        CAMPAN_STEPPER,
        max_velocity=CAMPAN_MAX_VELOCITY,
        acceleration=CAMPAN_ACCELERATION,
    )
    robot.step_enable(CAMPAN_STEPPER)
    time.sleep(0.1)

    # Sweep all three scan positions
    all_results: list[dict] = []
    for campan_deg in CAMPAN_POSITIONS_DEG:
        steps = campan_deg_to_steps(campan_deg)
        print(f"\n[TEST] ── pan to {campan_deg:+.1f}° ({steps} steps) ──")

        ok = robot.step_move(CAMPAN_STEPPER, steps, StepMoveType.ABSOLUTE, timeout=MOVE_TIMEOUT_S)
        if not ok:
            print(f"[TEST] FAIL: campan timed out at {campan_deg:+.1f}° — aborting")
            robot.step_disable(CAMPAN_STEPPER)
            return

        time.sleep(CAMPAN_SETTLE_S + VISION_FRAME_LAG_S)

        frame   = _capture_frame(CAMERA_DEVICE)
        results = _process_position(robot, campan_deg, frame, img_w, img_h)
        all_results.extend(results)

    # Return to center
    print("\n[TEST] ── returning campan to 0° ──")
    ok = robot.step_move(CAMPAN_STEPPER, 0, StepMoveType.ABSOLUTE, timeout=MOVE_TIMEOUT_S)
    if not ok:
        print("[TEST] WARNING: campan did not return to center")
    else:
        time.sleep(CAMPAN_SETTLE_S)
    robot.step_disable(CAMPAN_STEPPER)

    # Final summary
    print()
    print(f"[TEST] ══ SUMMARY ══  {len(all_results)} IK-reachable marshmallow(s) across 3 frames")

    if not all_results:
        print("[TEST] No marshmallows detected. Check lighting, placement, and MIN_CONFIDENCE_MARSHMALLOW.")
        print("[TEST] DONE — no arm motion was commanded.")
        return

    print("[TEST] All marshmallow detections:")
    for r in all_results:
        print(f"[TEST]   campan={r['campan_deg']:+.1f}°  conf={r['conf']:.2f}"
              f"  bearing={r['bearing']:+.1f}°  dist={r['dist_mm']:.0f} mm"
              f"  →  x={r['x_mm']:.0f} y={r['y_mm']:.0f} z={r['z_mm']:.0f} mm"
              f"  →  tt={r['turntable_deg']:+.1f}° sh={r['shoulder_deg']:.1f}° el={r['elbow_deg']:.1f}°")

    best = max(all_results, key=lambda r: r["conf"])
    print()
    print(f"[TEST] Best candidate:  conf={best['conf']:.2f}  campan={best['campan_deg']:+.1f}°  bearing={best['bearing']:+.1f}°")
    print(f"[TEST]   distance      : {best['dist_mm']:.0f} mm ({best['dist_mm'] / 25.4:.1f} in)")
    print(f"[TEST]   robot frame   : x={best['x_mm']:.0f} mm  y={best['y_mm']:.0f} mm  z={best['z_mm']:.0f} mm")
    print(f"[TEST]   IK angles     : turntable={best['turntable_deg']:+.1f}°  shoulder={best['shoulder_deg']:.1f}°  elbow={best['elbow_deg']:.1f}°")
    print(f"[TEST]   IK steps      : turntable = {best['tt_steps']} steps")
    print(f"[TEST]   PNG frames    : {SAVE_DIR}")
    print("[TEST] DONE — no arm motion was commanded.")
