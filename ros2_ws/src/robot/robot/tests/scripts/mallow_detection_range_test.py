"""
mallow_detection_range_test.py
==============================
Camera detection-range characterisation. Pans the campan through five
positions across the full ±66° envelope, captures the vision node's
annotated frame at each, and prints the estimated marshmallow coordinates
(camera-derived bearing / distance / height) for every detection.

No arm motion: the script only enables the campan stepper. The shoulder,
elbow, and gripper are NOT commanded — put the arm in a safe stow pose
before running.

If the lidar is enabled, the script also lists any lidar clusters within
±MATCH_DEG of each mallow detection so you can cross-check the camera
distance against the lidar reading. The lidar mount may not be inverted
yet; this script does NOT correct for inversion. Treat lidar output as a
sanity check rather than ground truth until the mount is finalised.

Outputs
-------
  * `/runtime_output/mallow_detection_range/scan_<run_id>_pan<deg>.jpg`
    — one annotated frame per pan angle (only if the vision node is
    launched with `debug_save_enabled:=true`).
  * Console log of every detection with estimated coordinates.
  * Final summary table.

Nodes required: bridge (auto) + vision + (optional) lidar.
"""
from __future__ import annotations

import math
import shutil
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, StepMoveType
from robot.robot import FirmwareState, Robot
from robot.tests.scripts._manipulator_config import (
    CAMERA_FORWARD_OFFSET_MM,
    CAMERA_HEIGHT_MM,
    CAMERA_HFOV_DEG,
    CAMPAN_ACCELERATION,
    CAMPAN_MAX_VELOCITY,
    CAMPAN_SETTLE_S,
    CAMPAN_STEPPER,
    MARSHMALLOW_CLASS,
    MARSHMALLOW_DIAMETER_MM,
    MIN_CONFIDENCE_MARSHMALLOW,
    TURNTABLE_X_IN_BODY_MM,
    campan_deg_to_steps,
    snap_to_venue_tier_mm,
)

# ── Scan plan ────────────────────────────────────────────────────────────────
# Five pan positions spaced across the campan's physical envelope.
SCAN_CAMPAN_DEG = [60.0, 30.0, 0.0, -30.0, -60.0]

# Wait this long after panning before reading detections so the vision
# stream catches up with the new frame.
PAN_SETTLE_S = CAMPAN_SETTLE_S + 0.4

# Confidence threshold for what counts as a "real" detection in the log.
# Same default as the live demo; tighten or loosen at the top of the file
# without editing _manipulator_config.py.
DET_MIN_CONFIDENCE = MIN_CONFIDENCE_MARSHMALLOW

# Lidar matching tolerance — bearings (camera frame) within this much of a
# lidar-cluster bearing are treated as the same target.
LIDAR_MATCH_DEG = 12.0

# Annotated-frame destination (mirrors the path turntable_pick_place_test uses).
VISION_LATEST_JPG = Path("/runtime_output/vision/latest.jpg")
SCAN_SNAPSHOT_DIR = Path("/runtime_output/mallow_detection_range")


# ── Helpers ──────────────────────────────────────────────────────────────────

def _campan_to_deg(robot: Robot, target_deg: float) -> None:
    robot.step_move(CAMPAN_STEPPER, campan_deg_to_steps(target_deg),
                    StepMoveType.ABSOLUTE, blocking=False)
    time.sleep(1.0)


def _bbox_center(det: dict) -> tuple[float, float]:
    bbox = det["bbox"]
    cx = bbox["x"] + bbox["width"] / 2.0
    cy = bbox["y"] + bbox["height"] / 2.0
    return cx, cy


def _bearing_in_camera_frame_deg(det: dict, img_w: int) -> float:
    cx, _ = _bbox_center(det)
    return ((cx / img_w) - 0.5) * CAMERA_HFOV_DEG if img_w > 0 else 0.0


def _mallow_dist_from_camera_mm(det: dict, img_w: int) -> float:
    """Pinhole distance estimate: real diameter × focal_px / pixel diameter.

    Uses the geometric mean of bbox width/height as the pixel diameter — a
    mallow is roughly spherical-ish so the bbox shouldn't be wildly
    asymmetric; the geomean is robust to occasional clipping on one side.
    """
    bbox = det["bbox"]
    px_diam = math.sqrt(max(1.0, bbox["width"] * bbox["height"]))
    focal_px = (img_w / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG / 2.0))
    return (MARSHMALLOW_DIAMETER_MM * focal_px) / px_diam


def _height_from_bbox_mm(det: dict, img_w: int, img_h: int, dist_mm: float) -> float:
    """Mallow centre z in robot frame from bbox vertical position + estimated
    distance, using the camera's vertical FOV (derived from HFOV + aspect)."""
    _, cy = _bbox_center(det)
    hfov_rad = math.radians(CAMERA_HFOV_DEG)
    vfov_rad = 2.0 * math.atan(math.tan(hfov_rad / 2.0) * img_h / img_w)
    elev_deg = -((cy - img_h / 2.0) / img_h) * math.degrees(vfov_rad)
    return CAMERA_HEIGHT_MM + dist_mm * math.tan(math.radians(elev_deg))


def _camera_to_robot_frame(world_bearing_deg: float, dist_mm: float) -> tuple[float, float]:
    """Convert (bearing from camera, distance from camera) to turntable-frame xy."""
    rad = math.radians(world_bearing_deg)
    x = CAMERA_FORWARD_OFFSET_MM + dist_mm * math.cos(rad)
    y = dist_mm * math.sin(rad)
    return x, y


def _lidar_clusters_in_zone(robot: Robot) -> list[tuple[float, float]]:
    """Return raw lidar points converted to turntable frame, filtered to the
    mallow zone (100–600 mm from turntable axis, ±90° bearing).

    NOTE: this does NOT correct for an inverted lidar mount. If the lidar is
    flipped, y will be mirrored — the cross-check will read on the wrong
    side. Document this in the output.
    """
    raw = robot.get_lidar_obstacles_robot()
    filtered: list[tuple[float, float]] = []
    for bx, by in raw:
        x = bx - TURNTABLE_X_IN_BODY_MM
        y = by
        r = math.hypot(x, y)
        if not (100.0 <= r <= 600.0):
            continue
        bearing = math.degrees(math.atan2(y, x))
        if abs(bearing) > 90.0:
            continue
        filtered.append((x, y))
    return filtered


def _nearest_lidar_to_bearing(
    lidar_points: list[tuple[float, float]],
    target_bearing_deg: float,
    match_deg: float,
) -> tuple[float, float] | None:
    """Find the lidar point whose bearing is closest to target_bearing_deg
    AND within ``match_deg``. Returns (x, y) or None."""
    best: tuple[float, float] | None = None
    best_err = match_deg
    for x, y in lidar_points:
        bearing = math.degrees(math.atan2(y, x))
        err = abs(bearing - target_bearing_deg)
        if err < best_err:
            best_err = err
            best = (x, y)
    return best


def _save_snapshot(run_id: str, label: str) -> Path | None:
    if not VISION_LATEST_JPG.is_file():
        return None
    try:
        SCAN_SNAPSHOT_DIR.mkdir(parents=True, exist_ok=True)
        dest = SCAN_SNAPSHOT_DIR / f"scan_{run_id}_{label}.jpg"
        shutil.copyfile(VISION_LATEST_JPG, dest)
        return dest
    except OSError as exc:
        print(f"[TEST] WARNING: failed to save snapshot {label}: {exc}")
        return None


# ── Result records ───────────────────────────────────────────────────────────

@dataclass
class MallowHit:
    pan_deg: float
    conf: float
    cam_bearing_deg: float
    world_bearing_deg: float
    cam_dist_mm: float
    x_mm: float
    y_mm: float
    z_raw_mm: float
    z_snap_mm: float
    lidar_x_mm: float | None
    lidar_y_mm: float | None


# ── Main FSM ─────────────────────────────────────────────────────────────────

def run(robot: Robot) -> None:  # noqa: C901
    state       = "INIT"
    period      = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick   = time.monotonic()

    print("[TEST] ── Mallow Detection Range ────────────────────────────────")
    print("[TEST] No arm motion. Put the arm in a safe stow pose first.")
    print(f"[TEST] Will pan campan to: {SCAN_CAMPAN_DEG} degrees")
    print(f"[TEST] Min confidence for log: {DET_MIN_CONFIDENCE:.2f}")
    print( "[TEST] ───────────────────────────────────────────────────────────")

    hits: list[MallowHit] = []
    lidar_enabled = False

    while True:

        # ── INIT ─────────────────────────────────────────────────────────────
        if state == "INIT":
            if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.enable_vision()
            try:
                robot.enable_lidar()
                lidar_enabled = True
                print("[TEST] Lidar enabled — will cross-check camera distances.")
                print("[TEST] NOTE: lidar inversion compensation is OFF — if the "
                      "mount is flipped, lidar y will be mirrored.")
            except Exception as exc:
                lidar_enabled = False
                print(f"[TEST] Lidar not available ({exc}); camera-only mode.")
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 255)
            state = "IDLE"

        # ── IDLE ─────────────────────────────────────────────────────────────
        elif state == "IDLE":
            if robot.get_button(Button.BTN_2):
                robot.shutdown()
                return
            for n in (3, 2, 1):
                print(f"[TEST] Starting in {n}...")
                time.sleep(1.0)
            print("[TEST] GO")
            state = "ENABLE_CAMPAN"

        # ── ENABLE_CAMPAN ───────────────────────────────────────────────────
        elif state == "ENABLE_CAMPAN":
            robot.step_set_config(CAMPAN_STEPPER, CAMPAN_MAX_VELOCITY, CAMPAN_ACCELERATION)
            robot.step_enable(CAMPAN_STEPPER)
            _campan_to_deg(robot, 0.0)
            time.sleep(0.3)
            state = "SCAN"

        # ── SCAN ────────────────────────────────────────────────────────────
        elif state == "SCAN":
            img_w, img_h = robot.get_detection_image_size()
            run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
            print(f"[TEST] SCAN run_id={run_id}  image={img_w}x{img_h}")

            for pan_deg in SCAN_CAMPAN_DEG:
                _campan_to_deg(robot, pan_deg)
                time.sleep(PAN_SETTLE_S)

                snap = _save_snapshot(run_id, f"pan{int(round(pan_deg)):+03d}")
                snap_str = str(snap) if snap is not None else "(no snapshot — debug_save_enabled?)"

                mallow_dets = robot.get_detections(MARSHMALLOW_CLASS)
                lidar_points = _lidar_clusters_in_zone(robot) if lidar_enabled else []

                print(f"\n[SCAN] pan={pan_deg:+5.1f}°  "
                      f"detections={len(mallow_dets)}  "
                      f"lidar_pts={len(lidar_points)}  "
                      f"frame={snap_str}")

                if not mallow_dets:
                    print("[SCAN]   (no mallow detections at this pan)")
                    continue

                for det in mallow_dets:
                    conf = float(det["confidence"])
                    cam_bearing = _bearing_in_camera_frame_deg(det, img_w)
                    world_bearing = pan_deg + cam_bearing
                    cam_dist = _mallow_dist_from_camera_mm(det, img_w)
                    x_mm, y_mm = _camera_to_robot_frame(world_bearing, cam_dist)
                    z_raw = _height_from_bbox_mm(det, img_w, img_h, cam_dist)
                    z_snap = snap_to_venue_tier_mm(z_raw)

                    lidar_xy = _nearest_lidar_to_bearing(
                        lidar_points, world_bearing, LIDAR_MATCH_DEG,
                    )
                    lidar_x = lidar_xy[0] if lidar_xy is not None else None
                    lidar_y = lidar_xy[1] if lidar_xy is not None else None

                    keep = conf >= DET_MIN_CONFIDENCE
                    tag = "KEEP" if keep else "skip"
                    print(f"[SCAN]   {tag} conf={conf:.2f}  "
                          f"cam_bearing={cam_bearing:+5.1f}°  "
                          f"world_bearing={world_bearing:+5.1f}°  "
                          f"cam_dist={cam_dist:5.0f} mm")
                    print(f"[SCAN]     robot_xyz=({x_mm:+5.0f}, {y_mm:+5.0f}, "
                          f"{z_raw:+5.0f}) mm  tier_z={z_snap:+5.0f} mm")
                    if lidar_xy is not None:
                        lidar_dist = math.hypot(lidar_x, lidar_y)
                        # Distance from camera origin to compare with cam_dist:
                        dx = lidar_x - CAMERA_FORWARD_OFFSET_MM
                        dy = lidar_y
                        lidar_cam_dist = math.hypot(dx, dy)
                        delta = lidar_cam_dist - cam_dist
                        print(f"[SCAN]     lidar nearest pt: "
                              f"xy=({lidar_x:+5.0f}, {lidar_y:+5.0f}) mm  "
                              f"from_camera={lidar_cam_dist:5.0f} mm  "
                              f"(Δ vs camera: {delta:+5.0f} mm)")
                    else:
                        print("[SCAN]     no lidar point within "
                              f"±{LIDAR_MATCH_DEG:.0f}° of this bearing")

                    if keep:
                        hits.append(MallowHit(
                            pan_deg=pan_deg,
                            conf=conf,
                            cam_bearing_deg=cam_bearing,
                            world_bearing_deg=world_bearing,
                            cam_dist_mm=cam_dist,
                            x_mm=x_mm,
                            y_mm=y_mm,
                            z_raw_mm=z_raw,
                            z_snap_mm=z_snap,
                            lidar_x_mm=lidar_x,
                            lidar_y_mm=lidar_y,
                        ))

            _campan_to_deg(robot, 0.0)
            state = "REPORT"

        # ── REPORT ──────────────────────────────────────────────────────────
        elif state == "REPORT":
            print("\n[TEST] ── Summary ────────────────────────────────────────")
            if not hits:
                print("[TEST] No mallow detections passed the confidence threshold.")
            else:
                print(f"[TEST] {len(hits)} detection(s) kept "
                      f"(conf ≥ {DET_MIN_CONFIDENCE:.2f}):\n")
                print("  pan°   conf  world_brg  cam_dist  robot_xyz (mm)              tier_z  lidar_xy")
                print("  ─────  ────  ─────────  ────────  ──────────────────────────  ──────  ─────────────")
                for h in hits:
                    if h.lidar_x_mm is not None:
                        lidar_str = f"({h.lidar_x_mm:+5.0f},{h.lidar_y_mm:+5.0f})"
                    else:
                        lidar_str = "       —"
                    print(f"  {h.pan_deg:+5.1f}  {h.conf:.2f}  "
                          f"{h.world_bearing_deg:+8.1f}°  "
                          f"{h.cam_dist_mm:7.0f}   "
                          f"({h.x_mm:+5.0f},{h.y_mm:+5.0f},{h.z_raw_mm:+5.0f})    "
                          f"{h.z_snap_mm:+5.0f}  {lidar_str}")
                best = max(hits, key=lambda h: h.conf)
                print(f"\n[TEST] Highest-confidence pick: "
                      f"world_bearing={best.world_bearing_deg:+.1f}°  "
                      f"xyz=({best.x_mm:+.0f}, {best.y_mm:+.0f}, {best.z_snap_mm:+.0f}) mm  "
                      f"conf={best.conf:.2f}")
            print( "[TEST] ──────────────────────────────────────────────────────")
            robot.step_disable(CAMPAN_STEPPER)
            state = "DONE"

        # ── DONE ────────────────────────────────────────────────────────────
        elif state == "DONE":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            robot.stop()

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
