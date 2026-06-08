"""
_cup_scan_logic.py
==================
Pure geometry + height-ranking helpers for the hardcoded-cup vision scan,
deliberately free of any ``robot.*`` / ``rclpy`` import so the logic can be
unit-tested and simulated off-hardware.  All tunables (FOV, camera height,
offsets, the known height set) are passed in by the caller — this module
holds algorithm, not configuration.

Used by ``hardcoded_cup_pick_place_test.py`` (which wires in the real config
constants) and by ``sim_cup_scan.py`` (which drives it with synthetic frames).
"""
from __future__ import annotations

import math


def camera_bearing_to_cup(x_mm: float, y_mm: float, campan_forward_offset_mm: float) -> float:
    """Campan-frame bearing (deg) to a turntable-frame point, turntable at 0°."""
    return math.degrees(math.atan2(y_mm, x_mm - campan_forward_offset_mm))


def detection_center_bearing_deg(det: dict, img_w: int, hfov_deg: float) -> float:
    """Horizontal bearing (deg) of a detection's bbox centre from frame centre."""
    if img_w <= 0:
        return 0.0
    bbox = det["bbox"]
    cx = bbox["x"] + bbox["width"] / 2.0
    return ((cx / img_w) - 0.5) * hfov_deg


def vfov_deg(img_w: int, img_h: int, hfov_deg: float) -> float:
    """Vertical FOV from horizontal FOV and the frame aspect ratio."""
    if img_w <= 0:
        return 0.0
    hfov_rad = math.radians(hfov_deg)
    return math.degrees(2.0 * math.atan(math.tan(hfov_rad / 2.0) * img_h / img_w))


def pixel_y_to_height_mm(
    cy_px: float, img_w: int, img_h: int, dist_mm: float,
    hfov_deg: float, cam_height_mm: float,
) -> float:
    """Project an image row (pixels, 0 = top) to a robot-frame z height (mm).

    Inverse of :func:`height_mm_to_pixel_y`.  A row above the frame centre
    (smaller cy) maps to a higher elevation angle and thus a taller object.
    """
    if img_w <= 0 or img_h <= 0:
        return cam_height_mm
    elev_deg = -((cy_px - img_h / 2.0) / img_h) * vfov_deg(img_w, img_h, hfov_deg)
    return cam_height_mm + dist_mm * math.tan(math.radians(elev_deg))


def height_mm_to_pixel_y(
    height_mm: float, img_w: int, img_h: int, dist_mm: float,
    hfov_deg: float, cam_height_mm: float,
) -> float:
    """Project a robot-frame z height (mm) to an image row (pixels).

    Inverse of :func:`pixel_y_to_height_mm`; used by the simulator to render a
    physically-consistent bbox for a cup of known height at a known distance.
    """
    if img_w <= 0 or img_h <= 0 or dist_mm == 0.0:
        return img_h / 2.0
    elev_deg = math.degrees(math.atan2(height_mm - cam_height_mm, dist_mm))
    v = vfov_deg(img_w, img_h, hfov_deg)
    if v == 0.0:
        return img_h / 2.0
    return img_h / 2.0 - (elev_deg / v) * img_h


def cup_top_height_mm(
    det: dict, img_w: int, img_h: int, dist_mm: float,
    hfov_deg: float, cam_height_mm: float,
) -> float:
    """Estimate robot-frame z of the TOP edge of a red-cup bbox."""
    return pixel_y_to_height_mm(
        det["bbox"]["y"], img_w, img_h, dist_mm, hfov_deg, cam_height_mm)


def bbox_center_height_mm(
    det: dict, img_w: int, img_h: int, dist_mm: float,
    hfov_deg: float, cam_height_mm: float,
) -> float:
    """Estimate robot-frame z of a bbox CENTRE (used for the marshmallow fallback)."""
    bbox = det["bbox"]
    cy = bbox["y"] + bbox["height"] / 2.0
    return pixel_y_to_height_mm(cy, img_w, img_h, dist_mm, hfov_deg, cam_height_mm)


def assign_heights_by_rank(cups: list[dict], known_z_sorted: list[float]) -> bool:
    """Rank cups by measured ``top_mm`` and map onto ``known_z_sorted``.

    Shortest measured top → shortest known height, upward.  Sets ``z_mm`` on
    every cup and returns True only when all cups have a measured top and the
    counts match; otherwise returns False (caller must fall back).
    """
    measured = [c for c in cups if c.get("top_mm") is not None]
    if len(measured) != len(cups) or len(cups) != len(known_z_sorted):
        return False
    for rank, cup in enumerate(sorted(cups, key=lambda c: c["top_mm"])):
        cup["z_mm"] = known_z_sorted[rank]
    return True
