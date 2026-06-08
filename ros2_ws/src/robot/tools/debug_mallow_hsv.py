"""
debug_mallow_hsv.py
===================
Diagnose why detect_marshmallow() is or isn't firing on a saved snapshot.

Usage:
    python3 ros2_ws/src/robot/tools/debug_mallow_hsv.py \
        ros2_ws/runtime_output/mallow_detection_range/scan_<run_id>_pan+00.jpg

What it does:
  1. Loads the JPG.
  2. Runs the live detect_marshmallow() from
     ros2_ws/src/vision/vision/rule_based_detection.py and prints the
     detections (or lack thereof).
  3. Also recomputes the HSV mask manually and reports how many pixels
     match each loosening of the range — so you can see if the bottleneck
     is hue, saturation, or value.
  4. Samples the HSV histogram of the central 30 % of the frame (where
     the mallow usually sits) and prints the dominant H/S/V values.
  5. Writes <snapshot>_mask.png next to the input so you can visually
     check what the current mask captures.

No ROS, no rebuild, no relaunch — pure OpenCV/numpy, runs in any env
with cv2 installed (e.g. inside the ros2_runtime container).
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import cv2
import numpy as np

# Parse the live HSV range and shape thresholds straight out of
# rule_based_detection.py — no Python import, so we don't drag in `ncnn`
# (which is only installed inside the ros2_runtime container).
THIS_DIR = Path(__file__).resolve().parent           # .../ros2_ws/src/robot/tools
SRC_DIR  = THIS_DIR.parent.parent                    # .../ros2_ws/src
DETECTOR_PATH = SRC_DIR / "vision" / "vision" / "rule_based_detection.py"


def _extract_current_hsv_range() -> tuple[tuple[int, int, int], tuple[int, int, int]]:
    """Scrape purple_low / purple_high from the live detector source.

    Falls back to a sane default if the regex misses (e.g. detector got
    rewritten).  That keeps this script useful even after refactors.
    """
    default_low  = (120,  60,  50)
    default_high = (160, 255, 255)
    try:
        text = DETECTOR_PATH.read_text()
    except OSError:
        return default_low, default_high

    def _grab(name: str) -> tuple[int, int, int] | None:
        m = re.search(
            name + r"\s*=\s*np\.array\(\[\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\]",
            text,
        )
        return (int(m.group(1)), int(m.group(2)), int(m.group(3))) if m else None

    return (_grab("purple_low") or default_low,
            _grab("purple_high") or default_high)


def _central_roi(frame_bgr: np.ndarray, frac: float = 0.30) -> np.ndarray:
    h, w = frame_bgr.shape[:2]
    rw, rh = int(w * frac), int(h * frac)
    x0 = (w - rw) // 2
    y0 = (h - rh) // 2
    return frame_bgr[y0 : y0 + rh, x0 : x0 + rw]


def _hsv_summary(label: str, hsv_pixels: np.ndarray) -> None:
    h_vals = hsv_pixels[..., 0].ravel()
    s_vals = hsv_pixels[..., 1].ravel()
    v_vals = hsv_pixels[..., 2].ravel()
    print(f"  {label:<22} H  median={np.median(h_vals):3.0f}  "
          f"p10={np.percentile(h_vals, 10):3.0f}  p90={np.percentile(h_vals, 90):3.0f}")
    print(f"  {'':<22} S  median={np.median(s_vals):3.0f}  "
          f"p10={np.percentile(s_vals, 10):3.0f}  p90={np.percentile(s_vals, 90):3.0f}")
    print(f"  {'':<22} V  median={np.median(v_vals):3.0f}  "
          f"p10={np.percentile(v_vals, 10):3.0f}  p90={np.percentile(v_vals, 90):3.0f}")


def _count_mask(hsv: np.ndarray, low: tuple, high: tuple) -> int:
    return int(cv2.inRange(hsv, np.array(low, dtype=np.uint8),
                            np.array(high, dtype=np.uint8)).sum() // 255)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", help="Path to a saved annotated frame (jpg)")
    args = parser.parse_args()

    snap = Path(args.snapshot)
    if not snap.is_file():
        print(f"ERROR: snapshot not found: {snap}")
        return 1

    frame = cv2.imread(str(snap))
    if frame is None:
        print(f"ERROR: cv2.imread returned None for: {snap}")
        return 1

    h, w = frame.shape[:2]
    print(f"Loaded {snap.name}  ({w}x{h})")

    cur_low, cur_high = _extract_current_hsv_range()
    print(f"Live detector HSV range from {DETECTOR_PATH.name}: "
          f"low={cur_low}  high={cur_high}")

    # ── 1. Recreate detect_marshmallow's pipeline on this frame ────────────
    # Same morphology + shape filters as rule_based_detection.detect_marshmallow.
    blurred_full = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv_full     = cv2.cvtColor(blurred_full, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_full, np.array(cur_low,  dtype=np.uint8),
                                  np.array(cur_high, dtype=np.uint8))
    open_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  open_kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, close_kernel)

    min_area_px    = 400
    max_area_px    = int(w * h * 0.12)
    min_fill_ratio = 0.30
    max_aspect     = 3.0

    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    print(f"\n[1] mask contours found: {len(contours)}")
    kept = 0
    for i, c in enumerate(contours):
        area = float(cv2.contourArea(c))
        x, y, ww, hh = cv2.boundingRect(c)
        bbox_area  = float(max(1, ww * hh))
        fill_ratio = area / bbox_area
        aspect = max(ww, hh) / float(max(1, min(ww, hh)))
        verdict = []
        if area < min_area_px:        verdict.append(f"area<{min_area_px}")
        if area > max_area_px:        verdict.append(f"area>{max_area_px}")
        if fill_ratio < min_fill_ratio: verdict.append(f"fill<{min_fill_ratio:.2f}")
        if aspect > max_aspect:       verdict.append(f"aspect>{max_aspect:.1f}")
        status = "KEEP" if not verdict else "drop(" + ",".join(verdict) + ")"
        print(f"    #{i}: bbox=({x},{y},{ww}x{hh}) area={area:.0f} "
              f"fill={fill_ratio:.2f} aspect={aspect:.2f}  -> {status}")
        if not verdict:
            kept += 1
    print(f"    => {kept} detection(s) would be returned by detect_marshmallow().")

    # ── 2. HSV summary on the central ROI ──────────────────────────────────
    roi_hsv = cv2.cvtColor(cv2.GaussianBlur(_central_roi(frame), (5, 5), 0),
                            cv2.COLOR_BGR2HSV)
    print("\n[2] HSV values across the central 30% of the frame "
          "(where the mallow usually sits):")
    _hsv_summary("central ROI", roi_hsv)

    # ── 3. Mask pixel counts for progressively looser ranges ───────────────
    print("\n[3] Mask pixel counts (how many pixels would the mask catch?):")
    cur_label = f"current detector  H{cur_low[0]}-{cur_high[0]} S{cur_low[1]}+  V{cur_low[2]}+"
    ranges = [
        (cur_label, cur_low, cur_high),
        ("loosen S         H120-160 S30+  V50+",  (120,  30,  50), (160, 255, 255)),
        ("loosen V         H120-160 S60+  V30+",  (120,  60,  30), (160, 255, 255)),
        ("widen H low      H100-160 S60+  V50+",  (100,  60,  50), (160, 255, 255)),
        ("widen H high     H120-175 S60+  V50+",  (120,  60,  50), (175, 255, 255)),
        ("very wide        H100-175 S25+  V25+",  (100,  25,  25), (175, 255, 255)),
        ("blue-only        H100-130 S40+  V40+",  (100,  40,  40), (130, 255, 255)),
        ("magenta-only     H140-175 S40+  V40+",  (140,  40,  40), (175, 255, 255)),
    ]
    for label, low, high in ranges:
        n = _count_mask(hsv_full, low, high)
        pct = 100.0 * n / (h * w)
        print(f"  {label:<42}  {n:>8d} px  ({pct:5.2f}% of frame)")

    # ── 4. Save mask of current detector range for visual inspection ───────
    out_raw   = snap.with_name(snap.stem + "_mask_raw.png")
    out_clean = snap.with_name(snap.stem + "_mask_cleaned.png")
    cv2.imwrite(str(out_raw),   mask)
    cv2.imwrite(str(out_clean), mask_clean)
    print(f"\n[4] Wrote masks (white = pixels that match current detector):")
    print(f"      {out_raw}")
    print(f"      {out_clean}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
