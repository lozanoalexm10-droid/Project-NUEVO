"""
sim_cup_scan.py
===============
Off-hardware harness for the hardcoded-cup vision scan.  Exercises the SAME
``_cup_scan_logic`` functions the robot uses, with no ROS required:

  Part A — real detection: run detect_red_cup / detect_marshmallow on a saved
           camera frame to prove the detection stage produces the bboxes the
           scan consumes.
  Part B — tiering + selection: for randomized cup layouts, synthesize
           physically-consistent red-cup bboxes (project each stack's true top
           height to a pixel row), add pixel noise, then run the real ranking +
           target-selection logic and check it recovers the right heights and
           the right marshmallow cup.

Run on the host:
    python3 ros2_ws/src/robot/robot/tests/scripts/sim_cup_scan.py
"""
from __future__ import annotations

import math
import os
import random
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _cup_scan_logic import (  # noqa: E402
    assign_heights_by_rank,
    cup_top_height_mm,
    height_mm_to_pixel_y,
)

# ── Constants mirrored from _manipulator_config.py (kept ROS-free here) ────────
CAMERA_HFOV_DEG          = 62.0
CAMERA_HEIGHT_MM         = -40.0
CAMERA_FORWARD_OFFSET_MM = 67.0 + 142.0          # ROBOT_FRONT_TO_TURNTABLE + PROTRUSION = 209
GROUND_Z_MM              = -247.0
CARDBOARD_MM             = 3.0
MALLOW_HALF_MM           = 14.0                  # mallow centre above the rim
IMG_W, IMG_H             = 1280, 720

# 9/11/14/16-cup stacks → mallow-centre z, sorted shortest→tallest.
STACK_HEIGHTS_MM = {9: 168.0, 11: 180.0, 14: 200.0, 16: 214.0}
MALLOW_Z = {n: GROUND_Z_MM + CARDBOARD_MM + h + MALLOW_HALF_MM for n, h in STACK_HEIGHTS_MM.items()}
KNOWN_MALLOW_Z_MM_SORTED = sorted(MALLOW_Z.values())

# The four hardcoded cup positions (camera-frame front_mm, side_mm).
HARDCODED_CUPS_CAMERA_MM = [(90.0, 204.0), (178.0, 103.0), (201.0, -15.6), (122.0, -183.0)]


def _cup_xy(front_mm: float, side_mm: float) -> tuple[float, float]:
    return front_mm + CAMERA_FORWARD_OFFSET_MM, -side_mm


def _synth_red_cup_det(rim_top_z_mm: float, dist_mm: float, noise_px: float) -> dict:
    """Render the red-cup bbox a camera would see for a stack of this rim height."""
    cy_top = height_mm_to_pixel_y(
        rim_top_z_mm, IMG_W, IMG_H, dist_mm, CAMERA_HFOV_DEG, CAMERA_HEIGHT_MM)
    cy_top += random.gauss(0.0, noise_px)
    return {"bbox": {"x": IMG_W / 2 - 90, "y": cy_top, "width": 180, "height": 160}}


def _run_scan(layout: dict[int, int], mallow_pos: int, noise_px: float) -> dict:
    """Simulate one full scan.

    layout: cup index (0-3) → stack size (9/11/14/16)
    Returns the result with per-cup tiering and the selected target.
    """
    cups = []
    for i, (front, side) in enumerate(HARDCODED_CUPS_CAMERA_MM):
        x, y = _cup_xy(front, side)
        dist = math.hypot(x - CAMERA_FORWARD_OFFSET_MM, y)
        true_mallow_z = MALLOW_Z[layout[i]]
        rim_top_z = true_mallow_z - MALLOW_HALF_MM
        det = _synth_red_cup_det(rim_top_z, dist, noise_px)
        cups.append({
            "id": float(i + 1), "x_mm": x, "y_mm": y,
            "top_mm": cup_top_height_mm(det, IMG_W, IMG_H, dist, CAMERA_HFOV_DEG, CAMERA_HEIGHT_MM),
            "true_z": true_mallow_z, "stack": layout[i],
            "mallow_conf": 0.88 if i == mallow_pos else 0.0,
        })
    ranked_ok = assign_heights_by_rank(cups, KNOWN_MALLOW_Z_MM_SORTED)
    target = max(cups, key=lambda c: c["mallow_conf"])
    return {"cups": cups, "ranked_ok": ranked_ok, "target": target}


def part_a_real_detection() -> None:
    print("=" * 70)
    print("PART A — real detectors on a saved camera frame")
    print("=" * 70)
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), *([".."] * 6)))
    frame = os.path.join(repo_root, "vision_latest.jpg")
    try:
        import cv2
        sys.modules.setdefault("ncnn", types.ModuleType("ncnn"))  # detect_* never uses it
        vision_root = os.path.join(repo_root, "ros2_ws", "src", "vision")
        sys.path.insert(0, vision_root)
        from vision.rule_based_detection import detect_marshmallow, detect_red_cup
        img = cv2.imread(frame)
        if img is None:
            print(f"  (no frame at {frame} — skipping real detection)")
            return
        cups, _ = detect_red_cup(img)
        malls, _ = detect_marshmallow(img)
        print(f"  frame {img.shape[1]}x{img.shape[0]}: {frame}")
        for d in cups:
            print(f"    red_cup     conf={d.confidence:.2f}  bbox=({d.x},{d.y},{d.width},{d.height})")
        for d in malls:
            print(f"    marshmallow conf={d.confidence:.2f}  bbox=({d.x},{d.y},{d.width},{d.height})")
        strong = [d for d in malls if d.confidence >= 0.60]
        print(f"  → {len(cups)} cup(s), {len(strong)} marshmallow(s) above the 0.60 threshold.")
    except Exception as exc:  # noqa: BLE001
        print(f"  (real detection skipped: {exc})")


def part_b_examples() -> None:
    print("\n" + "=" * 70)
    print("PART B — tiering + marshmallow selection (5 random layouts, noise=10px)")
    print("=" * 70)
    rng = random.Random(7)
    sizes = [9, 11, 14, 16]
    for t in range(5):
        order = sizes[:]
        rng.shuffle(order)
        layout = {i: order[i] for i in range(4)}
        mallow_pos = rng.randrange(4)
        # Use a fresh global RNG seed per layout so synth noise is deterministic.
        random.seed(1000 + t)
        res = _run_scan(layout, mallow_pos, noise_px=10.0)
        print(f"\n Layout #{t}: " + "  ".join(
            f"cup{c['id']:.0f}={c['stack']}cups" for c in res["cups"])
            + f"   (marshmallow on cup{mallow_pos + 1})")
        for c in res["cups"]:
            ok = "OK " if abs(c["z_mm"] - c["true_z"]) < 1e-6 else "ERR"
            tag = " <-- MALLOW" if c["mallow_conf"] > 0 else ""
            print(f"   cup{c['id']:.0f}: top_est={c['top_mm']:+6.1f}mm  "
                  f"assigned_z={c['z_mm']:+5.0f}  true_z={c['true_z']:+5.0f}  [{ok}]{tag}")
        tgt = res["target"]
        hit = (tgt["mallow_conf"] > 0 and abs(tgt["z_mm"] - tgt["true_z"]) < 1e-6)
        print(f"   → picked cup{tgt['id']:.0f}  z={tgt['z_mm']:+.0f}mm  "
              f"{'CORRECT' if hit else 'WRONG'}")


def part_b_montecarlo(trials: int, noise_px: float) -> None:
    print("\n" + "=" * 70)
    print(f"PART B — Monte-Carlo robustness: {trials} trials, noise={noise_px}px/σ")
    print("=" * 70)
    rng = random.Random(42)
    sizes = [9, 11, 14, 16]
    layout_perfect = target_correct = 0
    for t in range(trials):
        order = sizes[:]
        rng.shuffle(order)
        layout = {i: order[i] for i in range(4)}
        mallow_pos = rng.randrange(4)
        random.seed(t)
        res = _run_scan(layout, mallow_pos, noise_px=noise_px)
        if all(abs(c["z_mm"] - c["true_z"]) < 1e-6 for c in res["cups"]):
            layout_perfect += 1
        tgt = res["target"]
        if tgt["mallow_conf"] > 0 and abs(tgt["z_mm"] - tgt["true_z"]) < 1e-6:
            target_correct += 1
    print(f"  full-layout heights correct : {layout_perfect}/{trials} "
          f"({100 * layout_perfect / trials:.1f}%)")
    print(f"  target cup height correct   : {target_correct}/{trials} "
          f"({100 * target_correct / trials:.1f}%)")


if __name__ == "__main__":
    print(f"Known mallow-centre heights (sorted): "
          f"{[round(z) for z in KNOWN_MALLOW_Z_MM_SORTED]} mm")
    part_a_real_detection()
    part_b_examples()
    part_b_montecarlo(trials=3000, noise_px=10.0)
    part_b_montecarlo(trials=3000, noise_px=20.0)
