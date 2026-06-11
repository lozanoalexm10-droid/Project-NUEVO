from __future__ import annotations

import math

import cv2
import numpy as np

from vision.debug_utils import DebugOverlay
from vision.model_utils import DetectedObject


def detect_yellow_block(
    frame_bgr: np.ndarray,
) -> tuple[list[DetectedObject], list[DebugOverlay]]:
    """Detect a simple yellow block and return detections plus debug contours."""
    detections: list[DetectedObject] = []
    debug_overlays: list[DebugOverlay] = []

    blurred = cv2.GaussianBlur(frame_bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Yellow HSV range. These values were chosen to keep the example simple and
    # reasonably robust for bright classroom-colored yellow objects.
    yellow_hsv_low = (20, 110, 80)
    yellow_hsv_high = (38, 255, 255)
    mask = cv2.inRange(hsv, yellow_hsv_low, yellow_hsv_high)

    # Small open, then slightly larger close:
    # - open removes isolated noise specks
    # - close fills small gaps inside the detected block
    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

    # Minimum blob size and compactness filter:
    # - 500 px allows partially-clipped blocks at the frame edge
    # - 0.30 fill ratio allows partial/clipped bounding shapes
    min_area_px = 500
    min_fill_ratio = 0.30

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        contour_area = float(cv2.contourArea(contour))
        if contour_area < min_area_px:
            continue

        x, y, width, height = cv2.boundingRect(contour)
        bounding_box_area = float(max(1, width * height))
        fill_ratio = contour_area / bounding_box_area
        if fill_ratio < min_fill_ratio:
            continue

        confidence = yellow_detection_score(contour_area, min_area_px, fill_ratio)
        detection = DetectedObject(
            class_name="yellow block",
            confidence=confidence,
            x=int(x),
            y=int(y),
            width=int(width),
            height=int(height),
        )
        detection.add_attribute("color", "yellow", 1.0)
        detections.append(detection)
        debug_overlays.append(
            DebugOverlay(
                color=(0, 255, 255),
                contour=contour,
                label="yellow block",
                x=int(x),
                y=int(y),
            )
        )

    return detections, debug_overlays


def yellow_detection_score(contour_area: float, min_area_px: int, fill_ratio: float) -> float:
    """Build a simple 0-1 confidence score from area and shape compactness."""
    area_score = min(1.0, contour_area / float(max(1, min_area_px * 4)))
    score = 0.55 * area_score + 0.45 * max(0.0, min(1.0, fill_ratio))
    return max(0.0, min(1.0, score))


def detect_marshmallow(
    frame_bgr: np.ndarray,
) -> tuple[list[DetectedObject], list[DebugOverlay]]:
    """Detect a white marshmallow using HSV color + shape filtering.

    Marshmallows read as white/off-white: very low saturation, high value.
    Best used against a dark (e.g. blackboard) background so the white blob
    stands out cleanly.  The aspect-ratio filter rejects thin strips (paper
    edges, signage) and the area cap rejects large bright backgrounds.
    """
    detections: list[DetectedObject] = []
    debug_overlays: list[DebugOverlay] = []

    blurred = cv2.GaussianBlur(frame_bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # White / cream: any hue, moderate-low saturation, moderate-high value.
    # OpenCV HSV: H 0-180, S 0-255, V 0-255.
    # Thresholds were originally (V>=170, S<=60) for a brightly-lit white
    # marshmallow against neutral background. Loosened to (V>=110, S<=100)
    # because: the comp/practice background is matte black, the camera
    # auto-exposes for the dark scene and the mallow ends up dim cream
    # (sampled HSV median ≈ H 35, S 86, V 75; bright edge V ~140). The
    # old thresholds matched zero pixels on real frames so the detector
    # never fired. The tighter "real white" thresholds are still preserved
    # in the morphology + pink-ring + area + aspect filters below.
    white_low  = np.array([0,   0, 110], dtype=np.uint8)
    white_high = np.array([180, 100, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, white_low, white_high)

    # Open removes isolated noise; close fills small gaps inside the blob.
    open_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  open_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

    # Pink / magenta mask — used to REJECT the pink tennis ball, whose blown-out
    # highlight can read as "white". A real marshmallow has ~no pink around it,
    # whereas a white highlight on the pink ball is ringed by pink.
    # IMPORTANT: hue is capped at 168 to EXCLUDE the red-wrap (red reads as hue
    # ~170-179 in OpenCV), otherwise a red cup beneath the marshmallow would be
    # mistaken for pink and reject a perfectly good marshmallow.
    pink_low  = np.array([145, 60,  80], dtype=np.uint8)
    pink_high = np.array([168, 255, 255], dtype=np.uint8)
    pink_mask = cv2.inRange(hsv, pink_low, pink_high)

    # Minimum blob size: allows marshmallow to be small when far away (~12 in).
    # Maximum blob size: rejects large background regions (walls, ceiling, floor).
    # A real marshmallow (38 mm dia) fills at most ~12 % of a 1280×720 frame even
    # when held ~90 mm from the camera — anything larger is almost certainly clutter.
    frame_h, frame_w = frame_bgr.shape[:2]
    min_area_px    = 400
    max_area_px    = int(frame_w * frame_h * 0.12)
    min_fill_ratio = 0.30
    max_aspect     = 3.0   # rejects thin walls, paper edges, bright fixtures

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        contour_area = float(cv2.contourArea(contour))
        if contour_area < min_area_px or contour_area > max_area_px:
            continue

        x, y, width, height = cv2.boundingRect(contour)
        bbox_area  = float(max(1, width * height))
        fill_ratio = contour_area / bbox_area
        if fill_ratio < min_fill_ratio:
            continue

        aspect = max(width, height) / float(max(1, min(width, height)))
        if aspect > max_aspect:
            continue

        # Reject if the blob is ringed by pink — that's the pink tennis ball, not
        # a marshmallow. Check pink fraction in a region ~35 % larger than the bbox.
        ex, ey = int(width * 0.35), int(height * 0.35)
        rx0, ry0 = max(0, x - ex), max(0, y - ey)
        rx1, ry1 = min(frame_w, x + width + ex), min(frame_h, y + height + ey)
        region = pink_mask[ry0:ry1, rx0:rx1]
        region_area = max(1, region.shape[0] * region.shape[1])
        pink_frac = float(cv2.countNonZero(region)) / region_area
        if pink_frac > 0.15:
            continue

        confidence = _marshmallow_score(contour_area, min_area_px, fill_ratio, aspect)
        detection  = DetectedObject(
            class_name="marshmallow",
            confidence=confidence,
            x=int(x),
            y=int(y),
            width=int(width),
            height=int(height),
        )
        detection.add_attribute("color", "white", 1.0)
        detections.append(detection)
        debug_overlays.append(
            DebugOverlay(
                color=(255, 255, 255),
                contour=contour,
                label=f"marshmallow {confidence:.2f}",
                x=int(x),
                y=int(y),
            )
        )

    return detections, debug_overlays


def _marshmallow_score(
    contour_area: float, min_area_px: int, fill_ratio: float, aspect: float
) -> float:
    """Confidence in [0, 1] from area, fill compactness, and aspect closeness to 1."""
    area_score   = min(1.0, contour_area / float(max(1, min_area_px * 6)))
    fill_score   = max(0.0, min(1.0, fill_ratio))
    # aspect_score is 1.0 for a perfect square, 0.0 when aspect == max_aspect (3.0)
    aspect_score = max(0.0, 1.0 - (aspect - 1.0) / 2.0)
    return max(0.0, min(1.0, 0.40 * area_score + 0.35 * fill_score + 0.25 * aspect_score))


def detect_red_cup(
    frame_bgr: np.ndarray,
) -> tuple[list[DetectedObject], list[DebugOverlay]]:
    """Detect red Solo cups using dual-range HSV masking.

    Red wraps around H=0/180 in OpenCV HSV, so two ranges are combined.
    Filters by area, fill ratio, and aspect ratio to reject thin red strips
    and small noise blobs.
    """
    detections: list[DetectedObject] = []
    debug_overlays: list[DebugOverlay] = []

    blurred = cv2.GaussianBlur(frame_bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Red wraps around H=0 — use two ranges and OR them together.
    mask_lo = cv2.inRange(hsv, np.array([ 0, 100,  80], dtype=np.uint8),
                               np.array([10, 255, 255], dtype=np.uint8))
    mask_hi = cv2.inRange(hsv, np.array([165, 100,  80], dtype=np.uint8),
                               np.array([180, 255, 255], dtype=np.uint8))
    mask = cv2.bitwise_or(mask_lo, mask_hi)

    open_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  open_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

    min_area_px    = 800   # cup body should occupy a reasonable number of pixels
    min_fill_ratio = 0.30
    max_aspect     = 3.0   # reject very elongated red strips

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        contour_area = float(cv2.contourArea(contour))
        if contour_area < min_area_px:
            continue

        x, y, width, height = cv2.boundingRect(contour)
        bbox_area  = float(max(1, width * height))
        fill_ratio = contour_area / bbox_area
        if fill_ratio < min_fill_ratio:
            continue

        aspect = max(width, height) / float(max(1, min(width, height)))
        if aspect > max_aspect:
            continue

        confidence = _red_cup_score(contour_area, min_area_px, fill_ratio)
        detection  = DetectedObject(
            class_name="red_cup",
            confidence=confidence,
            x=int(x),
            y=int(y),
            width=int(width),
            height=int(height),
        )
        detection.add_attribute("color", "red", 1.0)
        detections.append(detection)
        debug_overlays.append(
            DebugOverlay(
                color=(0, 0, 200),
                contour=contour,
                label=f"red_cup {confidence:.2f}",
                x=int(x),
                y=int(y),
            )
        )

    return detections, debug_overlays


def _red_cup_score(contour_area: float, min_area_px: int, fill_ratio: float) -> float:
    """Confidence in [0, 1] from area and fill compactness."""
    area_score = min(1.0, contour_area / float(max(1, min_area_px * 5)))
    fill_score = max(0.0, min(1.0, fill_ratio))
    return max(0.0, min(1.0, 0.60 * area_score + 0.40 * fill_score))


def filter_marshmallows_on_red_cups(
    marshmallow_detections: list[DetectedObject],
    marshmallow_overlays: list[DebugOverlay],
    red_cup_detections: list[DetectedObject],
) -> tuple[list[DetectedObject], list[DebugOverlay]]:
    """Keep only marshmallows positioned on top of a red cup — when a cup is
    actually visible.

    A marshmallow qualifies when its horizontal center sits inside some cup's
    horizontal extent and its vertical center is at or above the cup's upper
    third — the slack lets a marshmallow rest in the cup rim instead of
    needing to float perfectly above the top edge.  When the cup is visible
    this drops background false-positives (originally important for the white
    detector; less important now that mallows are purple).

    If no cup is detected we pass the marshmallow detections through
    unfiltered, because on short stacks (~7 cups) the cup falls below the
    bottom of the camera frame and cup-gating would otherwise drop a
    perfectly good mallow.
    """
    if not marshmallow_detections:
        return [], []
    if not red_cup_detections:
        return list(marshmallow_detections), list(marshmallow_overlays)

    kept_detections: list[DetectedObject] = []
    kept_overlays: list[DebugOverlay] = []
    for detection, overlay in zip(marshmallow_detections, marshmallow_overlays):
        m_center_x = detection.x + detection.width / 2.0
        m_center_y = detection.y + detection.height / 2.0
        for cup in red_cup_detections:
            cup_left = cup.x
            cup_right = cup.x + cup.width
            cup_upper_third = cup.y + cup.height / 3.0
            if not (cup_left <= m_center_x <= cup_right):
                continue
            if m_center_y > cup_upper_third:
                continue
            kept_detections.append(detection)
            kept_overlays.append(overlay)
            break
    return kept_detections, kept_overlays
