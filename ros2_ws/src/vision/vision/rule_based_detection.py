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
    """Detect a white/off-white marshmallow using HSV color + shape filtering.

    Marshmallows are low-saturation (white/cream), high-brightness, roughly
    compact blobs. The aspect-ratio filter rejects thin strips (walls, paper
    edges) that also match the white HSV range.
    """
    detections: list[DetectedObject] = []
    debug_overlays: list[DebugOverlay] = []

    blurred = cv2.GaussianBlur(frame_bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # White/off-white: any hue, low saturation, high brightness.
    # OpenCV HSV: H 0-180, S 0-255, V 0-255.
    white_low  = np.array([ 0,   0, 160], dtype=np.uint8)
    white_high = np.array([180,  80, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, white_low, white_high)

    # Open removes isolated noise; close fills small gaps inside the blob.
    open_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  open_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

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
                color=(200, 200, 200),
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
