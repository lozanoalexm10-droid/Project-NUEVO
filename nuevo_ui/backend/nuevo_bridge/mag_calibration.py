"""
Bridge-side magnetometer calibration.

The firmware only enters/leaves calibration sampling mode and streams raw
magnetometer samples. The bridge collects those samples, fits a hard-iron
offset plus soft-iron 3x3 matrix, and sends the final calibration back in one
apply command.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from statistics import fmean
from typing import Callable, List, Optional, Sequence, Tuple


IDENTITY_3X3: Tuple[float, ...] = (
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
)


@dataclass
class MagCalibrationResult:
    offset: Tuple[float, float, float]
    matrix: Tuple[float, ...]
    span: Tuple[float, float, float]
    mean_norm: float
    std_norm: float


def _mat_vec_mul(matrix: Sequence[float], vector: Sequence[float]) -> Tuple[float, float, float]:
    return (
        matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
        matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2],
        matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2],
    )


def _jacobi_eigen_decomposition(matrix: Sequence[Sequence[float]]) -> Tuple[List[float], List[List[float]]]:
    a = [[float(matrix[r][c]) for c in range(3)] for r in range(3)]
    v = [[1.0 if r == c else 0.0 for c in range(3)] for r in range(3)]

    for _ in range(16):
        p, q = 0, 1
        max_offdiag = abs(a[0][1])
        for i, j in ((0, 2), (1, 2)):
            candidate = abs(a[i][j])
            if candidate > max_offdiag:
                max_offdiag = candidate
                p, q = i, j
        if max_offdiag < 1e-9:
            break

        if abs(a[p][p] - a[q][q]) < 1e-9:
            angle = math.pi / 4.0
            c = math.cos(angle)
            s = math.sin(angle)
        else:
            tau = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
            t = math.copysign(1.0 / (abs(tau) + math.sqrt(1.0 + tau * tau)), tau)
            c = 1.0 / math.sqrt(1.0 + t * t)
            s = t * c

        app = a[p][p]
        aqq = a[q][q]
        apq = a[p][q]
        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq
        a[p][q] = 0.0
        a[q][p] = 0.0

        for k in range(3):
            if k in (p, q):
                continue
            aik = a[k][p]
            akq = a[k][q]
            a[k][p] = c * aik - s * akq
            a[p][k] = a[k][p]
            a[k][q] = s * aik + c * akq
            a[q][k] = a[k][q]

        for k in range(3):
            vip = v[k][p]
            viq = v[k][q]
            v[k][p] = c * vip - s * viq
            v[k][q] = s * vip + c * viq

    eigenvalues = [a[0][0], a[1][1], a[2][2]]
    order = sorted(range(3), key=lambda idx: eigenvalues[idx], reverse=True)
    sorted_values = [eigenvalues[idx] for idx in order]
    sorted_vectors = [[v[row][idx] for idx in order] for row in range(3)]
    return sorted_values, sorted_vectors


def fit_soft_iron_calibration(samples: Sequence[Tuple[float, float, float]]) -> Optional[MagCalibrationResult]:
    if len(samples) < 16:
        return None

    xs = [sample[0] for sample in samples]
    ys = [sample[1] for sample in samples]
    zs = [sample[2] for sample in samples]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_z, max_z = min(zs), max(zs)
    span_x = max_x - min_x
    span_y = max_y - min_y
    span_z = max_z - min_z
    initial_offset = (
        (max_x + min_x) * 0.5,
        (max_y + min_y) * 0.5,
        (max_z + min_z) * 0.5,
    )

    def evaluate(offset: Tuple[float, float, float]) -> Optional[MagCalibrationResult]:
        centered = [
            (sample[0] - offset[0], sample[1] - offset[1], sample[2] - offset[2])
            for sample in samples
        ]

        cov = [[0.0, 0.0, 0.0] for _ in range(3)]
        inv_n = 1.0 / float(len(centered))
        for sample in centered:
            cov[0][0] += sample[0] * sample[0] * inv_n
            cov[0][1] += sample[0] * sample[1] * inv_n
            cov[0][2] += sample[0] * sample[2] * inv_n
            cov[1][1] += sample[1] * sample[1] * inv_n
            cov[1][2] += sample[1] * sample[2] * inv_n
            cov[2][2] += sample[2] * sample[2] * inv_n
        cov[1][0] = cov[0][1]
        cov[2][0] = cov[0][2]
        cov[2][1] = cov[1][2]

        eigenvalues, eigenvectors = _jacobi_eigen_decomposition(cov)
        if min(eigenvalues) <= 1e-6:
            return None

        inv_sqrt = [0.0] * 9
        for row in range(3):
            for col in range(3):
                inv_sqrt[row * 3 + col] = sum(
                    eigenvectors[row][k] * eigenvectors[col][k] / math.sqrt(eigenvalues[k])
                    for k in range(3)
                )

        unscaled_norms = []
        for sample in centered:
            corrected = _mat_vec_mul(inv_sqrt, sample)
            unscaled_norms.append(math.sqrt(corrected[0] ** 2 + corrected[1] ** 2 + corrected[2] ** 2))

        mean_unscaled_norm = fmean(unscaled_norms)
        if mean_unscaled_norm <= 1e-6:
            return None

        target_radius = max((span_x * span_y * span_z) ** (1.0 / 3.0) * 0.5, 1.0)
        scale = target_radius / mean_unscaled_norm
        matrix = tuple(value * scale for value in inv_sqrt)

        corrected_norms = []
        for sample in centered:
            corrected = _mat_vec_mul(matrix, sample)
            corrected_norms.append(math.sqrt(corrected[0] ** 2 + corrected[1] ** 2 + corrected[2] ** 2))

        mean_norm = fmean(corrected_norms)
        variance = fmean((norm - mean_norm) ** 2 for norm in corrected_norms)
        std_norm = math.sqrt(variance)

        return MagCalibrationResult(
            offset=offset,
            matrix=matrix,
            span=(span_x, span_y, span_z),
            mean_norm=mean_norm,
            std_norm=std_norm,
        )

    best = evaluate(initial_offset)
    if best is None:
        return None

    best_score = best.std_norm / best.mean_norm if best.mean_norm > 1e-6 else math.inf
    best_offset = initial_offset
    step = max(max(span_x, span_y, span_z) * 0.125, 0.5)

    for _ in range(12):
        improved = False
        for axis in range(3):
            for sign in (-1.0, 1.0):
                candidate_offset = list(best_offset)
                candidate_offset[axis] += sign * step
                candidate = evaluate((candidate_offset[0], candidate_offset[1], candidate_offset[2]))
                if candidate is None or candidate.mean_norm <= 1e-6:
                    continue
                candidate_score = candidate.std_norm / candidate.mean_norm
                if candidate_score + 1e-6 < best_score:
                    best = candidate
                    best_score = candidate_score
                    best_offset = candidate.offset
                    improved = True
        if not improved:
            step *= 0.5
            if step < 0.05:
                break

    return best


class MagCalibrationController:
    MIN_SAMPLES = 250
    MIN_DURATION_S = 6.0
    MAX_DURATION_S = 30.0
    MIN_AXIS_SPAN_UT = 12.0
    MIN_AXIS_RATIO = 0.30
    STABLE_WINDOW_S = 1.25
    SPAN_GROWTH_EPS_UT = 0.5
    MAX_STD_RATIO = 0.25
    MAX_SAMPLES = 4096

    def __init__(self, sender: Optional[Callable[[str, dict], bool]] = None):
        self._sender = sender
        self._reset()

    def set_sender(self, sender: Optional[Callable[[str, dict], bool]]) -> None:
        self._sender = sender

    def observe(self, topic: str, data: dict) -> None:
        if topic == "sensor_mag_cal_status":
            self._observe_status(data)
            return
        if topic == "sensor_imu":
            self._observe_imu(data)

    def _reset(self) -> None:
        self._sampling = False
        self._apply_sent = False
        self._samples: List[Tuple[float, float, float]] = []
        self._start_time = 0.0
        self._last_span_growth_time = 0.0
        self._min = [math.inf, math.inf, math.inf]
        self._max = [-math.inf, -math.inf, -math.inf]

    def _observe_status(self, data: dict) -> None:
        state = int(data.get("state", 0))
        if state == 1:
            if not self._sampling:
                now = time.monotonic()
                self._sampling = True
                self._apply_sent = False
                self._samples.clear()
                self._start_time = now
                self._last_span_growth_time = now
                self._min = [math.inf, math.inf, math.inf]
                self._max = [-math.inf, -math.inf, -math.inf]
            return

        if self._sampling and state in (0, 3, 4):
            self._reset()

    def _observe_imu(self, data: dict) -> None:
        if not self._sampling or self._apply_sent:
            return

        sample = (
            float(data.get("magX", 0.0)),
            float(data.get("magY", 0.0)),
            float(data.get("magZ", 0.0)),
        )
        if sample == (0.0, 0.0, 0.0):
            return

        now = time.monotonic()
        self._samples.append(sample)
        if len(self._samples) > self.MAX_SAMPLES:
            self._samples.pop(0)

        prev_spans = [
            0.0 if not math.isfinite(self._min[idx]) or not math.isfinite(self._max[idx])
            else (self._max[idx] - self._min[idx])
            for idx in range(3)
        ]
        for idx, value in enumerate(sample):
            if value < self._min[idx]:
                self._min[idx] = value
            if value > self._max[idx]:
                self._max[idx] = value
        spans = [self._max[idx] - self._min[idx] for idx in range(3)]
        if max(spans) > 0.0:
            span_growth = max(spans[idx] - prev_spans[idx] for idx in range(3))
            if span_growth >= self.SPAN_GROWTH_EPS_UT:
                self._last_span_growth_time = now

        elapsed = now - self._start_time
        if elapsed >= self.MAX_DURATION_S:
            self._send_command("sensor_mag_cal_cmd", {"command": 2})
            self._reset()
            return

        if elapsed < self.MIN_DURATION_S or len(self._samples) < self.MIN_SAMPLES:
            return

        if min(spans) < self.MIN_AXIS_SPAN_UT:
            return
        if min(spans) / max(spans) < self.MIN_AXIS_RATIO:
            return
        if (now - self._last_span_growth_time) < self.STABLE_WINDOW_S:
            return

        result = fit_soft_iron_calibration(self._samples)
        if result is None or result.mean_norm <= 1e-6:
            return
        if (result.std_norm / result.mean_norm) > self.MAX_STD_RATIO:
            return

        if self._send_command("sensor_mag_cal_cmd", {
            "command": 4,
            "offsetX": result.offset[0],
            "offsetY": result.offset[1],
            "offsetZ": result.offset[2],
            "softIronMatrix": list(result.matrix),
        }):
            self._apply_sent = True

    def _send_command(self, cmd: str, data: dict) -> bool:
        if self._sender is None:
            return False
        return bool(self._sender(cmd, data))
