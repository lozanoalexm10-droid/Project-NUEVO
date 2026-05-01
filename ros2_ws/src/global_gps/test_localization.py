"""
Standalone end-to-end test of the global_gps localization math.

Simulates a tripod-mounted camera observing 4 calibration tags + several
rover tags. Synthesises perfect (and then noisy) camera-frame tvecs,
runs the same calibration + world-pose pipeline used by the node, and
checks that recovered (x, y) world coordinates match ground truth.

Does NOT require ROS. Just numpy and the package's geometry_utils.
"""

from __future__ import annotations

import sys
import os
import json
import math
import numpy as np
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(__file__))
from global_gps.geometry_utils import (
    rigid_transform_svd,
    project_point_to_plane,
)


def euler_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX intrinsic rotation."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def make_tripod_T_cam_from_world() -> np.ndarray:
    """A plausible tripod pose: 2 m back from origin, 1.5 m up, looking
    down at the field at ~30° below horizontal."""
    pitch_down = np.radians(30.0)
    R_world_from_cam = euler_to_R(np.pi, pitch_down, 0.0)
    cam_pos_world = np.array([0.0, -2.0, 1.5])
    T_world_from_cam = np.eye(4)
    T_world_from_cam[:3, :3] = R_world_from_cam
    T_world_from_cam[:3, 3] = cam_pos_world
    return np.linalg.inv(T_world_from_cam)


def world_to_cam(T_cam_from_world: np.ndarray, P_world: np.ndarray) -> np.ndarray:
    Ph = np.hstack([P_world, np.ones((P_world.shape[0], 1))])
    return (T_cam_from_world @ Ph.T).T[:, :3]


def calibrate(P_world: np.ndarray,
              P_cam: np.ndarray,
              gp_point_world: np.ndarray,
              gp_normal_world: np.ndarray):
    """Mirror of GroundLocalizer._calibrate."""
    T_cam_from_world = rigid_transform_svd(P_world, P_cam)
    T_world_from_cam = np.linalg.inv(T_cam_from_world)
    R_cw = T_cam_from_world[:3, :3]
    t_cw = T_cam_from_world[:3, 3]
    n_cam = R_cw @ gp_normal_world
    p_cam = R_cw @ gp_point_world + t_cw
    plane = {"normal": n_cam, "d": -float(n_cam @ p_cam)}
    return T_world_from_cam, plane, T_cam_from_world


def localize(tvec: np.ndarray,
             T_world_from_cam: np.ndarray,
             plane: dict) -> tuple[float, float]:
    """Mirror of the (x, y) part of GroundLocalizer._compute_world_pose."""
    p_ground = project_point_to_plane(tvec, plane["normal"], plane["d"])
    p_world = T_world_from_cam @ np.append(p_ground, 1.0)
    return float(p_world[0]), float(p_world[1])


def localize_full(tvec: np.ndarray,
                  R_marker_world: np.ndarray,
                  T_world_from_cam: np.ndarray,
                  T_cam_from_world: np.ndarray,
                  plane: dict) -> tuple[float, float, float]:
    """Mirror of GroundLocalizer._compute_world_pose including theta.

    R_marker_world is the rover marker's pose in the WORLD frame
    (so we can synthesize the camera-frame rvec from it).
    """
    R_cam_from_world = T_cam_from_world[:3, :3]
    R_cam_marker = R_cam_from_world @ R_marker_world
    marker_x_cam = R_cam_marker[:, 0]
    R_world_from_cam = T_world_from_cam[:3, :3]
    marker_x_world = R_world_from_cam @ marker_x_cam
    n_world = np.array([0., 0., 1.])  # world frame normal (default)
    marker_x_world -= np.dot(marker_x_world, n_world) * n_world
    marker_x_world /= np.linalg.norm(marker_x_world)
    theta = float(np.arctan2(marker_x_world[1], marker_x_world[0]))
    x, y = localize(tvec, T_world_from_cam, plane)
    return x, y, theta


# ── Test scenarios ──────────────────────────────────────────────────────

def run_case(name: str,
             cal_world: np.ndarray,
             rover_world: np.ndarray,
             gp_point: np.ndarray,
             gp_normal: np.ndarray,
             tvec_noise_sigma_m: float = 0.0,
             rover_z_above_floor: float = 0.0,
             seed: int = 0) -> None:
    rng = np.random.default_rng(seed)
    print(f"\n=== {name} ===")
    T_cam_from_world_gt = make_tripod_T_cam_from_world()

    cal_cam_clean = world_to_cam(T_cam_from_world_gt, cal_world)
    cal_cam = cal_cam_clean + rng.normal(0, tvec_noise_sigma_m, cal_cam_clean.shape)

    T_world_from_cam, plane, T_cam_from_world = calibrate(
        cal_world, cal_cam, gp_point, gp_normal,
    )

    # Calibration residuals
    pred = (T_cam_from_world @ np.hstack(
        [cal_world, np.ones((cal_world.shape[0], 1))]
    ).T).T[:, :3]
    res_mm = np.linalg.norm(pred - cal_cam, axis=1) * 1000.0
    print(f"  calibration residuals (mm): max={res_mm.max():.2f} "
          f"rms={np.sqrt((res_mm**2).mean()):.2f}")

    # Localize each rover tag — rover sits at world (x,y,rover_z_above_floor)
    rover_world_3d = rover_world.copy()
    rover_world_3d[:, 2] += rover_z_above_floor
    rover_cam_clean = world_to_cam(T_cam_from_world_gt, rover_world_3d)
    rover_cam = rover_cam_clean + rng.normal(0, tvec_noise_sigma_m, rover_cam_clean.shape)

    errs_xy = []
    for i, (tvec, gt) in enumerate(zip(rover_cam, rover_world)):
        x, y = localize(tvec, T_world_from_cam, plane)
        err = np.hypot(x - gt[0], y - gt[1])
        errs_xy.append(err)
        print(f"  rover {i}: gt=({gt[0]:.3f},{gt[1]:.3f})  "
              f"est=({x:.3f},{y:.3f})  err={err*1000:.1f} mm")
    return float(np.max(errs_xy)) if errs_xy else 0.0


def main() -> None:
    # Floor calibration tags (coplanar, on z=0).
    cal_floor = np.array([
        [0.0, 0.0, 0.0],
        [2.4, 0.0, 0.0],
        [0.0, 1.8, 0.0],
        [2.4, 1.8, 0.0],
    ])
    # Non-coplanar tags (varied z) — the §2b case.
    cal_3d = np.array([
        [0.0, 0.0, 0.00],
        [2.4, 0.0, 0.05],
        [0.0, 1.8, 0.30],
        [2.4, 1.8, 0.35],
    ])
    rovers = np.array([
        [1.2, 0.9, 0.0],
        [0.5, 0.3, 0.0],
        [2.0, 1.5, 0.0],
        [1.8, 0.2, 0.0],
    ])

    # Case 1: coplanar floor tags, no noise — should be ~exact.
    e1 = run_case(
        "coplanar floor tags, zero noise",
        cal_floor, rovers,
        gp_point=np.array([0., 0., 0.]),
        gp_normal=np.array([0., 0., 1.]),
    )
    assert e1 < 1e-9, f"expected sub-nanometer recovery, got {e1}"

    # Case 2: non-coplanar tags (§2b), no noise — should still be ~exact.
    e2 = run_case(
        "non-coplanar tags (§2b), zero noise",
        cal_3d, rovers,
        gp_point=np.array([0., 0., 0.]),
        gp_normal=np.array([0., 0., 1.]),
    )
    assert e2 < 1e-9, f"expected sub-nanometer recovery, got {e2}"

    # Case 3: realistic per-tag tvec noise (5 mm sigma) on coplanar tags.
    e3 = run_case(
        "coplanar tags + 5 mm tvec noise",
        cal_floor, rovers,
        gp_point=np.array([0., 0., 0.]),
        gp_normal=np.array([0., 0., 1.]),
        tvec_noise_sigma_m=0.005,
        seed=7,
    )
    assert e3 < 0.05, f"expected <5 cm error under 5 mm noise, got {e3}"

    # Case 4: same noise on non-coplanar tags — should be no worse, often better.
    e4 = run_case(
        "non-coplanar tags + 5 mm tvec noise",
        cal_3d, rovers,
        gp_point=np.array([0., 0., 0.]),
        gp_normal=np.array([0., 0., 1.]),
        tvec_noise_sigma_m=0.005,
        seed=7,
    )
    assert e4 < 0.05, f"expected <5 cm error under 5 mm noise, got {e4}"

    # Case 5: §2c — non-z=0 ground plane.  Floor is tilted in world frame
    # (normal=[0,0.1,1], renormalised).  Rover tvecs sit on that plane.
    n = np.array([0.0, 0.1, 1.0])
    n /= np.linalg.norm(n)
    # adjust rover z so rovers sit on the tilted plane (n . p = 0):
    rovers_tilted = rovers.copy()
    rovers_tilted[:, 2] = -(n[0] * rovers_tilted[:, 0]
                            + n[1] * rovers_tilted[:, 1]) / n[2]
    e5 = run_case(
        "tilted ground plane (§2c), zero noise",
        cal_floor, rovers_tilted,
        gp_point=np.array([0., 0., 0.]),
        gp_normal=n,
    )
    assert e5 < 1e-9, f"tilted plane: expected exact recovery, got {e5}"

    # Case 6: rover NOT coplanar with calibration tags / not on the ground.
    # Rovers at world (x, y, 0.40) — tag mounted 40 cm above the floor.
    # Default ground plane is world z=0, n=[0,0,1]. The projection moves
    # the rover tvec along n_world to z=0, so recovered (x, y) should equal
    # the rover's true (x, y) exactly.
    e6 = run_case(
        "rover tag 40 cm above floor (not coplanar)",
        cal_floor, rovers,
        gp_point=np.array([0., 0., 0.]),
        gp_normal=np.array([0., 0., 1.]),
        rover_z_above_floor=0.40,
    )
    assert e6 < 1e-9, f"airborne rover: expected exact recovery, got {e6}"

    # Case 7: rovers at varied heights AND with calibration noise.
    e7 = run_case(
        "rovers at varied heights + 5 mm calib noise",
        cal_floor, rovers,
        gp_point=np.array([0., 0., 0.]),
        gp_normal=np.array([0., 0., 1.]),
        tvec_noise_sigma_m=0.005,
        rover_z_above_floor=0.40,
        seed=11,
    )
    assert e7 < 0.05, f"airborne rover + noise: got {e7}"

    # Case 8: rover on a ramp — pitched up while elevated.
    # Travels along world +Y, climbing a 15° ramp.  True (x,y) heading is +Y
    # (theta = pi/2). The marker is pitched 15° about world X.
    print("\n=== rover on a 15° ramp (pitched + elevated, default flat ground plane) ===")
    T_cam_from_world_gt = make_tripod_T_cam_from_world()
    cal_cam = world_to_cam(T_cam_from_world_gt, cal_floor)
    T_world_from_cam, plane, T_cam_from_world = calibrate(
        cal_floor, cal_cam,
        np.array([0., 0., 0.]), np.array([0., 0., 1.]),
    )

    pitch = np.radians(15.0)
    R_marker_world = euler_to_R(pitch, 0.0, np.pi / 2)  # heading +Y, pitched up
    # Sample 5 positions along the ramp, x=1.2, y=0..1.5, z=y*tan(pitch).
    max_xy_err = 0.0
    max_th_err = 0.0
    for s in np.linspace(0.0, 1.5, 5):
        rover_world = np.array([1.2, s, s * np.tan(pitch)])
        rover_cam = world_to_cam(T_cam_from_world_gt,
                                 rover_world.reshape(1, 3))[0]
        x, y, th = localize_full(
            rover_cam, R_marker_world,
            T_world_from_cam, T_cam_from_world, plane,
        )
        xy_err = np.hypot(x - rover_world[0], y - rover_world[1])
        th_err = abs(((th - np.pi / 2 + np.pi) % (2 * np.pi)) - np.pi)
        max_xy_err = max(max_xy_err, xy_err)
        max_th_err = max(max_th_err, th_err)
        print(f"  s={s:.2f}: gt=({rover_world[0]:.3f},{rover_world[1]:.3f},"
              f"z={rover_world[2]:.3f})  est=({x:.3f},{y:.3f})  "
              f"xy_err={xy_err*1000:.2f} mm  theta={np.degrees(th):.2f}° "
              f"(err={np.degrees(th_err):.3f}°)")
    assert max_xy_err < 1e-9, f"ramp xy_err: {max_xy_err}"
    assert max_th_err < 1e-9, f"pure pitch should not bias yaw, got {max_th_err}"

    # Case 9: ramp + sideways roll (mixed tilt) — heading should pick up bias.
    print("\n=== rover on ramp WITH 5° roll (mixed tilt) ===")
    R_marker_world_roll = euler_to_R(pitch, np.radians(5.0), np.pi / 2)
    rover_world = np.array([1.2, 0.8, 0.8 * np.tan(pitch)])
    rover_cam = world_to_cam(T_cam_from_world_gt, rover_world.reshape(1, 3))[0]
    x, y, th = localize_full(
        rover_cam, R_marker_world_roll,
        T_world_from_cam, T_cam_from_world, plane,
    )
    xy_err = np.hypot(x - rover_world[0], y - rover_world[1])
    th_err_deg = abs(np.degrees(th) - 90.0)
    print(f"  gt=({rover_world[0]:.3f},{rover_world[1]:.3f})  "
          f"est=({x:.3f},{y:.3f})  xy_err={xy_err*1000:.2f} mm  "
          f"theta={np.degrees(th):.3f}° (err={th_err_deg:.3f}°)")
    assert xy_err < 1e-9, f"xy still exact under mixed tilt, got {xy_err}"

    # ── Sentinel publishing path ────────────────────────────────────────
    # Replicates the empty-detections branch of GroundLocalizer._publish_detections.
    @dataclass
    class FakeTagDetection:
        tag_id: int = 0
        x: float = 0.0
        y: float = 0.0
        theta: float = 0.0

    def build_detections(rover_results: list) -> list:
        """Return the list that the node would publish/push.

        rover_results: list of (tag_id, x, y, theta) for each rover that
        produced a valid pose. Empty list means no rover seen.
        """
        detections = [
            FakeTagDetection(tag_id=int(mid), x=float(x), y=float(y),
                             theta=float(th))
            for (mid, x, y, th) in rover_results
        ]
        if not detections:
            detections = [FakeTagDetection(
                tag_id=-1,
                x=float("nan"), y=float("nan"), theta=float("nan"),
            )]
        return detections

    def tcp_payload(detections, stamp_sec: float = 1234.5) -> str:
        return json.dumps({
            "stamp": stamp_sec,
            "detections": [
                {"tag_id": d.tag_id, "x": d.x, "y": d.y, "theta": d.theta}
                for d in detections
            ],
        }) + "\n"

    print("\n=== sentinel publishing: no rovers visible ===")
    dets = build_detections([])
    assert len(dets) == 1, f"sentinel should be a single entry, got {len(dets)}"
    s = dets[0]
    assert s.tag_id == -1, f"expected tag_id=-1, got {s.tag_id}"
    assert math.isnan(s.x) and math.isnan(s.y) and math.isnan(s.theta), \
        "expected NaN for x, y, theta"

    payload = tcp_payload(dets)
    print(f"  payload: {payload.rstrip()}")
    # Round-trip JSON (Python's json.loads accepts NaN by default)
    parsed = json.loads(payload)
    assert parsed["detections"][0]["tag_id"] == -1
    assert math.isnan(parsed["detections"][0]["x"])
    assert math.isnan(parsed["detections"][0]["y"])
    assert math.isnan(parsed["detections"][0]["theta"])

    print("\n=== sentinel suppressed when rovers ARE visible ===")
    dets = build_detections([(11, 1.2, 0.9, 0.0), (12, 2.0, 1.5, 1.57)])
    assert len(dets) == 2, f"expected 2 detections, got {len(dets)}"
    assert all(d.tag_id != -1 for d in dets), "no sentinel should appear"
    assert not any(math.isnan(d.x) or math.isnan(d.y) or math.isnan(d.theta)
                   for d in dets), "no NaN values when rovers present"
    payload = tcp_payload(dets)
    parsed = json.loads(payload)
    assert [d["tag_id"] for d in parsed["detections"]] == [11, 12]
    print(f"  payload: {payload.rstrip()}")

    print("\nAll cases passed.")


if __name__ == "__main__":
    main()
