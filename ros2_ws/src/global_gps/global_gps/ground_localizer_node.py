"""
Ground localizer node.

Subscribes to a RealSense colour stream, detects ArUco markers, and
publishes 2-D world-frame poses for all detected rover tags.

Coordinate conventions
----------------------
* Camera frame: standard OpenCV (X right, Y down, Z forward).
* World frame:  defined by the user in ``calibration_layout.yaml`` (origin
                and axes are whatever convention the YAML's calibration-tag
                coordinates implicitly establish). World-Z is the
                configured ``ground_plane.normal`` (default ``[0, 0, 1]``).

Topic published
---------------
/global_gps/tag_detections  (bridge_interfaces/msg/TagDetectionArray)
    One entry per detected rover tag.  Corner anchors are excluded.
    The header stamp matches the colour image that triggered the detection.

Parameters
----------
marker_size : float  (default 0.10)
    Physical side length of the ArUco markers in metres.
corner_ids : int[]  (default [0, 1, 2, 3])
    Marker IDs used as fixed field-corner anchors.  All four must be
    visible simultaneously for the initial calibration to succeed.
rover_ids : int[]  (default [11-18])
    Marker IDs that can appear on rovers.  Only these are published.
calibration_layout_file : str  (default "")
    Path to a YAML file giving the world-frame ``(x, y, z)`` of each
    calibration tag under a top-level ``calibration_tags`` map. May also
    include an optional ``ground_plane`` block with ``point`` and
    ``normal`` length-3 vectors describing the ground plane in world
    frame (default if absent: ``point=[0,0,0]``, ``normal=[0,0,1]``).
    When empty, the packaged default at
    ``<share>/global_gps/config/calibration_layout.yaml`` is used.
"""

from __future__ import annotations

import json
import os
import socket
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from bridge_interfaces.msg import TagDetection, TagDetectionArray

from .geometry_utils import project_point_to_plane, rigid_transform_svd


# QoS used for all publishers and the camera-info subscriber.
_RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)

# Physical layout of a square marker's corners in marker-local frame
# (counter-clockwise from top-left, z=0 in marker plane).
# Overwritten at runtime with the configured marker_size.
_MARKER_OBJ_PTS_UNIT = np.array(
    [[-0.5,  0.5, 0.0],
     [ 0.5,  0.5, 0.0],
     [ 0.5, -0.5, 0.0],
     [-0.5, -0.5, 0.0]],
    dtype=np.float32,
)


class GroundLocalizer(Node):
    """Detect ArUco markers and publish 2-D world-frame rover poses."""

    def __init__(self) -> None:
        super().__init__("ground_localizer")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("marker_size", 0.10)
        self.declare_parameter("corner_ids", [0, 1, 2, 3])
        self.declare_parameter("rover_ids", [11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27])
        self.declare_parameter("tcp_port", 7777)
        self.declare_parameter("camera_stream_port", 7778)
        self.declare_parameter("camera_stream_fps", 5.0)
        self.declare_parameter("calibration_layout_file", "")
        self.declare_parameter(
            "transform_cache_file",
            "/runtime_output/global_gps/transform_cache.yaml",
        )
        self.declare_parameter("startup_cache_timeout_sec", 30.0)

        self._marker_size: float = float(self.get_parameter("marker_size").value)
        self._corner_ids: list[int] = list(self.get_parameter("corner_ids").value)
        self._rover_ids: list[int] = list(self.get_parameter("rover_ids").value)
        self._tcp_port: int = int(self.get_parameter("tcp_port").value)
        self._camera_stream_port: int = int(self.get_parameter("camera_stream_port").value)
        self._camera_stream_fps: float = float(self.get_parameter("camera_stream_fps").value)
        self._calibration_layout_file: str = str(
            self.get_parameter("calibration_layout_file").value
        )
        self._transform_cache_file: str = str(
            self.get_parameter("transform_cache_file").value
        )
        self._startup_cache_timeout_sec: float = float(
            self.get_parameter("startup_cache_timeout_sec").value
        )

        # Load and validate the world-frame calibration-tag layout.
        # Row order matches self._corner_ids so P_world[i] corresponds to
        # the tvec at P_cam[i] during the SVD calibration step.
        self._P_world: np.ndarray = self._load_calibration_layout()

        self._obj_pts = _MARKER_OBJ_PTS_UNIT * self._marker_size

        # ── OpenCV ArUco detector (supports both 4.6 and ≥ 4.7 APIs) ────────
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, "ArucoDetector"):
            # OpenCV ≥ 4.7
            aruco_params = cv2.aruco.DetectorParameters()
            self._detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            self._use_new_aruco_api = True
        else:
            # OpenCV 4.6 (shipped with Ubuntu 24.04 apt python3-opencv)
            self._aruco_dict = aruco_dict
            self._aruco_params = cv2.aruco.DetectorParameters_create()
            self._use_new_aruco_api = False

        # ── State ──────────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._K: np.ndarray | None = None
        self._D: np.ndarray | None = None
        self._ground_plane: dict | None = None    # {"normal": ..., "d": ...}
        self._T_world_from_cam: np.ndarray | None = None
        self._calibrated: bool = False
        self._using_cached_transform: bool = False
        self._cache_warning_emitted: bool = False
        self._startup_cache_timer = None

        # ── MJPEG preview stream ───────────────────────────────────────────
        self._stream_lock = threading.Lock()
        self._latest_frame_jpg: bytes | None = None
        self._last_stream_ts: float = 0.0
        self._start_mjpeg_server()

        # ── TCP push server (NAT-friendly: robots connect to us) ──────────
        # Robots on WiFi cannot be reached from the wired Jetson because the
        # WiFi AP performs NAT.  Instead, each robot's robot_gps node opens a
        # TCP connection to this server.  We push line-delimited JSON
        # detections over the established connection.
        self._tcp_clients: list[socket.socket] = []
        self._tcp_lock = threading.Lock()
        self._tcp_server_thread = threading.Thread(
            target=self._tcp_server_loop, daemon=True
        )
        self._tcp_server_thread.start()
        self.get_logger().info(
            f"TCP push server listening on port {self._tcp_port}"
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._detections_pub = self.create_publisher(
            TagDetectionArray,
            "/global_gps/tag_detections",
            _RELIABLE_QOS,
        )

        # ── Subscribers ───────────────────────────────────────────────────
        # Topics are relative ("image_raw", "camera_info") so the launch file
        # can remap them to either the RGB color stream or the wider-FOV IR
        # (infra1) stream.
        self.create_subscription(
            CameraInfo,
            "camera_info",
            self._on_camera_info,
            10,
        )

        self.create_subscription(
            Image,
            "image_raw",
            self._on_rgb,
            qos_profile=qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Ground localizer started | "
            f"marker_size={self._marker_size:.3f} m | "
            f"corner_ids={self._corner_ids} | "
            f"rover_ids={self._rover_ids} | "
            f"transform_cache_file={self._transform_cache_file} | "
            f"startup_cache_timeout_sec={self._startup_cache_timeout_sec:.1f}"
        )

        if self._startup_cache_timeout_sec > 0.0:
            self._startup_cache_timer = self.create_timer(
                self._startup_cache_timeout_sec,
                self._on_startup_cache_timeout,
            )

    # ── Camera info ───────────────────────────────────────────────────────

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._K is not None:
            return
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._D = np.array(msg.d, dtype=np.float64)
        self.get_logger().info("Camera intrinsics received.")

    # ── Main image callback ───────────────────────────────────────────────

    def _on_rgb(self, rgb_msg: Image) -> None:
        if self._K is None:
            return

        # Accept either color (bgr8) or IR (mono8) — CvBridge converts as needed.
        gray = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="mono8")

        if self._use_new_aruco_api:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._aruco_dict, parameters=self._aruco_params
            )

        if ids is None:
            self._maybe_update_stream_frame(gray, [], [])
            if self._calibrated:
                self._publish_detections(rgb_msg, {})
            return

        ids = ids.flatten().tolist()
        self._maybe_update_stream_frame(gray, corners, ids)
        marker_poses = self._estimate_poses(corners, ids)

        seen = [m for m in self._corner_ids if m in marker_poses]
        missing = [m for m in self._corner_ids if m not in marker_poses]

        if not self._calibrated or self._using_cached_transform:
            # Rigid 3D alignment needs >= 3 non-collinear correspondences.
            if len(seen) >= 3:
                if missing:
                    self.get_logger().warn(
                        f"Calibrating with {len(seen)}/{len(self._corner_ids)} "
                        f"corners; missing {missing}. Result may be less accurate."
                    )
                source = (
                    "live markers after cached startup"
                    if self._using_cached_transform
                    else "live markers"
                )
                self._calibrate(marker_poses, seen, source=source)
            elif not self._calibrated:
                self.get_logger().warn(
                    f"Waiting for calibration — need >=3 corners, "
                    f"have {len(seen)} (missing {missing}).",
                    throttle_duration_sec=5.0,
                )

        if not self._calibrated:
            return

        self._publish_detections(rgb_msg, marker_poses)

    # ── Pose estimation ───────────────────────────────────────────────────

    def _estimate_poses(
        self,
        corners: list,
        ids: list[int],
    ) -> dict[int, dict]:
        """Run solvePnP for every detected marker and return a pose dict."""
        poses: dict[int, dict] = {}
        for corner, mid in zip(corners, ids):
            success, rvec, tvec = cv2.solvePnP(
                self._obj_pts,
                corner[0],
                self._K,
                self._D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not success:
                continue
            poses[mid] = {"rvec": rvec, "tvec": tvec.reshape(3)}
        return poses

    # ── Calibration ───────────────────────────────────────────────────────

    def _load_calibration_layout(self) -> np.ndarray:
        """Resolve, parse and validate the calibration-tag world layout.

        Returns a ``(4, 3)`` float ndarray ordered to match
        ``self._corner_ids`` (row ``i`` is the world-frame ``(x, y, z)``
        of corner id ``self._corner_ids[i]``).
        """
        # Resolution order: explicit param wins; otherwise packaged default.
        path = self._calibration_layout_file
        if not path:
            try:
                share = get_package_share_directory("global_gps")
            except Exception as exc:
                self.get_logger().error(
                    f"Cannot locate global_gps share directory: {exc}"
                )
                rclpy.shutdown()
                raise
            path = os.path.join(share, "config", "calibration_layout.yaml")

        if not os.path.isfile(path):
            self.get_logger().error(
                f"Calibration layout file not found: {path}"
            )
            rclpy.shutdown()
            raise FileNotFoundError(path)

        try:
            with open(path, "r") as f:
                doc = yaml.safe_load(f)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to parse calibration layout {path}: {exc}"
            )
            rclpy.shutdown()
            raise

        raw = (doc or {}).get("calibration_tags")
        if not isinstance(raw, dict):
            self.get_logger().error(
                f"Calibration layout {path} missing 'calibration_tags' map."
            )
            rclpy.shutdown()
            raise ValueError("calibration_tags missing")

        # PyYAML loads `0:` as int but quoted `"0":` becomes str — coerce.
        coerced: dict[int, list[float]] = {}
        for k, v in raw.items():
            try:
                coerced[int(k)] = [float(x) for x in v]
            except (TypeError, ValueError) as exc:
                self.get_logger().error(
                    f"Bad calibration entry {k!r} -> {v!r}: {exc}"
                )
                rclpy.shutdown()
                raise

        # Validate that every required corner id is present.
        missing = [cid for cid in self._corner_ids if cid not in coerced]
        if missing:
            self.get_logger().error(
                f"Calibration layout {path} is missing corner_ids: {missing}"
            )
            rclpy.shutdown()
            raise KeyError(f"missing corner ids: {missing}")

        # Build (N, 3) array in self._corner_ids order — must match P_cam row order.
        P_world = np.array(
            [coerced[cid] for cid in self._corner_ids],
            dtype=np.float64,
        )
        if P_world.shape[1] != 3:
            self.get_logger().error(
                f"Calibration layout entries must be 3-vectors, got shape {P_world.shape}"
            )
            rclpy.shutdown()
            raise ValueError("layout entries must be length-3")

        # Reject only on collinearity in 3-D: SVD on the centered (N, 3)
        # cloud. sv[0] == 0 means coincident points; sv[1] / sv[0] < 1 %
        # means all on a line. A near-zero sv[2] (coplanar) is allowed —
        # non-coplanar layouts are preferred but coplanar still works.
        centered = P_world - P_world.mean(axis=0)
        sv = np.linalg.svd(centered, compute_uv=False)
        if sv[0] <= 0.0 or sv[1] / sv[0] < 0.01:
            self.get_logger().error(
                "Calibration layout is nearly collinear in 3-D "
                f"(singular values {sv.tolist()}); fix calibration_layout.yaml."
            )
            rclpy.shutdown()
            raise ValueError("collinear calibration layout")

        # Parse the optional ground_plane block and store on self as a
        # side effect (loader returns P_world only; ground-plane spec is
        # exposed via self._gp_point_world / self._gp_normal_world).
        gp = (doc or {}).get("ground_plane")
        if gp is None:
            gp_point = np.array([0.0, 0.0, 0.0], dtype=np.float64)
            gp_normal = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        else:
            if not isinstance(gp, dict):
                self.get_logger().error(
                    f"ground_plane in {path} must be a mapping with "
                    "'point' and 'normal' length-3 vectors."
                )
                rclpy.shutdown()
                raise ValueError("ground_plane must be a mapping")
            try:
                gp_point = np.array(
                    [float(x) for x in gp.get("point", [0.0, 0.0, 0.0])],
                    dtype=np.float64,
                )
                gp_normal = np.array(
                    [float(x) for x in gp.get("normal", [0.0, 0.0, 1.0])],
                    dtype=np.float64,
                )
            except (TypeError, ValueError) as exc:
                self.get_logger().error(
                    f"ground_plane entries in {path} must be numeric: {exc}"
                )
                rclpy.shutdown()
                raise
            if gp_point.shape != (3,) or gp_normal.shape != (3,):
                self.get_logger().error(
                    f"ground_plane.point and ground_plane.normal in {path} "
                    "must be length-3 vectors."
                )
                rclpy.shutdown()
                raise ValueError("ground_plane vectors must be length-3")

        n_norm = float(np.linalg.norm(gp_normal))
        if n_norm <= 0.0:
            self.get_logger().error(
                f"ground_plane.normal in {path} has zero length."
            )
            rclpy.shutdown()
            raise ValueError("ground_plane.normal has zero length")
        gp_normal = gp_normal / n_norm

        self._gp_point_world = gp_point
        self._gp_normal_world = gp_normal

        self.get_logger().info(
            f"Loaded calibration layout from {path} ({len(self._corner_ids)} tags); "
            f"ground_plane point={gp_point.tolist()} normal={gp_normal.tolist()}."
        )
        return P_world

    def _calibrate(
        self,
        marker_poses: dict[int, dict],
        seen_ids: list[int],
        source: str = "live markers",
    ) -> None:
        """SVD-align measured tag tvecs to the known world layout.

        ``seen_ids`` is the subset of ``self._corner_ids`` currently detected
        (length >= 3). Only those rows of ``self._P_world`` are used.
        """
        self.get_logger().info(
            f"Calibrating world frame via SVD using {len(seen_ids)} corners: "
            f"{seen_ids} ({source})"
        )

        # Pick the matching rows of P_world (preserving corner_ids order).
        idx = [self._corner_ids.index(cid) for cid in seen_ids]
        P_world = self._P_world[idx]
        P_cam = np.array(
            [marker_poses[cid]["tvec"] for cid in seen_ids],
            dtype=np.float64,
        )

        # With exactly 3 points, refuse to proceed if they're collinear —
        # SVD returns a degenerate transform (rotation about the line is
        # unconstrained) and downstream world poses become garbage.
        if len(seen_ids) == 3:
            v1 = P_world[1] - P_world[0]
            v2 = P_world[2] - P_world[0]
            area = float(np.linalg.norm(np.cross(v1, v2)))
            if area < 1e-6:
                self.get_logger().warn(
                    f"3 visible corners {seen_ids} are collinear in the world "
                    f"frame — cannot calibrate. Waiting for a 4th corner."
                )
                return

        T_cam_from_world = rigid_transform_svd(P_world, P_cam)
        self._T_world_from_cam = np.linalg.inv(T_cam_from_world)

        # Ground plane in camera frame is the image of the user-specified
        # world-frame plane (default world z = 0 if no ground_plane block).
        R_cw = T_cam_from_world[:3, :3]
        t_cw = T_cam_from_world[:3, 3]
        n_cam = R_cw @ self._gp_normal_world
        p_cam = R_cw @ self._gp_point_world + t_cw
        self._ground_plane = {"normal": n_cam, "d": -float(n_cam @ p_cam)}

        # Per-tag residuals in millimetres for operator feedback.
        P_world_h = np.hstack([P_world, np.ones((P_world.shape[0], 1))])
        P_cam_pred = (T_cam_from_world @ P_world_h.T).T[:, :3]
        errs = np.linalg.norm(P_cam_pred - P_cam, axis=1)
        max_mm = float(errs.max() * 1000.0)
        rms_mm = float(np.sqrt(np.mean(errs ** 2)) * 1000.0)
        per_tag = ", ".join(
            f"{cid}:{e * 1000.0:.1f}mm"
            for cid, e in zip(seen_ids, errs)
        )

        self._calibrated = True
        self._using_cached_transform = False
        self._cache_warning_emitted = False
        self._save_cached_transform()
        self._cancel_startup_cache_timer()
        self.get_logger().info(
            f"Calibration complete ({len(seen_ids)} corners). "
            f"Residuals max={max_mm:.1f}mm rms={rms_mm:.1f}mm per-tag=[{per_tag}]"
        )

    def _on_startup_cache_timeout(self) -> None:
        """Load the cached transform after the startup timeout, if needed."""
        self._cancel_startup_cache_timer()
        if self._calibrated:
            return
        if not self._load_cached_transform():
            self.get_logger().warn(
                "Startup calibration timeout expired and no valid cached "
                "transformation is available. Continuing to wait for live markers."
            )
            return
        self._calibrated = True
        self._using_cached_transform = True
        if not self._cache_warning_emitted:
            self.get_logger().warn(
                "Cannot detect the current localization markers after "
                f"{self._startup_cache_timeout_sec:.1f}s. Using cached transformation "
                f"from {self._transform_cache_file}."
            )
            self._cache_warning_emitted = True

    def _cancel_startup_cache_timer(self) -> None:
        if self._startup_cache_timer is None:
            return
        self._startup_cache_timer.cancel()
        self.destroy_timer(self._startup_cache_timer)
        self._startup_cache_timer = None

    def _save_cached_transform(self) -> None:
        """Persist the latest successful calibration to disk."""
        if self._T_world_from_cam is None or self._ground_plane is None:
            return
        try:
            cache_dir = os.path.dirname(self._transform_cache_file)
            if cache_dir:
                os.makedirs(cache_dir, exist_ok=True)

            payload = {
                "version": 1,
                "saved_at": datetime.now(timezone.utc).isoformat(),
                "marker_size": self._marker_size,
                "corner_ids": self._corner_ids,
                "transform_world_from_cam": self._T_world_from_cam.tolist(),
                "ground_plane": {
                    "normal": self._ground_plane["normal"].tolist(),
                    "d": float(self._ground_plane["d"]),
                },
            }

            with open(self._transform_cache_file, "w", encoding="utf-8") as f:
                yaml.safe_dump(payload, f, sort_keys=False)
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to save cached transformation to "
                f"{self._transform_cache_file}: {exc}"
            )

    def _load_cached_transform(self) -> bool:
        """Load a cached calibration transform from disk."""
        path = self._transform_cache_file
        if not path or not os.path.isfile(path):
            return False

        try:
            with open(path, "r", encoding="utf-8") as f:
                doc = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to read cached transformation {path}: {exc}"
            )
            return False

        try:
            cached_corner_ids = [int(x) for x in doc["corner_ids"]]
            cached_marker_size = float(doc["marker_size"])
            T_world_from_cam = np.array(
                doc["transform_world_from_cam"],
                dtype=np.float64,
            )
            gp_doc = doc["ground_plane"]
            gp_normal = np.array(gp_doc["normal"], dtype=np.float64)
            gp_d = float(gp_doc["d"])
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(
                f"Cached transformation {path} is malformed: {exc}"
            )
            return False

        if cached_corner_ids != self._corner_ids:
            self.get_logger().warn(
                f"Cached transformation corner_ids {cached_corner_ids} do not match "
                f"current corner_ids {self._corner_ids}; ignoring cache."
            )
            return False
        if abs(cached_marker_size - self._marker_size) > 1e-9:
            self.get_logger().warn(
                f"Cached transformation marker_size {cached_marker_size:.6f} does not "
                f"match current marker_size {self._marker_size:.6f}; ignoring cache."
            )
            return False
        if T_world_from_cam.shape != (4, 4):
            self.get_logger().warn(
                f"Cached transformation matrix has shape {T_world_from_cam.shape}, "
                "expected (4, 4); ignoring cache."
            )
            return False
        if gp_normal.shape != (3,):
            self.get_logger().warn(
                f"Cached ground-plane normal has shape {gp_normal.shape}, "
                "expected (3,); ignoring cache."
            )
            return False

        self._T_world_from_cam = T_world_from_cam
        self._ground_plane = {"normal": gp_normal, "d": gp_d}
        return True

    # ── Detection publishing ──────────────────────────────────────────────

    def _publish_detections(
        self,
        rgb_msg: Image,
        marker_poses: dict[int, dict],
    ) -> None:
        rover_ids_seen = [
            mid for mid in marker_poses
            if mid in self._rover_ids and mid not in self._corner_ids
        ]

        detections: list[TagDetection] = []
        for mid in rover_ids_seen:
            result = self._compute_world_pose(marker_poses[mid])
            if result is None:
                continue
            x, y, theta = result

            det = TagDetection()
            det.tag_id = int(mid)
            det.x = float(x)
            det.y = float(y)
            det.theta = float(theta)
            detections.append(det)

            self.get_logger().debug(
                f"Tag {mid}: x={x:.3f} m, y={y:.3f} m, "
                f"theta={np.degrees(theta):.1f}°"
            )

        # Always publish — even when no rover tag is visible — so consumers
        # see a steady stream and can distinguish "no rovers visible" from
        # "node is dead". An empty detections array is the no-rovers signal.
        msg = TagDetectionArray()
        msg.header = rgb_msg.header
        msg.detections = detections
        self._detections_pub.publish(msg)

        stamp_sec = (
            rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        )
        self._tcp_push(detections, stamp_sec)

        if detections:
            self.get_logger().info(
                f"Published {len(detections)} rover tag(s): "
                f"{[d.tag_id for d in detections]}"
            )
        else:
            self.get_logger().debug("No rover tags visible — published empty array.")

    # ── World-pose computation ────────────────────────────────────────────

    def _compute_world_pose(
        self,
        marker_data: dict,
    ) -> tuple[float, float, float] | None:
        """Project a marker pose into the world frame.

        Returns (x, y, theta) in world coordinates, or None if the marker
        orientation is degenerate (parallel to the world Z axis).
        """
        tvec: np.ndarray = marker_data["tvec"]
        rvec: np.ndarray = marker_data["rvec"]

        normal = self._ground_plane["normal"]
        d = self._ground_plane["d"]

        # Project marker centre onto the ground plane.
        p_ground = project_point_to_plane(tvec, normal, d)
        p_world = self._T_world_from_cam @ np.append(p_ground, 1.0)

        # Project marker X axis onto the ground plane to get heading.
        R_cam_marker, _ = cv2.Rodrigues(rvec)
        marker_x_cam = R_cam_marker[:, 0]
        R_world_from_cam = self._T_world_from_cam[:3, :3]
        marker_x_world = R_world_from_cam @ marker_x_cam

        # Remove the normal component so the direction is in-plane.
        marker_x_world -= np.dot(marker_x_world, normal) * normal
        norm = np.linalg.norm(marker_x_world)
        if norm < 1e-6:
            self.get_logger().warn("Degenerate marker orientation — skipping.")
            return None
        marker_x_world /= norm

        theta = float(np.arctan2(marker_x_world[1], marker_x_world[0]))
        return float(p_world[0]), float(p_world[1]), theta


    # ── MJPEG preview server ──────────────────────────────────────────────

    def _start_mjpeg_server(self) -> None:
        node = self

        class _Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if self.path != "/stream":
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame",
                )
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                try:
                    while True:
                        with node._stream_lock:
                            jpg = node._latest_frame_jpg
                        if jpg is None:
                            time.sleep(0.05)
                            continue
                        self.wfile.write(
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n"
                            + jpg
                            + b"\r\n"
                        )
                        self.wfile.flush()
                        time.sleep(1.0 / node._camera_stream_fps)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass

            def log_message(self, *_args) -> None:
                pass  # suppress per-request log noise

        srv = ThreadingHTTPServer(("0.0.0.0", self._camera_stream_port), _Handler)
        t = threading.Thread(target=srv.serve_forever, daemon=True)
        t.start()
        self.get_logger().info(
            f"MJPEG preview stream at http://0.0.0.0:{self._camera_stream_port}/stream"
            f" ({self._camera_stream_fps:.0f} FPS)"
        )

    def _maybe_update_stream_frame(
        self, gray: np.ndarray, corners: list, ids: list[int]
    ) -> None:
        """Throttle-encode and store the latest annotated preview frame."""
        now = time.monotonic()
        if now - self._last_stream_ts < 1.0 / self._camera_stream_fps:
            return
        self._last_stream_ts = now

        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if ids and corners:
            cv2.aruco.drawDetectedMarkers(bgr, corners)
            for corner, mid in zip(corners, ids):
                cx = int(corner[0][:, 0].mean())
                cy = int(corner[0][:, 1].mean())
                cv2.putText(
                    bgr,
                    str(mid),
                    (cx + 5, cy - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 50])
        if ok:
            with self._stream_lock:
                self._latest_frame_jpg = buf.tobytes()

    # ── TCP server ────────────────────────────────────────────────────────

    def _tcp_server_loop(self) -> None:
        """Accept robot connections in a background thread."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self._tcp_port))
        srv.listen(16)
        srv.settimeout(1.0)
        while rclpy.ok():
            try:
                conn, addr = srv.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self._tcp_lock:
                    self._tcp_clients.append(conn)
                self.get_logger().info(f"Robot connected from {addr}")
            except socket.timeout:
                continue
            except Exception as exc:
                self.get_logger().warn(f"TCP server error: {exc}")
        srv.close()

    def _tcp_push(self, detections: list[TagDetection], stamp_sec: float) -> None:
        """Serialize detections to JSON and push to all connected robots."""
        payload = json.dumps({
            "stamp": stamp_sec,
            "detections": [
                {"tag_id": d.tag_id, "x": d.x, "y": d.y, "theta": d.theta}
                for d in detections
            ],
        }) + "\n"
        data = payload.encode()
        dead: list[socket.socket] = []
        with self._tcp_lock:
            for conn in self._tcp_clients:
                try:
                    conn.sendall(data)
                except OSError:
                    dead.append(conn)
            for conn in dead:
                self._tcp_clients.remove(conn)
                conn.close()
                self.get_logger().info("Robot disconnected")


def main() -> None:
    rclpy.init()
    node = GroundLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
