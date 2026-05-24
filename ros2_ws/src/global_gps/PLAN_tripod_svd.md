# Plan: Tripod-mounted TrueVision with known calibration-tag coordinates + SVD pose alignment

## Goal

Adapt the `global_gps` package so the TrueVision (RealSense) camera can be mounted
on a **tripod looking down at the ground at an angle** rather than being suspended
directly overhead. Calibration tags (IDs 0, 1, 2, 3) have **known world-frame
coordinates** (measured by hand and entered in a config file). The world transform
is recovered from the four detected tag positions using an **SVD-based rigid
alignment** (Kabsch / Umeyama).

## Hard constraints (do NOT change)

- The published topic stays `/global_gps/tag_detections` with the existing
  `bridge_interfaces/msg/TagDetectionArray` message.
- Each `TagDetection` keeps the same fields: `tag_id`, `x`, `y`, `theta`.
- The TCP push server (port `7777`, line-delimited JSON, same payload schema with
  `stamp` + `detections[]` entries `{tag_id, x, y, theta}`) is unchanged.
- QoS, header stamping, and rover-tag filtering behavior are unchanged.
- The **communication protocol and physical link are unchanged**: detections
  continue to be delivered to robots over **Ethernet** via the existing TCP
  push server (NAT-friendly, robots dial in to the Jetson). No switch to UDP,
  multicast, MQTT, WebSocket, ROS-DDS-over-WiFi, or any other transport.
  Bind address, listen port, accept loop, and per-client `sendall` semantics
  stay as they are in `_tcp_server_loop` / `_tcp_push`.
- No other package in `ros2_ws`/`src` is touched other than `global_gps`

Downstream consumers must not need to be touched.

## What changes

### 1. Calibration input — Option A: explicit world coordinates

Add a YAML config file shipped with the package:

```text
ros2_ws/src/global_gps/config/calibration_layout.yaml
```

Contents (example):

```yaml
# World-frame coordinates of the four calibration tags, in metres.
# Tags do NOT need to be coplanar with the ground (see §2b); any known
# (x, y, z) layout is fine, and a non-coplanar layout is preferred for
# best calibration conditioning.
# The user defines the world frame however they like (e.g. tag 0 at
# origin, +X toward tag 1, +Y toward tag 2). Whatever convention is
# chosen here is what downstream rover poses will be expressed in.
calibration_tags:
  0: [0.000, 0.000, 0.00]
  1: [2.400, 0.000, 0.05]
  2: [0.000, 1.800, 0.30]
  3: [2.400, 1.800, 0.35]

# Optional: ground plane in world frame. Used to project rover-tag
# tvecs before reporting (x, y) detections. Default if omitted:
# point=(0,0,0), normal=(0,0,1) — i.e. world z = 0.
ground_plane:
  point:  [0.0, 0.0, 0.0]
  normal: [0.0, 0.0, 1.0]
```

The user fills in the four `(x, y, z)` triples based on their measured
tag positions and chosen world-frame convention. No solving for an
embedding from distances — coordinates are given directly.

### 2. New SVD utility in `geometry_utils.py`

Add one function:

```python
def rigid_transform_svd(P_src: np.ndarray,
                        P_dst: np.ndarray) -> np.ndarray:
    """
    Kabsch / Umeyama. Given N>=3 corresponding 3-D points (rows),
    return the 4x4 homogeneous transform T such that T @ P_src ~= P_dst
    in the least-squares sense, with a reflection guard.
    """
```

Algorithm:

1. `mu_src = P_src.mean(0)`, `mu_dst = P_dst.mean(0)`.
2. `H = (P_src - mu_src).T @ (P_dst - mu_dst)`.
3. `U, _, Vt = np.linalg.svd(H)`.
4. `D = diag(1, 1, sign(det(Vt.T @ U.T)))` to prevent reflections.
5. `R = Vt.T @ D @ U.T`, `t = mu_dst - R @ mu_src`.
6. Assemble homogeneous `T`.

`project_point_to_plane` is kept (used to project noisy rover tvecs onto the
ground plane, where the plane is now derived analytically from
`T_cam_from_world` rather than from a runtime fit). `fit_plane_svd` is no
longer called anywhere after this change and can be removed; if it is kept
for future use, mark it as unused. `build_world_transform` is removed
outright — its job is now done by `rigid_transform_svd` against the known
layout. The existing import line at `ground_localizer_node.py:48`
(`from .geometry_utils import fit_plane_svd, project_point_to_plane,
build_world_transform`) must be updated to drop the removed names.

### 2a. Approach choice — SVD on 4 tag tvecs (accepted)

We are deliberately running SVD on the **4 tag-center tvecs** produced by
per-tag `solvePnP`, not on the 16 individual marker corners. Rationale:

- The tripod is mounted at a moderate downward angle, not a steeply
  oblique angle. Per-tag tvec depth noise is bounded in this regime.
- The four calibration tags span a known rectangle, which keeps the
  Kabsch alignment well-conditioned.
- Keeps the implementation small and matches the user's explicit ask.

A more robust alternative for steeply oblique mounts (single coplanar
`solvePnP` over all 16 tag corners with known world coordinates) is
**not implemented**. If residuals later prove too large, that is the
upgrade path.

### 2b. Arbitrary known 3-D tag positions (no coplanarity required)

The 4 calibration tags are **not** required to lie on the ground plane,
nor on any single plane. As long as their world-frame coordinates
`(x, y, z)` are known, Kabsch / Umeyama SVD alignment recovers the rigid
transform `T_cam_from_world` correctly. Conditions:

- `N >= 3` correspondences (we have 4) ✓
- Points are not all collinear in 3-D
- Coplanarity is allowed but **non-coplanar layouts are better-
  conditioned**: a fully 3-D spread constrains all three rotation axes
  directly, so depth-direction noise in the per-tag tvecs is averaged
  out instead of amplified. In practice, mount the four tags at
  visibly different heights (e.g. tape two on the floor and two on
  short stands) for the best calibration quality.

The layout YAML therefore allows arbitrary `z` values:

```yaml
calibration_tags:
  0: [0.000, 0.000, 0.00]
  1: [2.400, 0.000, 0.05]
  2: [0.000, 1.800, 0.30]
  3: [2.400, 1.800, 0.35]
```

The non-collinearity validator (currently checks centered **XY**
singular values) must be generalised: compute SVD on the centered
`(N, 3)` cloud and reject only if the **largest** singular value is
zero (all points coincident) or the second-largest is below ~1 % of
the largest (all points on a line). A near-zero third singular value
(coplanar) is acceptable.

### 2c. Decoupling the ground plane from the calibration tags

Previously the ground plane was implicitly defined as the plane through
the four tags. Now that tags can sit anywhere in 3-D, the ground plane
must be specified **independently** in the world frame. Add a top-level
block to `calibration_layout.yaml`:

```yaml
# Ground plane in world frame. Rover tag tvecs are projected onto this
# plane before being reported as (x, y) detections.
# Default: world z = 0.
ground_plane:
  point:  [0.0, 0.0, 0.0]
  normal: [0.0, 0.0, 1.0]
```

The loader reads `ground_plane` (with the above default if the block is
absent — preserves existing behaviour for users whose tags are on the
floor) and stores the plane in **world frame**. The calibration step
maps it into camera frame via `T_cam_from_world` instead of hard-coding
`[0, 0, 1]` / `world z = 0`. Pseudocode:

```python
n_world = np.asarray(layout["ground_plane"]["normal"], float)
n_world /= np.linalg.norm(n_world)
p_world = np.asarray(layout["ground_plane"]["point"], float)

R_cw = T_cam_from_world[:3, :3]
t_cw = T_cam_from_world[:3, 3]
n_cam = R_cw @ n_world
p_cam = R_cw @ p_world + t_cw
self._ground_plane = {"normal": n_cam, "d": -float(n_cam @ p_cam)}
```

This replaces the `world z = 0` shortcut in §3.

### 2d. Drop the depth subscription (RGB-only operation)

The localizer must **not depend on any sensor other than the RGB
stream**. Specifically:

- The depth subscription (`/camera/camera/depth/image_rect_raw`) and
  the `ApproximateTimeSynchronizer` over RGB+depth at
  `ground_localizer_node.py:140-147` are removed.
- Replace with a plain `create_subscription` on
  `/camera/camera/color/image_raw` calling a new `_on_rgb(rgb_msg)`
  handler (rename of `_on_images` with the unused `depth_msg`
  parameter dropped — Pylance currently flags `depth_msg` as unused).
- Remove `from message_filters import Subscriber,
  ApproximateTimeSynchronizer`.
- The launch file may keep `align_depth.enable: true` off; depth is no
  longer required by this node. Other nodes that consume depth are
  unaffected.

The package contains no IMU subscription today (`grep -ri imu` in
`global_gps/` returns nothing), so there is no IMU dependency to
remove. If a future iteration needs gravity alignment for the ground
plane, supply it via the `ground_plane` config block in §2c rather
than via an IMU.

### 3. Calibration step in `ground_localizer_node.py`

Replace `_calibrate` (currently lines 224–238) with logic equivalent to:

```python
P_world = layout array, shape (4,3), ordered to match self._corner_ids
P_cam   = np.array([marker_poses[mid]["tvec"] for mid in self._corner_ids])

T_cam_from_world = rigid_transform_svd(P_world, P_cam)
self._T_world_from_cam = np.linalg.inv(T_cam_from_world)

# Ground plane: image of the user-specified world-frame plane (§2c).
# Default if unspecified: point=(0,0,0), normal=(0,0,1).
R_cw = T_cam_from_world[:3, :3]
t_cw = T_cam_from_world[:3, 3]
n_cam = R_cw @ self._gp_normal_world
p_cam = R_cw @ self._gp_point_world + t_cw
self._ground_plane = {"normal": n_cam, "d": -float(n_cam @ p_cam)}
self._calibrated = True
```

Also log per-tag residuals `‖(T_cam_from_world @ P_world_i) - P_cam_i‖`
(max + RMS) so the operator can judge tripod placement quality. Report
residuals in **millimetres** (the "max < 1 cm" rule of thumb is easier to
read as `mm`).

Before computing the SVD, validate the layout per §2b: SVD on the
centered `(N, 3)` cloud and reject only on collinearity (second-largest
singular value below ~1 % of the largest). Coplanarity is acceptable;
non-coplanar layouts are preferred.

The rest of `_on_images`, `_estimate_poses`, `_compute_world_pose`,
`_publish_detections`, `_tcp_push` remain untouched.

### 4. New ROS parameter

In `__init__`:

```python
self.declare_parameter("calibration_layout_file", "")
```

Resolution order:

1. If parameter is non-empty, load that file.
2. Else load the default at
   `<package_share>/config/calibration_layout.yaml`.

Loader builds a `(4,3)` ndarray whose **row order matches
`self._corner_ids`** (not YAML insertion order, not numeric sort). Add an
inline comment in the loader making this explicit, since `P_world` row `i`
must correspond to the tvec at `P_cam` row `i`.

YAML key handling: PyYAML loads `0:` as `int` but a quoted `"0":` becomes
`str`. The loader must coerce keys via `int(k)` and validate. If any
`corner_id` is missing from the YAML, or if the file is missing entirely
when no fallback default exists, fail fast with a clear log message and
shut the node down.

### 5. Launch file

`launch/global_gps.launch.py`:

- Add `calibration_layout_file` `DeclareLaunchArgument` (default `""` →
  node uses the packaged default).
- Pass it through to the node parameters.
- No other changes (RealSense launch unchanged).

### 6. Packaging

`setup.py`: install `config/calibration_layout.yaml` into the package
share directory so `get_package_share_directory("global_gps")` resolves
the default path. Concretely, add to `data_files`:

```python
(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
```

`package.xml` is unaffected.

### 7. README

Update the README with:

- A short section explaining the tripod use case.
- How to measure the tags and fill in `calibration_layout.yaml`.
- How to read the calibration residual log line and what magnitude is
  acceptable (rule of thumb: max residual under ~1 cm).

## Files touched

| File | Change |
| --- | --- |
| `global_gps/geometry_utils.py` | add `rigid_transform_svd`; remove `build_world_transform` |
| `global_gps/ground_localizer_node.py` | new param, layout loader (incl. `ground_plane` block), rewritten `_calibrate`, residual logging, drop depth subscription + `ApproximateTimeSynchronizer` (RGB-only) |
| `launch/global_gps.launch.py` | new `calibration_layout_file` arg |
| `config/calibration_layout.yaml` (new file) | example world coords for tags 0–3 + optional `ground_plane` block |
| `setup.py` | install `config/` into share dir |
| `README.md` | tripod setup + calibration instructions |
| `PLAN_tripod_svd.md` (this file) | design reference |

## Out of scope

- No change to the published message type, topic name, QoS, or TCP JSON schema.
- No change to rover-tag detection or per-tag `solvePnP` for rovers.
- No multi-frame calibration averaging (single-frame SVD is sufficient given
  known coordinates; can be added later if residuals are too high).
- No automatic recovery of the world frame from inter-tag distances — the
  user supplies coordinates directly (Option A).

## Test strategy

1. **Unit test** for `rigid_transform_svd`: synthesize a random rotation +
   translation, transform a point cloud, recover `T`, assert error < 1e-9.
2. **Unit test** for the reflection guard: add noise that would otherwise
   push the SVD toward a reflection (negative determinant), then assert
   `det(R) ≈ +1` after the guard.
3. **Bench test**: run the node against a recorded bag from a tripod-mounted
   camera; verify (a) calibration residuals are small, (b) published rover
   `(x, y, theta)` values match a hand-measured ground-truth position to
   within a few cm.
