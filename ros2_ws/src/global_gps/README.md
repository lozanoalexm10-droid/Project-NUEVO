# Global GPS — Field Localizer

Runs on the **Jetson Nano** (not on the robot RPis).

Uses a RealSense D4xx camera (overhead or tripod-mounted at an oblique angle —
see "Non-coplanar tags and the ground plane" below) to detect ArUco markers and
publish 2-D world-frame positions for all rover robots simultaneously.

## How it works

1. **Calibration**: On startup, the node waits until all four corner anchor
   markers (IDs 0–3 by default) are visible at once. It then SVD-aligns the
   four measured tag tvecs to the **known world-frame coordinates** that you
   provided in `config/calibration_layout.yaml` (Kabsch / Umeyama with a
   reflection guard). The ground plane in camera frame is derived
   analytically as the image of the world `z = 0` plane. Calibration is
   automatic — just make sure the camera can see all four corners when the
   node starts. The most recent successful transform is cached to
   `/runtime_output/global_gps/transform_cache.yaml` by default; if startup
   cannot find enough localization markers within 30 seconds, the node logs a
   warning and falls back to that cached transform.

2. **Tracking**: Once calibrated, the node detects any rover markers (IDs 11–18
   by default) in every frame and publishes their 2-D poses (x, y, theta) on
   `/global_gps/tag_detections` across the network.

3. **Robot side**: Each robot runs a `robot_gps` node (in the `sensors` package)
   that subscribes to the Jetson's global topic and re-publishes detections
   locally so the robot's main node can use them.

## Marker setup

| Role | IDs | Size |
|---|---|---|
| Field corner anchors | 0, 1, 2, 3 | configurable (`marker_size` param) |
| Rover markers | 11–18 | same size |

Place corner markers at the four corners of the field with the **same physical
size** as the rover markers. The world frame origin, axes, and units are
defined entirely by what you write in `config/calibration_layout.yaml` — see
the next section.

## Tripod / oblique camera mounts

The localizer works whether the RealSense is hung directly overhead or
mounted on a **tripod looking down at an angle**. In the tripod case the
ground plane is no longer trivially perpendicular to the camera, so the node
recovers the world frame by SVD-aligning the four detected corner-tag
positions to a layout you supply.

### Filling in `calibration_layout.yaml`

The packaged default lives at
`ros2_ws/src/global_gps/config/calibration_layout.yaml`:

```yaml
# All values in metres. z = 0 because the four tags lie on the ground.
calibration_tags:
  0: [0.000, 0.000, 0.0]
  1: [2.400, 0.000, 0.0]
  2: [0.000, 1.800, 0.0]
  3: [2.400, 1.800, 0.0]
```

Steps:

1. Put down corner markers 0, 1, 2, 3 anywhere you like on the ground.
2. Pick a world-frame convention. A typical choice: tag 0 at the origin,
   `+X` toward tag 1, `+Y` toward tag 2.
3. Measure the inter-tag distances by hand (tape measure) and fill in the
   `(x, y, 0)` triple for each tag in your chosen frame.
4. Either edit the packaged file directly, or write your own and pass
   `calibration_layout_file:=/path/to/your.yaml` to the launch.

Whatever convention you pick is what downstream rover poses are reported in.

### Non-coplanar tags and the ground plane

Calibration tags do **not** have to be coplanar with the floor — any
known `(x, y, z)` layout works, and a layout where the four tags sit at
visibly different heights (e.g. two on the floor, two on short stands)
is in fact better-conditioned than a flat one because the depth axis is
constrained directly. Because the tags can now sit anywhere in 3-D, the
ground plane is specified independently in the world frame via an
optional `ground_plane` block in `calibration_layout.yaml`:

```yaml
ground_plane:
  point:  [0.0, 0.0, 0.0]
  normal: [0.0, 0.0, 1.0]
```

If the block is omitted, the default is `point=(0,0,0)`,
`normal=(0,0,1)` — i.e. world `z = 0`, which preserves prior behaviour
for users whose tags lie flat on the floor. Rover tag tvecs are
projected onto this plane before being reported as `(x, y)` detections.

### RGB-only operation

The localizer subscribes only to `/camera/camera/color/image_raw`. The
RealSense depth stream is **no longer consumed** by this node — all
geometry is recovered from the calibrated camera pose plus the
configured ground plane. Other nodes that consume depth are unaffected.

### Reading the calibration log line

When calibration succeeds the node logs something like:

```
Calibration complete. Residuals max=4.1mm rms=2.7mm per-tag=[0:1.2mm, 1:3.8mm, 2:4.1mm, 3:2.0mm]
```

`max` is the worst per-tag fit error (Euclidean distance between the
measured tvec and the SVD-predicted tvec). Rule of thumb:

- **`max < 10 mm` (1 cm)** — healthy. Use as is.
- 1–3 cm — usable but suspect; re-measure inter-tag distances or check that
  the camera's intrinsic calibration is reasonable.
- `> 3 cm` — bad. Likely causes: typo in the YAML, mis-placed corner
  marker, very oblique tripod angle that pushes per-tag depth noise up, or
  one of the four tags partially occluded.

## ROS topic

| Topic | Type | Publisher |
|---|---|---|
| `/global_gps/tag_detections` | `bridge_interfaces/TagDetectionArray` | Jetson |
| `/tag_detections` | `bridge_interfaces/TagDetectionArray` | `robot_gps` (RPi) |

Each `TagDetection` in the array contains:

```
int32   tag_id   # ArUco marker ID
float32 x        # world-frame X (metres)
float32 y        # world-frame Y (metres)
float32 theta    # heading (radians, CCW from world X)
```

---

## Jetson setup

### Prerequisites (one-time, on the Jetson host)

1. **JetPack 6.x** (Ubuntu 22.04) installed and working.
2. **Docker** installed:
   ```bash
   curl -fsSL https://get.docker.com -o get-docker.sh
   sudo sh get-docker.sh
   # JetPack may not create the docker group automatically — create it if missing:
   sudo groupadd docker 2>/dev/null || true
   sudo usermod -aG docker $USER
   newgrp docker
   docker --version
   ```
3. Clone your fork of the repository:
   ```bash
   git clone https://github.com/<your-username>/Project-NUEVO.git
   cd Project-NUEVO
   ```

### Build and start the container

```bash
COMPOSE=ros2_ws/docker/docker-compose.jetson.yml

docker compose -f $COMPOSE build       # first time only (~5 min)
docker compose -f $COMPOSE up -d
docker compose -f $COMPOSE logs -f global_gps   # watch colcon build
```

Wait until the log shows `[entrypoint] Container ready`.

### Launch the localizer

```bash
docker compose -f $COMPOSE exec global_gps bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch global_gps global_gps.launch.py
```

To override default parameters:

```bash
ros2 launch global_gps global_gps.launch.py \
    marker_size:=0.15 \
    corner_ids:=[0,1,2,3] \
    rover_ids:=[11,12,13]
```

The cache fallback can be configured explicitly:

```bash
ros2 launch global_gps global_gps.launch.py \
    transform_cache_file:=/runtime_output/global_gps/transform_cache.yaml \
    startup_cache_timeout_sec:=30.0
```

### Boot-time startup service

This package now includes host-side startup assets:

- `scripts/start_global_gps_stack.sh`
- `systemd/global-gps-stack.service`

The service template does the two required startup actions:

1. `docker compose ... up -d --wait global_gps`
2. `ros2 launch global_gps global_gps.launch.py`

Because enabling `systemd` on the Jetson modifies host state outside
`src/global_gps`, installation is still a manual host step. Copy or symlink
the packaged `systemd/global-gps-stack.service` into `/etc/systemd/system/`,
adjust `WorkingDirectory` if the repo lives elsewhere, then run:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now global-gps-stack.service
```

### Capture a photo of the camera's field of view

A standalone script is included that saves a single JPEG from the RealSense
camera without needing the full ROS2 stack running.  It uses `pyrealsense2`
directly via the libusb/V4L2 backend and automatically repairs missing device
nodes inside the container after a USB re-enumeration.

**One-time setup** — install `pyrealsense2` inside the container:
```bash
docker compose -f $COMPOSE exec global_gps \
    pip3 install --break-system-packages pyrealsense2
```

**Capture a photo** (saves to `/tmp/gps_snapshot_<timestamp>.jpg` by default):
```bash
# From the Jetson host:
docker exec docker-global_gps-1 \
    python3 /ros2_ws/src/global_gps/capture_photo.py

# Or from inside the container shell:
python3 /ros2_ws/src/global_gps/capture_photo.py

# Custom output path:
python3 /ros2_ws/src/global_gps/capture_photo.py --output /tmp/field.jpg
```

The script waits for 15 warm-up frames (~0.5 s) so auto-exposure settles
before saving.  Additional options:

```
--output / -o   Output file path
--warmup        Warm-up frame count (default: 15)
--width         Stream width  (default: 640)
--height        Stream height (default: 480)
--fps           Stream FPS    (default: 30)
```

> **Note:** The GPS launch (`ros2 launch global_gps global_gps.launch.py`)
> must **not** be running when you capture a photo — both the launch and the
> script compete for exclusive camera access.  Stop the launch first, capture,
> then restart it if needed.

### Verify detections from any machine on the network

```bash
# Quick test: connect to the Jetson TCP server directly (no ROS needed)
# nc -v 192.168.8.120 7777   # should show JSON lines when markers are visible

# Or from inside the robot container:
ros2 topic echo /tag_detections
```

---

## Robot-side setup (RPi per robot)

The `robot_gps` node in the `sensors` package bridges the Jetson's global
topic into the robot's local domain.

### Start robot_gps

Open a terminal inside the robot's container and run it separately from
the bridge/robot nodes:

```bash
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml

docker compose -f $COMPOSE exec ros2_runtime bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run sensors robot_gps
```

No special environment variables are needed.  The node connects to the
Jetson's TCP server directly, which works through the lab WiFi NAT.

Once running, the topic `/tag_detections` becomes available locally and other
nodes (robot node, etc.) can subscribe to it with the RPi Docker default
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.

### Subscribe in the robot node

```python
from bridge_interfaces.msg import TagDetectionArray

self.create_subscription(
    TagDetectionArray,
    "/tag_detections",
    self._on_gps_update,
    10,
)

def _on_gps_update(self, msg: TagDetectionArray) -> None:
    for det in msg.detections:
        print(f"Robot {det.tag_id}: ({det.x:.2f}, {det.y:.2f})")
```

---

## Troubleshooting

**No detections published / calibration never completes**
- Make sure all four corner marker IDs (default 0–3) are visible in the camera
  frame simultaneously when the node starts.
- Check that `corner_ids` and `rover_ids` parameters match the printed markers.

**RealSense camera not found**
- Confirm the camera is connected: `rs-enumerate-devices` (inside the container).
- The container runs with `privileged: true` which is required for the
  librealsense2 USB driver. If the camera still isn't found, try
  `docker compose down` and `up -d` with the camera already plugged in.

**Robot cannot see `/tag_detections`**
- Confirm `robot_gps` is running in the RPi container.
- Confirm the Jetson and the RPi are on the same network.
- Confirm the Jetson TCP server is reachable from the RPi:
  `nc -v 192.168.8.120 7777`
- From the RPi container: `ros2 topic echo /tag_detections` should show data
  when rover markers are visible.

**Multiple `/bridge` nodes visible**
- See [`docs/ros2/troubleshooting.md`](../../../docs/ros2/troubleshooting.md).
