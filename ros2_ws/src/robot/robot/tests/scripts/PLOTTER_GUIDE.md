# Navigation Plotter — Usage Guide

Two scripts work together to capture and visualize a driving run:

| Script | Runs on | Purpose |
|---|---|---|
| `nav_data_recorder.py` | Pi (inside Docker) | Records live ROS data to a JSON file |
| `plot_nav_results.py` | Mac (no ROS needed) | Reads the JSON and generates PNG figures |

---

## Step 1 — Record a run on the Pi

Open a **5th terminal** (alongside your normal 4-terminal startup). Start the recorder **before** launching the robot node, then do your run normally.

```bash
# Enter the container
docker compose -f ros2_ws/docker/docker-compose.rpi.yml exec ros2_runtime bash

# Source ROS2
source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash

# Start recording (default output: /runtime_output/nav_run_YYYYMMDD_HHMMSS.json)
python3 /ros2_ws/src/robot/robot/tests/scripts/nav_data_recorder.py
```

To specify a custom output path:
```bash
python3 nav_data_recorder.py --out /runtime_output/my_run.json
```

**Stop recording:** press `Ctrl+C`. The JSON file is written at that point.

What gets recorded:

| Data | Source topic | Rate |
|---|---|---|
| Odometry (x, y, θ) | `/sensor_kinematics` | 5 Hz |
| GPS-fused pose (x, y, θ, gps_active) | `/fused_pose` | 5 Hz |
| Lidar point cloud (world frame) | `/scan` | 1 Hz (one snapshot/s) |
| Vision detections (class, confidence, attributes) | `/vision/detections` | every event |

The recorder is **read-only** — it never commands the robot. Run it alongside any driving script without interference.

---

## Step 2 — Copy the JSON to your Mac

```bash
scp mae162-s4-g7@192.168.8.152:/runtime_output/nav_run_*.json ./
```

Or a specific file:
```bash
scp mae162-s4-g7@192.168.8.152:/runtime_output/nav_run_20260603_142000.json ./
```

---

## Step 3 — Install dependencies (first time only)

```bash
pip install matplotlib numpy
```

---

## Step 4 — Generate figures

All commands run from the `tests/scripts/` directory (or use full paths).

### FSM state machine diagram (no run data needed)

```bash
python3 plot_nav_results.py --fsm
```

Generates `fig_fsm_diagram.png` — a diagram of the full-course FSM with all states, sub-phases, transitions, and start modes. This is already pre-generated at the repo root as `fig_fsm_diagram.png`.

### All figures from a recorded run

```bash
python3 plot_nav_results.py nav_run_20260603_142000.json
```

### All figures including the FSM diagram

```bash
python3 plot_nav_results.py nav_run_20260603_142000.json --fsm
```

### Write figures to a specific folder

```bash
python3 plot_nav_results.py nav_run_20260603_142000.json --fsm --out-dir ./slides/
```

---

## Output figures

| File | Contents | Needs run data? |
|---|---|---|
| `fig_fsm_diagram.png` | State machine: IDLE → SEG1 → SEG2 → SEG3 (3A/3B/3C) → SEG4 → DONE | No |
| `fig_trajectory_map.png` | Venue grid + planned path (yellow) + odometry (blue) + GPS-fused (green dashed) + lidar obstacle cloud (red) + checkpoints | Yes |
| `fig_heading_fusion.png` | Heading over time (odom vs fused) + GPS correction magnitude with shaded GPS-active windows | Yes |
| `fig_vision_timeline.png` | Detection confidence scatter over time, color-coded by class (green/red traffic light, stop sign, etc.) | Yes |

All figures use the dark NUEVO color scheme and save at 180 dpi.

---

## Tips

- **Run the recorder before starting the robot node** so it catches the very first odometry reset and all detections from the start.
- If the vision node is not running with `debug_save_enabled:=true`, vision detections still appear in the JSON (the recorder reads `/vision/detections` directly — independent of the debug image saver).
- The lidar snapshots are projected to world frame using the latest fused pose. If GPS is not active, the projection uses odometry, which drifts in the obstacle zone — that's expected and actually visible in the trajectory figure.
- For the cleanest trajectory figure, run `START_SEGMENT = 1` (full course from start) so the origin is at (0, 0) matching the plotter's course geometry.
- The JSON files can be large (a full run is ~1–3 MB). Copy them off the Pi promptly since `/runtime_output` is on a tmpfs and does not survive a reboot.
