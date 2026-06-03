# driving_state.md — Full-Course Driving: Session Handoff

> Purpose: let the next person (human or AI agent) immediately understand the
> current state of autonomous driving — what works, what's untested, and what to
> do next. Read top-to-bottom, then jump to **Quick Start**.

**Last updated:** 2026-06-03.
**Script:** `ros2_ws/src/robot/robot/tests/scripts/venue_full_course_test.py`
**Branch:** `tests-todd`

---

## TL;DR — current status by segment

| Segment | Route | Status |
|---|---|---|
| SEG1 | Start → CP1 (lane 1 north, first half top-left U-turn) | ✅ Working reliably |
| SEG2 | CP1 → CP2 (complete U-turn, lane 2 south, bottom turn) | ✅ Working reliably |
| SEG3A | CP2 east → obstacle zone entry (avoidance OFF) | ✅ Working reliably |
| SEG3B | North through obstacle field (avoidance ON) | ⚠️ Sometimes works — obstacle avoidance is the weak link |
| SEG3C | Exit obstacle zone → 90° right turn east → CP3 | 🔴 Untested (lidar pose reset just added) |
| SEG4 | CP3 → lane 5 south → stop sign → manipulation station | 🔴 Untested end-to-end (depends on SEG3C landing correctly) |

---

## Course geometry

All coordinates are in the absolute course frame: x = east (mm), y = north (mm).
Origin is at the robot's start position (lane 1 south).

```
                    NORTH WALL  y ≈ 3965
  ┌─────────────────────────────────────────────────────┐
  │   CP1(305,3660)          CP3(1983,3660)    NE corner│
  │      ↑ SEG1/2 U-turn        ↑ SEG3C/4 U-turn  (2745,3965)
  │                                                     │
  │  x=0    x=610    x=1525    x=2440                  │
  │  Lane1  Lane2    ObsMid    Lane5                    │
  │                 [OBSTACLE FIELD]                    │
  │                                                     │
  │      CP2(1068,610)                                  │
  └─────────────────────────────────────────────────────┘
                    SOUTH WALL  y ≈ 0
  Start: (0, -160), heading north (90°)
```

Key constants (all in `venue_full_course_test.py`):

| Name | Value (mm) | Meaning |
|---|---|---|
| `Y_START` | -160 | Robot start y |
| `Y_BOT` | 610 | Bottom turn y (CP2 row) |
| `Y_TOP` | 3500 | Top turn y (CP1/CP3 row) |
| `BACK_RIGHT_CORNER_ABS` | (2745, 3965) | NE corner used for lidar pose reset |
| `OBS_ENTRY_OFFSET_MM` | 400 | Gap before avoidance starts after the bottom turn |
| `OBS_EXIT_OFFSET_MM` | 400 | Gap before avoidance stops before the top turn |

---

## Checkpoints (restart positions)

Place the robot physically here, set `START_SEGMENT = N`, and relaunch.
Odometry resets to (0, 0) at the robot's current location.

| CP | Absolute position | Heading | SEG starts here |
|---|---|---|---|
| Start | (0, -160) | 90° N | SEG1 |
| CP1 | (305, 3660) | 0° E | SEG2 |
| CP2 | (1068, 610) | 0° E | SEG3 |
| CP3 | (1983, 3660) | 0° E | SEG4 |

---

## Run configuration (top of script)

```python
START_SEGMENT = 1     # 1–4: which segment to begin from
AUTO_START    = True  # True = 3-s countdown; False = green-light trigger (BTN_1 override)
```

**When `AUTO_START = False`**, the camera pan stepper pans to **−30°** (slightly
left) before the IDLE loop starts, to face the traffic light. The pan happens
once at startup and stays there until driving begins.

---

## Quick Start

```bash
# SSH into Pi and enter the container
ssh mae162-s4-g7@192.168.8.152
docker exec -it docker-ros2_runtime-1 bash

# Source + run the full course
source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash
python3 /ros2_ws/src/robot/robot/tests/scripts/venue_full_course_test.py
```

To restart from a checkpoint (e.g. CP3):
1. Edit `START_SEGMENT = 4` at the top of the script
2. Place robot at CP3 facing east
3. Run the script

---

## SEG3B — Obstacle avoidance (the unreliable part)

The robot drives north through the obstacle field (~x=1525, y=1010→3100) with the
`_nav_follow_pp_path` pure-pursuit + APF avoidance planner active.

**Known failure modes:**
- Robot gets trapped between two close obstacles and oscillates
- Avoidance impulse carries the robot too far left/right, missing the exit corridor
- Odometry heading drifts significantly during avoidance maneuvers (the core
  motivation for the SEG3C lidar reset below)

**Tuning knobs** (`SEG3B_CFG` in the script):
```python
SEG3B_CFG = dict(speed=90, lookahead=100, spacing=400,
                 safe_dist=150, lane_width=400, avoidance_delay=250)
```
- Lower `speed` gives the planner more reaction time
- Smaller `lookahead` makes it more reactive to the path (less corner cutting)
- `safe_dist` is the repulsion bubble radius; `lane_width` limits lateral excursion

---

## SEG3C — Lidar pose reset (new, untested)

**Why:** after SEG3B the odometry heading is unreliable. Without correction the
90° right turn at the top and the eastward run to CP3 drift significantly.

**What it does** (runs at the SEG3B→SEG3C transition):

1. `robot.stop()` + 0.3 s settle
2. Calls `_lidar_wall_pose_reset(robot, ox, oy)`:
   - Gets current lidar point cloud in robot frame (`robot.get_obstacles()`, mm)
   - Uses current (noisy) heading to approximate-transform points to world frame
   - Filters for north-wall candidates: `y_world > 3465` and `x_world < 2895`
   - Fits a line `wy = m*wx + b` to wall points
   - **Heading correction:** `arctan(m)` gives the yaw error; corrected θ = θ_odom − arctan(m)
   - **Y correction:** true north wall is at y=3965; fitted intercept gives y offset
   - **X correction:** point on the wall closest to corner (2745, 3965), if within 500 mm
3. Calls `set_initial_theta(corrected_theta)` + `reset_odometry()`
4. Updates `ox, oy` to the corrected absolute position so all downstream `_op()`
   calls (exit path + SEG4) use the corrected origin
5. If detection fails (< 8 wall points), falls through and uses the odometry estimate

**Expected conditions at the reset point:**
- Robot at approximately (1525, 3100), heading ~90° ± drift
- North wall at y = 3965, ~865 mm ahead
- NE corner at (2745, 3965), ~1220 mm to the right — may or may not be in FOV

**What to check on first test:**
- Look at `[LIDAR]` log lines: how many wall points found, what heading correction was applied
- Confirm the fitted wall y matches 3965 ± a small tolerance
- Check whether the corner is detected or not (log says "Corner at ..." or "not resolved")

---

## SEG4 — Stop sign + manipulation station

- Stop-sign detection active throughout SEG4
- First detection → 3-second dwell (red LED), then resume
- Final waypoint: (2440, 305) — manipulation station

SEG4 has never been reached in a full run because SEG3C is untested. When testing
SEG4 in isolation (`START_SEGMENT = 4`), place the robot at CP3 (1983, 3660) facing east.

---

## Camera pan behavior

The camera pan stepper (`CAMPAN_STEPPER = STEPPER_3`) is only initialized by this
driving script when `AUTO_START = False`. It pans to **−30°** (left) so the
traffic light is in frame.

- The manipulation code (`turntable_pick_place_test.py`, `marshmallow_pick.py`)
  manages the campan independently — it will re-home/re-position as needed.
- On `AUTO_START = True` runs, the campan is **not touched** by this script.

---

## Open issues / next steps (priority order)

### 1. Test SEG3C lidar pose reset  ← first thing to do
Run from `START_SEGMENT = 3` to exercise the full SEG3 path, including the new
lidar reset. Watch the `[LIDAR]` and `[FSM]` log output carefully.

Key questions:
- Are there enough wall points (need ≥ 8)?
- Is the heading correction reasonable (expect < ±20° after a clean obstacle run)?
- Does the corrected exit path aim the robot at CP3?

If the wall is not detected: the lidar FOV or range filter may be cutting it.
Check `LIDAR_RANGE_MAX_MM` (6000 mm) and `LIDAR_FOV_DEG` (−180 to +180) in
`hardware_map.py` — both should be wide enough. The most likely failure is too
few wall points if the robot ends up far from the wall at the exit boundary.

### 2. SEG3B reliability
The obstacle avoidance field is the single biggest risk to a full course run.
Options if avoidance keeps failing:
- Run without obstacles (confirm the path is correct first)
- Tune `safe_dist`/`lane_width`/`speed` iteratively
- Add a recovery behavior (e.g., back up and re-approach) — not currently implemented

### 3. Validate full-course end-to-end
Once SEG3C is working, do a full run from `START_SEGMENT = 1`, `AUTO_START = False`
with the real traffic light. The stop-sign detection in SEG4 and the handoff to
the manipulation arm are the remaining untested integration points.

### 4. Heading at CP3 for manipulation
The manipulation arm assumes the robot is at the station facing south. The SEG4
path ends facing south (the last segment of the path runs south along lane 5).
Confirm the final heading is correct before the arm code is called.

---

## Key files

| File | Purpose |
|---|---|
| `venue_full_course_test.py` | Main driving FSM — edit `START_SEGMENT`/`AUTO_START` here |
| `_manipulator_config.py` | Stepper/servo constants shared with manipulation code |
| `turntable_pick_place_test.py` | Manipulation arm FSM (picks marshmallow at station) |
| `hardware_map.py` | All hardware constants (lidar, wheel, motor IDs, etc.) |
| `venue_waypoints.png` | Diagram of the course waypoints |
| `venue_avoidance_map.png` | Diagram of the obstacle avoidance region |

---

## Pure-pursuit + APF parameters reference

For all segments without avoidance (`obstacle_avoidance=False`):
- `max_angular_speed = 1.5` rad/s
- `goal_tolerance = 20` mm
- `alpha_Ld = 0.7` (lookahead scaling)

For SEG3B (avoidance ON):
- `max_angular_speed = 1.0` rad/s
- `obstacles_range = 550` mm
- `view_angle = 65°`
- `avoidance_delay = 250` (ticks before avoidance engages)
- `alpha_Ld = 1.3`

Speed/lookahead per segment:

| Segment | Speed (mm/s) | Lookahead (mm) |
|---|---|---|
| SEG1 | 140 | 300 |
| SEG2 | 120 | 300 |
| SEG3A | 130 | 300 |
| SEG3B | 90 | 100 |
| SEG3C | 120 | 300 |
| SEG4 | 120 | 300 |
