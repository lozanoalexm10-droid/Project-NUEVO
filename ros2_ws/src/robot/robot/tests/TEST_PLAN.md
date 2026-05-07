# Mechanical Subsystem Test Plan

Coordination spec for all subsystem test scripts.
Each entry has the exact terminal commands needed, the launch file to use, pass criteria, and safety notes.

**Shared config:** All manipulator tests import hardware assignments from
`scripts/_manipulator_config.py` — change channel/pin numbers there, not in individual test files.

---

## Step 1 — Build and start Docker (Terminal 1)

Run from the repository root (`Project-NUEVO/`). Keep this terminal open — it shows live container logs.

```bash
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml
docker compose -f $COMPOSE build
docker compose -f $COMPOSE up -d
docker compose -f $COMPOSE logs -f ros2_runtime
```

> First build takes ~4 minutes. Subsequent starts take ~5 s.
> The bridge node starts automatically as the container's main process — do not start it manually.

---

## Step 2 — Enter the container (each new terminal)

Open a **new terminal tab** for every node or script you need to run. In each one:

```bash
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml
docker compose -f $COMPOSE exec ros2_runtime bash
```

Then **always source ROS2 inside each container shell before running anything**:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

---

## Step 3 — Start the required nodes

Open a new container terminal (Step 2) for each node below.

**Bridge node — already running. Do not start it.**
Docker started `bridge.launch.py` as its main process when you ran `docker compose up`.
It translates ROS2 topics ↔ Arduino serial commands and stays alive the whole session.

**Robot node** — starts your program (`main.py`)
```bash
ros2 launch robot robot.launch.py
```
> `robot.launch.py` starts only the robot node — not the bridge.
> It runs `main.py` immediately. Change the import in `main.py` to switch programs before launching.
> The robot will NOT move until the FSM receives a trigger (BTN_1, timer, etc.) —
> EXCEPT programs that auto-start (e.g. `venue_no_obstacles_test` starts after 3 s).

**LiDAR + GPS** — required for any test that drives or uses position fusion (D1, D2, D3, D5)
```bash
ros2 launch robot everything_but_robot.launch.py
```
> Starts two nodes together:
> - **rplidar** — LiDAR scan data for obstacle avoidance
> - **robot_gps** — connects to the lab Jetson over TCP, receives ArUco tag detections,
>   and republishes them as `/tag_detections` for position fusion
>
> Without robot_gps, `enable_gps()` / `set_tracked_tag_id()` will have no data to fuse.
> The Jetson must be reachable on the lab network for tag detections to arrive.

**Vision node** — required for detection test (M7) and manipulator demo (D4, D5)
```bash
ros2 run vision vision
```

---

## Step 4 — Run a test script

Open another container terminal (Step 2), then:

```bash
python3 /ros2_ws/src/robot/robot/tests/scripts/<script_name>.py
```

To save output and grep for results:
```bash
python3 /ros2_ws/src/robot/robot/tests/scripts/<script_name>.py | tee logs/<script_name>.log
grep '\[TEST\] PASS\|\[TEST\] FAIL' logs/<script_name>.log
```

---

## Manipulator Hardware Assignments

| Joint          | Type    | ID / Channel | Config constant         | Status   |
|---|---|---|---|---|
| Turntable      | Stepper | STEPPER_1    | `TURNTABLE_STEPPER`     | Confirmed |
| Shoulder       | Servo   | CH_1 (TBC)   | `SHOULDER_CHANNEL`      | Needs wiring check |
| Elbow          | Servo   | CH_2 (TBC)   | `ELBOW_CHANNEL`         | Needs wiring check |
| Gripper        | Servo   | CH_3         | `GRIPPER_CHANNEL`       | Confirmed |
| Heating wire   | DC PWM  | DC_M3 (TBC)  | `HEATING_WIRE_MOTOR_ID` | Needs firmware confirmation |
| Turntable home | Limit   | LIM_1        | `TURNTABLE_HOME_LIMIT`  | Needs wiring check |

Confirm `MICROSTEP` in `_manipulator_config.py` matches the firmware StepConfig before running any turntable test.
Gear ratio: 4:1 GT2 belt — `STEPS_PER_TURNTABLE_DEG` is derived automatically.

---

## Manipulator Tests

### M1 — Turntable Range Sweep

| Field       | Value |
|---|---|
| Script      | `scripts/turntable_range_test.py` |
| Launch file | `robot.launch.py` (bridge already running on RPi) |
| Nodes       | bridge + robot |
| Status      | Ready |

**Terminals needed:** 2 (robot node + script)
| Terminal | Command |
|---|---|
| T1 | `ros2 launch robot robot.launch.py` |
| T2 | `python3 /ros2_ws/src/robot/robot/tests/scripts/turntable_range_test.py` |

Sweeps turntable ±90° from center and returns. Arm must be fully stowed.

**Pass:** Both sweeps complete before timeout. No missed steps (no irregular clicking).

---

### M2 — Turntable Home

| Field       | Value |
|---|---|
| Script      | `scripts/turntable_home_test.py` |
| Launch file | `robot.launch.py` |
| Nodes       | bridge + robot |
| Status      | Ready |

**Terminals needed:** 2 (robot node + script)
| Terminal | Command |
|---|---|
| T1 | `ros2 launch robot robot.launch.py` |
| T2 | `python3 /ros2_ws/src/robot/robot/tests/scripts/turntable_home_test.py` |

Homes turntable against LIM_1 three times, checks limit switch triggers each run.

**Pass:** Limit switch triggers on all three runs. No grinding.

---

### M3 — Shoulder Range

| Field       | Value |
|---|---|
| Script      | `scripts/shoulder_range_test.py` |
| Launch file | `robot.launch.py` |
| Nodes       | bridge + robot |
| Status      | Ready — confirm `SHOULDER_CHANNEL` in `_manipulator_config.py` first |

**Terminals needed:** 2 (robot node + script). Elbow must stay stowed throughout.
| Terminal | Command |
|---|---|
| T1 | `ros2 launch robot robot.launch.py` |
| T2 | `python3 /ros2_ws/src/robot/robot/tests/scripts/shoulder_range_test.py` |

Sweeps shoulder through full safe arc incrementally (5°/step).

**Pass:** Full range reached without binding, cable snag, or collision.

---

### M4 — Elbow Range

| Field       | Value |
|---|---|
| Script      | `scripts/elbow_range_test.py` |
| Launch file | `robot.launch.py` |
| Nodes       | bridge + robot |
| Status      | Ready — confirm `ELBOW_CHANNEL` in `_manipulator_config.py` first |

**Terminals needed:** 2 (robot node + script). Script pins shoulder at stow before moving elbow.
| Terminal | Command |
|---|---|
| T1 | `ros2 launch robot robot.launch.py` |
| T2 | `python3 /ros2_ws/src/robot/robot/tests/scripts/elbow_range_test.py` |

**Pass:** Full range reached without collision with body or cables.

---

### M5 — Gripper Open/Close

| Field       | Value |
|---|---|
| Script      | `scripts/gripper_open_close_test.py` |
| Launch file | `robot.launch.py` |
| Nodes       | bridge + robot |
| Status      | Ready — CH_3 confirmed |

**Terminals needed:** 2 (robot node + script)
| Terminal | Command |
|---|---|
| T1 | `ros2 launch robot robot.launch.py` |
| T2 | `python3 /ros2_ws/src/robot/robot/tests/scripts/gripper_open_close_test.py` |

Cycles gripper open/close 3 times using incremental 5° steps to avoid stall.
If the jaw barely moves, widen the gap between `OPEN_DEG` and `CLOSE_DEG` in `_manipulator_config.py` by 10° at a time.

**Pass:** Jaw opens and closes fully each cycle. No grinding or gear slip.

---

### M6 — Arm Extend / Retract

| Field       | Value |
|---|---|
| Script      | `scripts/arm_extend_retract_test.py` |
| Launch file | `robot.launch.py` |
| Nodes       | bridge + robot |
| Status      | Ready — run after M3 and M4 pass |

**Terminals needed:** 2 (robot node + script)
| Terminal | Command |
|---|---|
| T1 | `ros2 launch robot robot.launch.py` |
| T2 | `python3 /ros2_ws/src/robot/robot/tests/scripts/arm_extend_retract_test.py` |

Coordinates shoulder and elbow: extend to reach pose, hold 2s, retract.
Enforces elbow-first retract order to prevent body collision.

**Pass:** Full extend-hold-retract cycle without collision.

---

### M7 — Vision Detection

| Field       | Value |
|---|---|
| Script      | `scripts/detection_vision_test.py` |
| Launch file | `robot.launch.py` + vision node separately |
| Nodes       | bridge + robot + vision |
| Status      | Ready — set `MARSHMALLOW_CLASS` to match vision model output |

**Terminals needed:** 3 (robot node + vision node + script)
| Terminal | Command |
|---|---|
| T1 | `ros2 launch robot robot.launch.py` |
| T2 | `ros2 run vision vision` |
| T3 | `python3 /ros2_ws/src/robot/robot/tests/scripts/detection_vision_test.py` |

Place a marshmallow in the detection zone (6–12 inch radius half-circle, forward of robot) before running.

**Pass:** Vision system active. Marshmallow detected above `MIN_CONFIDENCE`. No false positives from non-marshmallow objects.

---

### M8 — Heating Wire On/Off

| Field       | Value |
|---|---|
| Script      | `scripts/heating_wire_test.py` |
| Launch file | `robot.launch.py` |
| Nodes       | bridge + robot |
| Status      | Ready — confirm `HEATING_WIRE_MOTOR_ID` with firmware team before running |

**!! FIRE / BURN HAZARD !!** Run only after M1–M6 all pass and the gripper can safely hold a target object.

**Terminals needed:** 2 (robot node + script)
| Terminal | Command |
|---|---|
| T1 | `ros2 launch robot robot.launch.py` |
| T2 | `python3 /ros2_ws/src/robot/robot/tests/scripts/heating_wire_test.py` |

Powers heating wire at low PWM for 3s, then off.

**Pass:** Wire heats visibly. Cools after shutoff. No sparks or damage.

---

## Sequence

```
M1 → M2 → M3 → M4 → M5 → M6 → M7 → M8
```

Do not run M8 until M1–M6 all pass.

---

## Drive Programs (`robot/programs/`) and Drive Tests

To run any of these, edit the one import line in `main.py`:

```python
# robot/main.py — change this one line:
from robot.programs.final_demo import run  # noqa: F401
```

Then restart the robot node. All programs share the `run(robot: Robot)` entry point.

---

### D1 — Lane Switch Obstacle Test

| Field       | Value |
|---|---|
| File        | `tests/scripts/lane_switch_obstacle_test.py` |
| Nodes       | bridge (auto) + rplidar + sensors + robot |

Tests LiDAR-based obstacle avoidance along a straight segment.
Press BTN_1 to start, BTN_2 to stop.

**Terminals needed:** 2
| Terminal | Command |
|---|---|
| T1 | `ros2 launch robot everything_but_robot.launch.py` |
| T2 | `ros2 launch robot robot.launch.py` |

> Select in `main.py`: `from robot.tests.scripts.lane_switch_obstacle_test import run`

---

### D2 — Venue No Obstacles

| Field       | Value |
|---|---|
| File        | `tests/scripts/venue_no_obstacles_test.py` |
| Nodes       | bridge (auto) + rplidar + sensors + robot |

Drives the full venue path with obstacle avoidance disabled.
**Auto-starts 3 seconds after robot node launches** — be ready. Use to verify path geometry and GPS fusion before enabling obstacle avoidance.

**Terminals needed:** 2 (same as D1)
| Terminal | Command |
|---|---|
| T1 | `ros2 launch robot everything_but_robot.launch.py` |
| T2 | `ros2 launch robot robot.launch.py` |

> Select in `main.py`: `from robot.tests.scripts.venue_no_obstacles_test import run`

---

### D3 — Full Venue Route

| Field       | Value |
|---|---|
| File        | `programs/full_venue_route.py` |
| Nodes       | bridge (auto) + rplidar + sensors + robot |

Three-segment venue drive. Run after D1 and D2 both pass.

**Terminals needed:** 2 (same as D1)
| Terminal | Command |
|---|---|
| T1 | `ros2 launch robot everything_but_robot.launch.py` |
| T2 | `ros2 launch robot robot.launch.py` |

> Select in `main.py`: `from robot.programs.full_venue_route import run`

---

### D4 — Manipulator Demo

| Field       | Value |
|---|---|
| File        | `programs/manipulator_demo.py` |
| Nodes       | bridge (auto) + robot + vision |

Full manipulator sequence — no driving. Park robot at station first.
BTN_1 to start, BTN_2 to abort and restow.

**Terminals needed:** 2
| Terminal | Command |
|---|---|
| T1 | `ros2 run vision vision` |
| T2 | `ros2 launch robot robot.launch.py` |

> Select in `main.py`: `from robot.programs.manipulator_demo import run`

---

### D5 — Final Demo

| Field       | Value |
|---|---|
| File        | `programs/final_demo.py` |
| Nodes       | bridge (auto) + rplidar + sensors + robot + vision |

Full competition sequence. Run only after all D1–D4 and M1–M8 pass.
BTN_1 on green light, BTN_2 emergency stop during nav.

**Terminals needed:** 3
| Terminal | Command |
|---|---|
| T1 | `ros2 launch robot everything_but_robot.launch.py` |
| T2 | `ros2 run vision vision` |
| T3 | `ros2 launch robot robot.launch.py` |

> Select in `main.py`: `from robot.programs.final_demo import run`

---

## Subsystem test sequence (earlier tests)

| ID | Script | Nodes | Status |
|---|---|---|---|
| S1 | `servo_range_test.py` — generic servo channel sweep | bridge + robot | Ready |
| T1 | `stepper_basic_move_test.py` — basic stepper move/return | bridge + robot | Ready |

**Terminals needed:** 2 (robot node + script)
| Terminal | Command |
|---|---|
| T1 | `ros2 launch robot robot.launch.py` |
| T2 | `python3 /ros2_ws/src/robot/robot/tests/scripts/<script_name>.py` |

---

## Script Template

```python
from __future__ import annotations
import time
from robot.robot import FirmwareState, Robot
from _manipulator_config import ...  # import what you need

def run(robot: Robot) -> None:
    print("[TEST] my_subsystem_test")
    robot.set_state(FirmwareState.RUNNING)

    # setup → action → verify

    print("[TEST] PASS")
    # or
    print("[TEST] FAIL: <reason>")
```

All output lines use `[TEST]` prefix so logs are easy to grep.
