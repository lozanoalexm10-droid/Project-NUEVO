# Mechanical Subsystem Test Plan

Coordination spec for all subsystem test scripts.
Each entry has the exact terminal commands needed, the launch file to use, pass criteria, and safety notes.

**Shared config:** All manipulator tests import hardware assignments from
`scripts/_manipulator_config.py` — change channel/pin numbers there, not in individual test files.

---

## Step 1 — Start Docker

Run this once from the repository root (`Project-NUEVO/`) before any test session.

**On the Raspberry Pi (hardware robot):**
```bash
docker compose -f ros2_ws/docker/docker-compose.rpi.yml up -d --build --wait
```
Wait until the health check passes (up to ~4 minutes on first run, ~5s after that).
The bridge node starts automatically inside the container as the main process — you do not need to start it yourself.

**On a Mac/Windows dev machine (mock mode, no Arduino):**
```bash
docker compose -f ros2_ws/docker/docker-compose.vm.yml up -d --build --wait
```
Wait until the health check passes (~90s on first run). The container stays idle — you start nodes manually.

---

## Step 2 — Open terminals inside the container

Every node and test script runs **inside** the container. Open one shell tab per node you need.

To open a terminal inside the running container:
```bash
docker compose exec ros2_runtime bash
```

Run this command in as many separate terminal tabs as you need nodes.
You do not need to `cd` anywhere — the workspace is already sourced in the container's shell.

---

## Step 3 — Start the required nodes

**Bridge node**
- **RPi:** Already running. Skip this step.
- **VM (dev machine):** Open a terminal inside the container and run:
  ```bash
  ros2 run bridge bridge
  ```
  Leave this terminal open for the whole test session.

**Robot node** (required for every test)
Open a second terminal inside the container and run:
```bash
ros2 launch robot robot.launch.py
```
Leave this terminal open while running tests.

**Vision node** (only needed for M7 — detection test)
Open a third terminal inside the container and run:
```bash
ros2 run vision vision
```

---

## Step 4 — Run a test script

Open another terminal inside the container and run:
```bash
python3 /ros2_ws/src/robot/robot/tests/scripts/<script_name>.py
```

All output lines start with `[TEST]`. Grep logs for PASS or FAIL:
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

**Terminal setup:**
- Terminal 1: bridge (auto on RPi / `ros2 run bridge bridge` on VM)
- Terminal 2: `ros2 launch robot robot.launch.py`
- Terminal 3: `python3 .../turntable_range_test.py`

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

Same terminal setup as M1.

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

Same terminal setup as M1. Elbow must stay stowed throughout.

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

Same terminal setup as M1. Script pins shoulder at stow before moving elbow.

**Pass:** Full range reached without collision with body or cables.

---

### M5 — Gripper Open/Close

| Field       | Value |
|---|---|
| Script      | `scripts/gripper_open_close_test.py` |
| Launch file | `robot.launch.py` |
| Nodes       | bridge + robot |
| Status      | Ready — CH_3 confirmed |

Same terminal setup as M1.

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

Same terminal setup as M1.

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

**Terminal setup:**
- Terminal 1: bridge (auto on RPi / `ros2 run bridge bridge` on VM)
- Terminal 2: `ros2 launch robot robot.launch.py`
- Terminal 3: `ros2 run vision vision`
- Terminal 4: `python3 .../detection_vision_test.py`

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

Same terminal setup as M1.

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

To run any of these via the robot node, edit the one import line in `main.py`:

```python
# In robot/main.py — change this one line:
from robot.tests.scripts.lane_switch_obstacle_test import run  # noqa: F401
```

---

### D1 — Lane Switch Obstacle Test

| Field       | Value |
|---|---|
| File        | `tests/scripts/lane_switch_obstacle_test.py` |
| Launch file | `robot.launch.py` + `everything_but_robot.launch.py` (LiDAR + sensors) |
| Nodes       | bridge + robot + rplidar + sensors |

Tests LiDAR-based obstacle avoidance along a straight segment.
Press BTN_1 to start, BTN_2 to stop. Primary test for validating the LiDAR
lane-switch algorithm before committing it to the full venue route.

**Terminal setup:**
- Terminal 1: bridge (auto on RPi)
- Terminal 2: `ros2 launch robot everything_but_robot.launch.py`
- Terminal 3: `ros2 launch robot robot.launch.py`

---

### D2 — Venue No Obstacles

| Field       | Value |
|---|---|
| File        | `tests/scripts/venue_no_obstacles_test.py` |
| Launch file | `robot.launch.py` + `everything_but_robot.launch.py` |
| Nodes       | bridge + robot + rplidar + sensors |

Drives the full venue path with obstacle avoidance disabled.
Auto-starts 3 seconds after boot. Use to verify path geometry and GPS fusion
before enabling obstacle avoidance.

Same terminal setup as D1.

---

### D3 — Full Venue Route

| Field       | Value |
|---|---|
| File        | `programs/full_venue_route.py` |
| Launch file | `robot.launch.py` + `everything_but_robot.launch.py` |
| Nodes       | bridge + robot + rplidar + sensors |

Drives the full venue path in three segments: pre-obstacle (no avoidance),
obstacle field (avoidance on), post-obstacle (no avoidance).
Run after D1 and D2 both pass.

Same terminal setup as D1.

---

### D4 — Manipulator Demo

| Field       | Value |
|---|---|
| File        | `programs/manipulator_demo.py` |
| Launch file | `robot.launch.py` |
| Nodes       | bridge + robot |

Stub — manipulator sequence to be filled in as M1–M6 pass.

Same terminal setup as M1.

---

### D5 — Final Demo

| Field       | Value |
|---|---|
| File        | `programs/final_demo.py` |
| Launch file | `robot.launch.py` + `everything_but_robot.launch.py` |
| Nodes       | bridge + robot + rplidar + sensors + vision |

Full competition sequence. Run only after all D1–D4 and M1–M8 pass.

**Terminal setup:**
- Terminal 1: bridge (auto on RPi)
- Terminal 2: `ros2 launch robot everything_but_robot.launch.py`
- Terminal 3: `ros2 launch robot robot.launch.py`
- Terminal 4: `ros2 run vision vision`

---

## Subsystem test sequence (earlier tests)

| ID | Script | Nodes | Status |
|---|---|---|---|
| S1 | `servo_range_test.py` — generic servo channel sweep | bridge + robot | Ready |
| T1 | `stepper_basic_move_test.py` — basic stepper move/return | bridge + robot | Ready |

Same terminal setup as M1 for both.

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
