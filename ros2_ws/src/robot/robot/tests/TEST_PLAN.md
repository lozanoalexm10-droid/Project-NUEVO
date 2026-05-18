# Mechanical Subsystem Test Plan

All hardware constants live in two files — edit there, not in individual scripts:
    
    ros2_ws/src/robot/robot/hardware_map.py
    ros2_ws/src/robot/robot/tests/scripts/_manipulator_config.py

---

## How to start a session (every time)

### Terminal 1 — Start Docker (bridge opens automatically, keep this running)

From the repository root (`Project-NUEVO/`):

```bash
docker compose -f ros2_ws/docker/docker-compose.rpi.yml build
docker compose -f ros2_ws/docker/docker-compose.rpi.yml up -d
docker compose -f ros2_ws/docker/docker-compose.rpi.yml logs -f ros2_runtime
```

The container starts and immediately launches `ros2 launch bridge bridge.launch.py`.
This is the bridge node — it relays ROS2 topics to and from the Arduino over UART.
**Do not start the bridge manually.** It is already the container's main process.
The `logs` command keeps this terminal live so you can see what the container is doing.

> First build takes ~4 minutes. Subsequent starts take ~5 s.

---

### Terminal 2 (and every additional terminal) — Enter the container

```bash
docker compose -f ros2_ws/docker/docker-compose.rpi.yml exec ros2_runtime bash
```

**Always source ROS2 inside each new container shell before running anything:**

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

---

### Terminal 3 — Launch the robot node

In a container shell (Terminal 2 above):

```bash
ros2 launch robot robot.launch.py
```

This starts the robot node and runs whatever `run()` is imported in `main.py`.
The robot will NOT move until the FSM receives its trigger (button, timer, or vision event)
— except programs that auto-start (e.g. `venue_no_obstacles_test` starts after 3 s).

**To switch programs**, edit the one import line in `main.py` before launching:

```python
# /ros2_ws/src/robot/robot/main.py — change only this line:
from robot.tests.scripts.straight_line_test import run  # noqa: F401
```

---

### Optional — LiDAR + GPS node (required for any test that drives)

Open another container shell and run:

```bash
ros2 launch robot everything_but_robot.launch.py
```

Starts two nodes together:
- **rplidar** — LiDAR scan data for obstacle avoidance
- **robot_gps** — connects to the lab Jetson over TCP, receives ArUco tag detections,
  republishes them as `/tag_detections` for position fusion

Without this, `enable_gps()` and `set_tracked_tag_id()` will have no data.

---

### Optional — Vision node (required for detection, traffic light, and manipulator tests)

```bash
ros2 run vision vision
```

---

## Hardware Assignments

| Joint            | Type    | ID / Channel  | Config constant          | Status              |
|---|---|---|---|---|
| Turntable        | Stepper | STEPPER_1     | `TURNTABLE_STEPPER`      | Confirmed           |
| Camera pan       | Stepper | STEPPER_2     | `CAMPAN_STEPPER`         | Confirm microstep against firmware |
| Shoulder         | Servo   | CH_16         | `SHOULDER_CHANNEL`       | Confirmed           |
| Elbow            | Servo   | CH_15         | `ELBOW_CHANNEL`          | Confirmed           |
| Gripper          | Servo   | CH_14         | `GRIPPER_CHANNEL`        | Confirmed           |
| Heating wire     | DC PWM  | DC_M3 (TBC)   | `HEATING_WIRE_MOTOR_ID`  | Confirm with firmware |

Turntable angle convention: -90° = robot right (CW limit), 0° = forward, +180° = stow (CCW limit).
Step counts increase with angle. Confirm `MICROSTEP` in `_manipulator_config.py` matches firmware StepConfig.
Gear ratios: turntable 4:1 GT2 belt, camera pan TBD (confirm `CAMPAN_PULLEY_RATIO`).

---

## Test Sequence

Run phases in order. Do not advance to the next phase until the current one passes.

```
Phase 1 — Drive baseline:        D0 → D1 → D2 → D3
Phase 2 — Vision:                D_vision
Phase 3 — Servos (individual):   S1 → M3 → M4 → M5
Phase 4 — Coordinated arm:       M6
Phase 5 — Steppers:              T1 → M1 → M_campan → M2
Phase 6 — IK calibration:        M_ik  ← must complete before any demo
Phase 7 — Sensors:               M7
Phase 8 — Fire hazard last:       M8
Full demos (after all above):    D4 → D5
```

---

## Drive Tests

All drive tests use `main.py` to load the program. Set the import, then run:

```bash
ros2 launch robot robot.launch.py
```

### D0 — Straight Line (baseline pure pursuit)

| Field  | Value |
|---|---|
| File   | `tests/scripts/straight_line_test.py` |
| Nodes  | bridge (auto) + robot |
| Status | Ready — run first; confirms PID gains and pure pursuit work |

```python
# main.py
from robot.tests.scripts.straight_line_test import run  # noqa: F401
```

Drives 6 tiles straight ahead. No obstacles, no GPS.

**Pass:** Robot drives straight without spinning in place. Heading drift < 5° at end.

---

### D1 — Lane Switch Obstacle Avoidance

| Field  | Value |
|---|---|
| File   | `tests/scripts/lane_switch_obstacle_test.py` |
| Nodes  | bridge (auto) + everything_but_robot + robot |
| Status | Ready — run after D0 |

```python
# main.py
from robot.tests.scripts.lane_switch_obstacle_test import run  # noqa: F401
```

**Terminals needed:** 3
| Terminal | Command |
|---|---|
| T1 | Docker up (bridge auto-starts) |
| T2 | `ros2 launch robot everything_but_robot.launch.py` |
| T3 | `ros2 launch robot robot.launch.py` |

Press BTN_1 to start, BTN_2 to stop. Place an obstacle in the lane.

**Pass:** Robot detects obstacle and switches lanes without stopping. Returns to original lane after clearing.

---

### D2 — Venue No Obstacles

| Field  | Value |
|---|---|
| File   | `tests/scripts/venue_no_obstacles_test.py` |
| Nodes  | bridge (auto) + everything_but_robot + robot |
| Status | Ready — run after D1 |

```python
# main.py
from robot.tests.scripts.venue_no_obstacles_test import run  # noqa: F401
```

**Same terminals as D1.**

Drives the full venue path with obstacle avoidance disabled.
**Auto-starts 3 seconds after the robot node launches** — be ready.

**Pass:** Robot follows all waypoints and reaches the final position.

---

### D3 — Full Venue Route (3 segments)

| Field  | Value |
|---|---|
| File   | `programs/full_venue_route.py` |
| Nodes  | bridge (auto) + everything_but_robot + robot |
| Status | Ready — run after D1 and D2 pass |

```python
# main.py
from robot.programs.full_venue_route import run  # noqa: F401
```

**Same terminals as D1.**

Three-segment route: pre-obstacle (avoidance off) → obstacle field (avoidance on) → post-obstacle.

**Pass:** All three segments complete. Robot stops at final waypoint.

---

### D_vision — Traffic Light and Stop Sign

| Field  | Value |
|---|---|
| File   | `tests/scripts/vision_traffic_light_drive.py` |
| Nodes  | bridge (auto) + vision + robot |
| Status | Ready — requires vision node running |

```python
# main.py
from robot.tests.scripts.vision_traffic_light_drive import run  # noqa: F401
```

**Terminals needed:** 3
| Terminal | Command |
|---|---|
| T1 | Docker up (bridge auto-starts) |
| T2 | `ros2 run vision vision` |
| T3 | `ros2 launch robot robot.launch.py` |

Hold a red or green traffic light card in front of the camera. Then hold a stop sign.

**Pass:**
- Green light → drives forward, green LED on
- Red light → stops, red LED on
- Stop sign → stops immediately, red LED on (overrides green)
- Card removed → LEDs off within 3 s, robot stops

---

## Servo Tests (run with arm fully stowed and clear of obstructions)

For all servo and stepper tests the script is run directly — no main.py edit needed.

**Terminals needed:** 2
| Terminal | Command |
|---|---|
| T1 | `ros2 launch robot robot.launch.py` |
| T2 | `python3 /ros2_ws/src/robot/robot/tests/scripts/<script_name>.py` |

---

### S1 — Generic Servo Sweep

| Field  | Value |
|---|---|
| Script | `servo_range_test.py` |
| Nodes  | bridge (auto) + robot |
| Status | Ready |

Sweeps CH_16 (shoulder), CH_15 (elbow), CH_14 (gripper) — centers each to 90° in Phase 1, then sweeps in Phase 2. Raw sanity check before named joint tests.

**Pass:** Both channels reach all waypoints. No grinding or binding.

---

### M3 — Shoulder Range

| Field  | Value |
|---|---|
| Script | `shoulder_range_test.py` |
| Nodes  | bridge (auto) + robot |
| Status | Ready — confirm `SHOULDER_CHANNEL` in `_manipulator_config.py` first |

Sweeps shoulder through full safe arc (5°/step). **Keep elbow stowed throughout.**

**Pass:** Full safe range reached without binding, cable snag, or collision.

---

### M4 — Elbow Range

| Field  | Value |
|---|---|
| Script | `elbow_range_test.py` |
| Nodes  | bridge (auto) + robot |
| Status | Ready — confirm `ELBOW_CHANNEL` in `_manipulator_config.py` first |

Script holds shoulder at stow before sweeping elbow. **Do not run with elbow extended and shoulder raised.**

**Pass:** Full safe range reached without body or cable collision.

---

### M5 — Gripper Open / Close

| Field  | Value |
|---|---|
| Script | `gripper_open_close_test.py` |
| Nodes  | bridge (auto) + robot |
| Status | Ready — CH_14 confirmed |

Cycles gripper open/close 3× with incremental 5°/step moves to avoid stall.
If the jaw barely moves, increase the gap between `GRIPPER_OPEN_DEG` and `GRIPPER_CLOSE_DEG` in `_manipulator_config.py` by 10° at a time.

**Pass:** Jaw opens and closes fully each cycle. No grinding or gear slip.

---

### M6 — Arm Extend / Retract

| Field  | Value |
|---|---|
| Script | `arm_extend_retract_test.py` |
| Nodes  | bridge (auto) + robot |
| Status | Ready — run after M3 and M4 pass |

Coordinates shoulder and elbow: extend to reach pose → hold 2 s → retract.
Enforces elbow-first retract order to prevent body collision.

**Pass:** Full extend-hold-retract cycle without collision or servo stall.

---

## Stepper Tests

Same 2-terminal setup as servo tests above.

---

### T1 — Basic Stepper Move (sanity check)

| Field  | Value |
|---|---|
| Script | `stepper_basic_move_test.py` |
| Nodes  | bridge (auto) + robot |
| Status | Ready |

Moves STEPPER_1 forward 200 steps, back 200 steps. Lowest-level stepper sanity check.

**Pass:** Both moves complete before timeout. Stepper returns to start.

---

### M1 — Turntable Range Sweep

| Field  | Value |
|---|---|
| Script | `turntable_range_test.py` |
| Nodes  | bridge (auto) + robot |
| Status | Ready — run after T1 passes |

Sweeps turntable ±90° from center and returns. **Arm must be fully stowed.**
Valid travel: -90° (right) → 0° (forward) → +180° (stow, also CCW limit).

**Pass:** Both sweeps complete before timeout. No missed steps (no irregular clicking). Returns exactly to start.

---

### M_campan — Camera Pan Range Sweep

| Field  | Value |
|---|---|
| Script | `campan_range_test.py` |
| Nodes  | bridge (auto) + robot |
| Status | Ready — confirm `CAMPAN_MICROSTEP` and `CAMPAN_PULLEY_RATIO` in `_manipulator_config.py` first |

Sweeps camera pan stepper through all three scan positions (-60°, 0°, +60°) and returns to center.
Verify camera cable has enough slack for full ±60° travel before running.

**Pass:** All three position moves complete before timeout. Camera image visibly shifts between frames. Returns to forward-facing center.

---

### M2 — Turntable Home Angles

| Field  | Value |
|---|---|
| Script | `turntable_home_test.py` |
| Nodes  | bridge (auto) + robot |
| Status | Ready — run after M1 passes |

Moves turntable to known angles (45°, 90°, 135°) and returns to 0°.
Align the turntable to the forward mark before running — the script uses open-loop step counts from startup position.

**Pass:** Each move completes before timeout. Stepper returns to 0° without stalling or grinding.

---

### M_ik — IK Calibration (MUST COMPLETE BEFORE ANY DEMO)

| Field  | Value |
|---|---|
| Script | `arm_ik_calibration_test.py` |
| Nodes  | bridge (auto) + robot |
| Status | **Blocked — run after M3, M4, M6 pass** |

Step-by-step guided calibration. Records physical measurements and confirms servo offset values.
After running, populate these constants in `_manipulator_config.py`:

- `ARM_L1_MM`, `ARM_L2_MM` — measure upper arm and forearm lengths
- `ARM_SHOULDER_HEIGHT_MM`, `ARM_SHOULDER_OFFSET_MM` — measure from base plate
- `SHOULDER_SERVO_OFFSET`, `SHOULDER_SERVO_SIGN`
- `ELBOW_SERVO_OFFSET`, `ELBOW_SERVO_SIGN`

**Pass:** FK round-trip error < 5 mm at two known positions.

---

## Sensor / Subsystem Tests

---

### M7 — Vision Detection

| Field  | Value |
|---|---|
| Script | `detection_vision_test.py` |
| Nodes  | bridge (auto) + vision + robot |
| Status | Ready — set `MARSHMALLOW_CLASS` to match vision model output |

**Terminals needed:** 3
| Terminal | Command |
|---|---|
| T1 | Docker up (bridge auto-starts) |
| T2 | `ros2 run vision vision` |
| T3 | `python3 /ros2_ws/src/robot/robot/tests/scripts/detection_vision_test.py` |

Place a marshmallow in the detection zone (6–12 inch radius half-circle, forward of robot) before running.

**Pass:** Vision system active within timeout. Marshmallow detected above `MIN_CONFIDENCE`. No false positives from background objects.

---

### M8 — Heating Wire On / Off

| Field  | Value |
|---|---|
| Script | `heating_wire_test.py` |
| Nodes  | bridge (auto) + robot |
| Status | Ready — confirm `HEATING_WIRE_MOTOR_ID` with firmware team before running |

> !! FIRE / BURN HAZARD !! — **Run this last, after M1–M7 all pass.**
> The gripper must be closed around a marshmallow or heat-safe object. Have a fire extinguisher within reach.
> Start with `HEATING_WIRE_PWM_ON` at 64 or lower. Do not leave unattended.

**Terminals needed:** 2 (same as servo tests)

Powers wire at low PWM for a few seconds, then off. Verify temperature change visually.

**Pass:** Wire heats (faint glow or measurable warmth). Cools after shutoff. No sparks or damage.

---

## Full Demo Programs

### D4 — Manipulator Demo (no driving)

| Field  | Value |
|---|---|
| File   | `programs/manipulator_demo.py` |
| Nodes  | bridge (auto) + vision + robot |
| Status | Blocked — requires M_ik to pass and all TODOs in `_manipulator_config.py` resolved |

```python
# main.py
from robot.programs.manipulator_demo import run  # noqa: F401
```

**Terminals needed:** 3
| Terminal | Command |
|---|---|
| T1 | Docker up (bridge auto-starts) |
| T2 | `ros2 run vision vision` |
| T3 | `ros2 launch robot robot.launch.py` |

Park robot at the marshmallow station first. BTN_1 to start, BTN_2 to abort and restow.

State machine: `IDLE → ARM_HOME → SCANNING (3-frame camera pan) → RANGING (ultrasonic) → APPROACHING → PICKING → CARRYING → PLACING → ROASTING → RESTOWING → DONE`

**Note:** RANGING state requires `robot.get_ultrasonic_mm()` to be implemented (currently raises `NotImplementedError`). Until then, the demo falls through to RESTOWING after scanning.

---

### D5 — Final Demo (full competition sequence)

| Field  | Value |
|---|---|
| File   | `programs/final_demo.py` |
| Nodes  | bridge (auto) + everything_but_robot + vision + robot |
| Status | Blocked — run only after all D0–D4 and M1–M8 pass |

```python
# main.py
from robot.programs.final_demo import run  # noqa: F401
```

**Terminals needed:** 4
| Terminal | Command |
|---|---|
| T1 | Docker up (bridge auto-starts) |
| T2 | `ros2 launch robot everything_but_robot.launch.py` |
| T3 | `ros2 run vision vision` |
| T4 | `ros2 launch robot robot.launch.py` |

State machine:
1. **IDLE** — red LED on, waits for green traffic light (or BTN_1 override)
2. **Driving** — pre-obstacle → obstacle field (LiDAR avoidance) → post-obstacle
3. **AT_STOP_SIGN** — stops; after dwell, waits for stop sign to disappear from vision or BTN_1 to advance
4. **WAIT_ENCLOSURE** — timer for team to remove competition cover
5. **ARM sequence** — SCANNING → RANGING → APPROACHING → PICKING → CARRYING → PLACING → ROASTING → RESTOWING

BTN_2 = emergency stop and shutdown at any point.

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
    # or:
    print("[TEST] FAIL: <reason>")
```

Save output to a log:

```bash
python3 /ros2_ws/src/robot/robot/tests/scripts/my_test.py | tee /runtime_output/my_test.log
grep '\[TEST\] PASS\|\[TEST\] FAIL' /runtime_output/my_test.log
```
