# grip_marshmallow.md — Marshmallow Pick: Session Handoff

> Purpose: let the next Claude agent (and you) resume the marshmallow-pick work
> instantly without re-deriving everything. Read this top-to-bottom, then jump to
> **Quick Start**.

**Last session date:** 2026-06-01. **Status:** arm successfully grabs + lifts a
marshmallow off a red cup. **One open issue:** the gripper does not *stay* closed
after the script exits (see Open Issues #1).

---

## TL;DR — what works

A standalone script **`ros2_ws/src/robot/robot/tests/scripts/marshmallow_pick.py`**
physically moves the arm to pick a marshmallow off a red cup. The reliable,
proven command (run inside the Pi's ROS container) is:

```bash
python3 /ros2_ws/src/robot/robot/tests/scripts/marshmallow_pick.py \
    --turntable 0 --shoulder 90 --elbow 160 --grab
```

This: rotates the turntable to 0° (forward), drops the arm into the grab pose
(**shoulder 90°, elbow 160°** — empirically the jaws straddle the marshmallow on
the cup), opens the gripper, closes it (grab), and lifts to carry pose.

The grab pose was found **empirically by sweeping**, NOT by inverse kinematics —
the arm geometry in `_manipulator_config.py` is still `TODO: measure` placeholder,
so IK is intentionally not used. See **Calibration caveat**.

---

## System / environment (as of last session)

| Thing | Value |
|---|---|
| Pi host | `mae162-s4-g7@192.168.8.152` (SSH key already authorized from this Mac) |
| ROS container | `docker-ros2_runtime-1` (compose: `ros2_ws/docker/docker-compose.rpi.yml`) |
| Container source mount | `~/Project-NUEVO/ros2_ws/src` → `/ros2_ws/src` (**read-only**) |
| Build dir | volume; src is NOT symlink-installed → **must `colcon build` after editing `/ros2_ws/src`** (but scp'ing the script + running it via `python3` runs the src copy directly, no build needed) |
| Pi repo branch | `tests-todd` (NOT `main`) |
| Camera | Raspberry Pi camera via `pi-camera-feed.service` → V4L2 loopback `/dev/video10` (1280×720 or 640×480). NOT the Jetson RealSense. |
| Ultrasonic | SparkFun Qwiic TCT40 on RPi I2C bus 1, addr `0x2F`, node `qwiic_ultrasonic`, topic `/ultrasonic_range` (sensor_msgs/Range, **meters**), ~10–20 Hz |
| Bridge | `/bridge` node; firmware UART on `/dev/ttyAMA0`; firmware FSM via `/set_firmware_state` |

**Heartbeat / safety:** firmware `HEARTBEAT_TIMEOUT_MS = 500` (`firmware/arduino/src/config.h`)
— **actuators are disabled if the host heartbeat stops, and on IDLE/ESTOP/ERROR.**
This is central to Open Issue #1.

---

## Quick Start (next session)

1. **Reconnect / confirm the Pi is up:**
   ```bash
   ssh mae162-s4-g7@192.168.8.152 'uptime; docker ps --format "{{.Names}}\t{{.Status}}"'
   ```
   If SSH key fails: `ssh-copy-id mae162-s4-g7@192.168.8.152` (password: `password`).

2. **Make sure exactly ONE of each helper node is running** (duplicates are the
   #1 source of flakiness — see Open Issue #2). Check + clean:
   ```bash
   ssh mae162-s4-g7@192.168.8.152 'docker exec docker-ros2_runtime-1 pgrep -af "lib/sensors/qwiic_ultrasonic"; docker exec docker-ros2_runtime-1 pgrep -af "lib/vision/vision_node"'
   ```
   If more than one of either, kill all and restart one (commands below).

3. **Confirm firmware reaches RUNNING** (under load the bridge's 2 s inner
   handshake can report failure even when it succeeds — re-check state):
   ```bash
   ssh mae162-s4-g7@192.168.8.152 'docker exec docker-ros2_runtime-1 bash -lc "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState \"{target_state: 2, timeout_sec: 8.0}\""'
   ```

4. **Deploy the latest script** (it lives on this Mac AND the Pi; keep them in sync):
   ```bash
   scp ros2_ws/src/robot/robot/tests/scripts/marshmallow_pick.py \
       mae162-s4-g7@192.168.8.152:/home/mae162-s4-g7/Project-NUEVO/ros2_ws/src/robot/robot/tests/scripts/marshmallow_pick.py
   ```

5. **Run the grab:**
   ```bash
   ssh mae162-s4-g7@192.168.8.152 'docker exec docker-ros2_runtime-1 bash -lc "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && python3 /ros2_ws/src/robot/robot/tests/scripts/marshmallow_pick.py --turntable 0 --shoulder 90 --elbow 160 --grab"'
   ```

---

## The pick sequence (how it works)

1. Ensure firmware RUNNING (skips if already; tolerates the slow-handshake false-negative).
2. `enable_vision()` + `enable_ultrasonic()`; wait for active (10 s each; ultrasonic skipped in `--grab`).
3. Enable turntable stepper + shoulder/elbow/gripper servos; settle to stow.
4. **Target:** either `--turntable <deg>` (manual) or vision (marshmallow paired
   with a red_cup within 10° bearing; falls back to the cup itself if the
   marshmallow detector misses — it often does, see Open Issue #3).
5. Rotate turntable (`step_move ABSOLUTE`, formula `steps = turntable_deg_to_steps(TURNTABLE_HOME_OFFSET_DEG - target_deg)`).
6. Move shoulder/elbow to the search pose (`--shoulder`/`--elbow`).
7. `--grab`: open gripper → close to `--grip` (default 45°) → lift to carry → **exit**.
   Otherwise: ultrasonic-fed descent loop (step elbow toward `--elbow-min`, stop when
   `get_ultrasonic_mm() < --pick-threshold`).
8. `finally`: retract to carry + recenter turntable to 0°. In `--grab` it
   **skips `robot.shutdown()`** (which would set IDLE and release the gripper).

### Key empirical facts
- **Grab pose: turntable 0°, shoulder 90°, elbow 160°.** At this pose the
  ultrasonic reads a steady ~123 mm (its beam clears the marshmallow to the cup
  rim/floor behind) — so the ultrasonic is a RED HERRING here; the jaws are
  already around the marshmallow. Confirmed by the user: "you got the
  marshmallow, you just had to close the gripper."
- The elbow approach direction is "increase elbow toward 160" (the safe max).
  At shoulder 90, elbow 90→160 the sensor reading drops monotonically
  (160°→119mm, 157°→130mm, 154°→135mm). At elbow 90 NO shoulder angle aims at
  the cup (beam sails over to the wall ~1100 mm).

---

## CLI flags (script already supports these)

| Flag | Meaning |
|---|---|
| `--turntable <deg>` | Manual turntable angle; skips vision targeting (and the vision-active wait). |
| `--shoulder <deg>` | Search-pose shoulder angle (grab pose: **90**). |
| `--elbow <deg>` | Search-pose / start elbow angle (grab pose: **160**). |
| `--grab` | Go to the pose, open→close→lift, NO ultrasonic descent. Skips `shutdown()`. |
| `--grip <deg>` | Gripper close angle (default `GRIPPER_GRAB_DEG`=45; higher = tighter). |
| `--probe [--probe-s N]` | Hold the pose and stream ultrasonic for N s; no descent/grab. **Aiming tool.** |
| `--sweep shoulder\|elbow [--sweep-from A --sweep-to B]` | Step one joint across a range, stream ultrasonic, report the angle with the closest reading. **The tool that found the grab pose.** |
| `--pick-threshold <mm>` | Ultrasonic trigger distance for the descent loop (default 50). |
| `--elbow-min <deg>` | Descent floor for the elbow (default safe min 20). |
| `--no-grab` | Full descent but don't close (dry run). |

Servo channels (`_manipulator_config.py`): shoulder=CH_16, elbow=CH_15,
gripper=CH_14. Gripper: open=0°, grab=45°, full close=120°. Safe ranges:
shoulder [30,150], elbow [20,160]. Turntable: STEPPER_2, range [-90,180]°,
home offset -180°.

---

## Open Issues / TODO (most important first)

### 1. Gripper does NOT stay closed after the script exits  ← UNRESOLVED
**Symptom:** the grab closes and lifts fine, but the gripper reopens / drops the
marshmallow shortly after the script finishes. Happened with `--grip 45` twice.

**What we tried:** in `--grab` mode the script already SKIPS `robot.shutdown()`
(which sets firmware IDLE → would release). Did not fix it.

**Leading hypothesis (untested):** firmware `HEARTBEAT_TIMEOUT_MS = 500` disables
actuators when the *host heartbeat* stops. The bridge sends heartbeats
(`serial_manager.py: _heartbeat_loop`), so this *shouldn't* trip on script exit —
**this needs confirming.** Alternative hypotheses: (a) 45° is too loose and the
springy marshmallow forces the jaw open → try `--grip 90` or higher; (b) the
servo relaxes without periodic re-commands.

**Recommended next step:** add a **hold loop** to `--grab`: after the grab, keep
the process alive and re-issue `set_servo(GRIPPER_CHANNEL, grip)` every ~0.5 s for
a `--hold-s` duration (run it backgrounded). Observe whether the grip holds while
the process is alive (it held fine during the 14 s `--probe`, which is the clue
that *process-alive = grip-holds*). Also try a tighter `--grip`. Was mid-implementation when the session ended.

### 2. Duplicate helper nodes (flakiness)
Multiple `qwiic_ultrasonic` and `vision_node` processes accumulate across runs
(two people SSH'd, repeated launches). Duplicates fight over the I2C device / the
camera and the topic stops publishing → "ultrasonic not active" / frozen stream.
**Fix:** kill all, start one.
```bash
# Ultrasonic: kill all, start one
ssh mae162-s4-g7@192.168.8.152 'docker exec docker-ros2_runtime-1 pkill -9 -f "lib/sensors/qwiic_ultrasonic"; sleep 3; docker exec -d docker-ros2_runtime-1 bash -lc "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 run sensors qwiic_ultrasonic > /tmp/us.log 2>&1"'
```

### 3. Marshmallow vision detector is unreliable
The rule-based `detect_marshmallow` (HSV white-blob) fires on the **wall**
(`marshmallow 0.66–0.91` on beige walls) and **misses the real marshmallow when
it's centred on a cup** (its centroid becomes the cup). That's why the script
pairs marshmallow↔red_cup within 10° and falls back to targeting the red_cup.
For the proven `--turntable 0` flow this doesn't matter. A real fix would be
cup-pairing in the vision node or a tighter/size-bounded white mask.

### 4. Arm geometry uncalibrated → IK disabled
`ARM_GEOMETRY` in `_manipulator_config.py` is placeholder (`L1=150, L2=130`,
servo offsets, etc. all `TODO: measure`). So the script uses **preset poses +
empirical sweeps**, not `inverse_kinematics()`. To make picking general (any cup
position, ultrasonic-driven descent), measure the arm and calibrate via
`arm_ik_calibration_test.py`. Until then, re-sweep when the cup moves.

### 5. Turntable "no motion" timeout is benign
`step_move(blocking=True)` waits for motion to *start*; a zero-step / already-at-
target move "times out". The script treats this as non-fatal (warns, continues).

### 6. Pi stability
The Pi rebooted several times during the session and ran at load avg ~5. SSH and
the camera stream drop when it reboots. On reboot, firmware starts IDLE (servos
limp) and `pi-camera-feed` auto-starts.

---

## "Start with the arm vertical" (user request)

The user wants to start each session with the gripper **vertical, straight up over
the turntable**. Notes:
- When firmware is **RUNNING**, servos hold their last commanded PWM → the arm is
  "stuck" and can't be moved by hand. To free it for hand-positioning, set
  firmware **IDLE** (releases all actuators — arm goes limp, **support it so it
  doesn't drop**, and the gripper releases any held marshmallow):
  ```bash
  ssh mae162-s4-g7@192.168.8.152 'docker exec docker-ros2_runtime-1 bash -lc "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState \"{target_state: 1, timeout_sec: 8.0}\""'
  ```
- On power-up the firmware boots IDLE, so servos start limp — position the arm
  vertical by hand THEN proceed (the script enables servos and will move from
  there; `_move_servo` commands the target regardless of the real start, though
  the first step may be abrupt since there is no servo-angle readback).
- **TODO:** we never determined the servo angles for true "straight up" (would
  need a sweep next session). By the (uncalibrated) convention, upper-arm-up is
  shoulder_geo 90° → servo ~180° but `SHOULDER_SAFE_MAX` is 150°, so full vertical
  may not be reachable; ~servo 150 gives ~60° above horizontal. Add a `--park`
  mode (smooth move to a chosen pose) if you want a repeatable rest position.

---

## Live camera stream (debugging vision)

Vision node writes an annotated `latest.jpg`; a tiny HTTP server makes it
viewable at **http://192.168.8.152:8091/** (auto-refresh ~4 fps). To (re)start:
```bash
ssh mae162-s4-g7@192.168.8.152 'bash -s' <<'EOF'
docker exec docker-ros2_runtime-1 pkill -9 -f vision_node; pkill -9 -f "http.server 8091"; sleep 2
docker exec -d docker-ros2_runtime-1 bash -lc 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 run vision vision_node --ros-args -p camera_device:=/dev/video10 -p debug_save_enabled:=true -p debug_save_only_on_detection:=false -p debug_save_rate_hz:=4.0 -p debug_save_latest:=true > /tmp/vis.log 2>&1'
DIR=/home/mae162-s4-g7/Project-NUEVO/ros2_ws/runtime_output/vision
printf '%s' '<!doctype html><body style="margin:0;background:#111;text-align:center"><img id=f style="max-width:100%;height:auto"><script>setInterval(function(){document.getElementById("f").src="latest.jpg?"+Date.now()},250)</script>' > "$DIR/index.html"
cd "$DIR"; nohup setsid python3 -m http.server 8091 --bind 0.0.0.0 </dev/null >/tmp/camhttp.log 2>&1 & disown
EOF
```
Pull a single frame to inspect: `scp mae162-s4-g7@192.168.8.152:/home/mae162-s4-g7/Project-NUEVO/ros2_ws/runtime_output/vision/latest.jpg /tmp/`.

Camera dies after a `pi-camera-feed` restart because the V4L2 loopback splits
into Output-only — if the node logs `select() timeout` / "waiting for camera",
restart the `pi-camera-feed` service (sudo password `password`) then restart the
container or just retry until the node grabs the Capture side.

---

## Git state

- `marshmallow_pick.py` and `marshmallow_pick_solver_test.py` are **on this Mac's
  working tree and on the Pi (`tests-todd`)**, NOT committed to `main`. The pose
  solver (`marshmallow_pick_solver_test.py`, report-only x/y/z + turntable angle +
  IK) was committed to `main` earlier in `c44dfb7`; the **mover (`marshmallow_pick.py`)
  was never committed.** Commit + push it when ready.
- The 0.7 vision confidence filter (`min_display_confidence` in `vision_node.py`)
  is on `main` (`b1f7e8e`) but the Pi's `tests-todd` checkout has it as a working
  edit.

---

## One-liners cheat sheet

```bash
# Grab (proven):
... python3 .../marshmallow_pick.py --turntable 0 --shoulder 90 --elbow 160 --grab

# Aim/inspect at a pose (stream ultrasonic, no motion past the pose):
... python3 .../marshmallow_pick.py --turntable 0 --shoulder 90 --elbow 160 --probe --probe-s 10

# Find the aim from scratch (sweep elbow at shoulder 90):
... python3 .../marshmallow_pick.py --turntable 0 --shoulder 90 --sweep elbow --sweep-from 160 --sweep-to 20

# Release the grip / unlock arm (firmware IDLE):
... ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 1, timeout_sec: 8.0}"
```
(prefix the python/ros2 lines with the container source: `ssh mae162-s4-g7@192.168.8.152 'docker exec docker-ros2_runtime-1 bash -lc "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && <cmd>"'`)
