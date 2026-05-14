"""
vision_traffic_light_drive.py
==============================
Vision-triggered LED control and driving — traffic light and stop sign test.

Drives forward when a green light is detected, stops on red or stop sign.
Use this to verify the vision pipeline end-to-end before the full competition demo.

What the robot does:
  - Green traffic light detected  → green LED on, drive forward at DRIVE_SPEED_MM_S
  - Red traffic light detected    → red LED on, stop
  - Stop sign detected            → red LED on, stop immediately (overrides traffic light)
  - No detection for VISION_STALE_SEC → all LEDs off, stop

==============================================================================
HOW TO RUN — 3 terminals
==============================================================================

TERMINAL 1 — Start Docker (runs the bridge automatically, keep this open)
--------------------------------------------------------------------------
From the repository root (Project-NUEVO/):

    docker compose -f ros2_ws/docker/docker-compose.rpi.yml up -d --build --wait

The container starts and immediately launches:
    ros2 launch bridge bridge.launch.py

This opens the bridge node that relays ROS2 topics ↔ Arduino serial commands.
Do NOT start the bridge manually — it is already running as the container's main process.

Watch logs (optional, open in the same terminal or a side panel):

    docker compose -f ros2_ws/docker/docker-compose.rpi.yml logs -f ros2_runtime


TERMINAL 2 — Enter the container and start the vision node
-----------------------------------------------------------
Enter the running container:

    docker compose -f ros2_ws/docker/docker-compose.rpi.yml exec ros2_runtime bash

Source ROS2 (required in every new shell inside the container):

    source /ros2_ws/install/setup.bash

Start the vision node:

    ros2 run vision vision

Leave this terminal running. Vision must be active before the robot node starts.


TERMINAL 3 — Set the program, then launch the robot node
---------------------------------------------------------
Enter the container in a new terminal:

    docker compose -f ros2_ws/docker/docker-compose.rpi.yml exec ros2_runtime bash

Source ROS2:

    source /ros2_ws/install/setup.bash

Tell main.py to load this program (edit the one import line):

    sed -i 's|^from robot.*import run.*|from robot.tests.scripts.vision_traffic_light_drive import run  # noqa: F401|' \
        /ros2_ws/src/robot/robot/main.py

Launch the robot node:

    ros2 launch robot robot.launch.py

The robot starts watching for traffic light detections immediately.
Place a red or green traffic light sign in front of the camera and observe the LEDs.

==============================================================================
Pass criteria
==============================================================================
  - Green light shown → robot drives forward, green LED on
  - Red light shown   → robot stops, red LED on
  - Stop sign shown   → robot stops immediately (overrides green), red LED on
  - Light removed     → LEDs off within VISION_STALE_SEC, robot stops
  - No false positives from unrelated objects in the background

Nodes required:  bridge (auto) + vision + robot
"""
from __future__ import annotations

import time

from robot.hardware_map import (
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LED,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot

# ── Configuration ─────────────────────────────────────────────────────────────
LED_BRIGHTNESS              = 255
LIGHT_HOLD_SEC              = 2.0    # hold LED on this long after last detection
VISION_STALE_SEC            = 3.0    # treat vision as lost after this many seconds
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50
DRIVE_SPEED_MM_S            = 90.0


# ── Helpers ───────────────────────────────────────────────────────────────────

def _configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision()
    ok = robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    print(f"[CONFIG] odometry parameters: {'OK' if ok else 'FAILED'}")


def _dim_all_leds(robot: Robot) -> None:
    for led in (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE):
        robot.set_led(led, 0)


def _show_color(robot: Robot, color: str) -> None:
    if color == "red":
        robot.set_led(LED.RED, LED_BRIGHTNESS)
        robot.set_led(LED.GREEN, 0)
    elif color == "green":
        robot.set_led(LED.RED, 0)
        robot.set_led(LED.GREEN, LED_BRIGHTNESS)


def _find_traffic_light_color(robot: Robot) -> str | None:
    """Return 'red' or 'green' from the highest-confidence traffic light detection, or None."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None
    best_color = None
    best_conf  = -1.0
    for det in robot.get_detections("traffic light"):
        conf  = float(det["confidence"])
        color = det.get("attributes", {}).get("color", {}).get("value")
        if conf >= MIN_TRAFFIC_LIGHT_CONFIDENCE and color in ("red", "green") and conf > best_conf:
            best_conf  = conf
            best_color = str(color)
    return best_color


# ── Entry point ───────────────────────────────────────────────────────────────

def run(robot: Robot) -> None:
    _configure_robot(robot)

    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

    _dim_all_leds(robot)
    robot.stop()
    print("[TEST] WATCHING — show a red or green traffic light to the camera")

    lights_off_at    = 0.0
    last_shown_color = None

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        # Stop sign overrides everything.
        if robot.get_detections("stop sign"):
            robot.stop()
            _show_color(robot, "red")
            if last_shown_color != "stop":
                print("[TEST] STOP SIGN detected — stopping")
            last_shown_color = "stop"
            lights_off_at    = 0.0

        else:
            color = _find_traffic_light_color(robot)

            if color in ("red", "green"):
                _show_color(robot, color)
                lights_off_at = now + LIGHT_HOLD_SEC
                if color != last_shown_color:
                    print(f"[TEST] traffic light: {color}")
                last_shown_color = color

            elif lights_off_at > 0.0 and now >= lights_off_at:
                _dim_all_leds(robot)
                lights_off_at = 0.0
                if last_shown_color is not None:
                    print("[TEST] no recent detection — LEDs off, stopping")
                last_shown_color = None

            if color == "green":
                robot.set_velocity(DRIVE_SPEED_MM_S, 0)
            else:
                robot.stop()

        next_tick += period
        sleep_s = next_tick - now
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
