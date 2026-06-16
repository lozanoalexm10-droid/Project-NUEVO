"""
green_light_detect_test.py
==========================
Watches /vision/detections for green traffic light and prints loudly on each
hit. No driving. Use this to verify the vision pipeline is seeing green before
relying on it for autostart.

For annotated debug images saved on each detection, launch the vision node with:

    ros2 run vision vision \
        --ros-args \
        -p debug_save_enabled:=true \
        -p debug_save_only_on_detection:=true \
        -p debug_save_timestamped:=true

Images land in /runtime_output/vision/vision_<timestamp>.jpg (inside Docker).
Copy them off with:
    scp mae162-s4-g7@192.168.8.152:~/Project-NUEVO/ros2_ws/runtime_output/vision/*.jpg ./

==============================================================================
HOW TO RUN
==============================================================================
Terminal 1 — Docker already up.

Terminal 2 — Vision node (with debug save):
    docker compose -f ros2_ws/docker/docker-compose.rpi.yml exec ros2_runtime bash
    source /ros2_ws/install/setup.bash
    ros2 run vision vision \\
        --ros-args \\
        -p debug_save_enabled:=true \\
        -p debug_save_only_on_detection:=true \\
        -p debug_save_timestamped:=true

Terminal 3 — This test (set main.py import first):
    source /ros2_ws/install/setup.bash
    ros2 launch robot robot.launch.py

Point the camera at a green traffic light sign. You should see:
    [GREEN] conf=0.87  bbox=...  — image saved to /runtime_output/vision/
==============================================================================

Nodes required:  bridge (auto) + vision + robot
"""
from __future__ import annotations

import time

from robot.hardware_map import DEFAULT_FSM_HZ, LED
from robot.robot import FirmwareState, Robot

MIN_CONFIDENCE      = 0.50
VISION_STALE_SEC    = 3.0
VISION_WAIT_S       = 10.0
LED_BRIGHTNESS      = 255


def _set_green_led(robot: Robot) -> None:
    robot.set_led(LED.RED, 0)
    robot.set_led(LED.GREEN, LED_BRIGHTNESS)


def _leds_off(robot: Robot) -> None:
    for led in (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE):
        robot.set_led(led, 0)


def run(robot: Robot) -> None:
    print("[TEST] green_light_detect_test — watching for green traffic light")
    print(f"[TEST] Threshold: confidence >= {MIN_CONFIDENCE}")
    print("[TEST] Point camera at a green traffic light. Ctrl+C to stop.")
    print()

    robot.set_state(FirmwareState.RUNNING)
    robot.enable_vision()
    _leds_off(robot)

    print(f"[TEST] Waiting up to {VISION_WAIT_S}s for vision to come online...")
    deadline = time.monotonic() + VISION_WAIT_S
    while time.monotonic() < deadline:
        if robot.is_vision_active(timeout_s=1.0):
            break
        time.sleep(0.5)
    else:
        print("[TEST] FAIL: vision did not come online — is the vision node running?")
        return

    img_w, img_h = robot.get_detection_image_size()
    print(f"[TEST] Vision active — frame {img_w}x{img_h}")
    print()

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()
    last_state: str | None = None

    while True:
        now = time.monotonic()

        dets = robot.get_detections("traffic_light")
        best_color: str | None = None
        best_conf  = -1.0
        best_bbox  = None

        for det in dets:
            conf  = float(det["confidence"])
            color = det.get("attributes", {}).get("color", {}).get("value")
            if conf >= MIN_CONFIDENCE and color in ("red", "green") and conf > best_conf:
                best_conf  = conf
                best_color = color
                best_bbox  = det.get("bbox")

        if best_color == "green":
            _set_green_led(robot)
            if last_state != "green":
                print(f"[GREEN] conf={best_conf:.2f}  bbox={best_bbox}  — image saved to /runtime_output/vision/")
            last_state = "green"

        elif best_color == "red":
            _leds_off(robot)
            if last_state != "red":
                print(f"[red]   conf={best_conf:.2f}  bbox={best_bbox}")
            last_state = "red"

        else:
            _leds_off(robot)
            if last_state is not None:
                print("[---]   no traffic light detected")
            last_state = None

        next_tick += period
        sleep_s = next_tick - now
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
