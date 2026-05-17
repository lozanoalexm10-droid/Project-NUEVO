from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DCPidLoop,
    DEFAULT_FSM_HZ,
    LED,
    INITIAL_THETA_DEG,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    TAG_ID,
    VELOCITY_KD,
    VELOCITY_KI,
    VELOCITY_KP,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)

from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )

    robot.enable_gps()
    robot.set_tracked_tag_id(TAG_ID)

    robot.set_orientation_fusion_alpha(0.0)
    robot.set_position_fusion_alpha(0.1)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.ORANGE, 255)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 255)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

    robot.set_pid_gains(
        motor_id=LEFT_WHEEL_MOTOR,
        loop_type=DCPidLoop.VELOCITY,
        kp=VELOCITY_KP,
        ki=VELOCITY_KI,
        kd=VELOCITY_KD,
    )
    robot.set_pid_gains(
        motor_id=RIGHT_WHEEL_MOTOR,
        loop_type=DCPidLoop.VELOCITY,
        kp=VELOCITY_KP,
        ki=VELOCITY_KI,
        kd=VELOCITY_KD,
    )
    time.sleep(0.2)

    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=3.0):
        print("[WARN] odometry reset not confirmed within 3s — pose may be stale")
        robot.wait_for_pose_update(timeout=1.0)
    print(f"[CONFIG] pose after reset: {robot.get_odometry_pose()}")


FULL_PATH = [
    (0.0, 0.0),
    (0.0, 1200.0),  # Switch y back to 3100.0
    (610.0, 1200.0),# Swittch y back to 3100.0
    (610.0, 610.0),
    (1565.0, 610.0),
    (1565.0, 3110.0),
    (2530.0, 3110.0),
    (2530.0, 610.0),
    (2745.0, 305.0),
]


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    print(f"FSM period: {period:.3f} seconds")
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            print("[FSM] INIT (odometry reset)")

            path = densify_polyline(FULL_PATH, spacing=50.0)

            print("[FSM] Full path ready. Entering IDLE state.")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            time.sleep(3)

            if robot.get_button(Button.BTN_2):
                print("BTN_2 pressed. Stopping robot and saving trajectory.")
                robot.shutdown()
                return

            print("[FSM] IDLE - Auto-starting full path.")
            print("[FSM] MOVING")
            state = "MOVING"

        elif state == "MOVING":
            show_moving_leds(robot)
            robot.purepursuit_follow_path(
                waypoints=path,
                velocity=130.0,
                lookahead=100.0,
                tolerance=20.0,
                max_angular_rad_s=0.7,
                advance_radius=80.0,
                blocking=True,
            )
            print("[FSM] Full venue route complete.")
            robot.stop()
            state = "DONE"

        elif state == "DONE":
            show_idle_leds(robot)
            robot.stop()

        next_tick += period
        sleep_s = next_tick - time.monotonic()

        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()