from __future__ import annotations

import time

from robot.hardware_map import (
    DCPidLoop,
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LED,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    VELOCITY_KD,
    VELOCITY_KI,
    VELOCITY_KP,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot

DRIVE_SPEED_MM_S = 90.0
DRIVE_SECONDS = 2.5


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    ok = robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    print(f"[CONFIG] set_odometry_parameters: {'OK' if ok else 'FAILED'}")


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    ok = robot.set_state(FirmwareState.RUNNING)
    print(f"[CONFIG] set_state RUNNING: {'OK' if ok else 'FAILED'}")

    # PID gains MUST be sent after RUNNING is confirmed — firmware ignores
    # them in IDLE state.
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
    time.sleep(0.1)
    print("[CONFIG] PID gains applied")


def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    state = "FORWARD"
    state_entered_at = time.monotonic()

    robot.set_led(LED.GREEN, 255)
    robot.set_led(LED.ORANGE, 0)
    print("[FSM] FORWARD — driving both wheels forward")

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        if state == "FORWARD":
            robot.set_velocity(DRIVE_SPEED_MM_S, 0)
            if now - state_entered_at >= DRIVE_SECONDS:
                robot.stop()
                robot.set_led(LED.GREEN, 0)
                robot.set_led(LED.ORANGE, 255)
                print("[FSM] PAUSE")
                state = "PAUSE_1"
                state_entered_at = now

        elif state == "PAUSE_1":
            robot.stop()
            if now - state_entered_at >= 1.0:
                robot.set_led(LED.GREEN, 255)
                robot.set_led(LED.ORANGE, 0)
                print("[FSM] BACKWARD — driving both wheels backward")
                state = "BACKWARD"
                state_entered_at = now

        elif state == "BACKWARD":
            robot.set_velocity(-DRIVE_SPEED_MM_S, 0)
            if now - state_entered_at >= DRIVE_SECONDS:
                robot.stop()
                robot.set_led(LED.GREEN, 0)
                robot.set_led(LED.ORANGE, 255)
                print("[FSM] DONE — check if both wheels moved symmetrically")
                state = "DONE"

        elif state == "DONE":
            robot.stop()

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
