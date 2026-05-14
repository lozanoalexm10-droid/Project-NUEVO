from __future__ import annotations

import math
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
    robot.enable_lidar()
    robot.enable_gps()
    robot.set_tracked_tag_id(TAG_ID)

    robot.set_orientation_fusion_alpha(0.0)
    robot.set_position_fusion_alpha(0.10)


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
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
        robot.wait_for_pose_update(timeout=0.5)


PRE_OBSTACLE_PATH = [
    (0.0, 610.0),
    (0.0, 3660.0),
    (610.0, 3660.0),
    (610.0, 610.0),
    (1565.0, 610.0),
]

OBSTACLE_FIELD_PATH = [
    (1565.0, 610.0),
    (1565.0, 3660.0),
]

POST_OBSTACLE_PATH = [
    (1565.0, 3660.0),
    (2530.0, 3660.0),
    (2530.0, 610.0),
    (2745.0, 305.0),
]


def start_path_segment(
    robot: Robot,
    control_points: list[tuple[float, float]],
    *,
    obstacle_avoidance: bool,
    x_L: float = 0.0,
    path_spacing: float = 50.0,
) -> None:
    path = densify_polyline(control_points, spacing=path_spacing)

    robot._nav_follow_pp_path(
        lookahead_distance=100.0,
        max_linear_speed=90.0,
        max_angular_speed=1.5,
        goal_tolerance=20.0,
        obstacles_range=450.0,
        view_angle=math.radians(70.0),
        safe_dist=250.0,
        avoidance_delay=150,
        alpha_Ld=0.7,
        offset=305.0,
        lane_width=500.0,
        obstacle_avoidance=obstacle_avoidance,
        x_L=x_L,
    )

    robot._set_obstacle_avoidance_path(path)


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()
    state_entry_time = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            start_path_segment(robot, PRE_OBSTACLE_PATH, obstacle_avoidance=False)
            print("[FSM] INIT complete — pre-obstacle path loaded. Auto-starts in 3 s.")
            state = "IDLE"
            state_entry_time = time.monotonic()

        elif state == "IDLE":
            show_idle_leds(robot)
            if robot.get_button(Button.BTN_2):
                robot.stop()
                robot.shutdown()
                return
            if time.monotonic() - state_entry_time >= 3.0:
                print("[FSM] IDLE — auto-starting pre-obstacle segment.")
                show_moving_leds(robot)
                state = "MOVING_PRE_OBSTACLE"
                state_entry_time = time.monotonic()

        elif state == "MOVING_PRE_OBSTACLE":
            show_moving_leds(robot)
            nav_state = robot._nav_follow_pp_path_loop()

            if nav_state == "IDLE":
                print("[FSM] Pre-obstacle segment complete. Starting obstacle field.")
                start_path_segment(robot, OBSTACLE_FIELD_PATH, obstacle_avoidance=True, x_L=1565.0)
                state = "MOVING_OBSTACLE"
                state_entry_time = time.monotonic()

        elif state == "MOVING_OBSTACLE":
            show_moving_leds(robot)
            nav_state = robot._nav_follow_pp_path_loop()

            if nav_state == "IDLE":
                print("[FSM] Obstacle field complete. Starting post-obstacle segment.")
                start_path_segment(robot, POST_OBSTACLE_PATH, obstacle_avoidance=False)
                state = "MOVING_POST_OBSTACLE"
                state_entry_time = time.monotonic()

        elif state == "MOVING_POST_OBSTACLE":
            show_moving_leds(robot)
            nav_state = robot._nav_follow_pp_path_loop()

            if nav_state == "IDLE":
                print("[FSM] Full venue route complete.")
                robot.stop()
                state = "DONE"
                state_entry_time = time.monotonic()

        elif state == "DONE":
            show_idle_leds(robot)
            robot.stop()


        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()