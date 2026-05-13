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
from robot.util import densify_polyline

# Edit PATH to drive any sequence of waypoints from the robot's starting position.
# Initial heading is 90 deg (+Y is forward). One tile = 610 mm.
PATH = [
    (0.0, 0.0),
    (0.0, 3660.0),  # 6 tiles straight ahead
]


def run(robot: Robot) -> None:
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

    # Pure odometry — no GPS fusion
    robot.set_orientation_fusion_alpha(0.0)
    robot.set_position_fusion_alpha(0.0)

    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

    robot.set_pid_gains(LEFT_WHEEL_MOTOR, DCPidLoop.VELOCITY, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD)
    robot.set_pid_gains(RIGHT_WHEEL_MOTOR, DCPidLoop.VELOCITY, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD)
    time.sleep(0.2)

    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=3.0):
        print("[WARN] odometry reset not confirmed within 3s — pose may be stale")
        robot.wait_for_pose_update(timeout=1.0)
    x0, y0, th0 = robot.get_odometry_pose()
    print(f"[START] pose = ({x0:.1f}, {y0:.1f}) mm  θ={th0:.1f}°")

    path = densify_polyline(PATH, spacing=50.0)

    robot._nav_follow_pp_path(
        lookahead_distance=100.0,
        max_linear_speed=90.0,
        max_angular_speed=1.5,
        goal_tolerance=20.0,
        obstacles_range=450.0,
        view_angle=0.0,
        safe_dist=250.0,
        avoidance_delay=150,
        alpha_Ld=0.7,
        offset=305.0,
        lane_width=500.0,
        obstacle_avoidance=False,
        x_L=0.0,
    )
    robot._set_obstacle_avoidance_path(path)

    state = "MOVING"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    robot.set_led(LED.GREEN, 255)
    robot.set_led(LED.ORANGE, 0)
    print(f"[FSM] MOVING — {len(path)} densified points")

    while True:
        if state == "MOVING":
            nav_state = robot._nav_follow_pp_path_loop()
            if nav_state == "IDLE":
                robot.stop()
                robot.set_led(LED.GREEN, 0)
                robot.set_led(LED.ORANGE, 255)
                x1, y1, th1 = robot.get_odometry_pose()
                print(f"[DONE] pose = ({x1:.1f}, {y1:.1f}) mm  θ={th1:.1f}°")
                print(f"[DONE] heading drift = {th1 - th0:.1f}°")
                state = "DONE"

        elif state == "DONE":
            robot.stop()

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
