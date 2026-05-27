"""
uturn_test.py
=============
U-turn from lane 1 to lane 2 using pure pursuit over a semicircular waypoint path.

Geometry:
  - Lane width / center-to-center spacing: 610 mm
  - Turning radius: 305 mm (= 610 / 2)
  - 13 waypoints spaced every 15 deg along the arc (~80 mm apart)
  - Robot starts at (0, 0) facing forward, ends at (610, 0) facing 180 deg opposite
  - Front omni wheels, rear drive wheels — ICC lies on the rear axle

Pass criteria:
  - Final x ≈ +610 mm
  - Final y ≈    0 mm
  - Heading change ≈ 180 deg
"""
from __future__ import annotations

import math
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

LANE_WIDTH_MM  = 610.0
R              = LANE_WIDTH_MM / 2   # 305 mm

VELOCITY_MM_S  = 90.0
LOOKAHEAD_MM   = 150.0
TOLERANCE_MM   = 30.0

# Semicircle waypoints: 13 points every 15 deg from 0 to 180.
# x = R*(1 - cos θ),  y = R*sin θ  (right turn, ICC to the right)
UTURN_PATH = [
    (R * (1 - math.cos(math.radians(deg))),
     R *      math.sin(math.radians(deg)))
    for deg in range(0, 181, 15)
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

    robot.set_orientation_fusion_alpha(0.0)
    robot.set_position_fusion_alpha(0.0)

    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

    robot.set_pid_gains(LEFT_WHEEL_MOTOR,  DCPidLoop.VELOCITY, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD)
    robot.set_pid_gains(RIGHT_WHEEL_MOTOR, DCPidLoop.VELOCITY, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD)
    time.sleep(0.2)

    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=3.0):
        print("[WARN] odometry reset not confirmed — pose may be stale")
        robot.wait_for_pose_update(timeout=1.0)

    x0, y0, th0 = robot.get_odometry_pose()
    print(f"[UTURN] start pose = ({x0:.1f}, {y0:.1f}) mm  θ={th0:.1f}°")
    print(f"[UTURN] {len(UTURN_PATH)} waypoints, R={R:.0f} mm, speed={VELOCITY_MM_S:.0f} mm/s")

    robot.set_led(LED.GREEN, 255)
    robot.set_led(LED.ORANGE, 0)

    robot.purepursuit_follow_path(
        waypoints=UTURN_PATH,
        velocity=VELOCITY_MM_S,
        lookahead=LOOKAHEAD_MM,
        tolerance=TOLERANCE_MM,
        blocking=True,
    )

    robot.stop()
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.ORANGE, 255)

    robot.wait_for_pose_update(timeout=1.0)
    x1, y1, th1 = robot.get_odometry_pose()
    print(f"[UTURN] end pose   = ({x1:.1f}, {y1:.1f}) mm  θ={th1:.1f}°")
    print(f"[UTURN] delta x={x1 - x0:.1f} mm  (target +{LANE_WIDTH_MM:.0f} mm)")
    print(f"[UTURN] delta y={y1 - y0:.1f} mm  (target 0 mm)")
    print(f"[UTURN] heading change={th1 - th0:.1f}°  (target ±180°)")

    x_err = abs((x1 - x0) - LANE_WIDTH_MM)
    y_err = abs(y1 - y0)
    h_err = abs(abs(th1 - th0) - 180.0)
    if x_err < 50.0 and y_err < 50.0 and h_err < 10.0:
        print("[UTURN] PASS")
    else:
        print(f"[UTURN] FAIL — x_err={x_err:.1f} mm  y_err={y_err:.1f} mm  heading_err={h_err:.1f}°")

    period = 1.0 / float(DEFAULT_FSM_HZ)
    while True:
        robot.stop()
        time.sleep(period)
