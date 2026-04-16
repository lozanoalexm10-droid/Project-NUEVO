"""
main.py — student entry point
==============================
This is the only file students are expected to edit.

The structure is intentionally simple:
- keep one plain `state` variable
- write helper functions for robot actions
- use `if state == "..."` inside the main loop

To run:
    ros2 run robot robot
"""

from __future__ import annotations
import time

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.util import densify_polyline
from robot.path_planner import PurePursuitPlanner
import math


# ---------------------------------------------------------------------------
# Robot build configuration
# ---------------------------------------------------------------------------

POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True


def configure_robot(robot: Robot) -> None:
    """Apply the user unit plus robot-specific wheel mapping and odometry settings."""
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


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.ORANGE, 255)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 255)


def start_robot(robot: Robot) -> None:
    """Start the firmware and reset odometry before the main mission begins."""
    if not robot.set_state(FirmwareState.RUNNING):
        print("[FSM] ERROR: failed to transition firmware to RUNNING. Check bridge / serial status.")
    robot.reset_odometry()
    if not robot.wait_for_pose_update(timeout=0.2):
        print("[FSM] WARNING: timed out waiting for a pose update after odometry reset.")
    else:
        print("[FSM] Pose update received after odometry reset.")


def run(robot: Robot) -> None:
    configure_robot(robot)
    

    state = "INIT"
    drive_handle = None
    # FSM refresh rate control
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            print("[FSM] INIT (odometry reset)")
            path_control_points = [ #Define your path control points here (x, y) in mm
                # (0.0, 0.0),
                # (0.0, 150.0),
                # (0.0, 300.0),
                # (0.0, 450.0),
                # (0.0, 610.0),
                # (150.0, 610.0),
                # (300.0, 610.0),
                # (450.0, 610.0),
                # (610.0, 610.0),
                # (610.0, 450.0),
                # (610.0, 300.0),
                # (610.0, 150.0),
                # (610.0, 0.0),
                # (450.0, 0.0),
                # (300.0, 0.0),
                # (150.0, 0.0),
                # (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 610.0),
                (610.0, 610.0),
                (610.0, 0.0),
                (0.0, 0.0),
            ]    
            path1 = path_control_points
            #path1 = densify_polyline(path_control_points, spacing=20.0)
            remaining_path = path1.copy() 
            print("Path is ready, Entering IDLE state.")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            print("[FSM] IDLE - Press BTN_1 to enter MOVING state.")
            if robot.get_button(Button.BTN_1):
                LOOKAHEAD_DIST = 100.0 # Lookahead distance in mm (adjust as needed)
                planner1 = PurePursuitPlanner(
                    lookahead_dist=LOOKAHEAD_DIST, 
                    max_angular=1.5, # Max angular velocity in rad/s (adjust as needed)
                    goal_tolerance=20.0, # Distance in mm to consider the target reached (adjust as needed)
             )
                print("Pure Pursuit Planner is initialized. Start Moving!")
                print("[FSM] MOVING")
                state = "MOVING"

        elif state == "MOVING":
            show_moving_leds(robot)

            # Step 1: Get current pose
            current_x, current_y, current_theta_deg = robot.get_pose()

            # Step 2: Convert to radians
            current_theta_rad = math.radians(current_theta_deg)

            # Step 3: Advance remaining path
            remaining_path = robot._advance_remaining_path(remaining_path, current_x, current_y, advance_radius_mm=20.0)

            # Step 4: Get lookahead point
            current_pursuit_x, current_pursuit_y = planner1._lookahead_point(
                current_x,
                current_y,
                waypoints=remaining_path,
            )

            # Step 5: Compute velocity
            linear_velocity_cmd, angular_velocity_cmd_rad_s = planner1.compute_velocity(
                pose=(current_x, current_y, current_theta_rad),
                waypoints=remaining_path,
                max_linear=80.0,
            )

            # Step 6: Send velocity commands
            robot.set_velocity(
                linear_velocity_cmd,
                math.degrees(angular_velocity_cmd_rad_s),
            )

            # Step 7: Check if target reached
            if planner1.CurrentTargetReached(current_pursuit_x, current_pursuit_y, current_x, current_y):
                print("MOVING: Target reached! Stopping.")
                robot.stop()
                print("[FSM] IDLE")
                state = "IDLE"

            # Step 8: Debug prints
            print(f"Current Pose: ({current_x:.1f}, {current_y:.1f}, {current_theta_deg:.1f} deg)")
            print(f"Current Pursuit Point: ({current_pursuit_x:.1f}, {current_pursuit_y:.1f})")
            
        # FSM refresh rate control
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
