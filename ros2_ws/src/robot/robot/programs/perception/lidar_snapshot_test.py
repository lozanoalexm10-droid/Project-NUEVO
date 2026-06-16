"""Single lidar snapshot at the current robot position.

Place the robot manually where you want the snapshot (e.g. obstacle-zone
entry, facing north), then run this script. It enables the lidar, waits
2 s for scans to land, and saves /data/plot.png via the existing
_draw_lidar_obstacles() visualizer.

Plot conventions:
  x-axis: west(-)/east(+) in metres relative to the robot
  y-axis: south(-)/north(+) in metres relative to the robot
  Robot is at (0, 0) facing +y (north) because odom is reset to that pose.
"""
from __future__ import annotations

import time

from robot.hardware_map import POSITION_UNIT
from robot.robot import FirmwareState, Robot


def run(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)

    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

    robot.set_initial_theta(90.0)
    robot.reset_odometry()
    robot.wait_for_odometry_reset(timeout=3.0)

    robot.enable_lidar()
    time.sleep(2.0)

    robot._draw_lidar_obstacles()
    print("[SNAPSHOT] Saved /data/plot.png — robot at (0,0,90°)")
