# ── Active program ────────────────────────────────────────────────────────────
# Change this one line to switch which program the robot runs.
# All programs expose a run(robot: Robot) -> None entry point.
#
# Available programs:
#   from robot.programs.final_demo                         import run  # full competition sequence
#   from robot.programs.full_venue_route                   import run  # drive + obstacle avoidance
#   from robot.programs.manipulator_demo                   import run  # manipulator sequence
#   from robot.tests.scripts.lane_switch_obstacle_test     import run  # LiDAR obstacle avoidance test
#   from robot.tests.scripts.venue_no_obstacles_test       import run  # drive without obstacles
#
# TODO: tunable parameters (PID gains, path waypoints, servo angles, step counts)
# will move to a YAML config file so this file stays a one-liner permanently.

from robot.tests.scripts.lane_switch_obstacle_test import run  # noqa: F401
