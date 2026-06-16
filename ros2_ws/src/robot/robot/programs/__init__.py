"""
robot.programs
==============
Runtime hardware programs for the NUEVO capstone robot.

Layout:

  demos/         — final headline programs (full_competition_venue_run,
                    precision_stack_manipulator_demo)
  navigation/    — drive-only programs (straight-line, U-turn, lane-switch,
                    seg4 standalone, drive_both_wheels)
  manipulation/  — arm, servo, stepper, gripper, turntable, campan
  perception/    — vision + lidar tests
  analysis/      — data recording + plotting
  legacy/        — earlier draft programs kept for reference

  _cup_scan_logic.py — shared vision-driven scan helpers

Shared hardware constants live at the package root (robot/hardware_map.py,
robot/_manipulator_config.py), not inside this package.

All programs expose a `run(robot: Robot) -> None` entry point and are selected
for execution by uncommenting the matching import in main.py.
"""
