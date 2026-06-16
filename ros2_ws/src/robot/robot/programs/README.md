# robot.programs

Runtime hardware programs for the NUEVO capstone robot. Every file here runs on the physical hardware — nothing here is a pytest unit test.

---

## Layout

```
programs/
  README.md
  _cup_scan_logic.py             shared vision-driven cup-scan helpers

  demos/                         the two final headline programs
    full_competition_venue_run.py        complete graded venue route (driving + avoidance)
    precision_stack_manipulator_demo.py  vision-driven stack pick-and-place

  navigation/                    drive-only programs
    straight_line_test.py
    lane_switch_obstacle_test.py
    uturn_test.py
    seg4_standalone_test.py
    drive_both_wheels_test.py

  manipulation/                  arm, servo, stepper, gripper, turntable
    arm_ik_calibration_test.py        full IK round-trip calibration
    arm_extend_retract_test.py
    elbow_calibration_test.py
    elbow_range_test.py
    shoulder_range_test.py
    servo_calibration.py              guided servo offset calibration
    servo_range_test.py
    gripper_open_close_test.py
    campan_range_test.py
    turntable_range_test.py
    turntable_home_test.py
    stepper_basic_move_test.py
    disable_servos.py
    heating_wire_test.py

  perception/                    vision + lidar
    lidar_snapshot_test.py
    green_light_detect_test.py
    mallow_detection_range_test.py
    detection_vision_test.py

  analysis/                      data recording + plotting
    nav_data_recorder.py              logs odometry + heading to CSV
    plot_nav_results.py               matplotlib plots from recorder output
    PLOTTER_GUIDE.md                  plotting workflow notes

  legacy/                        earlier draft programs kept for reference
```

Shared hardware constants live at the package root:

- `robot/hardware_map.py` — pin/channel assignments, motor IDs, PID gains
- `robot/_manipulator_config.py` — arm geometry, servo offsets, scan tuning

Edit those files (not individual programs) when tuning hardware.

---

## Selecting a program

Edit the one uncommented import in `robot/main.py`:

```python
# robot/main.py — uncomment exactly one line
from robot.programs.demos.precision_stack_manipulator_demo import run  # noqa: F401
#from robot.programs.demos.full_competition_venue_run import run       # noqa: F401
#from robot.programs.navigation.straight_line_test import run          # noqa: F401
```

Then launch the robot node:

```bash
ros2 launch robot robot.launch.py
```

---

## Startup order

| Step | Terminal | Command |
|------|----------|---------|
| 1 | T1 | `docker compose -f ros2_ws/docker/docker-compose.rpi.yml up -d` |
| 2 | T2 | `ros2 launch robot everything_but_robot.launch.py` (LiDAR + GPS) |
| 3 | T3 | `ros2 launch vision vision_debug.launch.py` (if vision needed) |
| 4 | T4 | `ros2 launch robot robot.launch.py` (always last) |

Steps 2 and 3 can run in either order. The robot node must always start last because it resets odometry on startup.
