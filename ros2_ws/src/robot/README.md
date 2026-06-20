# Robot Package

High-level autonomy for MARSY: the Robot API, FSMs, planners, perception integration, and the demo programs that run the full graded course and the precision pick-and-place.

This package sits on top of the provided ROS 2 bridge (TLV ⇆ ROS) and below the sensor nodes that feed it. The team's contribution here is the [`programs/`](robot/programs/) tree, the manipulator FSM and inverse kinematics, the lane-switch avoidance customization, and the cup-pair vision logic. The `Robot` class itself and the path-planner scaffolding came from the teaching team and were extended for the venue.

---

## Layout

| Path | What it is |
|---|---|
| [`robot.py`](robot/robot.py) | The `Robot` class. Composes four mixins, owns ROS publishers/subscriptions, exposes the public API. |
| [`robot_impl/`](robot/robot_impl/) | The four mixin modules: `hardware.py` (motor/servo/stepper/LED), `sensors.py` (cached state from the spin thread), `navigation.py` (alternative planner integrations), `legacy.py` (the shipped venue-demo planner path). |
| [`path_planner.py`](robot/path_planner.py) | Path-planning algorithm classes. See Planners below. |
| [`arm_kinematics.py`](robot/arm_kinematics.py) | Closed-form 3-DOF inverse kinematics with explicit elbow-up branch selection. |
| [`hardware_map.py`](robot/hardware_map.py) | Pin mapping, motor IDs, LED/button enums, default rates. |
| [`_manipulator_config.py`](robot/_manipulator_config.py) | Manipulator geometry, joint limits, calibration constants. |
| [`sensor_fusion.py`](robot/sensor_fusion.py) | Complementary filters for heading and position. |
| [`obstacle_tracking.py`](robot/obstacle_tracking.py) | LiDAR cluster tracking that feeds the avoidance planner. |
| [`programs/`](robot/programs/) | All runnable hardware programs. See [`programs/README.md`](robot/programs/README.md). |
| [`examples/`](robot/examples/) | Smaller reference snippets for individual API areas. |
| [`robot_node.py`](robot/robot_node.py) | ROS node entry point. Builds a `Robot`, calls `main.run(robot)`. |
| [`main.py`](robot/main.py) | Selects which program runs by import. |

---

## Execution model

```
sensor ROS nodes (vision, LiDAR, GPS/ArUco, ultrasonic)
        │
        ▼
Robot class (robot.py + four mixins)
        │
        ▼
bridge node (TLV ⇆ ROS) ⇆ Arduino firmware over UART
```

The ROS spin thread continuously refreshes cached state from sensor topics. The active program runs an explicit FSM loop and issues commands through the Robot API. Path-following methods spin their own nav thread that calls planner `compute_velocity()` and writes drive commands.

Path planners are pure algorithm classes. They do not own ROS subscriptions or threads.

---

## Planners

The shipped venue demo ([`programs/demos/full_competition_venue_run.py`](robot/programs/demos/full_competition_venue_run.py)) uses **`PurePursuitPlannerWithAvoidance`** in [`path_planner.py`](robot/path_planner.py), invoked via [`robot_impl/legacy.py`](robot/robot_impl/legacy.py)'s `_nav_follow_pp_path`. When a cone is detected, the planner inserts a hat-shaped detour into the remaining waypoint list: lane offset on entry, hold past obstacle, return to center on exit.

The other planners in `path_planner.py` (`APFPlanner`, `LeashedAPFPlanner`, base `PurePursuitPlanner`) are alternative implementations explored during development but **not used in the final demo**. They remain in the tree as reference.

---

## Entry points

Two headline programs:

- [`programs/demos/full_competition_venue_run.py`](robot/programs/demos/full_competition_venue_run.py) for the full graded venue course
- [`programs/demos/precision_stack_manipulator_demo.py`](robot/programs/demos/precision_stack_manipulator_demo.py) for vision-driven pick-and-place

Select which one runs by uncommenting exactly one import in [`main.py`](robot/main.py).

---

## API conventions

- **Units**: lengths and velocities use the active `Unit` (set via `robot.set_unit(Unit.MM)` or `Unit.INCH`). Method names ending in `_mm` are always raw millimeters regardless of the active unit.
- **Angles**: heading is degrees in the public API. The one exception is `max_angular_rad_s` in path-following calls, which is rad/s.
- **Motion**: high-level motion calls return a `MotionHandle` and accept `blocking=True` (the default) or `blocking=False` (returns immediately; poll with `handle.is_finished()`). Only one high-level motion runs at a time.

Source of truth for all method signatures: [`robot.py`](robot/robot.py).
