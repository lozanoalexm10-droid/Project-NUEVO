# NUEVO Manipulator Task Plan — Competition Demo

How the robot autonomously locates, picks, and places the marshmallow during
the manipulation portion of the demo. Companion to `MANIPULATOR.md`
(geometry / kinematics reference); this doc covers the *task strategy* —
which sensor produces which coordinate, how those coordinates are derived,
and the FSM that uses them.

The implementation lives in
`robot/tests/scripts/lidar_cup_pick_place_test.py`.

---

## 1. What the robot has to figure out

Three pieces of information define a successful pick:

| Coordinate | Meaning | Source |
|---|---|---|
| `pick_x_mm`, `pick_y_mm` | Mallow horizontal position in robot frame (turntable axis = origin) | **LiDAR** (cup-stack centroid) |
| `pick_z_mm`              | Mallow centre height above robot base plate | **Camera** bbox height → snapped to one of three measured tiers |

The IK solver then turns `(pick_x, pick_y, pick_z)` into the turntable,
shoulder-servo, and elbow-servo angles needed to put the gripper there.
The IK math is in `arm_kinematics.inverse_kinematics()` — verified by the
manipulator simulator and known good as long as the calibration constants
in `_manipulator_config.py` are correct.

## 2. Why LiDAR for x, y and camera for z

The competition setup is three fixed cup stacks (8, 18, 24 cups), one of
which has a marshmallow on top. The marshmallow gets coloured purple with a
Sharpie before the run to give the camera a high-contrast target.

Each sensor has one job it does well:

* **LiDAR** scans one horizontal plane through every cup stack at lidar
  mount height. Cup stacks are vertical cylinders, so each stack lights up
  as a tight cluster of points. The point cloud gives ±5 mm radial accuracy
  and ±0.5° bearing — far more precise than the camera's pinhole-bbox
  distance estimate. It also sees all three stacks in one shot without
  having to move the arm into a "ranging pose."
* **Camera** can't measure depth precisely, but is great at recognising a
  *purple* mallow against a *red* cup background. We use it only to answer
  "which of the three stacks has the mallow on it" and to give a rough
  height estimate from the bbox vertical position.
* **Ultrasonic** is geometrically poor for this task — it's mounted on top
  of the forearm and the forearm has to point steeply down to range a low
  target, putting the sensor itself behind/above the cup centre line. We
  do not use it in the demo path. The sensor stays wired up and the node
  stays running for nav-time obstacle work; it just doesn't drive any
  manipulator coordinate.

## 3. Measured competition heights

Stacks sit on a 3 mm cardboard pad. Heights to the top of each stack,
measured 2026-06-07:

| Stack | Stack height above cardboard | Top-of-stack z in robot frame | Mallow-centre z in robot frame |
|---|---|---|---|
| 8 cups  | 162 mm | `floor + 3 + 162 = 165 mm`  → `z = −64 mm` | `−64 + 14 = −50 mm` |
| 18 cups | 225 mm | `floor + 3 + 225 = 228 mm`  → `z = −1 mm`  | `−1 + 14 = +13 mm` |
| 24 cups | 263 mm | `floor + 3 + 263 = 266 mm`  → `z = +37 mm` | `+37 + 14 = +51 mm` |

Robot frame: floor is at `GROUND_Z_MM = −229 mm` (base plate = 0). A 28 mm
tall marshmallow has its centre 14 mm above the surface it sits on.

These are encoded in `_manipulator_config.py` as

```python
MALLOW_Z_STACK_8_MM  = -50
MALLOW_Z_STACK_18_MM = +13
MALLOW_Z_STACK_24_MM = +51
```

and the helper `snap_to_venue_tier_mm(z)` rounds a noisy height estimate
to the nearest of those three values.

## 4. LiDAR cup-stack mapping

Implemented in `lidar_cup_pick_place_test._map_cup_stacks`.

```
1. Wait LIDAR_SETTLE_S = 0.5 s after entering LIDAR_MAP for fresh data.
2. Read robot.get_lidar_obstacles_robot() three times, ~120 ms apart, and
   accumulate all returned (x, y) points.  Three scans averages out the
   per-scan jitter on which beams happen to hit which cup.
3. Convert each point from body frame (origin = wheel midpoint) to
   turntable frame:   (x − TURNTABLE_X_IN_BODY_MM,  y).
4. Keep only points inside the mallow zone annulus:
       100 mm ≤ radius from turntable ≤ 600 mm
       bearing within ±90° of forward.
5. Greedy single-pass cluster: any point within 70 mm of an existing
   cluster's running centroid joins it; otherwise it starts a new cluster.
   70 mm catches the front arc of one cup while keeping adjacent stacks
   separated (stacks live ≥120 mm apart in the competition layout).
6. Drop clusters with fewer than 3 points (noise rejection).
7. Each remaining cluster's centroid is the cup-FRONT-arc centroid.
   Project to the cup CENTRE by extending the centroid outward by one
   cup radius (CUP_DIAMETER_MM / 2 = 32.5 mm) along the radial.
8. Return list of dicts:
       { x_mm, y_mm, bearing_deg, dist_mm, hits }
   sorted left → right (decreasing bearing).
```

Tunables live at the top of the test file: `CUP_ZONE_*`,
`CUP_CLUSTER_RADIUS_MM`, `CUP_MIN_POINTS_PER_STACK`,
`LIDAR_ACCUMULATE_SCANS`. Verify the lidar mount config on the robot —
the body→turntable conversion uses `TURNTABLE_X_IN_BODY_MM` from
`_manipulator_config.py`, which currently defaults to the lidar mount X
and should be measured directly on the chassis.

## 5. Vision target identification

Implemented in `_identify_target_stack`.

```
For each cup stack returned by LIDAR_MAP:
    a. Compute camera-frame bearing to the stack:
           dx = stack.x − CAMERA_FORWARD_OFFSET_MM
           dy = stack.y
           cam_bearing = atan2(dy, dx)
    b. Clamp to ±PICK_CAMPAN_MAX_DEG (66°) — the campan's physical limit.
    c. Pan the campan to that angle and wait PICK_CAMPAN_SETTLE_S.
    d. Pull robot.get_detections("marshmallow").
    e. Keep detections that pass MIN_CONFIDENCE_MARSHMALLOW (= 0.60)
       AND sit near the centre of the post-pan camera frame
       (within ±MALLOW_CUP_BEARING_MATCH_DEG = 10°).
    f. Estimate raw height from the bbox y-centre + camera vertical FOV.
       Snap to the nearest competition tier (snap_to_venue_tier_mm).

Pick the stack with the highest-confidence marshmallow detection.
A single retry is allowed if the first pass returns nothing (vision frames
can be momentarily empty).
```

After identification:
```
pick_x = target_stack.x_mm       (from LiDAR)
pick_y = target_stack.y_mm       (from LiDAR)
pick_z = target_stack.mallow_z   (from camera-snapped tier)
```

If the camera misses the mallow on all three stacks, the FSM goes to
RESTOW (graceful abort) rather than guessing.

## 6. IK + arm motion

Once `(pick_x, pick_y, pick_z)` is set:

```
tt_deg, sh_servo, el_servo = inverse_kinematics(
    pick_x, pick_y, pick_z, ARM_GEOMETRY
)
```

`inverse_kinematics` returns the elbow-up branch by default — this matches
the physical hinge direction. See MANIPULATOR.md §3 for why the elbow
branch matters.

The motion sequence in APPROACHING:

1. Safe-retract (shoulder → 105°, elbow → 90°, gripper → closed) before
   any turntable rotation. Always shoulder-first, never reverse this
   order.
2. Rotate turntable to `tt_deg`.
3. Open gripper to `GRIPPER_OPEN_DEG`.
4. Drive shoulder to `sh_servo`, then elbow to `el_servo`, in that order.

PICKING just closes the gripper to `GRIPPER_GRAB_DEG`. CARRY_TO_PLACE
safe-retracts again (keeping the gripper closed), rotates the turntable
to `PLACE_TURNTABLE_DEG = −45°`, and runs IK to the fixed place target
`(128, −226, −201) mm`.

## 7. State machine

```
INIT          — reset E-STOP, enable vision + lidar
IDLE          — 3-second start countdown
SAFE_RAISE    — shoulder up first, then elbow horizontal, then gripper open
ARM_HOME      — enable steppers, home turntable, pan campan to 0°
LIDAR_MAP     — cluster cup stacks → list[bearing, dist, x, y]
VISION_PICK_STACK
              — pan camera through each stack, find the purple mallow,
                snap height to nearest tier, pick the best detection
APPROACHING   — safe-retract, rotate turntable, run IK, drive to pick pose
PICKING       — close gripper
CARRY_TO_PLACE
              — safe-retract with grip held, rotate to −45°, IK to plate
PLACING       — drive shoulder + elbow to plate IK, open gripper
RESTOW        — safe-retract, return turntable to 0°, disable steppers
DONE          — green LED, robot.stop()
```

## 8. Known limitations and acceptable failures

* **Three-stack assumption.** The mapper expects 3 cup stacks. If it finds
  zero it aborts. If it finds more than 3, the extras are scanned anyway —
  the highest-confidence vision detection still wins. This is safe.
* **Tier-snap can be wrong by one tier** if the camera height estimate is
  noisy enough to land between two tiers. Mitigation: the three tiers are
  ≥63 mm apart, the camera's bbox-vertical accuracy is ~±20 mm at typical
  cup distances, so a one-tier error is unlikely but possible. If it
  becomes a problem, add a sanity-check using the cluster's z-projection
  from the lidar plane (lidar mount height vs. cup top height).
* **No closed-loop feedback during the pick.** Once IK is computed, the
  arm moves to the commanded angles open-loop. If the IK calibration is
  off, the gripper misses. The fix is calibration (servo offsets, link
  lengths) — see "Day 1" in the demo plan.
* **TURNTABLE_X_IN_BODY_MM defaults to the lidar mount X.** This is an
  approximation. Measure the actual distance from the wheel midpoint to
  the turntable axis on your robot before relying on the LiDAR x/y output.
* **No re-localisation if a stack moves between LIDAR_MAP and APPROACH.**
  Don't bump the table while the FSM is running.

## 9. How this hits the rubric

(Reference image discussed in chat — manipulation tasks section.)

* **5 pts — autonomous handoff from navigation:** the FSM picks up from
  IDLE without manual intervention, runs the full pick-place loop, and
  ends in DONE. Hand a green LED to the TA.
* **25 pts — task completion:** the demo target is one successful
  pick + place. If the IK calibration holds, this is a one-button run.
* **10 pts — manipulation task difficulty:** 3-DOF arm (turntable +
  shoulder + elbow) + gripper, lots of base-arm coordination during
  approach and carry. Auto-qualifies for the full 10.
* **+10 bonus — special solution beyond TA's experience:** the LiDAR +
  camera sensor-fusion approach (LiDAR for x/y, camera for z and target
  selection) is genuinely uncommon at the undergrad level. Lead with
  this when explaining the system — "we use the LiDAR's horizontal-plane
  ranging to localise the cup stacks and the camera only to identify the
  target on top, because the camera's pinhole distance estimate is too
  noisy for a 30 mm gripper opening." Sells the +10 tier without
  exaggeration.

## 10. Run it

```bash
# Inside the ros2_runtime container
colcon build --packages-select robot sensors
source /ros2_ws/install/setup.bash

# main.py target — set to:  RUN_TARGET = "lidar_cup_pick_place_test"
# then launch as usual:
ros2 launch robot robot.launch.py
```

The vision launch must include the marshmallow class (already configured).
The lidar node (`rplidar_c1_node`) is in the default launch.

## 11. Files touched

| File | Change |
|---|---|
| `_manipulator_config.py` | Added `CARDBOARD_HEIGHT_MM`, `STACK_*CUP_HEIGHT_MM`, `MARSHMALLOW_FULL_HEIGHT_MM`, `MALLOW_Z_STACK_*_MM`, `snap_to_venue_tier_mm`, `TURNTABLE_X_IN_BODY_MM` |
| `robot_impl/sensors.py` | Added `get_lidar_obstacles_robot()` to expose the raw robot-frame point cloud |
| `tests/scripts/lidar_cup_pick_place_test.py` | New — the full FSM and helpers |
| `MANIPULATOR_TASKS.md` | This document |
