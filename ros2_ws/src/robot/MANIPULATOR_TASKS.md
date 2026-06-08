# NUEVO Manipulator Task Plan — Competition Demo

How the robot autonomously locates, picks, and places the marshmallow during
the manipulation portion of the demo. Companion to `MANIPULATOR.md`
(geometry / kinematics reference); this doc covers the *task strategy* —
which sensor produces which coordinate, how those coordinates are derived,
and the FSM that uses them.

The implementation lives in
`robot/tests/scripts/lidar_cup_pick_place_test.py`.

---

## 0. Current Status (handoff snapshot — 2026-06-08)

What works:
* **Purple-mallow vision mask is live.** `rule_based_detection.detect_marshmallow`
  now uses a blue-violet HSV range (`H 120-160, S 60+, V 50+`) and labels
  detections `color=purple`. Verified on saved frames with the debug tool
  `ros2_ws/src/robot/tools/debug_mallow_hsv.py` — a 1280×720 snapshot of the
  competition mallow returns a single contour at conf ≈ 0.94. Run that script
  against any `/runtime_output/mallow_detection_range/scan_*.jpg` to retune
  the HSV range without rebuilding ROS.
* **Cup-gating relaxed for short stacks.** `filter_marshmallows_on_red_cups`
  used to drop every mallow detection when no red cup was in frame. It now
  passes mallows through unfiltered when no cup is detected (needed for the
  9-cup stack where the cup falls below the bottom of frame), but still
  gates on cup position when a cup *is* visible.
* **Single-frame test:** `mallow_detection_range_test.py` has been
  simplified to one forward capture (no campan sweep). Includes a
  vision-warm-up poll on `robot.get_detection_image_size()` so the test
  no longer races the vision node's first inference.

What does NOT work yet / needs attention:
* **LiDAR is mounted upside-down.** The mount needs to be flipped before
  the demo. Until that's done, lidar y is mirrored — which means
  `robot.get_lidar_obstacles_robot()` reports targets on the wrong side of
  the robot. **This will also corrupt nav-time obstacle avoidance**, not
  just the manipulator's cup-stack mapping — anything reading lidar in body
  or turntable frame will see obstacles flipped left↔right. Either rotate
  the lidar physically or add a y-negation in `sensors.py` /
  `get_lidar_obstacles_robot()` before relying on the data.
* **No camera-tilt term in the height kinematics.** The campan camera is
  pitched downward by ~10° at rest. `_height_from_bbox_mm` assumes a
  horizontal optical axis — every tier-z is biased low by
  `dist · tan(tilt)`. Either re-level the mount or add a
  `CAMERA_PITCH_DEG` constant to `_manipulator_config.py` and account for
  it in the elevation calc.
* **Lidar/camera test timing on the simplified test:** the no-campan test
  needs the warm-up poll added today to avoid a race. Don't remove it.

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

The competition setup is **four** fixed cup stacks (9, 11, 14, 16 cups) —
chosen because they are the four largest stacks that still fit entirely
inside the camera frame at the typical mallow-detection distance. One of
them has a marshmallow on top. The marshmallow is dyed **purple** before
the run to give the camera a high-contrast target; the rule-based detector
matches a blue-violet HSV range, not the original white range.

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
measured 2026-06-08:

| Stack   | Stack height above cardboard | Top-of-stack z in robot frame                  | Mallow-centre z in robot frame |
|---------|------------------------------|------------------------------------------------|--------------------------------|
| 9 cups  | 168 mm | `floor + 3 + 168 = 171 mm`  → `z = −76 mm` | `−76 + 14 = −62 mm` |
| 11 cups | 180 mm | `floor + 3 + 180 = 183 mm`  → `z = −64 mm` | `−64 + 14 = −50 mm` |
| 14 cups | 200 mm | `floor + 3 + 200 = 203 mm`  → `z = −44 mm` | `−44 + 14 = −30 mm` |
| 16 cups | 214 mm | `floor + 3 + 214 = 217 mm`  → `z = −30 mm` | `−30 + 14 = −16 mm` |

Robot frame: floor is at `GROUND_Z_MM = −247 mm` (base plate = 0). A 28 mm
tall marshmallow has its centre 14 mm above the surface it sits on.

These are encoded in `_manipulator_config.py` as

```python
MALLOW_Z_STACK_9_MM  ≈ -62
MALLOW_Z_STACK_11_MM ≈ -50
MALLOW_Z_STACK_14_MM ≈ -30
MALLOW_Z_STACK_16_MM ≈ -16
```

and the helper `snap_to_venue_tier_mm(z)` rounds a noisy height estimate
to the nearest of those four values.

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

> **⚠ Lidar mount is currently upside-down.** Until the physical mount is
> rotated (or a y-negation is added in `robot_impl/sensors.py`), the y
> coordinates returned by `get_lidar_obstacles_robot()` are mirrored. The
> cup-stack mapper will see the left-most stack on the right and vice
> versa — and **nav-time obstacle avoidance will be affected** too,
> because every consumer of the lidar topic shares this frame. Flip the
> mount before trusting any lidar-derived x/y.

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

* **Four-stack assumption.** The mapper now expects 4 cup stacks. If it
  finds zero it aborts. If it finds more than 4, the extras are scanned
  anyway — the highest-confidence vision detection still wins. This is
  safe. Adjust `CUP_MIN_POINTS_PER_STACK` if the lidar is fragmenting one
  stack into two clusters.
* **Tier-snap can be wrong by one tier** if the camera height estimate is
  noisy enough to land between two tiers. With the new 4-tier set the
  spacing is **tighter** (~12-20 mm centre-to-centre vs 63 mm in the old
  3-tier set), so a one-tier error is more likely than before. The
  camera-tilt bias documented in §0 makes this worse — fix that first.
  If it's still noisy, add a sanity-check using the cluster's z-projection
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
| `_manipulator_config.py` | Added `CARDBOARD_HEIGHT_MM`, `STACK_*CUP_HEIGHT_MM`, `MARSHMALLOW_FULL_HEIGHT_MM`, `MALLOW_Z_STACK_*_MM`, `snap_to_venue_tier_mm`, `TURNTABLE_X_IN_BODY_MM`.  Stack sizes were 7/10/13/18 (then 8/18/24 earlier); current set is **9/11/14/16 cups** with measured heights 168/180/200/214 mm. |
| `robot_impl/sensors.py` | Added `get_lidar_obstacles_robot()` to expose the raw robot-frame point cloud |
| `tests/scripts/lidar_cup_pick_place_test.py` | New — the full FSM and helpers |
| `tests/scripts/mallow_detection_range_test.py` | Simplified to a single forward capture (no campan sweep); added a vision-warm-up poll so detections aren't read before the node publishes its first frame |
| `vision/vision/rule_based_detection.py` | Swapped white HSV mask → purple (H 120-160 S 60+ V 50+), changed `color` attribute to "purple", made `filter_marshmallows_on_red_cups` pass mallows through when no red cup is in frame |
| `tools/debug_mallow_hsv.py` | New host/container script — runs the live detector against a saved snapshot, dumps mask images, and reports HSV stats so you can retune the purple range without rebuilding ROS |
| `MANIPULATOR_TASKS.md` | This document |
