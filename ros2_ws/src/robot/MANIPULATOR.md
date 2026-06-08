# NUEVO Manipulator Reference

Working notes for the 3-DOF pick/place arm — geometry, IK conventions, joint
limits, hardware-collision envelope, ranging pose, and the place coordinate
that the competition demo uses.

The numbers here mirror `_manipulator_config.py`.  Keep both in sync when
you recalibrate.

---

## 1. Arm geometry

| Constant | Value | Notes |
|---|---|---|
| `ARM_L1_MM` | 156 mm | upper arm: shoulder pivot → elbow pivot |
| `ARM_L2_MM` | 315 mm | forearm:   elbow pivot → gripper end |
| `ARM_SHOULDER_HEIGHT_MM` | 95 mm | shoulder pivot above base plate (z=0) |
| `ARM_SHOULDER_OFFSET_MM` | 14 mm | shoulder pivot forward of turntable axis |
| Max reach from shoulder | 471 mm | `L1 + L2` |
| Max horizontal reach from turntable axis | ~485 mm at shoulder height |

The shoulder pivot orbits the turntable axis at 14 mm radius — accounted for
inside `forward_kinematics` and `inverse_kinematics` automatically.

## 2. Chassis / base plate / sensor envelope

| Constant | Value | Notes |
|---|---|---|
| `BASE_PLATE_SIZE_MM` | 279.4 mm (= 11 in) | 11"×11" gray plate at z = 0 |
| `BASE_PLATE_FRONT_MM` | 67 mm | front edge of plate, +x of turntable axis |
| `FRONT_EXT_WIDTH_MM` | 70 mm | width of front sensor housing (centered LR) |
| `FRONT_EXT_LENGTH_MM` | 142 mm | sensor housing extends to the camera lens |
| `CAMERA_FORWARD_OFFSET_MM` | 209 mm | camera lens = front of robot + 142 |
| `CAMERA_HEIGHT_MM` | −40 mm | camera lens is 40 mm BELOW the base plate |

The sensor housing (lidar + campan + camera) is **flush with the plate top
at z = 0** and extends downward.  Treat the entire footprint as solid below
z = 0 — the arm must stay above z = 0 over the plate AND the sensor housing.

## 3. IK branch selection (IMPORTANT)

A 2-link planar IK has two solutions for any reachable point.  The NUEVO
elbow hinge folds in only one direction, so we always need the same branch:

* **Branch "elbow up"** (our arm): upper arm angled UP from shoulder, forearm
  angled DOWN to the target.  Elbow joint sits ABOVE the straight line from
  shoulder pivot to gripper.
* **Branch "elbow down"** (NOT achievable on our arm): upper arm DOWN, elbow
  below the shoulder→gripper line.  The physical elbow can't fold this way.

`inverse_kinematics()` defaults to `elbow_up=True`.  All test scripts and
programs (turntable_pick_place_test, manual_ik_pick_place_test,
campan_detect_ik_test, manipulator_demo, final_demo, etc.) automatically pick
up this default.  The wrong branch was the cause of "shoulder out of range"
results for legitimate pick targets — the math was solving for the
unachievable elbow direction.

## 4. Joint conventions

### Turntable

```
-90° = robot right (CW limit)
  0° = forward
+90° = robot left
+180° = stow (CCW limit, also home position)
```

Step convention: `firmware_steps = turntable_deg_to_steps(home_offset - target_deg)`.
After firmware homing, stow = step 0 = +180°.  Forward (0°) lives at +3200 steps.

### Shoulder

* `shoulder_geo`:  0° = upper arm horizontal forward; +90° = straight up.
* `shoulder_servo = SHOULDER_SERVO_OFFSET + SHOULDER_SERVO_SIGN × shoulder_geo`
* With our calibration (offset = 96, sign = +1):
  - servo = 90  → geo = −6°  (stow, near horizontal)
  - servo = 135 → geo = +39° (raised, used for ranging)
  - servo = 180 → geo = +84° (close to straight up)

### Elbow

* `elbow_geo`:  180° = fully straight; 0° = forearm folded against upper arm.
* `elbow_servo = ELBOW_SERVO_OFFSET + ELBOW_SERVO_SIGN × (elbow_geo − 180)`
* With our calibration (offset = 90, sign = −1):  `servo = 270 − elbow_geo`
  - servo = 0   → geo = 270° (forearm rotated 90° past straight, elbow-up direction)
  - servo = 90  → geo = 180° (straight)
  - servo = 180 → geo = 90°  (half folded the other way)

## 5. Safe joint limits (advisory)

The flat servo limits from `_manipulator_config.py` are conservative
baselines.  The **real** constraint on the elbow is collision with the
plate + sensor housing, which depends on shoulder angle.

| Limit | Value |
|---|---|
| `SHOULDER_SAFE_MIN/MAX` | 75 / 180 |
| `ELBOW_SAFE_MIN/MAX` | 20 / 160 |
| Physical servo range | 0 to 180 (both) |

### Dynamic safe elbow range (collision-driven)

Measured from `sim_arm_kinematics.py` with sensor housing top at z = 0:

| Shoulder servo | Safe elbow range (servo deg) |
|---|---|
| 75 | [70, 180] |
| 90 | [32, 180] |
| 100 | [16, 180] |
| 105 | [10, 180]  — "ideal" floor ~30 leaves ~20° margin |
| 110 | [4, 180] |
| **120 +** | **[0, 180] — full physical range, any turntable** |
| 135 | [0, 180]  |

Rule of thumb: **at shoulder ≥ 120°, any elbow / any turntable is safe.**
Below 120° you need to keep the elbow high enough that the forearm doesn't
dip into the sensor housing or below the plate edge.

## 6. Ranging pose

Used by the RANGING state in `turntable_pick_place_test.py` to point the
forearm-mounted ultrasonic + the camera at the cup stack.

| Constant | Value |
|---|---|
| `RANGING_SHOULDER_DEG` | 120 (geo +24°, upper arm up) |
| `RANGING_ELBOW_DEG`    | 4   (geo 266°, forearm angled ~62° below horizontal) |

Tuned for the **6"+cup-radius (=185 mm) mallow zone** — cup centres on a
semicircle around the camera at 185 mm.  Shoulder 120 is the lower edge of
the "any-elbow-safe" band, so the elbow can swing through its full physical
range without hitting the plate or sensor housing.  Elbow 4 angles the
forearm down enough that the US sensor sits behind the cup centre line at
all bearings — the beam sweeps outward through every cup instead of
overshooting them.  Verified in `sim_arm_kinematics.py`: across the entire
±90° workspace the US returns valid cup distances in the 55–193 mm range
with no hardware collisions.

**Why not the older sh=130 / el=8?**  That pose was tuned for an 11"
(=280 mm) cup zone.  When cups move in to 185 mm, the US sensor sits at
~309 mm radial (further out than the side cups at ~259 mm), so the beam
points *away* from those cups and misses 4 of 7 bearings.  The forearm tip
also brushes through the side cups — collision risk.

### Distance refinement — ultrasonic first, camera fallback

The RANGING state now tries two independent distance estimates and uses
whichever returns a valid reading first.

**1. Ultrasonic (preferred)** — sensor on the forearm, beam along the
forearm direction.  `d_raw` is the 3-D distance from the sensor to the cup's
NEAR wall.  Since the marshmallow sits on the cup's centre axis (not the
wall), we project past the near wall by one cup radius along the horizontal
forearm direction:

```
sensor_horiz = shoulder_offset + L1·cos(sh_geo)
             + ULTRASONIC_FOREARM_OFFSET_MM · cos(forearm_rad)

mallow_reach = sensor_horiz
             + d_raw · cos(forearm_rad)        ← horiz to cup wall
             + CUP_DIAMETER_MM / 2             ← + cup radius to cup centre
```

This back-calculates the radial horizontal distance from the turntable axis
to the cup CENTRE (= mallow xy centre).  Mallow z is the cup-tier height
already snapped during SCANNING.

The geometric assumption is that the beam is aimed at the cup centre, which
holds once the turntable has rotated to the target bearing.

**2. Cup-detection (fallback)** — pinhole formula on the cup bbox:

```
cup_dist = (CUP_DIAMETER_MM × focal_px) / cup_bbox_pixel_width
```

The bbox spans the cup's full outer diameter, so `cup_dist` directly gives
the distance from camera to the cup centre — no offset correction needed.
Used when US is out of range, returns None, or errors.

Both fall back to the SCANNING coarse estimate if neither refinement works.

**Why US first?**  Untested as of this writing — the rationale is that the
ultrasonic gives a direct line-of-sight depth read whereas the camera
formula depends on bbox accuracy and the calibrated focal length.  Verify
on hardware which one tracks ground truth more closely; the order is a
one-line swap in `turntable_pick_place_test.py` if you decide the camera
estimate is more reliable.

Caveat for the US path: the sensor is mounted **above** the forearm
centreline (29 mm) and points along the forearm — it hits the cup wall a
bit higher than centre.  Should be a small effect since the cup is a tall
vertical cylinder, but worth verifying on hardware.

## 7. Marshmallow workspace

Mallow centers live on the **11" semicircle around the camera**.

* Camera centre is at x = 209 mm from the turntable axis.
* Mallow at bearing α from camera: `(209 + 279.4·cos α, 279.4·sin α, z)`
* Heights: 6"–12" above the floor → `z ∈ [−76.6, +75.8]` mm in robot frame.

### Reach reality check

Even within the 11" semicircle, **directly forward (bearing 0°) is out of
arm reach**: the mallow lands at x = 488 mm, distance from shoulder pivot
~521 mm > 471 mm max.  Only bearings ≳ 45° away from straight-forward
are reachable.  Plan placements accordingly, or rotate the robot so the
mallow falls into the reachable arc.

## 8. Place position (constant, validated)

| Constant | Value |
|---|---|
| `PLACE_X_MM` | 128 |
| `PLACE_Y_MM` | −226 |
| `PLACE_Z_MM` | −201  (= 1" above floor) |

IK (elbow_up=True): turntable = −60.5°, shoulder = 98°, elbow = 14°.
The elbow is below the static `ELBOW_SAFE_MIN = 20`, but the collision
check shows the pose is clear because the gripper goes off to the right
side (y = −226 is well outside the plate footprint at ±139.7).  This is a
**known-good special case** — the static safe-range warning is advisory
and can be ignored for this exact pose.

## 9. Demo state machine (turntable_pick_place_test)

```
INIT → IDLE → SAFE_RAISE → ARM_HOME → SCANNING
       → RANGING → APPROACHING → PICKING
       → CARRY_TO_PLACE → PLACING → RESTOW → DONE
```

* **SAFE_RAISE** moves the shoulder UP first, then the elbow — never reverse
  this order or the forearm swings into the chassis.
* **SAFE retract before any turntable rotation**: same shoulder-first rule
  is applied by `_safe_arm_retract` before every `_turntable_to_deg`.
* **APPROACHING** uses the IK result from RANGING directly.  Now that IK
  picks the elbow_up branch by default, these angles match what the
  physical arm can actually execute.

---

## 10. Tools

* **`tools/sim_arm_kinematics.py`** — interactive 3D simulator.  Drag joint
  sliders for FK, target sliders for IK; click preset buttons to snap to
  Place / Plate / Mallow 6"/9"/12".  Shows live collision check, dynamic
  safe-elbow range, and the dynamic IK round-trip.
* **`tools/sim_obstacle_avoidance.py`** — pure-pursuit avoidance tuner.
  Separate scope (drive-time obstacle avoidance), but lives next to the
  arm sim because the workflow is the same: tweak constants in Mac sim,
  copy to test config.

## 11. Key files

| File | Purpose |
|---|---|
| `robot/arm_kinematics.py` | FK / IK (now defaults `elbow_up=True`) |
| `robot/tests/scripts/_manipulator_config.py` | All geometry + servo constants |
| `robot/tests/scripts/turntable_pick_place_test.py` | Competition pick/place demo |
| `robot/tests/scripts/manual_ik_pick_place_test.py` | Hand-pick (x, y, z) IK test |
| `tools/sim_arm_kinematics.py` | Mac simulator (no ROS2 needed) |
