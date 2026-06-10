# NUEVO Manipulator Task Plan — Competition Demo

How the robot autonomously locates, picks, and places the marshmallow during
the manipulation portion of the demo. Companion to `MANIPULATOR.md`
(geometry / kinematics reference); this doc covers the *task strategy* —
how each pick coordinate is derived, and the FSM that uses them.

The implementation lives in
`robot/tests/scripts/hardcoded_cup_pick_place_test.py`.

---

## 0. Current Status (handoff snapshot — 2026-06-10)

**Strategy change.** We do not have a sensor that gives us an accurate cup
position. Distance estimation from the pinhole camera was too noisy for a
30 mm gripper opening, and we are no longer using LiDAR or ultrasonic in the
manipulator path. The demo path is now:

* **Cup XY positions are hardcoded** — measured by hand, baked into
  `hardcoded_cup_pick_place_test.HARDCODED_CUPS_CAMERA_MM`.
* **Cup heights are picked from a known set by camera ranking** — the four
  stacks have known centre-of-mallow heights (9/11/14/16-cup stacks); the
  camera ranks them by apparent bbox-top height and maps that ordering onto
  the sorted known set. Absolute camera height is biased low by the tilted
  optical axis, but the *order* survives because every stack gets the same
  bias.
* **Marshmallow target selection is by camera** — the purple HSV detector
  picks the stack whose pan-window contains the highest-confidence
  marshmallow.
* **Only other sensing used during the manipulator task is the turntable
  limit switch** (LIM_1) for homing.

What works:
* **Purple-mallow vision mask is live.** `rule_based_detection.detect_marshmallow`
  uses a blue-violet HSV range (`H 120-160, S 60+, V 50+`) and labels
  detections `color=purple`. Verified on saved frames with the debug tool
  `ros2_ws/src/robot/tools/debug_mallow_hsv.py` — a 1280×720 snapshot of the
  competition mallow returns a single contour at conf ≈ 0.94. Run that script
  against any `/runtime_output/mallow_detection_range/scan_*.jpg` to retune
  the HSV range without rebuilding ROS.
* **Cup-gating relaxed for short stacks.** `filter_marshmallows_on_red_cups`
  passes mallows through unfiltered when no red cup is detected (needed for
  the 9-cup stack where the cup falls below the bottom of frame), but still
  gates on cup position when a cup *is* visible.
* **Manual IK pick** (`manual_ik_pick_place_test.py`) — full SAFE_RAISE →
  IK → squeeze → place sequence works end-to-end against a hardcoded mallow
  position. Used to verify the IK + grip mechanics independently of vision.
* **Hardcoded cup pick** (`hardcoded_cup_pick_place_test.py`, the active
  script) — adds the four-cup camera scan, height ranking, and
  marshmallow-on-top selection on top of the manual-IK mechanics.

What still needs attention before the demo:
* **End-to-end run on real hardware.** The hardcoded-cup script needs a
  clean dry run in the venue with the actual stack layout. Re-measure the
  four `HARDCODED_CUPS_CAMERA_MM` entries against the competition table
  before the run.
* **Camera-tilt bias.** The campan camera pitches downward by ~10° at rest.
  Absolute height estimates are biased low by `dist · tan(tilt)`. The
  rank-based assignment is robust to this bias (every cup is biased the
  same way), so it is not a blocker — but if you ever need the *absolute*
  bbox height (e.g. for the fallback path), add a `CAMERA_PITCH_DEG`
  constant to `_manipulator_config.py` and account for it in
  `_height_from_bbox_mm`.

---

## 1. What the robot has to figure out

Three pieces of information define a successful pick:

| Coordinate | Meaning | Source |
|---|---|---|
| `pick_x_mm`, `pick_y_mm` | Mallow horizontal position in robot frame (turntable axis = origin) | **Hardcoded** — measured by hand per cup |
| `pick_z_mm`              | Mallow centre height above robot base plate | **Camera** — apparent height *rank* maps onto the known set of four mallow-centre heights |

The IK solver then turns `(pick_x, pick_y, pick_z)` into the turntable,
shoulder-servo, and elbow-servo angles needed to put the gripper there.
The IK math is in `arm_kinematics.inverse_kinematics()` — verified by the
manipulator simulator and known good as long as the calibration constants
in `_manipulator_config.py` are correct.

The camera also answers the categorical "which of the four stacks has the
marshmallow on it" question via the purple-mallow detector.

## 2. Why hardcoded XY and ranked Z

The competition setup is **four** fixed cup stacks (9, 11, 14, 16 cups) —
chosen because they are the four largest stacks that still fit entirely
inside the camera frame at the typical mallow-detection distance. One of
them has a marshmallow on top. The marshmallow is dyed **purple** before
the run to give the camera a high-contrast target; the rule-based detector
matches a blue-violet HSV range, not the original white range.

We tried sensor-based XY localisation and it did not give us a 30 mm-gripper-quality
position. The camera's pinhole-bbox distance estimate is too noisy. So:

* **XY is hardcoded.** Each of the four cup positions is measured by hand
  and stored in `HARDCODED_CUPS_CAMERA_MM` as `(front_mm, side_mm)` in
  camera frame. Conversion to turntable frame at scan time:
  `x = front + CAMERA_FORWARD_OFFSET_MM, y = -side`.
* **Z is rank-assigned.** The camera measures the bbox top edge of each
  cup at its hardcoded pan angle, sorts the four cups by apparent height,
  and maps that ordering onto the sorted known mallow-centre heights
  (`MALLOW_Z_STACK_{9,11,14,16}_MM`). Relative ranking is robust to the
  camera-tilt bias documented in §0 — every cup is biased the same way, so
  the *order* survives even when the absolute height estimate is off.
* **Camera also picks the target.** During the same scan, the highest-
  confidence purple-mallow detection inside the per-cup pan window wins.

The only other sensor used during the manipulator task is the **turntable
limit switch** (`Limit.LIM_1`), which establishes a known zero for the
turntable stepper. Everything else (shoulder, elbow, gripper) is open-loop
servo angle commands.

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
to the nearest of those four values — used only by the fallback path when
rank-assignment cannot run.

## 4. Hardcoded cup positions and height ranking

Implemented in `hardcoded_cup_pick_place_test._scan_cups` and
`_assign_heights_by_rank`.

```
1. The four cup positions are baked into HARDCODED_CUPS_CAMERA_MM as
   (front_mm, side_mm) in camera frame. _build_cups() converts them to
   turntable-frame {id, x_mm, y_mm} dicts.
2. For each cup, compute the campan bearing that points the camera at the
   cup centre and clamp to ±PICK_CAMPAN_MAX_DEG (66°).
3. Pan the campan, wait PICK_CAMPAN_SETTLE_S, then sample
   robot.get_detections() SCAN_SAMPLES (=6) times at SCAN_SAMPLE_DT (=0.2 s)
   intervals — the first read after a pan often returns stale cache.
4. Keep the best-centred red-cup detection (smallest bearing offset from
   the expected in-frame position, within ±SCAN_CENTER_TOLERANCE_DEG = 4°).
   Record its bbox-top z as cup["top_mm"].
5. Keep the highest-confidence marshmallow detection inside the same
   ±4° window. Record confidence and the detection dict.
6. After all four cups are scanned, sort by top_mm ascending and assign
   each cup the corresponding mallow-centre z from
   sorted(MALLOW_Z_STACK_{9,11,14,16}_MM) — shortest-measured gets the
   9-cup z, tallest-measured gets the 16-cup z.
7. The target cup is the one with the highest marshmallow confidence.
8. Pick coordinates: pick_x = cup.x_mm, pick_y = cup.y_mm,
   pick_z = cup.z_mm + PICK_Z_LIFT_MM (+12 mm; the arm sags under load).
```

If the height scan is incomplete (any cup missing its `top_mm`), the
fallback `_mallow_fallback_z_mm` snaps the target cup's own measured
height (cup top, else marshmallow bbox) to the nearest known tier via
`snap_to_venue_tier_mm`.

Tunables live at the top of the test file:
`HARDCODED_CUPS_CAMERA_MM`, `KNOWN_MALLOW_Z_MM_SORTED`, `PICK_Z_LIFT_MM`,
`SCAN_SAMPLES`, `SCAN_SAMPLE_DT`, `SCAN_CENTER_TOLERANCE_DEG`,
`PICK_CAMPAN_SETTLE_S`, `PICK_CAMPAN_MAX_DEG`.

## 5. Vision target identification

Folded into the same scan loop as height ranking — see §4 step 5.

The chosen stack is the one with the highest-confidence marshmallow
detection inside its per-cup ±4° pan window during the multi-sample read.
The pan-window narrowness is what stops adjacent stacks from bleeding into
each other (the four cups are ~15–25° apart in bearing).

If the camera misses the mallow on all four cups, the FSM goes to RESTOW
(graceful abort) rather than guessing.

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

The motion sequence in SAFE_RAISE and APPROACHING:

1. **SAFE_RAISE** — tight-fold the elbow first (to 25°) so the arm's COM
   sits over the shoulder pivot, then lift the shoulder to SAFE_SHOULDER_DEG
   (110°), then unfold the elbow back to SAFE_ELBOW_DEG (70°). The COM
   move first is critical — lifting a flat-out arm overdraws the shoulder
   servo.
2. Rotate the turntable to the target bearing.
3. Open the gripper to `GRIPPER_OPEN_DEG` (29°).
4. Drive the shoulder to `sh_servo`, then the elbow to `el_servo`, in that
   order.

PICKING squeezes the gripper to `SQUEEZE_DEG` (≈ 84° — 90 % of the way
between OPEN and CLOSE) and holds it via `_hold_grip` (re-asserts every
0.4 s because the firmware relaxes idle actuators). CARRY_TO_PLACE
safe-retracts (keeping the squeeze), rotates the turntable to
`PLACE_TURNTABLE_DEG = −80°`, and runs IK to the fixed place target
`(PLACE_X_MM, PLACE_Y_MM, PLACE_Z_MM)` from `_manipulator_config.py`.

## 7. State machine

```
INIT             — reset E-STOP, enable vision
IDLE             — 3-second start countdown
SAFE_RAISE       — tight-fold elbow → lift shoulder → unfold elbow → open gripper
ARM_HOME         — enable steppers, home turntable on LIM_1, centre campan,
                   park turntable at 0°
VISION_FIND_CUP  — pan to each of the 4 hardcoded cup bearings; rank
                   measured cup-top heights → known mallow-z tiers; pick
                   the cup with the highest-confidence marshmallow detection
IK_COMPUTE       — rotate turntable to pick bearing; run inverse_kinematics
APPROACHING     — open gripper, drive shoulder then elbow to pick servos;
                   FK sanity-check vs target
PICKING          — squeeze gripper, hold
CARRY_TO_PLACE   — safe-retract with squeeze held, rotate turntable to
                   PLACE_TURNTABLE_DEG, IK to place target
PLACING          — drive shoulder + elbow to place IK, open gripper
RESTOW           — safe-retract, return turntable to 0°, recentre campan,
                   disable steppers
DONE             — green LED, robot.stop()
```

## 8. Known limitations and acceptable failures

* **Hardcoded XY assumes the table doesn't move.** Re-measure
  `HARDCODED_CUPS_CAMERA_MM` against the actual competition table before
  the run. A 10 mm shift is the gripper's miss margin.
* **Rank-assignment requires all four cups to be detected.** If the
  scan misses a cup top, `_assign_heights_by_rank` returns False and the
  code falls back to snapping the target cup's own height to the nearest
  known tier. That's a one-cup-only height estimate and is more sensitive
  to the camera-tilt bias — try to scan cleanly.
* **Tier-snap can be wrong by one tier** when the fallback path runs and
  the bias puts a measurement between two tiers. The new 4-tier set
  spacing is ~12-20 mm centre-to-centre vs 63 mm in the old 3-tier set.
* **No closed-loop feedback during the pick.** Once IK is computed, the
  arm moves to the commanded angles open-loop. If the IK calibration is
  off, the gripper misses. The fix is calibration (servo offsets, link
  lengths) — see `elbow_calibration_test.py`.
* **Limit switch is the only zeroing reference.** If LIM_1 fails to
  trigger during ARM_HOME, the homing code prints a warning and falls
  back to `home_offset = 0°`, which will mis-aim every subsequent
  turntable command. Watch the home log line on startup.
* **No re-scan if a stack moves mid-pick.** Don't bump the table while the
  FSM is running.

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
* **+10 bonus — special solution beyond TA's experience:** the
  rank-based height assignment is the angle to lead with. The pitch:
  "we can't measure cup heights to gripper precision from a pinhole
  camera, but we can rank them; we know the four stack sizes ahead of
  time, so ranking + sorted known heights gives us a tier-accurate z
  that is robust to camera-tilt bias because every cup is biased the
  same way." That plus the purple-HSV marshmallow detector sells the
  +10 tier without overstating the system.

## 10. Run it

```bash
# Inside the ros2_runtime container
colcon build --packages-select robot sensors
source /ros2_ws/install/setup.bash

# main.py target — already set to:
#   from robot.tests.scripts.hardcoded_cup_pick_place_test import run
# then launch as usual:
ros2 launch robot robot.launch.py
```

The vision launch must include the marshmallow class (already configured).
No LiDAR node is needed — the manipulator path does not consume any
LiDAR or ultrasonic data.

To dry-run the camera scan without picking, set `PICK_SCAN_ONLY=1` in the
environment — the FSM will pan to each cup, save annotated frames to
`/runtime_output/vision/`, log per-cup detections, and exit.

## 11. Files touched

| File | Change |
|---|---|
| `_manipulator_config.py` | Added `CARDBOARD_HEIGHT_MM`, `STACK_*CUP_HEIGHT_MM`, `MARSHMALLOW_FULL_HEIGHT_MM`, `MALLOW_Z_STACK_*_MM`, `snap_to_venue_tier_mm`. Stack sizes were 7/10/13/18 (then 8/18/24 earlier); current set is **9/11/14/16 cups** with measured heights 168/180/200/214 mm. |
| `tests/scripts/hardcoded_cup_pick_place_test.py` | **Active demo script.** Full FSM with hardcoded cup XY + camera-ranked Z + marshmallow target selection. |
| `tests/scripts/_cup_scan_logic.py` | Helpers shared by the cup-scan code: `assign_heights_by_rank`, `cup_top_height_mm`, `bbox_center_height_mm`, `detection_center_bearing_deg`, `camera_bearing_to_cup`. |
| `tests/scripts/manual_ik_pick_place_test.py` | Standalone IK verification — hardcoded mallow position, cosmetic camera pan, same SAFE_RAISE + squeeze mechanics as the active script. |
| `tests/scripts/mallow_detection_range_test.py` | Single forward capture with a vision-warm-up poll so detections aren't read before the node publishes its first frame. |
| `vision/vision/rule_based_detection.py` | Swapped white HSV mask → purple (H 120-160 S 60+ V 50+), changed `color` attribute to "purple", made `filter_marshmallows_on_red_cups` pass mallows through when no red cup is in frame. |
| `tools/debug_mallow_hsv.py` | Host/container script — runs the live detector against a saved snapshot, dumps mask images, and reports HSV stats so you can retune the purple range without rebuilding ROS. |
| `MANIPULATOR_TASKS.md` | This document. |
