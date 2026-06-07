"""
Shared hardware constants for all manipulator test scripts.
Edit channel/pin assignments here — all tests import from this file.
"""
from __future__ import annotations

from robot.arm_kinematics import ArmGeometry
from robot.hardware_map import Motor, ServoChannel, Stepper

# ── Turntable ─────────────────────────────────────────────────────────────────
TURNTABLE_STEPPER       = Stepper.STEPPER_2
MOTOR_STEPS_PER_REV     = 200               # native steps (1.8°/step)
MICROSTEP               = 8                 # 1/8 microstep — M0=HIGH, M1=HIGH, M2=LOW on DRV8825
PULLEY_RATIO            = 4.0               # GT2 belt: 4 motor revs per 1 turntable rev
TURNTABLE_MAX_VELOCITY  = 2800              # steps/sec at motor shaft (~156°/sec at turntable) — validated no-load max is 3200; margin for belt+turntable inertia
TURNTABLE_ACCELERATION  = 1000             # steps/sec²
# Angle convention (CCW positive, looking down from above):
#   -90° = robot's right side   (max CW — cable hard limit)
#     0° = forward
#   +90° = robot's left side
#  +180° = straight backward = stow position (max CCW — cable hard limit)
# Step counts increase with angle, so ABSOLUTE moves naturally spin CW when
# leaving stow (step count decreases) and CCW when returning to stow (increases).
# Never command outside [TURNTABLE_MIN_DEG, TURNTABLE_MAX_DEG].
TURNTABLE_MIN_DEG       = -90.0            # rightmost allowed position (CW hard limit)
TURNTABLE_MAX_DEG       = 180.0            # rearmost allowed position — also stow (CCW hard limit)
TURNTABLE_SCAN_ARC_DEG  = 90.0            # half-width of front scanning zone (targets within ±90°)

# Derived — do not edit
STEPS_PER_TURNTABLE_DEG = (MOTOR_STEPS_PER_REV * MICROSTEP * PULLEY_RATIO) / 360.0

def turntable_deg_to_steps(degrees: float) -> int:
    return round(degrees * STEPS_PER_TURNTABLE_DEG)

# ── Shoulder ──────────────────────────────────────────────────────────────────
SHOULDER_CHANNEL     = ServoChannel.CH_16
SHOULDER_STOW_DEG    = 90.0                 # safe resting position
SHOULDER_UP_DEG      = 60.0                 # raised for reach
SHOULDER_SAFE_MIN    = 75.0                 # physical mechanical limit — shoulder
                                            # cannot rotate past this. Do not lower.
SHOULDER_SAFE_MAX    = 180.0

# ── Elbow ─────────────────────────────────────────────────────────────────────
ELBOW_CHANNEL        = ServoChannel.CH_15
ELBOW_STOW_DEG       = 90.0                 # folded/retracted
ELBOW_EXTEND_DEG     = 45.0                 # extended toward target
ELBOW_SAFE_MIN       = 5.0                  # was 20 — same pulse-mapping rationale
ELBOW_SAFE_MAX       = 160.0

# ── Gripper ───────────────────────────────────────────────────────────────────
GRIPPER_CHANNEL      = ServoChannel.CH_14
GRIPPER_OPEN_DEG     = 29.0                 # fully open — approach position before grab
GRIPPER_GRAB_DEG     = 58.0                 # hold position for marshmallow (tightened +8° from 50)
GRIPPER_CLOSE_DEG    = 90.0                 # fully closed hard stop
GRIPPER_ROAST_DEG    = 70.0                 # slightly open during roasting (≈⅔ from open)

# ── Heating wire ──────────────────────────────────────────────────────────────
# Controlled via 1-channel 5V optical relay module (active LOW, Handson MDU1150).
# Wiring: remove VCC↔JD-VCC jumper; JD-VCC→5V, VCC→3.3V, GND→GND, IN1→GPIO pin below.
# Relay output: heating wire between COM and NO terminals.
HEATING_WIRE_GPIO_PIN = 17     # TODO: confirm BCM GPIO pin wired to relay IN1
# Relay is wired to DC motor output DC_M3 on the NUEVO PCB (confirm with firmware).
# Active LOW relay: PWM 255 = relay OFF (line held high), PWM 0 = relay ON (line pulled low).
# TODO: verify correct DC channel and polarity before powering heating wire.
HEATING_WIRE_MOTOR_ID    = Motor.DC_M3   # TODO: confirm DC channel on NUEVO PCB
HEATING_WIRE_PWM_ON      = 255           # TODO: active LOW relay — confirm correct value
HEATING_WIRE_PWM_OFF     = 0            # relay inactive

# ── Camera pan stepper (Stepper 3) ────────────────────────────────────────────
# Motor: 28BYJ-48 rewired as bipolar (center tap removed).
# Internal gear reduction is 64:1, giving 2048 full-step pulses per output revolution.
# No microstepping required — the gear reduction already provides ~0.176°/step resolution.
# Three scan positions at ±45° with 62° HFOV gives 17° frame overlap (no blind spots).
CAMPAN_STEPPER          = Stepper.STEPPER_3
CAMPAN_STEPS_PER_REV    = 2048             # 28BYJ-48 bipolar full-step: 32 motor steps × 64:1 internal gear
CAMPAN_MICROSTEP        = 1               # no microstepping — DRV8825 M0/M1/M2 not connected on this channel
CAMPAN_PULLEY_RATIO     = 1.0             # direct drive — no belt or pulley
CAMPAN_MAX_VELOCITY     = 200             # steps/sec (output shaft) — conservative for 28BYJ-48 low torque
CAMPAN_ACCELERATION     = 100             # steps/sec² (output shaft)
CAMPAN_POSITIONS_DEG    = [-66.0, 0.0, 66.0]   # left → center → right (CCW negative = left)
CAMPAN_SETTLE_S         = 0.3             # seconds to wait after panning before capturing

STEPS_PER_CAMPAN_DEG = (CAMPAN_STEPS_PER_REV * CAMPAN_MICROSTEP * CAMPAN_PULLEY_RATIO) / 360.0

def campan_deg_to_steps(degrees: float) -> int:
    return round(degrees * STEPS_PER_CAMPAN_DEG)

# ── Ultrasonic sensor ──────────────────────────────────────────────────────────
# Mounted on the forearm. Measure from the physical robot and replace these values.
ULTRASONIC_FOREARM_OFFSET_MM = 215.0    # TODO: distance from elbow pivot to sensor along forearm
ULTRASONIC_HEIGHT_OFFSET_MM  = 29.0    # TODO: sensor height above forearm centerline (+ = above)

# ── Arm geometry (measure from physical robot, all in mm) ─────────────────────
ARM_L1_MM               = 155.0   # upper arm: shoulder pivot → elbow pivot
ARM_L2_MM               = 298.0    # forearm:   elbow pivot   → gripper end where grabbing happens
ARM_SHOULDER_HEIGHT_MM  = 95.0    # shoulder pivot height above robot base plate (measured)
ARM_SHOULDER_OFFSET_MM  = 14.0    # forward distance from turntable axis to shoulder pivot (~1 inch — measure)

# Distance from turntable rotation axis to the front face of the robot chassis (mm).
ROBOT_FRONT_TO_TURNTABLE_MM = 67.0

# How far the camera/campan axis sits forward of the front chassis face (mm).
# camera_forward_offset_mm = ROBOT_FRONT_TO_TURNTABLE_MM + CAMERA_PROTRUSION_MM.
# Used by detection_to_robot_frame() to translate camera distances into
# turntable-axis coordinates for IK.  A marshmallow 6 in (152 mm) from the
# camera is (152 + CAMERA_FORWARD_OFFSET_MM) from the turntable axis.
CAMERA_PROTRUSION_MM        = 142.0
CAMERA_FORWARD_OFFSET_MM    = ROBOT_FRONT_TO_TURNTABLE_MM + CAMERA_PROTRUSION_MM

# Servo calibration — run arm_ik_calibration_test.py to determine these values.
# shoulder_servo_offset: servo angle (deg) when upper arm is horizontal (geometric 0°)
# shoulder_servo_sign:   +1 if higher servo angle raises the arm, -1 if it lowers it
# elbow_servo_offset:    servo angle (deg) when arm is fully straight (geometric 180°)
# elbow_servo_sign:      +1 if higher servo angle opens the elbow
SHOULDER_SERVO_OFFSET   = 96.0     # servo angle when upper arm is horizontal forward
SHOULDER_SERVO_SIGN     = 1.0      # higher servo angle raises the arm
ELBOW_SERVO_OFFSET      = 90.0     # servo angle when forearm is collinear with upper arm (fully straight)
ELBOW_SERVO_SIGN        = 1.0      # higher servo angle = forearm angled UP (elbow-up).
                                   # Confirmed empirically: under sign=-1 + elbow_up=True the IK
                                   # output servo 53.3° produced elbow-DOWN physically, so the
                                   # true hardware mapping is the opposite of MANIPULATOR.md's
                                   # documented sign=-1 convention. Servo 126.7° = elbow-up.

# Assembled geometry object — import this in programs and tests that use IK.
ARM_GEOMETRY = ArmGeometry(
    L1                       = ARM_L1_MM,
    L2                       = ARM_L2_MM,
    shoulder_height_mm       = ARM_SHOULDER_HEIGHT_MM,
    shoulder_offset_mm       = ARM_SHOULDER_OFFSET_MM,
    shoulder_servo_offset    = SHOULDER_SERVO_OFFSET,
    shoulder_servo_sign      = SHOULDER_SERVO_SIGN,
    elbow_servo_offset       = ELBOW_SERVO_OFFSET,
    elbow_servo_sign         = ELBOW_SERVO_SIGN,
    camera_forward_offset_mm = CAMERA_FORWARD_OFFSET_MM,
)

# ── Servo motion ─────────────────────────────────────────────────────────────
ARM_SERVO_STEP_DEG   = 3.0      # degrees per step for incremental servo moves
ARM_SERVO_STEP_DWELL = 0.06     # seconds between steps

# ── Arm search and carry poses (servo degrees — tune after calibration) ────────
# Search: arm extended into the detection zone so the camera can see targets
ARM_SEARCH_SHOULDER_DEG  = 110.0   # shoulder servo while vision is scanning
ARM_SEARCH_ELBOW_DEG     = 90.0    # elbow servo while vision is scanning (arm near horizontal)
# Carry: safe retracted pose for rotating turntable between pick and place
ARM_CARRY_SHOULDER_DEG   = 100.0
ARM_CARRY_ELBOW_DEG      = 90.0
# Ranging: forearm-mounted ultrasonic + camera tilted down at the cup stack.
# Shoulder 120 sits at the boundary of the "any-elbow-safe" zone — at this or
# higher shoulder the elbow can swing through its entire 0–180° physical range
# without colliding with the plate or sensor housing at any turntable angle.
# Elbow 4 tilts the forearm ~62° below horizontal — enough to keep the US
# sensor *behind* the cup centre line at the 6"+cup-radius (=185 mm) mallow
# zone so the beam sweeps outward through every cup at ±90° instead of
# overshooting them.  Verified in sim_arm_kinematics.py: every cup across
# the ±90° workspace returns a 55–193 mm US distance at this pose.
ARM_RANGING_SHOULDER_DEG = 120.0
ARM_RANGING_ELBOW_DEG    =   4.0

# ── Hardware envelope (used for collision-aware safe-range checks) ──────────
# Base plate: 11"×11" square at z = 0, turntable axis is centered LR, sits
# ROBOT_FRONT_TO_TURNTABLE_MM behind the plate's front edge.  Treated as a
# solid sheet — anything inside the plate XY footprint with z < 0 collides.
BASE_PLATE_SIZE_MM         = 11.0 * 25.4   # 279.4 mm
BASE_PLATE_FRONT_X_MM      = ROBOT_FRONT_TO_TURNTABLE_MM  # front edge in robot x

# Front sensor housing (lidar + campan + camera body) — a 70 mm wide stub
# from the plate front out to the camera lens.  Top is FLUSH with the plate
# (z = 0); extends downward toward the floor.  The arm must stay above z = 0
# over this footprint just like over the plate.
FRONT_EXT_WIDTH_MM         = 70.0
FRONT_EXT_LENGTH_MM        = CAMERA_PROTRUSION_MM    # 142 mm
SENSORS_Z_TOP_MM           =    0.0
SENSORS_Z_BOTTOM_MM        = -120.0   # extends past the camera at z=-40

# Dynamic safe-elbow rule of thumb (from sim_arm_kinematics.py):
#   shoulder ≥ 120°  → ANY elbow in [0, 180] safe, at ANY turntable angle.
#   shoulder = 105°  → elbow safe roughly [10, 180]; recommend keep ≥ 30 margin.
#   shoulder = 90°   → elbow safe roughly [32, 180].
# See MANIPULATOR.md §5 for the full table.  These are advisory; the
# authoritative check is `tools/sim_arm_kinematics.py arm_collides()`.

# ── Turntable home offset ─────────────────────────────────────────────────────
# Firmware homing: step_home() sets position 0 at stow (LIM1 trigger = 180°).
# Direction convention: CW = positive firmware steps, CCW = negative.
# Formula: firmware_steps = turntable_deg_to_steps(home_offset - target_deg)
#   After firmware homing (stow = step 0): offset = +180  → forward(0°) = +3200 steps CW ✓
#   Manual alignment at forward (forward = step 0): offset = 0   → forward(0°) = 0 steps ✓
# NEVER use target+offset — that sends moves back into the limit switch.
TURNTABLE_HOME_OFFSET_DEG = 169.0    # limit switch triggers 11° short of full stow (measured 2026-06-07; was 171° but post-home turntable overshot 0° by 2° CW)

# ── Target geometry (measure from competition venue, all mm, robot frame) ─────
# Robot frame: x = forward, y = left, z = up; origin = turntable axis at base plate.
#
# Ground level: the competition floor sits 229 mm BELOW the robot base plate.
# All absolute z heights must be expressed relative to the base plate (z = 0).
# Floor surface = GROUND_Z_MM = -229 mm.  Objects on the floor have z < 0.
GROUND_Z_MM              = -247.0  # floor surface in robot frame (measured)

MARSHMALLOW_HEIGHT_MM    = 80.0     # fallback height — runtime uses vision estimate
MARSHMALLOW_DISTANCE_MM  = 200.0   # TODO: measure horizontal distance from turntable axis
# Plate is always 3 in (76.2 mm) forward of the robot front face.
# Robot front to turntable axis = ROBOT_FRONT_TO_TURNTABLE_MM = 101.6 mm.
# Plate center x = 76.2 + 101.6 = 177.8 mm from turntable axis.
PLATE_X_MM               = 177.8   # 3 in forward of robot front (76.2 mm) + turntable offset (101.6 mm)
PLATE_Y_MM               = 0.0     # plate is directly forward
PLATE_Z_MM               = GROUND_Z_MM + 15.0  # floor + graham cracker (~10 mm) + small clearance

# ── Marshmallow place position (where the gripper releases after picking) ────
# Tuned in sim_arm_kinematics.py: turntable ≈ -60.5°, shoulder ≈ 98°, elbow ≈ 15°
# (just below the elbow safe-min of 20° — verify on hardware before running).
PLACE_X_MM               =  240.0   # forward of turntable axis (mm)
PLACE_Y_MM               = -226.0   # right of turntable axis (negative = right)
PLACE_Z_MM               = -201.0   # ≈ 28 mm above floor (1.1" above ground)

# ── Camera field of view ──────────────────────────────────────────────────────
# Raspberry Pi Camera Module v2 with default lens: ~62° HFOV.
CAMERA_HFOV_DEG          = 62.0    # TODO: verify for your specific lens

# Height of the camera lens above the robot base plate (mm).
# Used to convert bounding-box elevation angle + distance into target height.
# Measure: hold a ruler from the base plate up to the camera lens centerline.
CAMERA_HEIGHT_MM         = -40.0

# Physical diameter of a standard large marshmallow (~1.5 in / 38 mm).
# Used in the pinhole distance estimate: dist_mm = (MARSHMALLOW_DIAMETER_MM * focal_px) / px_diameter
# Measure your actual marshmallow and update if needed.
MARSHMALLOW_DIAMETER_MM  = 25.0
MARSHMALLOW_RADIUS_MM    = MARSHMALLOW_DIAMETER_MM / 2.0  # center-height above support surface

# ── Red Solo cup height tiers ──────────────────────────────────────────────────
# Standard red Solo cup: ~94mm tall. Marshmallow sits on top → center at +19mm above rim.
# All heights are in robot frame (z relative to base plate). Cups sit on the floor
# (GROUND_Z_MM = -229 mm), so all tiers are negative in the robot frame.
# TODO: measure actual cup height and marshmallow diameter for your specific cups/marshmallows.
CUP_SINGLE_HEIGHT_MM     = 94.0     # height of one Solo cup (mm)
# Marshmallow centre z in robot frame (base plate = 0, floor = GROUND_Z_MM):
MALLOW_HEIGHT_0_CUPS_MM  = GROUND_Z_MM + MARSHMALLOW_RADIUS_MM                            # floor + radius  ≈ -210 mm
MALLOW_HEIGHT_1_CUP_MM   = GROUND_Z_MM + CUP_SINGLE_HEIGHT_MM + MARSHMALLOW_RADIUS_MM    # 1 cup  ≈ -116 mm
MALLOW_HEIGHT_2_CUPS_MM  = GROUND_Z_MM + 2 * CUP_SINGLE_HEIGHT_MM + MARSHMALLOW_RADIUS_MM # 2 cups ≈  -22 mm
MALLOW_HEIGHT_3_CUPS_MM  = GROUND_Z_MM + 3 * CUP_SINGLE_HEIGHT_MM + MARSHMALLOW_RADIUS_MM # 3 cups ≈  +72 mm

_CUP_TIER_HEIGHTS_MM = (
    MALLOW_HEIGHT_0_CUPS_MM,
    MALLOW_HEIGHT_1_CUP_MM,
    MALLOW_HEIGHT_2_CUPS_MM,
    MALLOW_HEIGHT_3_CUPS_MM,
)

def snap_to_cup_tier_mm(height_mm: float) -> float:
    """Round a noisy height estimate to the nearest known cup-tier height."""
    return min(_CUP_TIER_HEIGHTS_MM, key=lambda h: abs(h - height_mm))

# ── Competition cup-stack tiers (measured at venue) ───────────────────────────
# The competition uses three fixed stacks of red Solo cups, each on a 3 mm
# cardboard pad. Heights are measured from the cardboard top to the top of the
# cup stack. Verified 2026-06-07.
CARDBOARD_HEIGHT_MM       = 3.0
STACK_8CUP_HEIGHT_MM      = 162.0   # 8-cup stack height above cardboard
STACK_18CUP_HEIGHT_MM     = 225.0   # 18-cup stack height above cardboard
STACK_24CUP_HEIGHT_MM     = 263.0   # 24-cup stack height above cardboard

# Mallow is a cylinder (vertical axis) ~28 mm tall — pickup centre is half-height.
MARSHMALLOW_FULL_HEIGHT_MM = 28.0
_MALLOW_CENTER_OFFSET_MM   = MARSHMALLOW_FULL_HEIGHT_MM / 2.0   # 14 mm

# Mallow-centre z in robot frame (floor = GROUND_Z_MM = -229 mm):
#   floor + cardboard + stack_height + half_mallow
MALLOW_Z_STACK_8_MM  = GROUND_Z_MM + CARDBOARD_HEIGHT_MM + STACK_8CUP_HEIGHT_MM  + _MALLOW_CENTER_OFFSET_MM  # ≈ -50 mm
MALLOW_Z_STACK_18_MM = GROUND_Z_MM + CARDBOARD_HEIGHT_MM + STACK_18CUP_HEIGHT_MM + _MALLOW_CENTER_OFFSET_MM  # ≈ +13 mm
MALLOW_Z_STACK_24_MM = GROUND_Z_MM + CARDBOARD_HEIGHT_MM + STACK_24CUP_HEIGHT_MM + _MALLOW_CENTER_OFFSET_MM  # ≈ +51 mm

_VENUE_TIER_HEIGHTS_MM = (
    MALLOW_Z_STACK_8_MM,
    MALLOW_Z_STACK_18_MM,
    MALLOW_Z_STACK_24_MM,
)

def snap_to_venue_tier_mm(height_mm: float) -> float:
    """Round a noisy height estimate to the nearest competition-tier height."""
    return min(_VENUE_TIER_HEIGHTS_MM, key=lambda h: abs(h - height_mm))

# ── Lidar → turntable-frame conversion ────────────────────────────────────────
# LIDAR_MOUNT_X_MM in hardware_map.py gives the lidar position in BODY frame
# (origin = wheel midpoint). Manipulator IK uses TURNTABLE frame (origin =
# turntable axis). Need the body-frame x of the turntable axis to convert.
# TODO: measure on robot — distance from wheel midpoint to turntable axis along
# forward direction (positive = turntable is forward of wheel midpoint).
TURNTABLE_X_IN_BODY_MM   = 320.7    # approximation: assumes turntable axis aligned with lidar mount

# ── Detection ─────────────────────────────────────────────────────────────────
MARSHMALLOW_CLASS        = "marshmallow"      # class name from vision model
CUP_CLASS                = "red_cup"          # class name from rule_based_detection.detect_red_cup

# Per-class confidence thresholds — tune independently during vision testing.
MIN_CONFIDENCE               = 0.60           # legacy default used by detection_vision_test
MIN_CONFIDENCE_MARSHMALLOW   = 0.60
MIN_CONFIDENCE_ROASTING_STICK= 0.65
MIN_CONFIDENCE_PLATE         = 0.70
MIN_CONFIDENCE_CUP           = 0.50

# Cup geometry — used for pinhole distance estimate and cup-mallow bearing match.
CUP_DIAMETER_MM              = 65.0           # top rim outer diameter of a standard red Solo cup (mm)
MALLOW_CUP_BEARING_MATCH_DEG = 10.0           # max bearing separation to pair a mallow with a cup

# ── Competition sequence timing ───────────────────────────────────────────────
STOP_SIGN_DWELL_S    = 3.0    # pause at stop sign before manipulator sequence begins
ENCLOSURE_WAIT_S     = 10.0   # wait after stop for team to remove competition enclosure
SCAN_TIMEOUT_S       = 15.0   # give up scanning if no marshmallow found in this time
ROAST_TIME_S         = 5.0    # seconds to run heating wire

VISION_WAIT_S        = 5.0    # seconds to wait for first detection (legacy)
