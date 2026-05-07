"""
Shared hardware constants for all manipulator test scripts.
Edit channel/pin assignments here — all tests import from this file.
"""
from __future__ import annotations

from robot.arm_kinematics import ArmGeometry
from robot.hardware_map import Limit, ServoChannel, Stepper

# ── Turntable ─────────────────────────────────────────────────────────────────
TURNTABLE_STEPPER       = Stepper.STEPPER_1
TURNTABLE_HOME_LIMIT    = Limit.LIM_1       # limit switch at the 0° hard stop
MOTOR_STEPS_PER_REV     = 200               # native steps (1.8°/step)
MICROSTEP               = 16                # confirm against firmware StepConfig
PULLEY_RATIO            = 4.0               # GT2 belt: 4 motor revs per 1 turntable rev
TURNTABLE_MAX_VELOCITY  = 2000              # steps/sec at motor shaft
TURNTABLE_ACCELERATION  = 1000             # steps/sec² at motor shaft
TURNTABLE_SAFE_ARC_DEG  = 180.0            # usable sweep (front half-circle)

# Derived — do not edit
STEPS_PER_TURNTABLE_DEG = (MOTOR_STEPS_PER_REV * MICROSTEP * PULLEY_RATIO) / 360.0

def turntable_deg_to_steps(degrees: float) -> int:
    return round(degrees * STEPS_PER_TURNTABLE_DEG)

# ── Shoulder ──────────────────────────────────────────────────────────────────
SHOULDER_CHANNEL     = ServoChannel.CH_1    # TODO: confirm wiring
SHOULDER_STOW_DEG    = 90.0                 # safe resting position
SHOULDER_UP_DEG      = 60.0                 # raised for reach
SHOULDER_SAFE_MIN    = 30.0                 # hard mechanical limit — do not exceed
SHOULDER_SAFE_MAX    = 150.0

# ── Elbow ─────────────────────────────────────────────────────────────────────
ELBOW_CHANNEL        = ServoChannel.CH_2    # TODO: confirm wiring
ELBOW_STOW_DEG       = 90.0                 # folded/retracted
ELBOW_EXTEND_DEG     = 45.0                 # extended toward target
ELBOW_SAFE_MIN       = 20.0
ELBOW_SAFE_MAX       = 160.0

# ── Gripper ───────────────────────────────────────────────────────────────────
GRIPPER_CHANNEL      = ServoChannel.CH_3
GRIPPER_OPEN_DEG     = 30.0
GRIPPER_CLOSE_DEG    = 110.0
GRIPPER_ROAST_DEG    = 50.0                 # slightly open during roasting

# ── Heating wire ──────────────────────────────────────────────────────────────
# TODO: confirm whether this is a DC motor channel in PWM mode or a GPIO relay.
# Current assumption: DC_M3 in PWM mode through set_motor_pwm().
HEATING_WIRE_MOTOR_ID = 3                   # Motor.DC_M3
HEATING_WIRE_PWM_ON   = 64                  # −255 to 255 scale — start very low, increase only after verifying temp
HEATING_WIRE_PWM_OFF  = 0

# ── Arm geometry (measure from physical robot, all in mm) ─────────────────────
# TODO: replace placeholder values with physical measurements before using IK.
ARM_L1_MM               = 150.0    # upper arm: shoulder pivot → elbow pivot
ARM_L2_MM               = 130.0    # forearm:   elbow pivot   → gripper center
ARM_SHOULDER_HEIGHT_MM  = 120.0    # shoulder pivot height above robot base plate
ARM_SHOULDER_OFFSET_MM  = 20.0     # forward distance from turntable axis to shoulder pivot

# Servo calibration — run arm_ik_calibration_test.py to determine these values.
# shoulder_servo_offset: servo angle (deg) when upper arm is horizontal (geometric 0°)
# shoulder_servo_sign:   +1 if higher servo angle raises the arm, -1 if it lowers it
# elbow_servo_offset:    servo angle (deg) when arm is fully straight (geometric 180°)
# elbow_servo_sign:      +1 if higher servo angle opens the elbow
SHOULDER_SERVO_OFFSET   = 90.0     # TODO: calibrate
SHOULDER_SERVO_SIGN     = 1.0      # TODO: confirm direction
ELBOW_SERVO_OFFSET      = 90.0     # TODO: calibrate
ELBOW_SERVO_SIGN        = 1.0      # TODO: confirm direction

# Assembled geometry object — import this in programs and tests that use IK.
ARM_GEOMETRY = ArmGeometry(
    L1                  = ARM_L1_MM,
    L2                  = ARM_L2_MM,
    shoulder_height_mm  = ARM_SHOULDER_HEIGHT_MM,
    shoulder_offset_mm  = ARM_SHOULDER_OFFSET_MM,
    shoulder_servo_offset = SHOULDER_SERVO_OFFSET,
    shoulder_servo_sign   = SHOULDER_SERVO_SIGN,
    elbow_servo_offset    = ELBOW_SERVO_OFFSET,
    elbow_servo_sign      = ELBOW_SERVO_SIGN,
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

# ── Turntable home offset ─────────────────────────────────────────────────────
# After step_home(), position 0 = limit switch hard stop.
# This offset is the angle from that hard stop to robot forward direction.
# Set during turntable homing calibration.
TURNTABLE_HOME_OFFSET_DEG = 90.0   # TODO: measure after first homing run

# ── Target geometry (measure from competition venue, all mm, robot frame) ─────
# Robot frame: x = forward, y = left, z = up; origin = turntable axis at base plate.
MARSHMALLOW_HEIGHT_MM    = 80.0    # TODO: measure z of marshmallow above base plate
MARSHMALLOW_DISTANCE_MM  = 200.0   # TODO: measure horizontal distance from turntable axis
PLATE_X_MM               = 150.0   # TODO: measure plate center position
PLATE_Y_MM               = 0.0     # plate is directly forward
PLATE_Z_MM               = 30.0    # TODO: measure plate height above base plate

# ── Camera field of view ──────────────────────────────────────────────────────
# Horizontal bearing from bounding-box pixel centre → turntable target angle.
# Raspberry Pi Camera Module v2 with default lens: ~62° HFOV.
CAMERA_HFOV_DEG          = 62.0    # TODO: verify for your specific lens

# ── Detection ─────────────────────────────────────────────────────────────────
MARSHMALLOW_CLASS        = "marshmallow"      # class name from vision model
ROASTING_STICK_CLASS     = "roasting_stick"   # TODO: confirm class name with vision team
PLATE_CLASS              = "plate"            # TODO: confirm class name with vision team

# Per-class confidence thresholds — tune independently during vision testing.
MIN_CONFIDENCE               = 0.60           # legacy default used by detection_vision_test
MIN_CONFIDENCE_MARSHMALLOW   = 0.60
MIN_CONFIDENCE_ROASTING_STICK= 0.65
MIN_CONFIDENCE_PLATE         = 0.70

# ── Competition sequence timing ───────────────────────────────────────────────
STOP_SIGN_DWELL_S    = 3.0    # pause at stop sign before manipulator sequence begins
ENCLOSURE_WAIT_S     = 10.0   # wait after stop for team to remove competition enclosure
SCAN_TIMEOUT_S       = 15.0   # give up scanning if no marshmallow found in this time
ROAST_TIME_S         = 5.0    # seconds to run heating wire

VISION_WAIT_S        = 5.0    # seconds to wait for first detection (legacy)
