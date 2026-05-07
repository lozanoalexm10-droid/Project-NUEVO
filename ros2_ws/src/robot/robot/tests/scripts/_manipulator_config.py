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

# ── Detection ─────────────────────────────────────────────────────────────────
MARSHMALLOW_CLASS    = "marshmallow"        # class name from vision model
MIN_CONFIDENCE       = 0.60                 # minimum detection confidence to act on
VISION_WAIT_S        = 5.0                  # seconds to wait for first detection
