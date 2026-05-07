"""
arm_kinematics.py
=================
Forward and inverse kinematics for the 3-DOF manipulator arm.

Arm structure (base to tip):
  1. Turntable  — stepper, rotates in the horizontal plane
  2. Shoulder   — servo, rotates in the arm's vertical plane
  3. Elbow      — servo, rotates in the arm's vertical plane
  4. Gripper    — servo, open/close only (no effect on position)

Coordinate system (robot frame):
  +x = forward
  +y = left
  +z = up
  origin = turntable rotation axis at the robot base plate

Geometric angle conventions (used internally by this module):
  shoulder_geo:  angle above horizontal of the upper arm.
                 0° = upper arm pointing forward horizontally
                 90° = upper arm pointing straight up
  elbow_geo:     opening angle at the elbow joint.
                 180° = arm fully straight
                 0° = forearm folded back against upper arm

Servo angles are different from geometric angles because servos are
physically mounted at various orientations. Use ArmGeometry.to_servo()
to convert, and calibrate the offsets in _manipulator_config.py.

Usage example (in a program or test):
    from robot.arm_kinematics import ArmGeometry, inverse_kinematics, OutOfReachError
    from robot.tests.scripts._manipulator_config import ARM_GEOMETRY

    try:
        turntable_deg, shoulder_deg, elbow_deg = inverse_kinematics(
            x_mm=200.0, y_mm=50.0, z_mm=100.0, geom=ARM_GEOMETRY
        )
        robot.set_servo(SHOULDER_CHANNEL, shoulder_deg)
        robot.set_servo(ELBOW_CHANNEL, elbow_deg)
        # turntable_deg → steps via turntable_deg_to_steps()
    except OutOfReachError as e:
        print(f"Target unreachable: {e}")
"""
from __future__ import annotations

import math
from dataclasses import dataclass


class OutOfReachError(ValueError):
    """Raised when a target position is outside the arm's workspace."""


@dataclass
class ArmGeometry:
    """
    Physical measurements and servo calibration for the arm.
    All lengths in millimetres. Measure from the physical robot.

    Fields:
        L1                  upper arm length — shoulder pivot to elbow pivot
        L2                  forearm length   — elbow pivot to gripper centre
        shoulder_height_mm  height of shoulder pivot above the robot base plate
        shoulder_offset_mm  forward distance from turntable axis to shoulder pivot
                            (the "slight offset" in the design — positive = forward)

        shoulder_servo_offset   servo angle (deg) that corresponds to the arm
                                pointing horizontally forward (geometric 0°).
                                If the servo reads 90° when the arm is horizontal,
                                this value is 90.0.
        shoulder_servo_sign     +1 if increasing servo angle raises the arm,
                                -1 if increasing servo angle lowers the arm.

        elbow_servo_offset      servo angle (deg) that corresponds to the arm
                                being fully straight (geometric 180°).
        elbow_servo_sign        +1 if increasing servo angle opens the elbow,
                                -1 if increasing servo angle closes the elbow.

    Calibration procedure:
        1. Set shoulder servo to 90°. Measure geometric angle of upper arm from
           horizontal. shoulder_servo_offset = 90 - measured_geometric_angle.
        2. Increase servo angle by 10°. If arm rises, shoulder_servo_sign = +1.
        3. Repeat for elbow with arm fully extended.
    """
    L1: float
    L2: float
    shoulder_height_mm: float
    shoulder_offset_mm: float
    shoulder_servo_offset: float = 90.0
    shoulder_servo_sign: float = 1.0
    elbow_servo_offset: float = 90.0
    elbow_servo_sign: float = 1.0

    def shoulder_geo_to_servo(self, geo_deg: float) -> float:
        """Convert shoulder geometric angle (deg above horizontal) to servo angle."""
        return self.shoulder_servo_offset + self.shoulder_servo_sign * geo_deg

    def elbow_geo_to_servo(self, geo_deg: float) -> float:
        """Convert elbow geometric angle (deg, 180=straight) to servo angle."""
        # Geometric elbow is measured from fully-straight (180°). Servo offset is
        # defined relative to that same fully-straight position.
        return self.elbow_servo_offset + self.elbow_servo_sign * (geo_deg - 180.0)

    def shoulder_servo_to_geo(self, servo_deg: float) -> float:
        return (servo_deg - self.shoulder_servo_offset) / self.shoulder_servo_sign

    def elbow_servo_to_geo(self, servo_deg: float) -> float:
        return 180.0 + (servo_deg - self.elbow_servo_offset) / self.elbow_servo_sign

    @property
    def max_reach_mm(self) -> float:
        """Maximum horizontal reach from the shoulder pivot."""
        return self.L1 + self.L2

    @property
    def min_reach_mm(self) -> float:
        """Minimum horizontal reach (arm folded back on itself)."""
        return abs(self.L1 - self.L2)


# ── Core kinematics ────────────────────────────────────────────────────────────

def inverse_kinematics(
    x_mm: float,
    y_mm: float,
    z_mm: float,
    geom: ArmGeometry,
) -> tuple[float, float, float]:
    """
    Compute joint angles to place the gripper at (x_mm, y_mm, z_mm) in robot frame.

    Returns:
        (turntable_deg, shoulder_servo_deg, elbow_servo_deg)

    turntable_deg is a rotation angle — convert to stepper steps with
    turntable_deg_to_steps() from _manipulator_config.py.

    Raises OutOfReachError if the target is outside the arm workspace.
    """
    # ── Step 1: turntable azimuth ─────────────────────────────────────────────
    # Point the arm plane toward the target in the horizontal plane.
    turntable_deg = math.degrees(math.atan2(y_mm, x_mm))

    # ── Step 2: target coordinates in the arm's vertical plane ───────────────
    # Horizontal distance from turntable axis to target, then subtract the
    # forward offset of the shoulder pivot from the turntable axis.
    horizontal_dist = math.hypot(x_mm, y_mm)
    reach = horizontal_dist - geom.shoulder_offset_mm

    # Height relative to the shoulder pivot (not the base plate).
    height = z_mm - geom.shoulder_height_mm

    # ── Step 3: 2-link planar IK for shoulder and elbow ──────────────────────
    d_sq = reach ** 2 + height ** 2
    d = math.sqrt(d_sq)

    if d > geom.L1 + geom.L2:
        raise OutOfReachError(
            f"Target at ({x_mm:.0f}, {y_mm:.0f}, {z_mm:.0f}) mm is "
            f"{d:.0f} mm from shoulder pivot — max reach is {geom.L1 + geom.L2:.0f} mm."
        )
    if d < abs(geom.L1 - geom.L2):
        raise OutOfReachError(
            f"Target at ({x_mm:.0f}, {y_mm:.0f}, {z_mm:.0f}) mm is too close "
            f"({d:.0f} mm) — min reach is {abs(geom.L1 - geom.L2):.0f} mm."
        )

    cos_elbow_geo = (d_sq - geom.L1 ** 2 - geom.L2 ** 2) / (2.0 * geom.L1 * geom.L2)
    cos_elbow_geo = max(-1.0, min(1.0, cos_elbow_geo))  # clamp floating point noise
    # elbow_geo convention: 180° = straight, 0° = fully folded (supplement of the
    # raw law-of-cosines angle, which is 0 when straight).
    elbow_geo = 180.0 - math.degrees(math.acos(cos_elbow_geo))

    # Shoulder geometric angle (above horizontal).
    # Because elbow_geo = 180 - q2_standard, sin(elbow_geo)=sin(q2) and
    # cos(elbow_geo)=-cos(q2), so the standard beta denominator L1+L2*cos(q2)
    # becomes L1 - L2*cos(elbow_geo).
    alpha = math.atan2(height, reach)
    beta = math.atan2(
        geom.L2 * math.sin(math.radians(elbow_geo)),
        geom.L1 - geom.L2 * math.cos(math.radians(elbow_geo)),
    )
    shoulder_geo = math.degrees(alpha - beta)

    # ── Step 4: convert geometric angles to servo commands ───────────────────
    shoulder_servo = geom.shoulder_geo_to_servo(shoulder_geo)
    elbow_servo = geom.elbow_geo_to_servo(elbow_geo)

    return turntable_deg, shoulder_servo, elbow_servo


def forward_kinematics(
    turntable_deg: float,
    shoulder_servo_deg: float,
    elbow_servo_deg: float,
    geom: ArmGeometry,
) -> tuple[float, float, float]:
    """
    Compute gripper position (x_mm, y_mm, z_mm) from joint angles.
    Inverse of inverse_kinematics — useful for verifying IK solutions and
    checking servo angle calibration.
    """
    shoulder_geo = geom.shoulder_servo_to_geo(shoulder_servo_deg)
    elbow_geo    = geom.elbow_servo_to_geo(elbow_servo_deg)

    turntable_rad = math.radians(turntable_deg)
    shoulder_rad  = math.radians(shoulder_geo)
    elbow_rad     = math.radians(elbow_geo)

    # Position of elbow pivot relative to shoulder pivot (arm's vertical plane)
    elbow_reach  = geom.L1 * math.cos(shoulder_rad)
    elbow_height = geom.L1 * math.sin(shoulder_rad)

    # Forearm absolute angle from horizontal.
    # q2_standard (fold from straight) = 180° - elbow_geo, so:
    # forearm_angle = shoulder_geo + q2_standard = shoulder_geo + (180° - elbow_geo)
    # In radians: shoulder_rad + (pi - elbow_rad)
    forearm_rad = shoulder_rad + (math.pi - elbow_rad)
    gripper_reach  = elbow_reach  + geom.L2 * math.cos(forearm_rad)
    gripper_height = elbow_height + geom.L2 * math.sin(forearm_rad)

    # Apply shoulder horizontal offset and project into robot frame
    total_reach = geom.shoulder_offset_mm + gripper_reach
    x_mm = total_reach * math.cos(turntable_rad)
    y_mm = total_reach * math.sin(turntable_rad)
    z_mm = geom.shoulder_height_mm + gripper_height

    return x_mm, y_mm, z_mm


def is_reachable(x_mm: float, y_mm: float, z_mm: float, geom: ArmGeometry) -> bool:
    """Return True if the target is within the arm's workspace."""
    try:
        inverse_kinematics(x_mm, y_mm, z_mm, geom)
        return True
    except OutOfReachError:
        return False


# ── Detection frame conversion ────────────────────────────────────────────────

def detection_to_robot_frame(
    distance_mm: float,
    bearing_rad: float,
    height_mm: float,
) -> tuple[float, float, float]:
    """
    Convert a detection result to robot-frame (x, y, z) coordinates.

    Args:
        distance_mm   horizontal distance from robot to target (from sensor)
        bearing_rad   angle from robot forward axis, CCW positive (from sensor)
        height_mm     height of target above robot base plate (from sensor or known)

    Returns:
        (x_mm, y_mm, z_mm) in robot frame, ready to pass to inverse_kinematics()
    """
    x_mm = distance_mm * math.cos(bearing_rad)
    y_mm = distance_mm * math.sin(bearing_rad)
    return x_mm, y_mm, height_mm
