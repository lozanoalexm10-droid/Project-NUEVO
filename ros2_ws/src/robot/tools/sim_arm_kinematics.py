#!/usr/bin/env python3
"""
sim_arm_kinematics.py
=====================
Interactive 3D simulator for the NUEVO 3-DOF arm.

Uses the same arm_kinematics.inverse_kinematics / forward_kinematics that the
robot uses, with the geometry values mirrored from _manipulator_config.py.
No ROS2 needed — runs on Mac/Linux with matplotlib only.

HOW TO USE
----------
  python3 ros2_ws/src/robot/tools/sim_arm_kinematics.py

  Drag any JOINT slider  → FK mode (arm moves, target XYZ updates to show gripper).
  Drag any TARGET slider → IK mode (arm computes joint angles to reach target).
  Click a preset button  → IK to that preset position (Plate, Mallow tiers).
  Drag the 3D plot       → rotate the camera view.
  Reset button           → return to stow pose (turntable 0, shoulder/elbow 90).

Reading the plot:
  Gold disk      — robot base plate at z = 0
  Gray plane     — floor at z = -229 mm
  Vertical line  — turntable rotation axis
  Blue link      — upper arm (L1)
  Orange link    — forearm  (L2)
  Cyan marker    — camera lens position
  Red dots       — preset target positions
  Green sphere   — gripper end (turns red if target unreachable)
  Wireframe arc  — maximum reach hemisphere from shoulder pivot

INFO PANEL
----------
  Mode               which sliders are driving (FK or IK).
  Turntable          joint angle in degrees (azimuth from forward).
  Shoulder           geometric angle (0 = horizontal, 90 = up) and the servo
                     angle the firmware actually receives.
  Elbow              geometric angle (180 = straight, 0 = folded) and the
                     servo angle the firmware sends.
  Gripper xyz        end-effector position in robot frame (mm).
  Reach              horizontal distance from turntable axis.
  Status             OK or OUT OF REACH with the limiting distance.
"""
from __future__ import annotations

import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ── Make the robot package importable from Mac (no ROS2 needed) ───────────────
_PKG_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from robot.arm_kinematics import (  # noqa: E402
    ArmGeometry, OutOfReachError, forward_kinematics, inverse_kinematics,
)

# ─────────────────────────────────────────────────────────────────────────────
# Geometry (mirror of _manipulator_config.py — keep in sync)
# ─────────────────────────────────────────────────────────────────────────────
ARM_L1_MM              = 156.0
ARM_L2_MM              = 315.0
ARM_SHOULDER_HEIGHT_MM = 95.0
ARM_SHOULDER_OFFSET_MM = 14.0

SHOULDER_SERVO_OFFSET  = 96.0
SHOULDER_SERVO_SIGN    = 1.0
ELBOW_SERVO_OFFSET     = 90.0
ELBOW_SERVO_SIGN       = -1.0

ROBOT_FRONT_TO_TURNTABLE_MM = 67.0
CAMERA_PROTRUSION_MM        = 142.0
CAMERA_FORWARD_OFFSET_MM    = ROBOT_FRONT_TO_TURNTABLE_MM + CAMERA_PROTRUSION_MM
CAMERA_HEIGHT_MM            = -40.0

GROUND_Z_MM = -229.0

PLATE_X_MM = 177.8
PLATE_Y_MM = 0.0
PLATE_Z_MM = GROUND_Z_MM + 15.0

CUP_SINGLE_HEIGHT_MM   = 94.0
MARSHMALLOW_RADIUS_MM  = 12.5

MALLOW_Z = [
    GROUND_Z_MM + MARSHMALLOW_RADIUS_MM,
    GROUND_Z_MM + 1*CUP_SINGLE_HEIGHT_MM + MARSHMALLOW_RADIUS_MM,
    GROUND_Z_MM + 2*CUP_SINGLE_HEIGHT_MM + MARSHMALLOW_RADIUS_MM,
    GROUND_Z_MM + 3*CUP_SINGLE_HEIGHT_MM + MARSHMALLOW_RADIUS_MM,
]

# Joint SAFE limits — match the SAFE_MIN/MAX in _manipulator_config.py.
# These are advisory only — sliders are intentionally wider so out-of-safe-range
# IK solutions can still be displayed (just flagged in the status line).
TT_MIN, TT_MAX = -90.0, 180.0
SH_MIN, SH_MAX = 75.0,  180.0
EL_MIN, EL_MAX = 20.0,  160.0

# Slider ranges — physical servo travel is 0..180 for both shoulder and elbow.
# The dynamic "safe elbow range" computed each tick replaces the static EL_MIN
# / EL_MAX flag (the real constraint is collision with hardware, not a flat
# servo bound).  Shoulder kept a touch wider to allow showing IK solutions
# that exceed servo travel.
TT_SLIDER_MIN, TT_SLIDER_MAX = -180.0, 180.0
SH_SLIDER_MIN, SH_SLIDER_MAX =    0.0, 200.0
EL_SLIDER_MIN, EL_SLIDER_MAX =    0.0, 180.0

# ── Hardware envelope for collision detection ────────────────────────────────
# The base plate is treated as a solid sheet at z=0; any arm point inside the
# plate XY footprint with z < 0 is a collision (the arm passed through the
# plate to get there).
# The front sensor housing (lidar/campan/camera) is at the same height as the
# chassis top — its TOP is flush with the base plate at z=0 and it extends
# downward toward the floor.  So the clearance requirement above it is the
# same as over the rest of the plate: stay above z=0.
SENSORS_Z_TOP    =  0.0    # flush with the base-plate top
SENSORS_Z_BOTTOM = -120.0  # extends down past the camera lens (z=-40)

# Stow / default pose
TT_STOW = 0.0
SH_STOW = 90.0
EL_STOW = 90.0

# Ranging pose — used by RANGING in turntable_pick_place_test.py.  Mirrors
# ARM_RANGING_SHOULDER_DEG / ARM_RANGING_ELBOW_DEG in _manipulator_config.py.
# Tuned for the 6"+cup-radius (=185 mm) mallow zone: sh=120 puts the arm in
# the any-elbow-safe band while el=4 makes the forearm angle (~62° below
# horizontal) reach all 7 cup bearings across ±90°.  US returns 55–193 mm
# across the workspace, no hardware collisions.
SH_RANGING = 120.0
EL_RANGING =   4.0

# Base plate spec (matches the real robot): 11"×11" gray square, turntable
# centred left-right.  The turntable axis sits ROBOT_FRONT_TO_TURNTABLE_MM
# behind the front edge of the plate (= 67 mm per _manipulator_config.py).
BASE_PLATE_SIZE_MM    = 11.0 * 25.4                  # 279.4 mm
BASE_PLATE_FRONT_MM   = ROBOT_FRONT_TO_TURNTABLE_MM  # 67 mm (front edge of plate)

# Front sensor housing: a 70 mm wide stub that extends from the front edge of
# the plate forward to the camera lens position (CAMERA_PROTRUSION_MM long).
# It represents the lidar, campan motor, and camera body — physical hardware
# the arm must not swing into.
FRONT_EXT_WIDTH_MM    = 70.0
FRONT_EXT_LENGTH_MM   = CAMERA_PROTRUSION_MM         # 142 mm
# Cup/mallow semicircle radius from the camera centre to the NEAREST cup
# centre: 6" mallow pick zone + 32.5 mm cup radius = 184.9 mm.  The cup's
# near wall sits at the 6" boundary; the cup centre is one cup-radius further
# out at 185 mm.  This is the closest a cup centre can be to the camera.
MALLOW_RADIUS_MM_CAM  = 6.0 * 25.4 + 65.0 / 2.0     # 184.9 mm (cup centre)

# Marshmallow height envelope (measured from FLOOR, not base plate).
# Competition setup: lowest mallow is 6" above floor, tallest is 12" above floor.
MALLOW_MIN_ABOVE_GROUND_MM =  6.0 * 25.4   # 152.4 mm above floor → z = -76.6 mm
MALLOW_MAX_ABOVE_GROUND_MM = 12.0 * 25.4   # 304.8 mm above floor → z = +75.8 mm
MALLOW_MID_ABOVE_GROUND_MM =  9.0 * 25.4   # 228.6 mm above floor → z =  -0.4 mm
MALLOW_Z_MIN = GROUND_Z_MM + MALLOW_MIN_ABOVE_GROUND_MM   # ≈ -76.6
MALLOW_Z_MID = GROUND_Z_MM + MALLOW_MID_ABOVE_GROUND_MM   # ≈  -0.4
MALLOW_Z_MAX = GROUND_Z_MM + MALLOW_MAX_ABOVE_GROUND_MM   # ≈ +75.8

# Marshmallow place position — keep in sync with PLACE_X/Y/Z_MM in
# _manipulator_config.py.  Tuned in this sim: turntable ≈ -60.5°,
# shoulder ≈ 98°, elbow ≈ 15° (just outside the 20° elbow safe min).
PLACE_X_MM =  128.0
PLACE_Y_MM = -226.0
PLACE_Z_MM = -201.0

# Red Solo cup outer dimensions — used to draw cup-stack cylinders.
CUP_OUTER_RADIUS_MM = 65.0 / 2.0    # CUP_DIAMETER_MM/2 from _manipulator_config.py
MARSHMALLOW_RADIUS_MM = 12.5        # half of MARSHMALLOW_DIAMETER_MM

# Ultrasonic sensor mount on the forearm (mirrors _manipulator_config.py):
#   * 215 mm along the forearm from the elbow pivot
#   * 29 mm above the forearm centerline (in the arm's vertical plane)
# Beam points along the forearm direction (same direction as the gripper).
ULTRASONIC_FOREARM_OFFSET_MM = 215.0
ULTRASONIC_HEIGHT_OFFSET_MM  =  29.0

# Workspace cup stacks: (bearing_deg_from_camera, mallow_z_in_robot_frame).
# Placed around the 11" semicircle at mixed heights so the user can drag
# the turntable slider and verify the ranging pose points at each stack.
WORKSPACE_CUPS = [
    (-90.0, MALLOW_Z_MID),
    (-60.0, MALLOW_Z_MIN),
    (-30.0, MALLOW_Z_MAX),
    (  0.0, MALLOW_Z_MID),
    ( 30.0, MALLOW_Z_MIN),
    ( 60.0, MALLOW_Z_MAX),
    ( 90.0, MALLOW_Z_MID),
]

GEOM = ArmGeometry(
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

# ─────────────────────────────────────────────────────────────────────────────
# Preset target list — (label, x, y, z, color)
# Mallow positions sit on a 11" semicircle around the camera (matches the
# physical setup). bearing 0° = directly forward of camera, + = camera-left.
# ─────────────────────────────────────────────────────────────────────────────
def _mallow_on_semicircle(bearing_deg: float, z: float) -> tuple[float, float, float]:
    """Point on the 11" semicircle around the camera, at given bearing from forward."""
    rad = math.radians(bearing_deg)
    x = CAMERA_FORWARD_OFFSET_MM + MALLOW_RADIUS_MM_CAM * math.cos(rad)
    y = MALLOW_RADIUS_MM_CAM * math.sin(rad)
    return x, y, z

_M_LO  = _mallow_on_semicircle(45.0, MALLOW_Z_MIN)   # 6"  above floor, bearing +45°
_M_MID = _mallow_on_semicircle(45.0, MALLOW_Z_MID)   # 9"  above floor, bearing +45°
_M_HI  = _mallow_on_semicircle(45.0, MALLOW_Z_MAX)   # 12" above floor, bearing +45°

# Position from manual_ik_pick_place_test.py: bearing 0°, 193.8 mm from camera,
# height -81.37 mm (floor + 2.33 mm cardboard + 6 Solo cups + mallow half-height).
_M_MANUAL = (CAMERA_FORWARD_OFFSET_MM + 193.8, 0.0, -81.37)

PRESETS = [
    ("Place",        PLACE_X_MM, PLACE_Y_MM, PLACE_Z_MM,  "#ffaa44"),
    ("Plate (old)",  PLATE_X_MM, PLATE_Y_MM, PLATE_Z_MM,  "#ddaa44"),
    ('Manual Test',  *_M_MANUAL,                          "#ff5555"),
    ('Mallow 9"',    *_M_MID,                             "#ff7755"),
    ('Mallow 12"',   *_M_HI,                              "#ff9988"),
]

MAX_REACH = ARM_L1_MM + ARM_L2_MM            # 471 mm from shoulder pivot
MIN_REACH = abs(ARM_L1_MM - ARM_L2_MM)       # 159 mm from shoulder pivot

# ─────────────────────────────────────────────────────────────────────────────
# Joint positions for plotting
# ─────────────────────────────────────────────────────────────────────────────

def joint_positions(tt_deg: float, sh_servo_deg: float, el_servo_deg: float,
                    geom: ArmGeometry):
    """
    Return (base, shoulder_pivot, elbow_pivot, gripper) as np.array(3,) each.
    Coordinates are the robot frame: x=forward, y=left, z=up.
    Origin = turntable rotation axis at base plate (z=0).
    """
    sh_geo = geom.shoulder_servo_to_geo(sh_servo_deg)
    el_geo = geom.elbow_servo_to_geo(el_servo_deg)

    tt_rad = math.radians(tt_deg)
    sh_rad = math.radians(sh_geo)
    # forearm absolute angle from horizontal, in the arm's vertical plane
    fa_rad = sh_rad + math.pi - math.radians(el_geo)

    # In arm vertical plane: (reach, height) coordinates
    sh_2d = (geom.shoulder_offset_mm, geom.shoulder_height_mm)
    el_2d = (sh_2d[0] + geom.L1 * math.cos(sh_rad),
             sh_2d[1] + geom.L1 * math.sin(sh_rad))
    gr_2d = (el_2d[0] + geom.L2 * math.cos(fa_rad),
             el_2d[1] + geom.L2 * math.sin(fa_rad))

    def to_3d(reach, height):
        return np.array([reach * math.cos(tt_rad),
                         reach * math.sin(tt_rad),
                         height])

    return (
        np.array([0.0, 0.0, 0.0]),
        to_3d(*sh_2d),
        to_3d(*el_2d),
        to_3d(*gr_2d),
    )


# ─────────────────────────────────────────────────────────────────────────────
# Hardware collision detection
# ─────────────────────────────────────────────────────────────────────────────

def _point_in_plate(p) -> bool:
    """True if point p is inside the base plate solid (z<0 under footprint)."""
    x, y, z = p[0], p[1], p[2]
    x_min = BASE_PLATE_FRONT_MM - BASE_PLATE_SIZE_MM
    x_max = BASE_PLATE_FRONT_MM
    y_half = BASE_PLATE_SIZE_MM / 2.0
    return (x_min <= x <= x_max and -y_half <= y <= y_half and z < 0.0)


def _point_in_sensors(p) -> bool:
    """True if point p is inside the front sensor housing box."""
    x, y, z = p[0], p[1], p[2]
    x_min = BASE_PLATE_FRONT_MM
    x_max = BASE_PLATE_FRONT_MM + FRONT_EXT_LENGTH_MM
    y_half = FRONT_EXT_WIDTH_MM / 2.0
    return (x_min <= x <= x_max and -y_half <= y <= y_half
            and SENSORS_Z_BOTTOM <= z <= SENSORS_Z_TOP)


def _segment_collides(p1, p2, n: int = 25) -> bool:
    """Sample n points along segment p1→p2 and check hardware intersection."""
    p1 = np.asarray(p1, dtype=float)
    p2 = np.asarray(p2, dtype=float)
    for t in np.linspace(0.0, 1.0, n):
        p = p1 + t * (p2 - p1)
        if _point_in_plate(p) or _point_in_sensors(p):
            return True
    return False


def arm_collides(tt_deg, sh_servo_deg, el_servo_deg, geom):
    """
    Check whether any link of the arm enters the hardware solids.
    Returns (collides: bool, where: str — 'upper arm' | 'forearm' | '').
    """
    _, shoulder, elbow, gripper = joint_positions(
        tt_deg, sh_servo_deg, el_servo_deg, geom,
    )
    if _segment_collides(shoulder, elbow):
        return True, "upper arm in hardware"
    if _segment_collides(elbow, gripper):
        return True, "forearm in hardware"
    return False, ""


# ─────────────────────────────────────────────────────────────────────────────
# Ultrasonic ray-casting
# ─────────────────────────────────────────────────────────────────────────────

def ultrasonic_pose(tt_deg, sh_servo_deg, el_servo_deg, geom):
    """
    Return (us_position_3d, beam_direction_3d_unit) for the US sensor at the
    given joint configuration.  Sensor mount is along the forearm; beam points
    along the forearm direction (same direction the gripper faces).
    """
    sh_geo = geom.shoulder_servo_to_geo(sh_servo_deg)
    el_geo = geom.elbow_servo_to_geo(el_servo_deg)

    tt_rad = math.radians(tt_deg)
    sh_rad = math.radians(sh_geo)
    fa_rad = sh_rad + math.pi - math.radians(el_geo)

    # Elbow position in arm plane (reach, height)
    elbow_reach  = geom.shoulder_offset_mm + geom.L1 * math.cos(sh_rad)
    elbow_height = geom.shoulder_height_mm + geom.L1 * math.sin(sh_rad)

    cos_fa = math.cos(fa_rad)
    sin_fa = math.sin(fa_rad)

    # US position in arm plane: forearm offset + perpendicular ("above") offset.
    # Perpendicular direction in arm plane = forearm direction rotated +90°,
    # i.e. (-sin_fa, cos_fa).
    us_reach  = (elbow_reach
                 + ULTRASONIC_FOREARM_OFFSET_MM * cos_fa
                 + ULTRASONIC_HEIGHT_OFFSET_MM * (-sin_fa))
    us_height = (elbow_height
                 + ULTRASONIC_FOREARM_OFFSET_MM * sin_fa
                 + ULTRASONIC_HEIGHT_OFFSET_MM * cos_fa)

    us_pos = np.array([
        us_reach * math.cos(tt_rad),
        us_reach * math.sin(tt_rad),
        us_height,
    ])
    beam_dir = np.array([
        cos_fa * math.cos(tt_rad),
        cos_fa * math.sin(tt_rad),
        sin_fa,
    ])
    n = float(np.linalg.norm(beam_dir))
    if n > 1e-9:
        beam_dir = beam_dir / n
    return us_pos, beam_dir


def _ray_cylinder_t(origin, direction, cx, cy, r, z_bot, z_top):
    """Smallest positive t at which the ray hits a vertical cylinder (or None)."""
    ox, oy, oz = origin
    dx, dy, dz = direction
    A = dx*dx + dy*dy
    if A < 1e-9:
        return None
    px, py = ox - cx, oy - cy
    B = 2.0 * (px*dx + py*dy)
    C = px*px + py*py - r*r
    disc = B*B - 4.0*A*C
    if disc < 0:
        return None
    sqrt_disc = math.sqrt(disc)
    for t in sorted(((-B - sqrt_disc)/(2.0*A), (-B + sqrt_disc)/(2.0*A))):
        if t <= 1e-3:
            continue
        z = oz + t * dz
        if z_bot <= z <= z_top:
            return t
    return None


def _ray_plane_t(origin, direction, z_plane):
    """Smallest positive t at which the ray hits a horizontal plane."""
    dz = direction[2]
    if abs(dz) < 1e-9:
        return None
    t = (z_plane - origin[2]) / dz
    return t if t > 1e-3 else None


def ultrasonic_measure(tt_deg, sh_servo_deg, el_servo_deg, geom, cup_layout):
    """
    Ray-cast the US beam at the given pose against cup_layout + the floor.

    Returns (us_pos, hit_pos, distance_mm, hit_label) — hit_pos is None and
    distance_mm is None if the beam hits nothing.
    """
    us_pos, beam_dir = ultrasonic_pose(tt_deg, sh_servo_deg, el_servo_deg, geom)

    best_t: float | None = None
    best_label = "no hit"

    for idx, (bearing_deg, mallow_z) in enumerate(cup_layout):
        rad = math.radians(bearing_deg)
        cx  = CAMERA_FORWARD_OFFSET_MM + MALLOW_RADIUS_MM_CAM * math.cos(rad)
        cy  = MALLOW_RADIUS_MM_CAM * math.sin(rad)
        z_top = mallow_z - MARSHMALLOW_RADIUS_MM
        t = _ray_cylinder_t(us_pos, beam_dir, cx, cy,
                            CUP_OUTER_RADIUS_MM, GROUND_Z_MM, z_top)
        if t is not None and (best_t is None or t < best_t):
            best_t     = t
            best_label = f"cup at bearing {bearing_deg:+.0f}°"

    t_floor = _ray_plane_t(us_pos, beam_dir, GROUND_Z_MM)
    if t_floor is not None and (best_t is None or t_floor < best_t):
        best_t     = t_floor
        best_label = "floor"

    if best_t is None:
        return us_pos, None, None, best_label
    hit_pos = us_pos + best_t * beam_dir
    return us_pos, hit_pos, best_t, best_label


def safe_elbow_ranges(tt_deg: float, sh_servo_deg: float, geom,
                      step: float = 2.0) -> list[tuple[float, float]]:
    """
    Sweep elbow_servo from 0..180 at the given turntable + shoulder, and return
    a list of (start, end) sub-ranges that don't collide with hardware.

    Empty list = no safe elbow position (the shoulder itself collides, or every
    elbow angle drives some link into the plate / sensor housing).
    """
    ranges: list[tuple[float, float]] = []
    in_safe = False
    start: float | None = None
    el = 0.0
    while el <= 180.0 + 1e-6:
        el_v = min(180.0, el)
        collides, _ = arm_collides(tt_deg, sh_servo_deg, el_v, geom)
        if not collides:
            if not in_safe:
                start = el_v
                in_safe = True
            last_safe = el_v
        else:
            if in_safe and start is not None:
                ranges.append((start, last_safe))
                in_safe = False
        el += step
    if in_safe and start is not None:
        ranges.append((start, 180.0))
    return ranges


# ─────────────────────────────────────────────────────────────────────────────
# GUI state
# ─────────────────────────────────────────────────────────────────────────────
_state = {
    "mode": "FK",          # "FK" or "IK"
    "tt_deg":     TT_STOW,
    "sh_servo":   SH_STOW,
    "el_servo":   EL_STOW,
    "target_x":   300.0,
    "target_y":   0.0,
    "target_z":   MALLOW_Z[1],
    "last_status": "OK",
    "last_msg":    "",
}

_suspend = False   # set True while we update sliders programmatically

# ─────────────────────────────────────────────────────────────────────────────
# Figure setup
# ─────────────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(16, 10))
fig.patch.set_facecolor("#1e1e2e")
fig.canvas.manager.set_window_title("NUEVO Arm Kinematics Simulator")

ax = fig.add_axes([0.02, 0.30, 0.56, 0.68], projection="3d")
ax.set_facecolor("#12121f")

# Info panel axes (no plot — just text)
ax_info = fig.add_axes([0.60, 0.30, 0.38, 0.68])
ax_info.axis("off")
_info_text = ax_info.text(
    0.0, 1.0, "", transform=ax_info.transAxes,
    fontfamily="monospace", fontsize=9.0,
    color="white", verticalalignment="top",
)

# ── Sliders ───────────────────────────────────────────────────────────────────
# Two columns: joint sliders (left) and target sliders (right).
_S_LEFT_X  = 0.07
_S_RIGHT_X = 0.55
_S_WIDTH   = 0.36
_S_HEIGHT  = 0.025
_S_Y_TOP   = 0.22
_S_VSTEP   = 0.045

def _make_slider(left, y, label, vmin, vmax, vinit, fmt="%.1f", color="#5555aa"):
    sax = fig.add_axes([left, y, _S_WIDTH, _S_HEIGHT])
    sax.set_facecolor("#2a2a3e")
    sl = Slider(sax, label, vmin, vmax, valinit=vinit,
                valfmt=fmt, color=color, track_color="#2a2a3e")
    sl.label.set_color("white"); sl.label.set_fontsize(8)
    sl.valtext.set_color("#aaaaff"); sl.valtext.set_fontsize(8)
    return sl

# Joint sliders (left column) — drive FK.
# Slider range is wider than the safe range so IK solutions that exceed safe
# limits can still be displayed; safe range is shown as a tick mark.
sl_tt = _make_slider(_S_LEFT_X, _S_Y_TOP - 0*_S_VSTEP, "Turntable (deg)",
                     TT_SLIDER_MIN, TT_SLIDER_MAX, TT_STOW, "%.0f", "#4488dd")
sl_sh = _make_slider(_S_LEFT_X, _S_Y_TOP - 1*_S_VSTEP, "Shoulder servo (deg)",
                     SH_SLIDER_MIN, SH_SLIDER_MAX, SH_STOW, "%.0f", "#4488dd")
sl_el = _make_slider(_S_LEFT_X, _S_Y_TOP - 2*_S_VSTEP, "Elbow servo (deg)",
                     EL_SLIDER_MIN, EL_SLIDER_MAX, EL_STOW, "%.0f", "#4488dd")

# Target sliders (right column) — drive IK
sl_tx = _make_slider(_S_RIGHT_X, _S_Y_TOP - 0*_S_VSTEP, "Target X (mm)",
                     -600.0, 600.0, _state["target_x"], "%.0f", "#dd8844")
sl_ty = _make_slider(_S_RIGHT_X, _S_Y_TOP - 1*_S_VSTEP, "Target Y (mm)",
                     -600.0, 600.0, _state["target_y"], "%.0f", "#dd8844")
sl_tz = _make_slider(_S_RIGHT_X, _S_Y_TOP - 2*_S_VSTEP, "Target Z (mm)",
                     GROUND_Z_MM, 300.0, _state["target_z"], "%.0f", "#dd8844")

# ── Buttons ───────────────────────────────────────────────────────────────────
_BTN_W = 0.085
_BTN_H = 0.038
_BTN_Y = 0.02

# Row 1 (very bottom): preset buttons + reset
preset_buttons = []
for i, (label, _x, _y, _z, _c) in enumerate(PRESETS):
    bx = 0.07 + i * (_BTN_W + 0.012)
    bax = fig.add_axes([bx, _BTN_Y, _BTN_W, _BTN_H])
    btn = Button(bax, label, color="#2a2a3e", hovercolor="#44446e")
    btn.label.set_color("white"); btn.label.set_fontsize(8.5)
    preset_buttons.append(btn)

ax_reset = fig.add_axes([0.07 + len(PRESETS)*(_BTN_W+0.012), _BTN_Y, _BTN_W, _BTN_H])
btn_reset = Button(ax_reset, "Stow Reset", color="#2a2a3e", hovercolor="#44446e")
btn_reset.label.set_color("white"); btn_reset.label.set_fontsize(8.5)

# Ranging pose: shoulder=130, elbow=0 (the RANGING state in turntable_pick_place_test).
# Keeps the current turntable angle so you can drag it to verify across bearings.
ax_ranging = fig.add_axes([0.07 + (len(PRESETS)+1)*(_BTN_W+0.012), _BTN_Y, _BTN_W, _BTN_H])
btn_ranging = Button(ax_ranging, "Ranging Pose", color="#2a3e2a", hovercolor="#446e44")
btn_ranging.label.set_color("white"); btn_ranging.label.set_fontsize(8.5)

# Mode buttons (right side of buttons row)
ax_fk = fig.add_axes([0.78, _BTN_Y, _BTN_W, _BTN_H])
btn_fk = Button(ax_fk, "FK Mode", color="#2a2a3e", hovercolor="#44446e")
btn_fk.label.set_color("white"); btn_fk.label.set_fontsize(8.5)

ax_ik = fig.add_axes([0.88, _BTN_Y, _BTN_W, _BTN_H])
btn_ik = Button(ax_ik, "IK Mode", color="#2a2a3e", hovercolor="#44446e")
btn_ik.label.set_color("white"); btn_ik.label.set_fontsize(8.5)


# ─────────────────────────────────────────────────────────────────────────────
# Plot helpers
# ─────────────────────────────────────────────────────────────────────────────

def _draw_base_plate(ax):
    """
    11"×11" gray base plate at z=0, plus a 70 mm wide front extension that
    represents the lidar / campan motor / camera housing.  The extension runs
    from the plate's front edge forward to the camera lens position.

    Turntable axis is centred LR (y=0) and sits ROBOT_FRONT_TO_TURNTABLE_MM
    behind the plate's front edge.
    """
    # ── Main 11"×11" plate ──
    x_front = +BASE_PLATE_FRONT_MM
    x_back  = x_front - BASE_PLATE_SIZE_MM
    y_half  = BASE_PLATE_SIZE_MM / 2.0

    plate_verts = [[
        (x_back,  -y_half, 0.0),
        (x_front, -y_half, 0.0),
        (x_front, +y_half, 0.0),
        (x_back,  +y_half, 0.0),
    ]]
    plate = Poly3DCollection(plate_verts, facecolor="#6a6a78",
                             edgecolor="#aaaabb", linewidths=1.4, alpha=1.0)
    plate.set_zorder(1)
    ax.add_collection3d(plate)

    # ── Front sensor housing as a 3D box (lidar/campan/camera envelope) ──
    ext_x_back  = x_front
    ext_x_front = x_front + FRONT_EXT_LENGTH_MM
    ext_y_half  = FRONT_EXT_WIDTH_MM / 2.0
    z0, z1 = SENSORS_Z_BOTTOM, SENSORS_Z_TOP

    sensor_faces = [
        # bottom
        [(ext_x_back, -ext_y_half, z0), (ext_x_front, -ext_y_half, z0),
         (ext_x_front, +ext_y_half, z0), (ext_x_back, +ext_y_half, z0)],
        # top
        [(ext_x_back, -ext_y_half, z1), (ext_x_front, -ext_y_half, z1),
         (ext_x_front, +ext_y_half, z1), (ext_x_back, +ext_y_half, z1)],
        # -y wall
        [(ext_x_back, -ext_y_half, z0), (ext_x_front, -ext_y_half, z0),
         (ext_x_front, -ext_y_half, z1), (ext_x_back, -ext_y_half, z1)],
        # +y wall
        [(ext_x_back, +ext_y_half, z0), (ext_x_front, +ext_y_half, z0),
         (ext_x_front, +ext_y_half, z1), (ext_x_back, +ext_y_half, z1)],
        # -x wall (back / facing the plate)
        [(ext_x_back, -ext_y_half, z0), (ext_x_back, +ext_y_half, z0),
         (ext_x_back, +ext_y_half, z1), (ext_x_back, -ext_y_half, z1)],
        # +x wall (front / facing forward)
        [(ext_x_front, -ext_y_half, z0), (ext_x_front, +ext_y_half, z0),
         (ext_x_front, +ext_y_half, z1), (ext_x_front, -ext_y_half, z1)],
    ]
    sensors = Poly3DCollection(sensor_faces, facecolor="#88556a",
                               edgecolor="#cc99aa", linewidths=0.9, alpha=0.45)
    sensors.set_zorder(2)
    ax.add_collection3d(sensors)

    # Small marker dot for the turntable rotation axis on the plate.
    ax.scatter(0.0, 0.0, 0.0, color="#ffcc44", s=35, edgecolor="black",
               linewidth=0.5, depthshade=False, zorder=3)

    # Label the hardware zone near its centre.
    ax.text((ext_x_back + ext_x_front) / 2, 0.0, 6.0,
            "sensors", color="#ffaabb", fontsize=6.5,
            ha="center", va="bottom", zorder=4)


def _draw_floor(ax, x_lim, y_lim, z=GROUND_Z_MM):
    """Translucent floor plane."""
    xx, yy = np.meshgrid(np.linspace(x_lim[0], x_lim[1], 4),
                         np.linspace(y_lim[0], y_lim[1], 4))
    zz = np.full_like(xx, z)
    ax.plot_surface(xx, yy, zz, color="#3a3a4a", alpha=0.18,
                    linewidth=0, shade=False)


def _draw_reach_hemisphere(ax, shoulder, max_r=MAX_REACH, n=18):
    """Wireframe upper hemisphere centred on shoulder pivot — max-reach surface."""
    u = np.linspace(0, 2*np.pi, n)
    v = np.linspace(0, np.pi/2, n//2)
    uu, vv = np.meshgrid(u, v)
    xs = shoulder[0] + max_r * np.cos(uu) * np.sin(vv)
    ys = shoulder[1] + max_r * np.sin(uu) * np.sin(vv)
    zs = shoulder[2] + max_r * np.cos(vv)
    ax.plot_wireframe(xs, ys, zs, color="#445588", linewidth=0.35, alpha=0.32)


def _draw_arm(ax, base, shoulder, elbow, gripper, reachable=True,
              collides=False):
    # Turntable axis (vertical line through base)
    ax.plot([base[0], base[0]], [base[1], base[1]],
            [GROUND_Z_MM, 250], color="#666688", linewidth=0.7,
            linestyle="--", alpha=0.55)

    # Shoulder pivot stub from base → shoulder (the small forward offset + height)
    ax.plot([base[0], shoulder[0]], [base[1], shoulder[1]],
            [base[2], shoulder[2]], color="#888899", linewidth=2.2)
    ax.scatter(*shoulder, color="#aaaaff", s=60, edgecolor="white",
               linewidth=0.6, depthshade=False)

    # Arm link colours — turn red when the pose collides with hardware.
    upper_color   = "#ff3333" if collides else "#4488ff"
    forearm_color = "#ff3333" if collides else "#ff8844"

    # Upper arm
    ax.plot([shoulder[0], elbow[0]], [shoulder[1], elbow[1]],
            [shoulder[2], elbow[2]], color=upper_color, linewidth=4.5,
            solid_capstyle="round")
    ax.scatter(*elbow, color="#aaccff", s=55, edgecolor="white",
               linewidth=0.6, depthshade=False)

    # Forearm
    ax.plot([elbow[0], gripper[0]], [elbow[1], gripper[1]],
            [elbow[2], gripper[2]], color=forearm_color, linewidth=4.5,
            solid_capstyle="round")

    # Gripper end — red if target unreachable, green otherwise.
    gc = "#44ee44" if reachable else "#ff3333"
    ax.scatter(*gripper, color=gc, s=140, edgecolor="white",
               linewidth=0.9, depthshade=False, zorder=10)


def _draw_mallow_zone(ax, n=40):
    """
    Draw the 180° semicircle of possible mallow positions at the LO (6") and
    HI (12") heights, so you can see the entire envelope you might need to reach.
    """
    th = np.linspace(-math.pi/2, math.pi/2, n)
    xs = CAMERA_FORWARD_OFFSET_MM + MALLOW_RADIUS_MM_CAM * np.cos(th)
    ys = MALLOW_RADIUS_MM_CAM * np.sin(th)

    for z, label_z in ((MALLOW_Z_MIN, '6"'), (MALLOW_Z_MAX, '12"')):
        zs = np.full_like(xs, z)
        ax.plot(xs, ys, zs, color="#ff7766", linewidth=0.8,
                alpha=0.55, linestyle="--")
        # End-arc tick label
        ax.text(xs[-1] + 10, ys[-1] + 10, z, f'mallow {label_z}',
                color="#ff9988", fontsize=6.5, ha="left", va="bottom")


def _draw_cup_stack(ax, cx, cy, top_z, n_theta=18,
                    cup_radius=CUP_OUTER_RADIUS_MM,
                    color="#cc4444", alpha=0.55):
    """Draw a cylindrical cup stack from the floor to top_z at (cx, cy)."""
    theta = np.linspace(0.0, 2.0*math.pi, n_theta)
    z_arr = np.array([GROUND_Z_MM, top_z])
    theta_grid, z_grid = np.meshgrid(theta, z_arr)
    x_grid = cx + cup_radius * np.cos(theta_grid)
    y_grid = cy + cup_radius * np.sin(theta_grid)
    ax.plot_surface(x_grid, y_grid, z_grid, color=color, alpha=alpha,
                    linewidth=0, shade=False)
    # Top rim outline for a bit of definition
    rim_x = cx + cup_radius * np.cos(theta)
    rim_y = cy + cup_radius * np.sin(theta)
    rim_z = np.full_like(theta, top_z)
    ax.plot(rim_x, rim_y, rim_z, color="#882222", linewidth=0.6, alpha=0.7)


def _draw_workspace_cups(ax):
    """
    Draw cup-stack cylinders at multiple bearings around the 11" semicircle,
    each with a mallow dot on top.  Drag the turntable slider to verify the
    arm at the ranging pose (sh=130, el=0) points at each stack.
    """
    for bearing_deg, mallow_z in WORKSPACE_CUPS:
        rad = math.radians(bearing_deg)
        cx = CAMERA_FORWARD_OFFSET_MM + MALLOW_RADIUS_MM_CAM * math.cos(rad)
        cy = MALLOW_RADIUS_MM_CAM * math.sin(rad)
        # Top of the cup stack sits just below the mallow centre.
        top_z = mallow_z - MARSHMALLOW_RADIUS_MM
        _draw_cup_stack(ax, cx, cy, top_z)
        # Mallow on top of the cup stack.
        ax.scatter(cx, cy, mallow_z, color="#ff9977", s=45, alpha=0.9,
                   edgecolor="white", linewidth=0.4, depthshade=False)


def _draw_targets(ax, presets, active_xyz=None):
    for label, x, y, z, c in presets:
        ax.scatter(x, y, z, color=c, s=70, edgecolor="white",
                   linewidth=0.5, depthshade=False)
        ax.text(x, y, z + 22, label, color=c, fontsize=7,
                ha="center", va="bottom")

    # Camera lens marker
    ax.scatter(CAMERA_FORWARD_OFFSET_MM, 0.0, CAMERA_HEIGHT_MM,
               color="#44ddff", s=60, marker="^", edgecolor="white",
               linewidth=0.5, depthshade=False)
    ax.text(CAMERA_FORWARD_OFFSET_MM, 0.0, CAMERA_HEIGHT_MM + 22,
            "Camera", color="#88eeff", fontsize=7,
            ha="center", va="bottom")

    if active_xyz is not None:
        x, y, z = active_xyz
        # Crosshair pillar from floor to target — makes the height obvious
        ax.plot([x, x], [y, y], [GROUND_Z_MM, z], color="#ffff44",
                linewidth=0.7, linestyle=":", alpha=0.85)
        ax.scatter(x, y, z, color="#ffff44", s=120, marker="x",
                   linewidth=2.0, depthshade=False)


# ─────────────────────────────────────────────────────────────────────────────
# Replot
# ─────────────────────────────────────────────────────────────────────────────

def _draw_ultrasonic(ax, us_pos, hit_pos, beam_dir):
    """Draw the US sensor marker, the beam, and the hit point."""
    # US sensor marker — small cyan triangle on the forearm
    ax.scatter(*us_pos, color="#66ccff", s=55, marker="^",
               edgecolor="white", linewidth=0.6, depthshade=False, zorder=11)
    ax.text(us_pos[0] + 8, us_pos[1] + 8, us_pos[2] + 8, "US",
            color="#aaeeff", fontsize=6.5)

    # Beam — dotted line from US to hit; if no hit, draw a short ghost segment
    if hit_pos is not None:
        ax.plot([us_pos[0], hit_pos[0]],
                [us_pos[1], hit_pos[1]],
                [us_pos[2], hit_pos[2]],
                color="#aaeeff", linewidth=1.1, linestyle=":", alpha=0.85)
        ax.scatter(*hit_pos, color="#ff44aa", s=90, marker="x",
                   linewidth=2.0, depthshade=False, zorder=12)
    else:
        tip = us_pos + 200.0 * beam_dir
        ax.plot([us_pos[0], tip[0]], [us_pos[1], tip[1]], [us_pos[2], tip[2]],
                color="#aaeeff", linewidth=1.0, linestyle=":", alpha=0.45)


def _info_panel(tt_deg, sh_servo, el_servo, gripper, status, msg,
                collides, where, safe_ranges,
                us_pos, us_dist, us_label):
    sh_geo = GEOM.shoulder_servo_to_geo(sh_servo)
    el_geo = GEOM.elbow_servo_to_geo(el_servo)
    horiz = math.hypot(gripper[0], gripper[1])
    d_from_shoulder = math.hypot(
        horiz - GEOM.shoulder_offset_mm,
        gripper[2] - GEOM.shoulder_height_mm,
    )
    status_line = f"  Status         : {status}"
    if msg:
        status_line += f"  ({msg})"

    # Hardware-collision section
    if collides:
        coll_line = f"  Collision      : YES — {where}"
    else:
        coll_line = "  Collision      : NONE (pose clear of hardware)"

    if not safe_ranges:
        safe_str = "  Safe elbow     : NONE — shoulder collides for any elbow"
    else:
        parts = [f"[{a:.0f}, {b:.0f}]" for a, b in safe_ranges]
        safe_str = "  Safe elbow     : " + " ∪ ".join(parts) + " deg"

    # Is the current elbow inside any safe range?
    in_safe = any(a - 0.5 <= el_servo <= b + 0.5 for a, b in safe_ranges)
    elbow_mark = "in safe range" if in_safe else "OUTSIDE safe range"

    return (
        f"─── Mode ────────────────────────────\n"
        f"  Active driver  : {_state['mode']}\n"
        f"  (drag a joint slider for FK,\n"
        f"   drag a target slider for IK)\n"
        f"\n"
        f"─── Joint angles ────────────────────\n"
        f"  Turntable      : {tt_deg:+7.1f} deg  "
        f"(0 = fwd, +CCW = left)\n"
        f"  Shoulder geo   : {sh_geo:+7.1f} deg  "
        f"(0 = horiz, 90 = up)\n"
        f"  Shoulder servo : {sh_servo:7.1f} deg  "
        f"physical [0..180]\n"
        f"  Elbow geo      : {el_geo:+7.1f} deg  "
        f"(180 = straight, 0 = folded)\n"
        f"  Elbow servo    : {el_servo:7.1f} deg  "
        f"({elbow_mark})\n"
        f"\n"
        f"─── Gripper position ────────────────\n"
        f"  x (forward)    : {gripper[0]:+7.1f} mm\n"
        f"  y (left)       : {gripper[1]:+7.1f} mm\n"
        f"  z (up)         : {gripper[2]:+7.1f} mm\n"
        f"  horizontal     : {horiz:7.1f} mm  from turntable\n"
        f"  shoulder dist  : {d_from_shoulder:7.1f} mm  "
        f"(max {MAX_REACH:.0f})\n"
        f"\n"
        f"─── Reach ───────────────────────────\n"
        f"{status_line}\n"
        f"\n"
        f"─── Hardware collision ──────────────\n"
        f"{coll_line}\n"
        f"{safe_str}\n"
        f"  (range is for current turntable + shoulder)\n"
        f"\n"
        f"─── Ultrasonic ──────────────────────\n"
        f"  US position    : ({us_pos[0]:+6.0f}, {us_pos[1]:+6.0f}, {us_pos[2]:+6.0f}) mm\n"
        f"  Reading        : {('%.0f mm' % us_dist) if us_dist is not None else 'no hit'}\n"
        f"  Hits           : {us_label}\n"
        f"\n"
        f"─── Geometry ────────────────────────\n"
        f"  L1 / L2        : {ARM_L1_MM:.0f} / {ARM_L2_MM:.0f} mm\n"
        f"  shoulder       : {ARM_SHOULDER_HEIGHT_MM:.0f} mm up, "
        f"{ARM_SHOULDER_OFFSET_MM:.0f} mm fwd of axis\n"
        f"  camera (x)     : {CAMERA_FORWARD_OFFSET_MM:.0f} mm fwd of axis\n"
        f"  sensor box z   : [{SENSORS_Z_BOTTOM:.0f}, {SENSORS_Z_TOP:.0f}] "
        f"(tune if wrong)\n"
        f"\n"
        f"─── Tips ────────────────────────────\n"
        f"  Drag joint slider  -> FK\n"
        f"  Drag target slider -> IK\n"
        f"  Click preset       -> IK to target\n"
        f"  Drag the 3D view   -> rotate camera\n"
    )


def replot():
    tt  = _state["tt_deg"]
    sh  = _state["sh_servo"]
    el  = _state["el_servo"]

    base, shoulder, elbow, gripper = joint_positions(tt, sh, el, GEOM)

    # Reachability check on the current target XYZ
    tx, ty, tz = _state["target_x"], _state["target_y"], _state["target_z"]
    target_reachable = True
    try:
        inverse_kinematics(tx, ty, tz, GEOM)
    except OutOfReachError:
        target_reachable = False

    # Re-draw 3D plot
    ax.cla()
    ax.set_facecolor("#12121f")
    # Make the background panes dark
    try:
        ax.xaxis.set_pane_color((0.07, 0.07, 0.12, 0.9))
        ax.yaxis.set_pane_color((0.07, 0.07, 0.12, 0.9))
        ax.zaxis.set_pane_color((0.07, 0.07, 0.12, 0.9))
    except Exception:
        pass

    # Axes labels and limits
    ax.set_xlabel("x  fwd (mm)", color="#aaaacc", fontsize=8.5, labelpad=4)
    ax.set_ylabel("y  left (mm)", color="#aaaacc", fontsize=8.5, labelpad=4)
    ax.set_zlabel("z  up (mm)",  color="#aaaacc", fontsize=8.5, labelpad=4)
    ax.tick_params(colors="#888899", labelsize=7)
    ax.set_xlim(-550, 550)
    ax.set_ylim(-550, 550)
    ax.set_zlim(GROUND_Z_MM - 30, 350)

    # Collision check at the current pose + sweep elbow for the safe range.
    collides, where = arm_collides(tt, sh, el, GEOM)
    safe_ranges     = safe_elbow_ranges(tt, sh, GEOM)

    # Ultrasonic ray-cast: US sensor pose + beam hit against cups + floor.
    us_pos, hit_pos, us_dist, us_label = ultrasonic_measure(
        tt, sh, el, GEOM, WORKSPACE_CUPS,
    )
    _, beam_dir = ultrasonic_pose(tt, sh, el, GEOM)

    # Floor + base + reach hemisphere + arm + targets + cup-stacks + US
    _draw_floor(ax, (-550, 550), (-550, 550))
    _draw_base_plate(ax)
    _draw_reach_hemisphere(ax, shoulder)
    _draw_mallow_zone(ax)
    _draw_workspace_cups(ax)
    _draw_targets(ax, PRESETS, active_xyz=(tx, ty, tz))
    _draw_arm(ax, base, shoulder, elbow, gripper,
              reachable=target_reachable or _state["mode"] == "FK",
              collides=collides)
    _draw_ultrasonic(ax, us_pos, hit_pos, beam_dir)

    ax.set_title("NUEVO Arm — 3D Kinematics", color="white", fontsize=10, pad=8)

    # Refresh the info panel
    status = _state["last_status"]
    msg    = _state["last_msg"]
    _info_text.set_text(_info_panel(tt, sh, el, gripper, status, msg,
                                    collides, where, safe_ranges,
                                    us_pos, us_dist, us_label))

    fig.canvas.draw_idle()


# ─────────────────────────────────────────────────────────────────────────────
# Slider / button callbacks
# ─────────────────────────────────────────────────────────────────────────────

def _push_joint_sliders(tt, sh, el):
    """Update joint sliders without retriggering callbacks."""
    global _suspend
    _suspend = True
    sl_tt.set_val(tt)
    sl_sh.set_val(sh)
    sl_el.set_val(el)
    _suspend = False


def _push_target_sliders(x, y, z):
    """Update target sliders without retriggering callbacks."""
    global _suspend
    _suspend = True
    sl_tx.set_val(x)
    sl_ty.set_val(y)
    sl_tz.set_val(z)
    _suspend = False


def _on_joint_changed(_val):
    if _suspend:
        return
    _state["mode"]     = "FK"
    _state["tt_deg"]   = float(sl_tt.val)
    _state["sh_servo"] = float(sl_sh.val)
    _state["el_servo"] = float(sl_el.val)
    # Compute gripper position via FK and mirror into target sliders
    gx, gy, gz = forward_kinematics(
        _state["tt_deg"], _state["sh_servo"], _state["el_servo"], GEOM,
    )
    _state["target_x"], _state["target_y"], _state["target_z"] = gx, gy, gz
    _push_target_sliders(gx, gy, gz)
    _state["last_status"] = "OK"
    _state["last_msg"]    = ""
    replot()


def _on_target_changed(_val):
    if _suspend:
        return
    _state["mode"]     = "IK"
    _state["target_x"] = float(sl_tx.val)
    _state["target_y"] = float(sl_ty.val)
    _state["target_z"] = float(sl_tz.val)
    _run_ik_and_apply()


def _run_ik_and_apply():
    """
    Solve IK for the current target and display the actual joint angles even
    if they fall outside the safe range — clamping was hiding the real shape
    of unreachable poses.  The status line flags any limit violation.
    """
    tx, ty, tz = _state["target_x"], _state["target_y"], _state["target_z"]
    try:
        tt_deg, sh_servo, el_servo = inverse_kinematics(tx, ty, tz, GEOM)
    except OutOfReachError as e:
        _state["last_status"] = "OUT OF REACH (distance)"
        msg = str(e)
        _state["last_msg"]    = msg.split("—")[-1].strip() if "—" in msg else ""
        replot()
        return

    warnings = []
    if not (TT_MIN <= tt_deg <= TT_MAX):
        warnings.append(f"turntable {tt_deg:.0f} not in [{TT_MIN:.0f},{TT_MAX:.0f}]")
    if not (SH_MIN <= sh_servo <= SH_MAX):
        warnings.append(f"shoulder {sh_servo:.0f} not in [{SH_MIN:.0f},{SH_MAX:.0f}]")
    if not (EL_MIN <= el_servo <= EL_MAX):
        warnings.append(f"elbow {el_servo:.0f} not in [{EL_MIN:.0f},{EL_MAX:.0f}]")

    # Keep slider values inside their own range so set_val() does not error.
    tt_disp = max(TT_SLIDER_MIN, min(TT_SLIDER_MAX, tt_deg))
    sh_disp = max(SH_SLIDER_MIN, min(SH_SLIDER_MAX, sh_servo))
    el_disp = max(EL_SLIDER_MIN, min(EL_SLIDER_MAX, el_servo))

    _state["tt_deg"]   = tt_disp
    _state["sh_servo"] = sh_disp
    _state["el_servo"] = el_disp
    _push_joint_sliders(tt_disp, sh_disp, el_disp)

    if warnings:
        _state["last_status"] = "OUTSIDE SAFE LIMITS"
        _state["last_msg"]    = "; ".join(warnings)
    else:
        _state["last_status"] = "OK"
        _state["last_msg"]    = ""
    replot()


def _on_preset(i):
    def handler(_event):
        label, x, y, z, _c = PRESETS[i]
        _state["mode"] = "IK"
        _state["target_x"], _state["target_y"], _state["target_z"] = x, y, z
        _push_target_sliders(x, y, z)
        _run_ik_and_apply()
    return handler


def _on_reset(_event):
    _state["mode"] = "FK"
    _state["tt_deg"]   = TT_STOW
    _state["sh_servo"] = SH_STOW
    _state["el_servo"] = EL_STOW
    _push_joint_sliders(TT_STOW, SH_STOW, EL_STOW)
    gx, gy, gz = forward_kinematics(TT_STOW, SH_STOW, EL_STOW, GEOM)
    _state["target_x"], _state["target_y"], _state["target_z"] = gx, gy, gz
    _push_target_sliders(gx, gy, gz)
    _state["last_status"] = "OK"
    _state["last_msg"]    = ""
    replot()


def _on_ranging(_event):
    """Snap shoulder + elbow to the ranging pose; keep current turntable."""
    _state["mode"] = "FK"
    _state["sh_servo"] = SH_RANGING
    _state["el_servo"] = EL_RANGING
    _push_joint_sliders(_state["tt_deg"], SH_RANGING, EL_RANGING)
    gx, gy, gz = forward_kinematics(
        _state["tt_deg"], SH_RANGING, EL_RANGING, GEOM,
    )
    _state["target_x"], _state["target_y"], _state["target_z"] = gx, gy, gz
    _push_target_sliders(gx, gy, gz)
    _state["last_status"] = "OK"
    _state["last_msg"]    = ""
    replot()


def _on_mode_fk(_event):
    _state["mode"] = "FK"
    # Use current joint sliders → recompute target
    _on_joint_changed(None)


def _on_mode_ik(_event):
    _state["mode"] = "IK"
    # Use current target sliders → recompute joints
    _on_target_changed(None)


sl_tt.on_changed(_on_joint_changed)
sl_sh.on_changed(_on_joint_changed)
sl_el.on_changed(_on_joint_changed)
sl_tx.on_changed(_on_target_changed)
sl_ty.on_changed(_on_target_changed)
sl_tz.on_changed(_on_target_changed)

for i, btn in enumerate(preset_buttons):
    btn.on_clicked(_on_preset(i))
btn_reset.on_clicked(_on_reset)
btn_ranging.on_clicked(_on_ranging)
btn_fk.on_clicked(_on_mode_fk)
btn_ik.on_clicked(_on_mode_ik)


# ─────────────────────────────────────────────────────────────────────────────
# Initial view
# ─────────────────────────────────────────────────────────────────────────────
# Sync target sliders to the current stow pose, then render.
_gx, _gy, _gz = forward_kinematics(TT_STOW, SH_STOW, EL_STOW, GEOM)
_state["target_x"], _state["target_y"], _state["target_z"] = _gx, _gy, _gz
_push_target_sliders(_gx, _gy, _gz)

# Default 3D camera angle: looking at the front-right of the robot.
ax.view_init(elev=22, azim=-55)
replot()

plt.show()
