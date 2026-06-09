#!/usr/bin/env python3
"""
sim_obstacle_avoidance.py
=========================
Interactive 2D simulator for PurePursuitPlannerWithAvoidance.

Runs the exact same planner code used in lane_switch_obstacle_test.py and
venue_full_course_test.py SEG3B — no robot needed.  Lets you place cones,
tune parameters with sliders, and see the trajectory replay immediately.

HOW TO USE
----------
  python3 ros2_ws/src/robot/tools/sim_obstacle_avoidance.py

Controls:
  Left-click on field  → add a cone
  Right-click on field → remove nearest cone (within 120 mm cursor radius)
  Sliders              → change any parameter and replay auto-triggers
  Reset Cones button   → restore the default cone layout
  Default Params       → restore all sliders to default values
  Venue Seg3B toggle   → switch between lane-switch (single straight) and
                         the full obstacle zone from venue_full_course_test

Reading the plot:
  Blue line     → normal driving trajectory
  Orange line   → trajectory while avoidance is active
  Orange dots   → detour waypoints the planner injected (hat-shaped path)
  Green dashed  → original reference path
  Red circles   → cones (cone radius = CONE_RADIUS_MM)
  Red rings     → safe_dist radius around each cone (avoidance triggers when
                  the next waypoint enters this ring)
  Blue arrow    → final robot position and heading
  Magenta dot   → lookahead point at the last recorded frame

PARAMETER GUIDE
---------------
  lookahead    How far ahead on the path to aim.  Small = twitchy/precise,
               large = smooth/wide corners.  Doesn't affect avoidance logic
               directly but shapes the approach and exit curves.

  safe_dist    The minimum clearance the planner demands between the next
               waypoint and any cone.  Also the amount of lateral offset it
               tries to achieve (if no cone is right on the centerline).
               Too small → clears too close.  Too large → huge detours.

  offset       Maximum lateral shift applied to the detour waypoints (mm).
               Think of this as "how far left/right does it go to dodge".
               Set to slightly more than safe_dist + cone_radius.

  obs_range    How far ahead the lidar looks for cones.  Cones beyond this
               are ignored.  Large = earlier reaction; too large = false
               triggers from irrelevant cones.

  avoid_delay  After clearing an obstacle, stay in avoidance mode for this
               many FSM ticks (1 tick = 20 ms at 50 Hz).  Prevents the robot
               from snapping back to center before it is fully past the cone.
               avoid_delay / 50 = seconds of extra hold.

  alpha_Ld     Lookahead scale factor during avoidance.  alpha_Ld < 1 →
               shorter lookahead = sharper tracking of detour waypoints.
               alpha_Ld > 1 → longer lookahead = smoother path but may
               cut corners on the detour.

  lane_width   Cones outside ±(lane_width/2) mm from x_L are ignored.
               Prevents the planner reacting to obstacles in adjacent lanes.

  speed        Linear speed in mm/s.  Lower = more time to react; higher =
               covers the field faster but overshoots more.
"""

from __future__ import annotations

import math
import os
import sys

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider

# ── Make the robot package importable from Mac (no ROS2 needed) ───────────────
_PKG_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from robot.path_planner import PurePursuitPlannerWithAvoidance  # noqa: E402
from robot.util import densify_polyline  # noqa: E402

# ─────────────────────────────────────────────────────────────────────────────
# Sim constants
# ─────────────────────────────────────────────────────────────────────────────

FSM_HZ      = 50          # matches DEFAULT_FSM_HZ from hardware_map.py
DT          = 1.0 / FSM_HZ
MAX_STEPS   = 6000        # 120-second timeout at 50 Hz
CONE_RADIUS = 75.0        # visual radius of traffic cone base (mm)

# ── Scenario definitions ──────────────────────────────────────────────────────
# lane_switch: matches lane_switch_obstacle_test.py exactly.
#   Robot at (0,0) facing north, straight path to (0,2000).
# obs_zone:    matches venue_full_course_test.py SEG3B.
#   Straight northbound run through the obstacle zone.
#   Field is 500 mm wide, 2500 mm long.

# ── Venue course geometry (mirrors venue_full_course_test.py SEG3) ────────────
# Kept in sync with that file so the sim reflects the real course layout.
X_LANE1   =    0.0
X_LANE2   =  610.0
X_OBS_MID = 1525.0
X_LANE5   = 2440.0
Y_TOP     = 3500.0
Y_BOT     =  460.0

OBS_ENTRY_OFFSET_MM = 400.0    # SEG3A drives this far north of the corner before
                               # SEG3B's avoidance takes over.
OBS_EXIT_OFFSET_MM  = 400.0    # SEG3B stops this far short of the north wall.
                               # Wall safety is handled by the goal-y obstacle
                               # filter in path_planner, not by this offset.

# SEG3 sub-paths in absolute course frame (x=east, y=north).
SEG3_APPROACH_PTS = [
    (X_LANE2 + 457.5, Y_BOT),                        # CP2
    (X_OBS_MID,       Y_BOT),                        # corner (east → north)
    (X_OBS_MID,       Y_BOT + OBS_ENTRY_OFFSET_MM),  # avoidance start
]
SEG3_OBS_PTS = [
    (X_OBS_MID, Y_BOT + OBS_ENTRY_OFFSET_MM),
    (X_OBS_MID, Y_TOP - OBS_EXIT_OFFSET_MM),
]
SEG3_EXIT_PTS = [
    (X_OBS_MID, Y_TOP - OBS_EXIT_OFFSET_MM),
    (X_OBS_MID, Y_TOP),                              # corner (north → east)
    (X_OBS_MID + 457.5, Y_TOP),                      # CP3
]

# Per-segment configs — must mirror SEG3{A,B,C}_CFG in venue_full_course_test.py.
SEG3A_CFG = dict(speed=130, lookahead=300, spacing=50)
SEG3B_CFG = dict(
    speed=75, lookahead=290, spacing=400,
    safe_dist=230, lane_width=451, avoidance_delay=245,
    obstacles_range=570, view_angle=70.0, alpha_Ld=0.90, offset=324,
)
SEG3C_CFG = dict(speed=120, lookahead=300, spacing=50)

# Avoidance-OFF planner defaults (mirrors the `else` branch in
# venue_full_course_test._load_path).
_AVOID_OFF_DEFAULTS = dict(
    obstacles_range = 450.0,
    view_angle_deg  =  70.0,
    safe_dist       = 250.0,
    avoidance_delay = 150,
    alpha_Ld        =   0.7,
    offset          = 305.0,
    lane_width      = 500.0,
    max_angular     =   1.5,
    goal_tolerance  =  20.0,
)

SCENARIOS = {
    "lane_switch": dict(
        path_pts      = [(0.0, 0.0), (0.0, 2000.0)],
        spacing       = 400.0,
        start_pose    = [0.0, 0.0, math.pi / 2],   # (x, y, theta) facing north
        x_L           = 0.0,
        field_xlim    = (-700.0, 700.0),
        field_ylim    = (-200.0, 2300.0),
        lane_lines_x  = [-200.0, 200.0],
        label         = "Lane-switch (straight 2 m)",
        default_cones = [
            (  80.0,  700.0),
            (-110.0, 1200.0),
            (  60.0, 1650.0),
        ],
    ),
    "obs_zone": dict(
        path_pts      = [(0.0, 0.0), (0.0, 2500.0)],
        spacing       = 400.0,
        start_pose    = [0.0, 0.0, math.pi / 2],
        x_L           = 0.0,
        field_xlim    = (-700.0, 700.0),
        field_ylim    = (-200.0, 2800.0),
        lane_lines_x  = [-250.0, 250.0],
        label         = "Venue SEG3B obstacle zone",
        default_cones = [
            (  80.0,  600.0),
            (-120.0, 1100.0),
            (  90.0, 1600.0),
            (-100.0, 2100.0),
        ],
    ),
    # Real venue geometry — coordinates in COURSE frame (x=east, y=north, mm).
    # Path: SEG3B obstacle phase from (1525, Y_BOT+400=1010) to
    # (1525, Y_TOP-OBS_EXIT_OFFSET_MM=2900) after the fix bumped the offset to 600.
    # Cones: the three test obstacles reported on the real course, ~305 mm off
    # centerline — which is the case the GUI defaults DON'T cover and is why
    # avoidance fires hard in sim but barely in reality.
    # Full SEG3 = 3a approach (avoidance OFF) → 3b obstacle zone (avoidance ON)
    # → 3c exit (avoidance OFF). Runs all three planners in sequence so the
    # diagonal-up-left drift at the SEG3B→SEG3C handoff is visible end-to-end.
    "seg3_full": dict(
        phased        = True,
        start_pose    = [X_LANE2 + 457.5, Y_BOT, 0.0],   # CP2 facing east
        x_L           = X_OBS_MID,
        field_xlim    = (150.0,  2750.0),
        field_ylim    = (200.0,  3800.0),
        label         = "Venue SEG3 FULL (CP2 → CP3, avoidance ON only in 3B)",
        # Walls / lane corridor bounds drawn explicitly.
        walls         = dict(south_y=Y_BOT, north_y=Y_TOP),
        lane_bounds   = dict(
            lane23_x = X_OBS_MID - 610.0,    #  915 — physical west bound
            lane45_x = X_OBS_MID + 610.0,    # 2135 — physical east bound
        ),
        # Avoidance-ON region (where SEG3B actually runs, lane corridor only).
        avoid_zone    = dict(
            y_lo = Y_BOT + OBS_ENTRY_OFFSET_MM,   #  860
            y_hi = Y_TOP - OBS_EXIT_OFFSET_MM,    # 2600
            x_lo = X_OBS_MID - SEG3B_CFG["lane_width"] / 2.0,
            x_hi = X_OBS_MID + SEG3B_CFG["lane_width"] / 2.0,
        ),
        # Three sub-paths, each with its own config + avoidance flag.
        phases        = [
            dict(name="3A approach", path_pts=SEG3_APPROACH_PTS, cfg=SEG3A_CFG, avoidance=False),
            dict(name="3B obstacle", path_pts=SEG3_OBS_PTS,      cfg=SEG3B_CFG, avoidance=True),
            dict(name="3C exit",     path_pts=SEG3_EXIT_PTS,     cfg=SEG3C_CFG, avoidance=False),
        ],
        # Three cones in the obstacle zone, alternating L/R, matching the
        # "venue_real" default layout but in this scenario's coordinates.
        default_cones = [
            (X_OBS_MID + 305.0, 1565.0),
            (X_OBS_MID - 305.0, 2175.0),
            (X_OBS_MID + 305.0, 2785.0),
        ],
    ),
    "venue_real": dict(
        path_pts      = [(1525.0, 1010.0), (1525.0, 2900.0)],
        spacing       = 400.0,
        start_pose    = [1525.0, 1010.0, math.pi / 2],
        x_L           = 1525.0,
        field_xlim    = (1000.0, 2050.0),
        field_ylim    = (800.0, 3100.0),
        lane_lines_x  = [1525.0 - 225.5, 1525.0 + 225.5],   # ±lane_width/2 = 451/2
        label         = "Venue REAL SEG3B (course-frame, cones @ ±305 mm)",
        default_cones = [
            (1830.0, 1565.0),
            (1220.0, 2175.0),
            (1830.0, 2785.0),
        ],
    ),
}

_active_scenario_key = "lane_switch"

# ── Default planner params (tuned values from sim session 2026-06-05) ─────────
# Slider starting values — kept in sync with SEG3B_CFG so the sliders accurately
# reflect what SEG3B will run with in the phased seg3_full scenario. Sliders
# only override SEG3B_CFG once the user actually moves them off these defaults
# (see _build_planner_params).
DEFAULT_PARAMS = dict(
    lookahead_distance = 290.0,
    max_linear_speed   = 75.0,
    max_angular_speed  = 1.0,
    goal_tolerance     = 30.0,
    obstacles_range    = 570.0,
    view_angle         = math.radians(70.0),
    safe_dist          = 230.0,
    avoidance_delay    = 245,
    alpha_Ld           = 0.90,
    offset             = 324.0,
    lane_width         = 451.0,
    obstacle_avoidance = True,
)

# ─────────────────────────────────────────────────────────────────────────────
# Simulation engine
# ─────────────────────────────────────────────────────────────────────────────

def run_simulation(
    params: dict,
    cones_world: list[tuple[float, float]],
    scenario_key: str,
) -> dict:
    """
    Run PurePursuitPlannerWithAvoidance to completion and return trajectory data.

    The planner is fed robot-frame obstacle positions at each tick, mirroring
    what the lidar provides on the real robot.
    """
    sc = SCENARIOS[scenario_key]
    path = densify_polyline(sc["path_pts"], spacing=sc["spacing"])

    full_params = dict(params)
    full_params["x_L"] = sc["x_L"]

    planner = PurePursuitPlannerWithAvoidance(**full_params)
    planner.set_path(path)

    pose = list(sc["start_pose"])

    traj       = [tuple(pose[:2])]
    avoid_flag = [False]
    detour_snaps: list[list[tuple]] = []  # remaining_path snapshot each tick

    avoidance_triggers = 0
    was_avoiding = False
    status = "TIMEOUT"

    for step in range(MAX_STEPS):
        cos_th = math.cos(pose[2])
        sin_th = math.sin(pose[2])

        # Transform cones from world frame → robot frame (x=fwd, y=left).
        obs_r_list = []
        for cx, cy in cones_world:
            dx, dy = cx - pose[0], cy - pose[1]
            fwd  =  cos_th * dx + sin_th * dy
            left = -sin_th * dx + cos_th * dy
            obs_r_list.append([fwd, left])
        obs_r = np.array(obs_r_list) if obs_r_list else np.empty((0, 2))

        v, w = planner.compute_velocity(pose, obs_r)
        pose = planner.motion(pose, v, w, DT)

        active = planner.avoidance_active
        traj.append(tuple(pose[:2]))
        avoid_flag.append(active)
        detour_snaps.append(list(planner.remaining_path))

        if active and not was_avoiding:
            avoidance_triggers += 1
        was_avoiding = active

        if planner.TargetReached(planner.remaining_path, pose[0], pose[1]):
            status = "GOAL REACHED"
            break
        if not planner.remaining_path:
            status = "PATH EMPTY"
            break

    return dict(
        trajectory         = np.array(traj),
        avoidance          = avoid_flag,
        detour_snaps       = detour_snaps,
        final_pose         = pose,
        status             = status,
        steps              = step + 1,
        path               = np.array(path),
        avoidance_triggers = avoidance_triggers,
    )


def _build_planner_params(cfg: dict, avoidance: bool, x_L: float,
                          slider_overrides: dict | None = None) -> dict:
    """
    Build the kwargs for PurePursuitPlannerWithAvoidance from a venue cfg dict
    (mirrors venue_full_course_test._load_path). For avoidance-OFF phases we
    use the same fallback defaults the FSM sends down. slider_overrides only
    applies to the avoidance-ON phase (SEG3B) so sliders don't accidentally
    perturb the 3A/3C straight runs.
    """
    if avoidance:
        params = dict(
            lookahead_distance = float(cfg["lookahead"]),
            max_linear_speed   = float(cfg["speed"]),
            max_angular_speed  = 1.0,
            goal_tolerance     = 30.0,
            obstacles_range    = float(cfg.get("obstacles_range", 450.0)),
            view_angle         = math.radians(float(cfg.get("view_angle", 70.0))),
            safe_dist          = float(cfg.get("safe_dist", 181.0)),
            avoidance_delay    = int(cfg.get("avoidance_delay", 245)),
            alpha_Ld           = float(cfg.get("alpha_Ld", 0.70)),
            offset             = float(cfg.get("offset", 324.0)),
            lane_width         = float(cfg.get("lane_width", 451.0)),
            obstacle_avoidance = True,
            x_L                = x_L,
        )
        # Slider overrides apply only when the user has actually moved a
        # slider away from its DEFAULT_PARAMS value — otherwise the slider's
        # starting position (legacy defaults) would silently overwrite the
        # tuned SEG3B_CFG values every replay.
        if slider_overrides:
            for k, v in slider_overrides.items():
                if k not in params:
                    continue
                default_v = DEFAULT_PARAMS.get(k)
                if default_v is None:
                    params[k] = v
                    continue
                if k == "avoidance_delay":
                    if int(round(v)) != int(default_v):
                        params[k] = int(round(v))
                elif abs(float(v) - float(default_v)) > 1e-6:
                    params[k] = float(v)
    else:
        d = _AVOID_OFF_DEFAULTS
        params = dict(
            lookahead_distance = float(cfg["lookahead"]),
            max_linear_speed   = float(cfg["speed"]),
            max_angular_speed  = d["max_angular"],
            goal_tolerance     = d["goal_tolerance"],
            obstacles_range    = d["obstacles_range"],
            view_angle         = math.radians(d["view_angle_deg"]),
            safe_dist          = d["safe_dist"],
            avoidance_delay    = d["avoidance_delay"],
            alpha_Ld           = d["alpha_Ld"],
            offset             = d["offset"],
            lane_width         = d["lane_width"],
            obstacle_avoidance = False,
            x_L                = x_L,
        )
    return params


def run_simulation_phased(
    slider_overrides: dict,
    cones_world: list[tuple[float, float]],
    scenario_key: str,
) -> dict:
    """
    Run the full SEG3 sequence: 3A (avoidance OFF) → 3B (avoidance ON) → 3C
    (avoidance OFF). Pose carries across phase boundaries exactly as the FSM
    does in venue_full_course_test.py.
    """
    sc = SCENARIOS[scenario_key]
    pose = list(sc["start_pose"])

    all_traj    = [tuple(pose[:2])]
    all_avoid   = [False]
    all_phase   = [-1]            # -1 = before any phase starts (initial point)
    detour_snaps = []
    paths_per_phase = []
    phase_end_steps = []          # absolute tick at end of each phase

    avoidance_triggers = 0
    was_avoiding = False
    total_steps  = 0
    status_per_phase: list[str] = []

    for phase_idx, phase in enumerate(sc["phases"]):
        cfg       = phase["cfg"]
        avoid_on  = phase["avoidance"]
        path_pts  = phase["path_pts"]
        spacing   = float(cfg.get("spacing", 50))
        path      = densify_polyline(path_pts, spacing=spacing)
        paths_per_phase.append(np.array(path))

        params  = _build_planner_params(cfg, avoid_on, sc["x_L"], slider_overrides)
        planner = PurePursuitPlannerWithAvoidance(**params)
        planner.set_path(path)

        phase_status = "TIMEOUT"
        budget = MAX_STEPS - total_steps
        for _ in range(budget):
            cos_th = math.cos(pose[2])
            sin_th = math.sin(pose[2])
            obs_r_list = []
            for cx, cy in cones_world:
                dx, dy = cx - pose[0], cy - pose[1]
                fwd  =  cos_th * dx + sin_th * dy
                left = -sin_th * dx + cos_th * dy
                obs_r_list.append([fwd, left])
            obs_r = np.array(obs_r_list) if obs_r_list else np.empty((0, 2))

            v, w = planner.compute_velocity(pose, obs_r)
            pose = planner.motion(pose, v, w, DT)

            active = planner.avoidance_active
            all_traj.append(tuple(pose[:2]))
            all_avoid.append(active)
            all_phase.append(phase_idx)
            detour_snaps.append(list(planner.remaining_path))

            if active and not was_avoiding:
                avoidance_triggers += 1
            was_avoiding = active
            total_steps += 1

            if planner.TargetReached(planner.remaining_path, pose[0], pose[1]):
                phase_status = "GOAL"
                break
            if not planner.remaining_path:
                phase_status = "EMPTY"
                break

        phase_end_steps.append(total_steps)
        status_per_phase.append(f"{phase['name']}: {phase_status}")
        if phase_status != "GOAL":
            break

    overall = "FULL SEG3 DONE" if all(s.endswith("GOAL") for s in status_per_phase) \
                                  and len(status_per_phase) == len(sc["phases"]) else "STOPPED EARLY"
    return dict(
        trajectory         = np.array(all_traj),
        avoidance          = all_avoid,
        phase              = all_phase,
        phase_end_steps    = phase_end_steps,
        detour_snaps       = detour_snaps,
        paths_per_phase    = paths_per_phase,
        final_pose         = pose,
        status             = overall,
        status_per_phase   = status_per_phase,
        steps              = total_steps,
        avoidance_triggers = avoidance_triggers,
    )


# ─────────────────────────────────────────────────────────────────────────────
# GUI state
# ─────────────────────────────────────────────────────────────────────────────

_cones: list[list[float]] = []   # [[x, y], ...] world frame, editable

# ─────────────────────────────────────────────────────────────────────────────
# Figure setup
# ─────────────────────────────────────────────────────────────────────────────

fig = plt.figure(figsize=(16, 10))
fig.patch.set_facecolor("#1e1e2e")

# Main 2D field  [left, bottom, width, height] in figure fraction
ax_field = fig.add_axes([0.04, 0.08, 0.54, 0.88])
ax_field.set_facecolor("#12121f")
ax_field.set_aspect("equal")
ax_field.tick_params(colors="white")
ax_field.title.set_color("white")
for spine in ax_field.spines.values():
    spine.set_edgecolor("#555577")

# Info text panel (no axes frame — just text via fig.text)
ax_info = fig.add_axes([0.60, 0.58, 0.38, 0.40])
ax_info.set_facecolor("#1a1a2e")
ax_info.axis("off")
for spine in ax_info.spines.values():
    spine.set_edgecolor("#555577")
_info_text = ax_info.text(
    0.02, 0.97, "", transform=ax_info.transAxes,
    va="top", ha="left", fontsize=8.5, color="white",
    fontfamily="monospace",
)

# ── Sliders ───────────────────────────────────────────────────────────────────
# Six key parameters as sliders.  Right column, evenly spaced.
_SLIDER_SPECS = [
    # (label,            attr_key,            vmin,   vmax,  valinit, fmt)
    ("Lookahead (mm)",   "lookahead_distance", 30.0,  500.0, DEFAULT_PARAMS["lookahead_distance"], "%.0f"),
    ("Safe dist (mm)",   "safe_dist",          30.0,  600.0, DEFAULT_PARAMS["safe_dist"],          "%.0f"),
    ("Offset (mm)",      "offset",             30.0,  500.0, DEFAULT_PARAMS["offset"],             "%.0f"),
    ("Obs range (mm)",   "obstacles_range",   100.0,  900.0, DEFAULT_PARAMS["obstacles_range"],    "%.0f"),
    ("Avoid delay (tk)", "avoidance_delay",     0.0,  700.0, float(DEFAULT_PARAMS["avoidance_delay"]), "%.0f"),
    ("alpha_Ld",         "alpha_Ld",            0.3,    2.5, DEFAULT_PARAMS["alpha_Ld"],           "%.2f"),
    ("Lane width (mm)",  "lane_width",         100.0,  900.0, DEFAULT_PARAMS["lane_width"],        "%.0f"),
    ("Speed (mm/s)",     "max_linear_speed",    30.0,  200.0, DEFAULT_PARAMS["max_linear_speed"],  "%.0f"),
]

_slider_axes   = []
_slider_widgets = []

_SLIDER_LEFT   = 0.62
_SLIDER_WIDTH  = 0.35
_SLIDER_HEIGHT = 0.025
_SLIDER_Y_TOP  = 0.54      # y of top slider
_SLIDER_VSTEP  = 0.063     # vertical gap between sliders

for i, (label, key, vmin, vmax, vinit, fmt) in enumerate(_SLIDER_SPECS):
    y = _SLIDER_Y_TOP - i * _SLIDER_VSTEP
    sax = fig.add_axes([_SLIDER_LEFT, y, _SLIDER_WIDTH, _SLIDER_HEIGHT])
    sax.set_facecolor("#2a2a3e")
    sl = Slider(
        sax, label, vmin, vmax, valinit=vinit,
        color="#5555aa", track_color="#2a2a3e",
    )
    sl.label.set_color("white")
    sl.label.set_fontsize(8)
    sl.valtext.set_color("#aaaaff")
    sl.valtext.set_fontsize(8)
    _slider_axes.append(sax)
    _slider_widgets.append(sl)

# ── Buttons ───────────────────────────────────────────────────────────────────
ax_btn_reset  = fig.add_axes([0.62, 0.04, 0.10, 0.035])
ax_btn_params = fig.add_axes([0.74, 0.04, 0.12, 0.035])
ax_btn_scene  = fig.add_axes([0.88, 0.04, 0.10, 0.035])

btn_reset  = Button(ax_btn_reset,  "Reset Cones",    color="#2a2a3e", hovercolor="#44446e")
btn_params = Button(ax_btn_params, "Default Params", color="#2a2a3e", hovercolor="#44446e")
btn_scene  = Button(ax_btn_scene,  "Toggle Scene",   color="#2a2a3e", hovercolor="#44446e")

for btn in (btn_reset, btn_params, btn_scene):
    btn.label.set_color("white")
    btn.label.set_fontsize(8.5)


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _current_params() -> dict:
    p = dict(DEFAULT_PARAMS)
    for i, (_, key, *_rest) in enumerate(_SLIDER_SPECS):
        val = _slider_widgets[i].val
        if key == "avoidance_delay":
            p[key] = int(round(val))
        else:
            p[key] = float(val)
    return p


def _draw_robot_arrow(ax, x, y, theta, length=100.0, width=50.0, color="#4488ff"):
    """Draw a filled arrow showing robot position and heading."""
    dx = math.cos(theta) * length
    dy = math.sin(theta) * length
    ax.annotate(
        "", xy=(x + dx, y + dy), xytext=(x, y),
        arrowprops=dict(
            arrowstyle=f"->,head_width={width/length:.2f},head_length=0.4",
            color=color, lw=2.5,
        ),
    )
    # Robot body rectangle
    bw, bl = width * 0.8, length * 0.9
    rect = mpatches.FancyBboxPatch(
        (x - bw / 2, y - bl * 0.3), bw, bl,
        boxstyle="round,pad=0.1",
        transform=ax.transData,
        angle=math.degrees(theta) - 90,
        color=color, alpha=0.35, linewidth=0,
    )
    # Can't rotate a patch easily without affine2d — just draw a circle
    ax.plot(x, y, "o", color=color, markersize=8, zorder=10)


def _params_summary(params: dict) -> str:
    avoid_s  = params["avoidance_delay"] / FSM_HZ
    view_deg = math.degrees(params.get("view_angle", math.radians(65)))
    return (
        f"─── Active Parameters ───────────────\n"
        f"  lookahead_distance : {params['lookahead_distance']:.0f} mm\n"
        f"  safe_dist          : {params['safe_dist']:.0f} mm\n"
        f"  offset             : {params['offset']:.0f} mm\n"
        f"  obstacles_range    : {params['obstacles_range']:.0f} mm\n"
        f"  avoidance_delay    : {params['avoidance_delay']} tks ({avoid_s:.1f} s)\n"
        f"  alpha_Ld           : {params['alpha_Ld']:.2f}\n"
        f"  lane_width         : {params['lane_width']:.0f} mm\n"
        f"  max_linear_speed   : {params['max_linear_speed']:.0f} mm/s\n"
        f"  view_angle         : {view_deg:.0f}°\n"
        f"  obstacle_avoidance : {params.get('obstacle_avoidance', True)}\n"
    )


def _draw_seg3_full_overlays(ax, sc):
    """Walls, lane bounds, and avoidance-on shading for the seg3_full scenario."""
    walls = sc.get("walls", {})
    bounds = sc.get("lane_bounds", {})
    az = sc.get("avoid_zone", {})

    xlo, xhi = sc["field_xlim"]

    # Avoidance-ON zone shading (lane corridor × y-range where SEG3B runs).
    if az:
        ax.axhspan(az["y_lo"], az["y_hi"], xmin=0, xmax=1,
                   color="#aa5500", alpha=0.10, zorder=0,
                   label="Avoidance ON (y-range)")
        # Narrow lane corridor where cones are actually considered (lane_width
        # filter inside the planner). Drawn as a vertical strip.
        ax.axvspan(az["x_lo"], az["x_hi"], ymin=0, ymax=1,
                   color="#ff8800", alpha=0.06, zorder=0,
                   label="Cone-consider corridor (lane_width)")

    # Physical walls — solid red horizontal lines.
    if "south_y" in walls:
        ax.axhline(walls["south_y"], color="#ff3333", linewidth=1.6,
                   linestyle="-", alpha=0.85, label="Wall (south/north)")
    if "north_y" in walls:
        ax.axhline(walls["north_y"], color="#ff3333", linewidth=1.6,
                   linestyle="-", alpha=0.85)

    # Lane 2/3 and 4/5 boundaries — yellow dashed verticals.
    if "lane23_x" in bounds:
        ax.axvline(bounds["lane23_x"], color="#ffcc44", linewidth=1.2,
                   linestyle="--", alpha=0.85, label="Lane 2/3 & 4/5 bound")
    if "lane45_x" in bounds:
        ax.axvline(bounds["lane45_x"], color="#ffcc44", linewidth=1.2,
                   linestyle="--", alpha=0.85)

    # Annotate the avoidance ON/OFF y-bands on the right side of the field.
    if az:
        ymin, ymax = sc["field_ylim"]
        ax.text(xhi - 50, ymin + 80, "3A  avoid OFF",
                color="#88cc88", fontsize=7.5, ha="right", va="bottom",
                fontfamily="monospace")
        ax.text(xhi - 50, (az["y_lo"] + az["y_hi"]) / 2.0,
                "3B  avoid ON", color="#ffaa66", fontsize=8.5, ha="right",
                va="center", fontfamily="monospace", fontweight="bold")
        ax.text(xhi - 50, ymax - 80, "3C  avoid OFF",
                color="#88cc88", fontsize=7.5, ha="right", va="top",
                fontfamily="monospace")
        # Horizontal dashed lines marking the exact ON/OFF transitions.
        ax.axhline(az["y_lo"], color="#ffaa66", linewidth=0.8,
                   linestyle=":", alpha=0.7)
        ax.axhline(az["y_hi"], color="#ffaa66", linewidth=0.8,
                   linestyle=":", alpha=0.7)


def _replay_phased():
    """Replay path for scenarios with phases=True (currently seg3_full)."""
    sc = SCENARIOS[_active_scenario_key]
    params = _current_params()
    cones = [(c[0], c[1]) for c in _cones]
    result = run_simulation_phased(params, cones, _active_scenario_key)

    ax_field.cla()
    ax_field.set_facecolor("#12121f")
    ax_field.set_aspect("equal")
    ax_field.set_xlim(*sc["field_xlim"])
    ax_field.set_ylim(*sc["field_ylim"])
    ax_field.set_xlabel("x (mm) — east→", color="#aaaacc", fontsize=8)
    ax_field.set_ylabel("y (mm) — north↑", color="#aaaacc", fontsize=8)
    ax_field.set_title(sc["label"], color="white", fontsize=9)
    ax_field.tick_params(colors="#888899", labelsize=7)
    ax_field.grid(True, color="#2a2a4a", linewidth=0.5, linestyle="--")

    _draw_seg3_full_overlays(ax_field, sc)

    # Reference paths — one per phase, each in its own colour.
    phase_colors = ["#44aa44", "#aaaa44", "#aa44aa"]   # 3A green, 3B yellow, 3C purple
    phase_labels = [p["name"] for p in sc["phases"]]
    for idx, path in enumerate(result["paths_per_phase"]):
        ax_field.plot(path[:, 0], path[:, 1], "--", color=phase_colors[idx],
                      linewidth=1.0, alpha=0.55,
                      label=f"Ref {phase_labels[idx]}", zorder=2)

    # Trajectory — split by phase AND avoidance flag for colour.
    traj  = result["trajectory"]
    flags = result["avoidance"]
    phs   = result["phase"]
    # Skip the initial sentinel index 0 (phase=-1).
    i = 1
    legend_seen: set[str] = set()
    while i < len(traj) - 1:
        j = i + 1
        while j < len(flags) and flags[j] == flags[i] and phs[j] == phs[i]:
            j += 1
        seg = traj[i:j + 1]
        if flags[i]:
            color = "#ff8800"
            lbl   = "Avoidance active"
        else:
            color = ["#66ccff", "#4488ff", "#dd66dd"][phs[i]]
            lbl   = f"Driving — {phase_labels[phs[i]]}"
        plot_lbl = lbl if lbl not in legend_seen else None
        legend_seen.add(lbl)
        ax_field.plot(seg[:, 0], seg[:, 1], color=color, linewidth=2.0,
                      label=plot_lbl, zorder=5)
        i = j

    # Mark phase-boundary points along the trajectory so the user can see
    # exactly where avoidance turns on / off relative to the wall + last cone.
    for end_step in result["phase_end_steps"][:-1]:
        if 0 < end_step < len(traj):
            px, py = traj[end_step]
            ax_field.plot(px, py, "D", color="#ffffff", markersize=8,
                          markeredgecolor="#000000", markeredgewidth=0.8,
                          zorder=11)
            ax_field.text(px + 30, py, f"phase end @ t={end_step}",
                          color="white", fontsize=6.5, va="center",
                          fontfamily="monospace")

    # Detour waypoint snapshots from the first avoidance activation.
    for snap in result["detour_snaps"]:
        if snap and len(snap) > 1:
            wx = [w[0] for w in snap]
            wy = [w[1] for w in snap]
            ax_field.plot(wx, wy, "o--", color="#ff8800", markersize=3,
                          linewidth=0.8, alpha=0.35, zorder=3)
            break

    # Start, goal (CP3), final pose, cones.
    sp = sc["start_pose"]
    ax_field.plot(sp[0], sp[1], "^", color="#44ee44", markersize=10,
                  zorder=8, label="Start (CP2)")
    cp3 = sc["phases"][-1]["path_pts"][-1]
    ax_field.plot(cp3[0], cp3[1], "*", color="#ffff44", markersize=12,
                  zorder=8, label="Goal (CP3)")
    fp = result["final_pose"]
    ax_field.annotate(
        "", xy=(fp[0] + 80 * math.cos(fp[2]), fp[1] + 80 * math.sin(fp[2])),
        xytext=(fp[0], fp[1]),
        arrowprops=dict(arrowstyle="->,head_width=0.3,head_length=0.4",
                        color="#4488ff", lw=2.5),
        zorder=10,
    )
    ax_field.plot(fp[0], fp[1], "o", color="#4488ff", markersize=9, zorder=10,
                  label="Final pose")

    safe_d = params["safe_dist"]
    for ci, (cx, cy) in enumerate(cones):
        ax_field.add_patch(plt.Circle((cx, cy), CONE_RADIUS, color="#ff3333",
                                       alpha=0.9, zorder=7))
        ax_field.add_patch(plt.Circle((cx, cy), safe_d, color="#ff3333",
                                       fill=False, linestyle=":", linewidth=0.9,
                                       alpha=0.35, zorder=6))
        ax_field.text(cx, cy + CONE_RADIUS + 18, str(ci + 1),
                      color="#ffaaaa", fontsize=7, ha="center", va="bottom",
                      zorder=8)

    ax_field.legend(loc="upper left", fontsize=6.5, facecolor="#1a1a2e",
                    labelcolor="white", framealpha=0.85, ncol=1)

    # Status bar
    secs = result["steps"] * DT
    dist = float(np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1))) / 1000.0
    status_color = "#44ee44" if result["status"] == "FULL SEG3 DONE" else "#ff8844"
    ax_field.text(
        0.02, 0.01,
        f"Status: {result['status']}  |  {result['steps']} ticks ({secs:.1f} s)  "
        f"|  dist {dist:.2f} m  |  {result['avoidance_triggers']} avoidance trigger(s)  "
        f"|  {len(cones)} cone(s)",
        transform=ax_field.transAxes, va="bottom", ha="left",
        fontsize=7.5, color=status_color, fontfamily="monospace",
    )

    # Info panel: per-phase status + final pose summary + params.
    fp_x, fp_y, fp_th = result["final_pose"]
    fp_th_deg = math.degrees(fp_th)
    # Lateral offset from centerline at end of SEG3B is the key diagnostic for
    # the "diagonal up-left into the wall" failure mode.
    seg3b_end_idx = result["phase_end_steps"][1] if len(result["phase_end_steps"]) >= 2 else None
    seg3b_end_str = ""
    if seg3b_end_idx is not None and seg3b_end_idx < len(traj):
        ex, ey = traj[seg3b_end_idx]
        seg3b_end_str = (
            f"  SEG3B exit pose : x={ex:+.0f}  y={ey:+.0f}  "
            f"(offset from x_L={ex - X_OBS_MID:+.0f} mm)\n"
        )

    info = (
        "─── Phase status ────────────────────\n"
        + "\n".join(f"  {s}" for s in result["status_per_phase"])
        + f"\n\n─── Final pose ──────────────────────\n"
        + f"  x={fp_x:+.0f}  y={fp_y:+.0f}  θ={fp_th_deg:+.1f}°\n"
        + seg3b_end_str
        + f"\n─── SEG3B (avoidance) params ────────\n"
        + _params_summary(params)
        + f"\n─── Cone positions (mm) ─────────────\n"
        + "\n".join(f"  #{i+1}: ({c[0]:+.0f}, {c[1]:+.0f})" for i, c in enumerate(cones))
        + ("\n  (none)" if not cones else "")
        + f"\n\n─── Tip ─────────────────────────────\n"
          f"  Left-click  → add cone\n"
          f"  Right-click → remove nearest cone\n"
          f"  Sliders affect SEG3B (avoidance) only.\n"
    )
    _info_text.set_text(info)
    fig.canvas.draw_idle()


def replay():
    """Re-run the simulation with current params and cones and redraw."""
    sc = SCENARIOS[_active_scenario_key]
    if sc.get("phased"):
        _replay_phased()
        return

    params = _current_params()
    cones = [(c[0], c[1]) for c in _cones]

    result = run_simulation(params, cones, _active_scenario_key)

    ax_field.cla()
    ax_field.set_facecolor("#12121f")
    ax_field.set_aspect("equal")
    ax_field.set_xlim(*sc["field_xlim"])
    ax_field.set_ylim(*sc["field_ylim"])
    ax_field.set_xlabel("x (mm) — east→", color="#aaaacc", fontsize=8)
    ax_field.set_ylabel("y (mm) — north↑", color="#aaaacc", fontsize=8)
    ax_field.set_title(sc["label"], color="white", fontsize=9)
    ax_field.tick_params(colors="#888899", labelsize=7)
    ax_field.grid(True, color="#2a2a4a", linewidth=0.5, linestyle="--")

    # Lane boundary lines
    for lx in sc["lane_lines_x"]:
        ax_field.axvline(lx, color="#555588", linewidth=0.8, linestyle="--", alpha=0.7)

    # Reference path (green dashed)
    path = result["path"]
    ax_field.plot(path[:, 0], path[:, 1], "--", color="#44aa44", linewidth=1.2,
                  alpha=0.6, label="Reference path", zorder=2)

    # Trajectory — split by avoidance state for colour
    traj = result["trajectory"]
    flags = result["avoidance"]
    i = 0
    first_normal = True
    first_avoid  = True
    while i < len(traj) - 1:
        j = i + 1
        while j < len(flags) and flags[j] == flags[i]:
            j += 1
        seg = traj[i:j + 1]
        if flags[i]:
            lbl = "Avoidance active" if first_avoid else None
            ax_field.plot(seg[:, 0], seg[:, 1], color="#ff8800", linewidth=2.0,
                          label=lbl, zorder=5)
            first_avoid = False
        else:
            lbl = "Normal driving" if first_normal else None
            ax_field.plot(seg[:, 0], seg[:, 1], color="#4488ff", linewidth=1.8,
                          label=lbl, zorder=5)
            first_normal = False
        i = j

    # Detour waypoints at the moment of first avoidance trigger
    for snap in result["detour_snaps"]:
        if snap and len(snap) > 1:
            wx = [w[0] for w in snap]
            wy = [w[1] for w in snap]
            ax_field.plot(wx, wy, "o--", color="#ff8800", markersize=3,
                          linewidth=0.8, alpha=0.35, zorder=3)
            break

    # Start marker
    sp = sc["start_pose"]
    ax_field.plot(sp[0], sp[1], "^", color="#44ee44", markersize=10,
                  zorder=8, label="Start")

    # Goal marker (last path point)
    ax_field.plot(path[-1, 0], path[-1, 1], "*", color="#ffff44", markersize=12,
                  zorder=8, label="Goal")

    # Final robot position
    fp = result["final_pose"]
    ax_field.annotate(
        "",
        xy=(fp[0] + 80 * math.cos(fp[2]), fp[1] + 80 * math.sin(fp[2])),
        xytext=(fp[0], fp[1]),
        arrowprops=dict(arrowstyle="->,head_width=0.3,head_length=0.4",
                        color="#4488ff", lw=2.5),
        zorder=10,
    )
    ax_field.plot(fp[0], fp[1], "o", color="#4488ff", markersize=9, zorder=10,
                  label="Final pose")

    # Cones
    safe_d = params["safe_dist"]
    for ci, (cx, cy) in enumerate(cones):
        # Cone body
        cone_patch = plt.Circle((cx, cy), CONE_RADIUS, color="#ff3333",
                                 alpha=0.9, zorder=7)
        ax_field.add_patch(cone_patch)
        # safe_dist ring (avoidance triggers when next waypoint enters this)
        safe_ring = plt.Circle((cx, cy), safe_d, color="#ff3333",
                               fill=False, linestyle=":", linewidth=0.9,
                               alpha=0.35, zorder=6)
        ax_field.add_patch(safe_ring)
        ax_field.text(cx, cy + CONE_RADIUS + 18, str(ci + 1),
                      color="#ffaaaa", fontsize=7, ha="center", va="bottom",
                      zorder=8)

    ax_field.legend(loc="upper right", fontsize=7, facecolor="#1a1a2e",
                    labelcolor="white", framealpha=0.8)

    # Status info
    secs = result["steps"] * DT
    dist = float(np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1))) / 1000.0
    status_color = "#44ee44" if result["status"] == "GOAL REACHED" else "#ff4444"
    ax_field.text(
        0.02, 0.01,
        f"Status: {result['status']}  |  {result['steps']} ticks ({secs:.1f} s)  "
        f"|  dist {dist:.2f} m  |  {result['avoidance_triggers']} avoidance trigger(s)  "
        f"|  {len(cones)} cone(s)",
        transform=ax_field.transAxes, va="bottom", ha="left",
        fontsize=7.5, color=status_color, fontfamily="monospace",
    )

    # Info panel text
    info = (
        f"─── Simulation Result ───────────────\n"
        f"  Status            : {result['status']}\n"
        f"  Steps             : {result['steps']} ({secs:.1f} s)\n"
        f"  Distance          : {dist:.3f} m\n"
        f"  Avoidance triggers: {result['avoidance_triggers']}\n"
        f"  Cones             : {len(cones)}\n"
        f"\n"
        + _params_summary(params)
        + f"\n─── Cone positions (mm) ─────────────\n"
        + "\n".join(f"  #{i+1}: ({c[0]:+.0f}, {c[1]:+.0f})" for i, c in enumerate(cones))
        + ("\n  (none)" if not cones else "")
        + f"\n\n─── Tip ─────────────────────────────\n"
          f"  Left-click  → add cone\n"
          f"  Right-click → remove nearest cone\n"
    )
    _info_text.set_text(info)

    fig.canvas.draw_idle()


# ─────────────────────────────────────────────────────────────────────────────
# Event handlers
# ─────────────────────────────────────────────────────────────────────────────

def _on_slider_change(_val):
    replay()


def _on_click(event):
    """Add cone on left-click; remove nearest on right-click."""
    if event.inaxes is not ax_field:
        return
    x, y = event.xdata, event.ydata
    if x is None or y is None:
        return

    if event.button == 1:   # left click → add
        _cones.append([x, y])
        replay()

    elif event.button == 3:  # right click → remove nearest within 120mm
        if not _cones:
            return
        dists = [math.hypot(c[0] - x, c[1] - y) for c in _cones]
        nearest_i = int(np.argmin(dists))
        if dists[nearest_i] < 120.0:
            _cones.pop(nearest_i)
            replay()


def _on_reset_cones(_event):
    global _cones
    sc = SCENARIOS[_active_scenario_key]
    _cones = [list(c) for c in sc["default_cones"]]
    replay()


def _on_default_params(_event):
    for i, (_, key, *_rest) in enumerate(_SLIDER_SPECS):
        val = DEFAULT_PARAMS[key]
        if key == "avoidance_delay":
            val = float(val)
        _slider_widgets[i].set_val(val)
    # replay triggered by slider on_changed callbacks


def _on_toggle_scene(_event):
    global _active_scenario_key
    keys = list(SCENARIOS.keys())
    idx = keys.index(_active_scenario_key)
    _active_scenario_key = keys[(idx + 1) % len(keys)]
    # Reset cones to default for new scenario
    _cones.clear()
    _cones.extend([list(c) for c in SCENARIOS[_active_scenario_key]["default_cones"]])
    replay()


# ── Wire up events ─────────────────────────────────────────────────────────────
for sl in _slider_widgets:
    sl.on_changed(_on_slider_change)

fig.canvas.mpl_connect("button_press_event", _on_click)
btn_reset.on_clicked(_on_reset_cones)
btn_params.on_clicked(_on_default_params)
btn_scene.on_clicked(_on_toggle_scene)

# ─────────────────────────────────────────────────────────────────────────────
# Initial run
# ─────────────────────────────────────────────────────────────────────────────

_cones.extend([list(c) for c in SCENARIOS[_active_scenario_key]["default_cones"]])
replay()

plt.show()
