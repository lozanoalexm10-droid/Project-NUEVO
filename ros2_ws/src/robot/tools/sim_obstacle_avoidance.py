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
    # Full Segment 3: CP2 → east → 90° left → north through cones → 90° right →
    # east → CP3.  Mirrors venue_full_course_test.py's three SEG3 sub-paths
    # joined into one continuous trajectory.  The planner runs avoidance for
    # the entire path here (unlike real life where 3A/3C disable avoidance
    # around the corners) — useful for confirming the avoidance doesn't fire
    # spuriously during the cornering phases when no cones are nearby.
    "venue_seg3_full": dict(
        path_pts      = [
            (1067.5,  610.0),   # CP2
            (1525.0,  610.0),   # 90° left corner (east → north)
            (1525.0, 3500.0),   # 90° right corner (north → east)
            (1982.5, 3500.0),   # CP3
        ],
        spacing       = 400.0,
        start_pose    = [1067.5, 610.0, 0.0],   # CP2 facing east
        x_L           = 1525.0,                  # avoidance lane centre = SEG3B centreline
        field_xlim    = (   900.0, 2150.0),
        field_ylim    = (   400.0, 3700.0),
        lane_lines_x  = [1525.0 - 225.5, 1525.0 + 225.5],
        label         = "Venue SEG3 FULL (CP2 → cones → CP3)",
        default_cones = [
            (1830.0, 1565.0),
            (1220.0, 2175.0),
            (1830.0, 2785.0),
        ],
    ),
}

_active_scenario_key = "lane_switch"

# ── Default planner params (tuned values from sim session 2026-06-05) ─────────
DEFAULT_PARAMS = dict(
    lookahead_distance = 115.0,
    max_linear_speed   = 75.0,
    max_angular_speed  = 1.0,
    goal_tolerance     = 30.0,
    obstacles_range    = 450.0,
    view_angle         = math.radians(70.0),
    safe_dist          = 181.0,
    avoidance_delay    = 245,
    alpha_Ld           = 0.70,
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


def replay():
    """Re-run the simulation with current params and cones and redraw."""
    sc = SCENARIOS[_active_scenario_key]
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
