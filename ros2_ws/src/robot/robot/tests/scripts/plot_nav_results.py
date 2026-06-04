"""
plot_nav_results.py — offline presentation figure generator.

Requires: matplotlib, numpy  (pip install matplotlib numpy)
Does NOT require ROS2 — run this on your Mac after copying the JSON data file.

Usage:
    # Just the FSM diagram (no run data needed):
    python3 plot_nav_results.py --fsm

    # Trajectory + vision figures from a recorded run:
    python3 plot_nav_results.py nav_run_20260603_142000.json

    # All figures:
    python3 plot_nav_results.py nav_run_20260603_142000.json --fsm

Output: PNG files written next to this script (or --out-dir to change).
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

# ── Venue geometry (must match venue_full_course_test.py) ─────────────────────
GRID_MM       = 610.0
X_LANE1       =    0.0
X_LANE2       =  610.0
X_OBS_MID     = 1525.0
X_LANE5       = 2440.0
Y_START       = -160.0
Y_BOT         =  610.0
Y_TOP         = 3500.0
FINISH_X      = 2440.0
FINISH_Y      =  305.0
CP1           = (  305.0, 3660.0)
CP2           = ( 1067.5,  610.0)
CP3           = ( 1982.5, 3660.0)
VENUE_LEFT    = -305.0
VENUE_RIGHT   = 2745.0
VENUE_BOTTOM  =    0.0
VENUE_TOP_ABS = 3965.0

# Planned path waypoints (in absolute course frame)
_PLANNED_PATH = [
    (X_LANE1, Y_START),
    (X_LANE1, Y_TOP),
    (X_LANE1 + 305, Y_TOP),       # CP1
    (X_LANE2 + 20,  Y_TOP),
    (X_LANE2 + 20,  Y_BOT),
    (X_LANE2 + 457.5, Y_BOT),     # CP2
    (X_OBS_MID, Y_BOT),
    (X_OBS_MID, Y_TOP),
    (X_OBS_MID + 457.5, Y_TOP),   # CP3
    (X_LANE5, Y_TOP),
    (FINISH_X, FINISH_Y),
]

# ── Colour palette (matches the dark-theme NUEVO aesthetic) ───────────────────
BG        = "#1a1f2e"
PANEL_BG  = "#252b3d"
GRID_CLR  = "#2d3550"
TEXT_CLR  = "#c8d0e8"
ODOM_CLR  = "#60a5fa"   # blue
FUSED_CLR = "#4ade80"   # green
PLAN_CLR  = "#facc15"   # yellow
LIDAR_CLR = "#f87171"   # red (low alpha)
GPS_DOT   = "#22d3ee"   # cyan


def _dark_fig(w: float, h: float):
    fig, ax = plt.subplots(figsize=(w, h), facecolor=BG)
    ax.set_facecolor(PANEL_BG)
    for spine in ax.spines.values():
        spine.set_edgecolor(GRID_CLR)
    ax.tick_params(colors=TEXT_CLR)
    ax.xaxis.label.set_color(TEXT_CLR)
    ax.yaxis.label.set_color(TEXT_CLR)
    ax.title.set_color(TEXT_CLR)
    return fig, ax


def _dark_fig_multi(rows: int, cols: int, w: float, h: float):
    fig, axes = plt.subplots(rows, cols, figsize=(w, h), facecolor=BG)
    fig.patch.set_facecolor(BG)
    for ax in (axes.flat if hasattr(axes, "flat") else [axes]):
        ax.set_facecolor(PANEL_BG)
        for spine in ax.spines.values():
            spine.set_edgecolor(GRID_CLR)
        ax.tick_params(colors=TEXT_CLR)
        ax.xaxis.label.set_color(TEXT_CLR)
        ax.yaxis.label.set_color(TEXT_CLR)
        ax.title.set_color(TEXT_CLR)
    return fig, axes


# ── Figure 1: FSM diagram ─────────────────────────────────────────────────────

def _box(ax, x, y, w, h, label, color, fontsize=9, text_color="white"):
    rect = mpatches.FancyBboxPatch(
        (x - w / 2, y - h / 2), w, h,
        boxstyle="round,pad=0.02",
        facecolor=color, edgecolor="white", linewidth=1.2, alpha=0.9,
    )
    ax.add_patch(rect)
    ax.text(x, y, label, ha="center", va="center",
            fontsize=fontsize, color=text_color, fontweight="bold",
            wrap=True, multialignment="center")


def _arrow(ax, x0, y0, x1, y1, label="", color="white", rad=0.0):
    ax.annotate(
        "", xy=(x1, y1), xytext=(x0, y0),
        arrowprops=dict(arrowstyle="-|>", color=color, lw=1.2,
                        connectionstyle=f"arc3,rad={rad}"),
    )
    if label:
        mx, my = (x0 + x1) / 2, (y0 + y1) / 2
        ax.text(mx, my, label, ha="center", va="center",
                fontsize=7, color="#facc15",
                bbox=dict(facecolor=BG, edgecolor="none", alpha=0.7, pad=1.5))


def plot_fsm_diagram(out_dir: Path) -> Path:
    fig, ax = plt.subplots(figsize=(13, 7), facecolor=BG)
    ax.set_facecolor(BG)
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 7)
    ax.axis("off")
    ax.set_title("Venue Full-Course FSM", color=TEXT_CLR, fontsize=13, fontweight="bold", pad=12)

    # ── States ────────────────────────────────────────────────────────────────
    states = {
        "IDLE":  (1.2, 3.5),
        "SEG1":  (3.2, 5.5),
        "SEG2":  (5.4, 5.5),
        "SEG3":  (7.5, 3.5),
        "SEG4":  (10.2, 3.5),
        "DONE":  (12.0, 3.5),
    }
    colors = {
        "IDLE": "#374151", "SEG1": "#1d4ed8", "SEG2": "#1d4ed8",
        "SEG3": "#0f766e", "SEG4": "#7c3aed", "DONE": "#166534",
    }
    for name, (x, y) in states.items():
        _box(ax, x, y, 1.5, 0.7, name, colors[name])

    # SEG3 sub-states
    sub_y   = 1.8
    sub_xs  = [5.8, 7.5, 9.2]
    sub_lbl = ["3A\nApproach\n(avoid OFF)", "3B\nObstacle\n(avoid ON)", "3C\nExit + turn\n(lidar reset)"]
    sub_clr = ["#0f766e", "#065f46", "#0f766e"]
    for x, lbl, clr in zip(sub_xs, sub_lbl, sub_clr):
        _box(ax, x, sub_y, 1.6, 1.1, lbl, clr, fontsize=7.5)
    # bracket
    ax.annotate("", xy=(sub_xs[0] - 0.8, sub_y + 0.8),
                 xytext=(sub_xs[-1] + 0.8, sub_y + 0.8),
                 arrowprops=dict(arrowstyle="-", color="#4ade80", lw=1))
    ax.text(7.5, 2.75, "SEG3 sub-phases", ha="center", fontsize=7.5,
            color="#4ade80")

    # ── Main transitions ──────────────────────────────────────────────────────
    ix, iy = states["IDLE"]
    # IDLE → SEG1
    ax.annotate("", xy=(states["SEG1"][0] - 0.75, states["SEG1"][1]),
                 xytext=(ix + 0.75, iy + 0.25),
                 arrowprops=dict(arrowstyle="-|>", color="white", lw=1.1,
                                 connectionstyle="arc3,rad=-0.25"))
    ax.text(2.1, 4.9, "green light /\nBTN_1 / 3 s", ha="center", fontsize=6.5,
            color="#facc15",
            bbox=dict(facecolor=BG, edgecolor="none", alpha=0.7, pad=1.5))

    # IDLE → any SEG (restart)
    ax.text(1.2, 2.6, "START_SEGMENT=N\n(checkpoint restart)", ha="center",
            fontsize=6.5, color="#94a3b8",
            bbox=dict(facecolor=BG, edgecolor="none", alpha=0.5, pad=1))

    # SEG1 → SEG2
    _arrow(ax, states["SEG1"][0] + 0.75, states["SEG1"][1],
               states["SEG2"][0] - 0.75, states["SEG2"][1],
           "CP1 reached")

    # SEG2 → SEG3
    ax.annotate("", xy=(states["SEG3"][0] - 0.75, states["SEG3"][1] + 0.2),
                 xytext=(states["SEG2"][0] + 0.75, states["SEG2"][1]),
                 arrowprops=dict(arrowstyle="-|>", color="white", lw=1.1,
                                 connectionstyle="arc3,rad=0.25"))
    ax.text(6.7, 5.0, "CP2 reached", ha="center", fontsize=6.5, color="#facc15",
            bbox=dict(facecolor=BG, edgecolor="none", alpha=0.7, pad=1.5))

    # SEG3 → SEG4
    _arrow(ax, states["SEG3"][0] + 0.75, states["SEG3"][1],
               states["SEG4"][0] - 0.75, states["SEG4"][1],
           "CP3 reached")

    # SEG4 → DONE
    _arrow(ax, states["SEG4"][0] + 0.75, states["SEG4"][1],
               states["DONE"][0] - 0.75, states["DONE"][1],
           "finish")

    # BTN_2 estop (dashed, from anywhere)
    ax.annotate("", xy=(ix + 0.0, iy - 0.35),
                 xytext=(states["SEG4"][0], states["SEG4"][1] - 0.35),
                 arrowprops=dict(arrowstyle="-|>", color="#f87171", lw=1,
                                 linestyle="dashed",
                                 connectionstyle="arc3,rad=0.3"))
    ax.text(5.5, 2.3, "BTN_2 → E-STOP / shutdown", ha="center",
            fontsize=6.5, color="#f87171")

    # SEG3 sub-phase arrows
    _arrow(ax, sub_xs[0] + 0.8, sub_y, sub_xs[1] - 0.8, sub_y, "OBS entry")
    _arrow(ax, sub_xs[1] + 0.8, sub_y, sub_xs[2] - 0.8, sub_y, "OBS exit +\nlidar reset")

    # Vertical link: SEG3 main box ↔ sub-states
    _arrow(ax, 7.5, states["SEG3"][1] - 0.35, 7.5, sub_y + 0.55, "")

    # Stop-sign annotation on SEG4
    ax.text(states["SEG4"][0], states["SEG4"][1] - 0.55,
            "stop sign:\n3 s dwell", ha="center", fontsize=6.5, color="#fbbf24",
            bbox=dict(facecolor="#78350f", edgecolor="#fbbf24",
                      alpha=0.7, pad=1.5, boxstyle="round"))

    # IDLE inset: two start modes
    ax.text(ix, iy + 1.1, "AUTO_START=True\n→ 3 s countdown",
            ha="center", fontsize=6.5, color="#94a3b8",
            bbox=dict(facecolor=PANEL_BG, edgecolor=GRID_CLR, alpha=0.8,
                      pad=2, boxstyle="round"))
    ax.text(ix, iy + 1.85, "AUTO_START=False\n→ traffic light watch\n(campan −30°)",
            ha="center", fontsize=6.5, color="#94a3b8",
            bbox=dict(facecolor=PANEL_BG, edgecolor=GRID_CLR, alpha=0.8,
                      pad=2, boxstyle="round"))

    fig.tight_layout()
    out = out_dir / "fig_fsm_diagram.png"
    fig.savefig(out, dpi=180, bbox_inches="tight", facecolor=BG)
    plt.close(fig)
    print(f"[plot] saved {out}")
    return out


# ── Figure 2: Trajectory map ──────────────────────────────────────────────────

def plot_trajectory_map(data: dict, out_dir: Path) -> Path:
    odom  = data.get("odom",  [])
    fused = data.get("fused", [])
    lidar = data.get("lidar", [])

    fig, ax = _dark_fig(9, 11)
    ax.set_aspect("equal")
    ax.set_xlabel("X  (mm, east)", fontsize=9)
    ax.set_ylabel("Y  (mm, north)", fontsize=9)
    ax.set_title("Navigation Trajectory — Odometry vs GPS-Fused", fontsize=11)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(610))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(610))
    ax.grid(True, color=GRID_CLR, linewidth=0.5)

    # Venue boundary
    vx = [VENUE_LEFT, VENUE_RIGHT, VENUE_RIGHT, VENUE_LEFT, VENUE_LEFT]
    vy = [VENUE_BOTTOM, VENUE_BOTTOM, VENUE_TOP_ABS, VENUE_TOP_ABS, VENUE_BOTTOM]
    ax.plot(vx, vy, color="#475569", linewidth=1.5, linestyle="--", label="Venue boundary")

    # Lane centre lines
    for lx, name in [(X_LANE1, "L1"), (X_LANE2, "L2"), (X_OBS_MID, "Obs"), (X_LANE5, "L5")]:
        ax.axvline(lx, color=GRID_CLR, linewidth=0.6, linestyle=":")
        ax.text(lx, VENUE_BOTTOM - 130, name, color="#64748b", ha="center", fontsize=7)

    # Planned path
    px, py = zip(*_PLANNED_PATH)
    ax.plot(px, py, color=PLAN_CLR, linewidth=1.4, linestyle="--",
            label="Planned path", alpha=0.7, zorder=3)

    # Lidar point cloud (obstacle zone only: x 900–2100, y 1000–3500)
    if lidar:
        all_wx = [x for snap in lidar for x in snap["xs"]
                  if 900 < x < 2100 and 1000 < snap["robot_y"] < 3500]
        all_wy = [y for snap in lidar for y, x in
                  zip(snap["ys"], snap["xs"]) if 900 < x < 2100
                  and 1000 < snap["robot_y"] < 3500]
        if all_wx:
            ax.scatter(all_wx, all_wy, s=1.5, color=LIDAR_CLR, alpha=0.25,
                       label="Lidar obstacles", zorder=2)

    # Odometry
    if odom:
        ox = [p["x"] for p in odom]
        oy = [p["y"] for p in odom]
        ax.plot(ox, oy, color=ODOM_CLR, linewidth=1.8, label="Odometry", zorder=5)
        ax.scatter([ox[0]], [oy[0]], color="#22c55e", s=60, zorder=7, label="Start")
        ax.scatter([ox[-1]], [oy[-1]], color="#ef4444", s=60, zorder=7, label="End")

    # Fused pose
    if fused:
        fx = [p["x"] for p in fused]
        fy = [p["y"] for p in fused]
        ax.plot(fx, fy, color=FUSED_CLR, linewidth=1.4,
                linestyle="--", label="GPS-fused", zorder=6, alpha=0.85)
        # Highlight GPS-active segments
        gps_x = [p["x"] for p in fused if p["gps_active"]]
        gps_y = [p["y"] for p in fused if p["gps_active"]]
        if gps_x:
            ax.scatter(gps_x, gps_y, s=8, color=GPS_DOT, alpha=0.5,
                       label="GPS active", zorder=6)

    # Checkpoints
    for name, (cx, cy) in [("CP1", CP1), ("CP2", CP2), ("CP3", CP3)]:
        ax.scatter([cx], [cy], s=100, marker="D", color="#fbbf24",
                   edgecolors="white", linewidth=0.8, zorder=8)
        ax.text(cx + 60, cy + 60, name, color="#fbbf24", fontsize=7.5)

    # Obstacle zone shading
    obs_rect = mpatches.Rectangle(
        (X_OBS_MID - 300, Y_BOT + 400), 600, Y_TOP - Y_BOT - 800,
        facecolor="#dc2626", alpha=0.05, edgecolor="#dc2626",
        linewidth=0.8, linestyle=":")
    ax.add_patch(obs_rect)
    ax.text(X_OBS_MID, (Y_BOT + Y_TOP) / 2, "Obstacle\nZone",
            color="#f87171", ha="center", va="center", fontsize=8, alpha=0.6)

    legend = ax.legend(fontsize=7.5, loc="upper left",
                       facecolor=PANEL_BG, edgecolor=GRID_CLR,
                       labelcolor=TEXT_CLR)
    fig.tight_layout()
    out = out_dir / "fig_trajectory_map.png"
    fig.savefig(out, dpi=180, bbox_inches="tight", facecolor=BG)
    plt.close(fig)
    print(f"[plot] saved {out}")
    return out


# ── Figure 3: Heading + GPS correction ───────────────────────────────────────

def plot_heading_fusion(data: dict, out_dir: Path) -> Path:
    odom  = data.get("odom",  [])
    fused = data.get("fused", [])
    if not odom:
        print("[plot] no odom data — skipping heading figure")
        return None

    fig, axes = _dark_fig_multi(2, 1, 10, 6)
    ax_h, ax_e = axes
    fig.suptitle("Sensor Fusion — Heading & Position", color=TEXT_CLR,
                 fontsize=11, fontweight="bold")

    # Heading over time
    ot = [p["t"] for p in odom]
    oh = [math.degrees(p["theta"]) for p in odom]
    ax_h.plot(ot, oh, color=ODOM_CLR, linewidth=1.5, label="Odometry heading")
    if fused:
        ft = [p["t"] for p in fused]
        fh = [math.degrees(p["theta"]) for p in fused]
        ax_h.plot(ft, fh, color=FUSED_CLR, linewidth=1.5,
                  linestyle="--", label="Fused heading")
    ax_h.set_ylabel("Heading (°)", fontsize=9)
    ax_h.set_ylim(-200, 200)
    ax_h.axhline(0,   color=GRID_CLR, linewidth=0.6, linestyle=":")
    ax_h.axhline(90,  color=GRID_CLR, linewidth=0.6, linestyle=":")
    ax_h.axhline(-90, color=GRID_CLR, linewidth=0.6, linestyle=":")
    ax_h.grid(True, color=GRID_CLR, linewidth=0.4)
    ax_h.legend(fontsize=8, facecolor=PANEL_BG, edgecolor=GRID_CLR, labelcolor=TEXT_CLR)
    ax_h.tick_params(colors=TEXT_CLR)

    # Position correction magnitude (fused − odom)
    if fused and odom:
        ot_set = {p["t"]: p for p in odom}
        ft_set = {p["t"]: p for p in fused}
        common_t = sorted(set(ot_set) & set(ft_set))
        if common_t:
            diffs = [
                math.hypot(ft_set[t]["x"] - ot_set[t]["x"],
                           ft_set[t]["y"] - ot_set[t]["y"])
                for t in common_t
            ]
            gps_active = [ft_set[t]["gps_active"] for t in common_t]
            ax_e.plot(common_t, diffs, color=FUSED_CLR, linewidth=1.5,
                      label="|fused − odom| (mm)")
            # Shade GPS-active regions
            in_gps = False
            t_start = 0
            for ti, gps in zip(common_t, gps_active):
                if gps and not in_gps:
                    t_start = ti
                    in_gps = True
                elif not gps and in_gps:
                    ax_e.axvspan(t_start, ti, alpha=0.12, color=GPS_DOT)
                    in_gps = False
            if in_gps:
                ax_e.axvspan(t_start, common_t[-1], alpha=0.12, color=GPS_DOT,
                             label="GPS active")
        ax_e.set_ylabel("Position correction (mm)", fontsize=9)
        ax_e.set_xlabel("Time (s)", fontsize=9)
        ax_e.grid(True, color=GRID_CLR, linewidth=0.4)
        ax_e.legend(fontsize=8, facecolor=PANEL_BG, edgecolor=GRID_CLR, labelcolor=TEXT_CLR)
        ax_e.tick_params(colors=TEXT_CLR)

    fig.tight_layout()
    out = out_dir / "fig_heading_fusion.png"
    fig.savefig(out, dpi=180, bbox_inches="tight", facecolor=BG)
    plt.close(fig)
    print(f"[plot] saved {out}")
    return out


# ── Figure 4: Vision detection timeline ──────────────────────────────────────

_VISION_CLASSES = {
    "traffic light": {"green": "#4ade80", "red": "#f87171", "unknown": "#94a3b8"},
    "stop sign":     {"default": "#fb923c"},
    "person":        {"default": "#a78bfa"},
    "red_cup":       {"default": "#f87171"},
    "marshmallow":   {"default": "#fde68a"},
}

def plot_vision_timeline(data: dict, out_dir: Path) -> Path:
    detections = data.get("detections", [])
    if not detections:
        print("[plot] no detection data — skipping vision figure")
        return None

    # Group by class
    classes = sorted({d["class_name"] for d in detections})
    fig, ax = _dark_fig(11, 5)
    ax.set_xlabel("Time (s)", fontsize=9)
    ax.set_ylabel("Detection confidence", fontsize=9)
    ax.set_title("Vision Detections Over Time", fontsize=11)
    ax.set_ylim(0, 1.1)
    ax.axhline(0.5, color=GRID_CLR, linewidth=0.6, linestyle="--")
    ax.text(0, 0.51, "min conf threshold (0.50)", color="#64748b", fontsize=6.5)
    ax.grid(True, color=GRID_CLR, linewidth=0.4, axis="x")

    for cls in classes:
        class_dets = [d for d in detections if d["class_name"] == cls]
        color_map = _VISION_CLASSES.get(cls, {})

        for d in class_dets:
            attr_color = d["attrs"].get("color", "")
            color = color_map.get(attr_color, color_map.get("default", "#94a3b8"))
            ax.scatter(
                d["t"], d["confidence"],
                s=40, color=color, alpha=0.8,
                edgecolors="white", linewidths=0.4, zorder=5,
            )

    # Legend: one proxy per class
    handles = []
    for cls in classes:
        color_map = _VISION_CLASSES.get(cls, {})
        c = list(color_map.values())[0] if color_map else "#94a3b8"
        handles.append(mpatches.Patch(color=c, label=cls))
    # Extra patches for traffic light colors
    for sub, col in [("green light", "#4ade80"), ("red light", "#f87171")]:
        if "traffic light" in classes:
            handles.append(mpatches.Patch(color=col, label=f"  → {sub}"))

    ax.legend(handles=handles, fontsize=8, loc="upper right",
              facecolor=PANEL_BG, edgecolor=GRID_CLR, labelcolor=TEXT_CLR)
    fig.tight_layout()
    out = out_dir / "fig_vision_timeline.png"
    fig.savefig(out, dpi=180, bbox_inches="tight", facecolor=BG)
    plt.close(fig)
    print(f"[plot] saved {out}")
    return out


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data_file", nargs="?", help="JSON file from nav_data_recorder.py")
    parser.add_argument("--fsm",     action="store_true", help="Generate FSM diagram")
    parser.add_argument("--out-dir", default=".", help="Output directory for PNGs")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if not args.data_file and not args.fsm:
        parser.print_help()
        return

    data: dict = {}
    if args.data_file:
        path = Path(args.data_file)
        if not path.exists():
            print(f"[error] file not found: {path}")
            return
        data = json.loads(path.read_text())
        meta = data.get("meta", {})
        print(f"[plot] loaded: {meta.get('recorded_at','?')}  "
              f"duration={meta.get('duration_s','?')} s  "
              f"odom={meta.get('n_odom',0)}  fused={meta.get('n_fused',0)}  "
              f"lidar={meta.get('n_lidar',0)}  detections={meta.get('n_detections',0)}")

    if args.fsm or not args.data_file:
        plot_fsm_diagram(out_dir)

    if data:
        plot_trajectory_map(data, out_dir)
        plot_heading_fusion(data, out_dir)
        plot_vision_timeline(data, out_dir)

    print(f"[plot] done — figures in {out_dir.resolve()}")


if __name__ == "__main__":
    main()
