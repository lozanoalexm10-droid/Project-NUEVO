#!/usr/bin/env python3
"""
make_system_state_diagram.py
============================
Generates Figure 2.X: top-level system state machine for the final report.

Four operational states + one safety sink:
  IDLE → NAVIGATE → MANIPULATE → DONE,   BTN_2 → ESTOP / SHUTDOWN

NAVIGATE and MANIPULATE are composite — their internal FSMs are detailed
in Sections 4 and 5.3.2 respectively. Composite states are drawn with a
dashed inner border + a slightly different fill so they read as
"expanded elsewhere" rather than primitive states.

Run:
    python3 ros2_ws/src/robot/tools/make_system_state_diagram.py

Output:
    /tmp/system_state_diagram.png
"""
from __future__ import annotations

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

# ─── Canvas ───────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(13, 6))
ax.set_xlim(0, 13)
ax.set_ylim(0, 6)
ax.set_aspect("equal")
ax.axis("off")
fig.patch.set_facecolor("white")

SIMPLE_FILL    = "#e8eef7"   # light blue-grey for primitive states
COMPOSITE_FILL = "#fff4e0"   # warm cream for composite states
ESTOP_FILL     = "#fde0e0"   # light red-pink for safety sink
ESTOP_COLOR    = "#a02020"   # dark red for ESTOP arrows


def add_state(x, y, w, h, label, sublabel=None, composite=False, fill=None):
    fill = fill or (COMPOSITE_FILL if composite else SIMPLE_FILL)
    box = FancyBboxPatch(
        (x - w / 2, y - h / 2), w, h,
        boxstyle="round,pad=0.06,rounding_size=0.18",
        linewidth=2.0, edgecolor="black", facecolor=fill,
    )
    ax.add_patch(box)
    if composite:
        inner = FancyBboxPatch(
            (x - w / 2 + 0.12, y - h / 2 + 0.12), w - 0.24, h - 0.24,
            boxstyle="round,pad=0.03,rounding_size=0.12",
            linewidth=1.0, edgecolor="#666666", facecolor="none",
            linestyle="--",
        )
        ax.add_patch(inner)
    ax.text(x, y + (0.14 if sublabel else 0.0), label,
            ha="center", va="center", fontsize=13, fontweight="bold")
    if sublabel:
        ax.text(x, y - 0.22, sublabel,
                ha="center", va="center", fontsize=9, style="italic", color="#555")


def add_arrow(p1, p2, label=None, label_offset=(0, 0.28),
              connectionstyle="arc3,rad=0", color="black",
              linestyle="-", linewidth=1.6, label_color="black"):
    arrow = FancyArrowPatch(
        p1, p2, arrowstyle="-|>", mutation_scale=18,
        linewidth=linewidth, color=color, linestyle=linestyle,
        connectionstyle=connectionstyle,
    )
    ax.add_patch(arrow)
    if label:
        mx = (p1[0] + p2[0]) / 2 + label_offset[0]
        my = (p1[1] + p2[1]) / 2 + label_offset[1]
        ax.text(mx, my, label,
                ha="center", va="center", fontsize=9.5, color=label_color,
                bbox=dict(boxstyle="round,pad=0.25", facecolor="white",
                          edgecolor="none", alpha=0.95))


# ─── Main row of states (y = 4.0) ─────────────────────────────────────────────
y_main = 4.0
h_main = 1.15
w_s = 1.55       # simple states
w_c = 2.20       # composite states

x_idle, x_nav, x_man, x_done = 1.10, 4.30, 8.20, 11.50

add_state(x_idle, y_main, w_s, h_main, "IDLE")
add_state(x_nav,  y_main, w_c, h_main, "NAVIGATE",
          sublabel="expanded in Section 4", composite=True)
add_state(x_man,  y_main, w_c, h_main, "MANIPULATE",
          sublabel="expanded in Section 5.3.2", composite=True)
add_state(x_done, y_main, w_s, h_main, "DONE")

# ─── ESTOP sink (y = 1.0) ─────────────────────────────────────────────────────
y_estop = 1.0
h_estop = 0.95
w_estop = 2.6
x_estop = 6.30
add_state(x_estop, y_estop, w_estop, h_estop, "ESTOP / SHUTDOWN", fill=ESTOP_FILL)

# ─── Entry arrow (power on) ───────────────────────────────────────────────────
add_arrow((-0.05, y_main), (x_idle - w_s / 2, y_main))
ax.text(-0.10, y_main + 0.35, "power on",
        ha="left", va="center", fontsize=9, style="italic", color="#555")

# ─── Main-flow arrows (forward progression) ───────────────────────────────────
add_arrow(
    (x_idle + w_s / 2, y_main),
    (x_nav  - w_c / 2, y_main),
    label="green light\nor BTN_1",
)
add_arrow(
    (x_nav  + w_c / 2, y_main),
    (x_man  - w_c / 2, y_main),
    label="stop sign at finish\ndrive halted",
)
add_arrow(
    (x_man  + w_c / 2, y_main),
    (x_done - w_s / 2, y_main),
    label="marshmallow\nplaced",
)

# ─── ESTOP arrows (BTN_2 from every operational state) ────────────────────────
# Each main state drops down to the top of the ESTOP box. We fan them in
# with mild curvature so labels and lines do not collide.
estop_top_y = y_estop + h_estop / 2
estop_xs = [x_estop - 1.0, x_estop - 0.4, x_estop + 0.4, x_estop + 1.0]
rads     = [-0.20,         -0.05,          0.05,          0.20]
src_xs   = [x_idle,         x_nav,         x_man,         x_done]

for sx, ex, rad in zip(src_xs, estop_xs, rads):
    add_arrow(
        (sx, y_main - h_main / 2),
        (ex, estop_top_y),
        connectionstyle=f"arc3,rad={rad}",
        color=ESTOP_COLOR, linewidth=1.3, linestyle=(0, (4, 2)),
    )

ax.text(x_estop, y_estop + h_estop / 2 + 0.55,
        "BTN_2 (any state)",
        ha="center", va="center", fontsize=9.5, style="italic",
        color=ESTOP_COLOR,
        bbox=dict(boxstyle="round,pad=0.25", facecolor="white",
                  edgecolor="none", alpha=0.95))

# ─── Title ────────────────────────────────────────────────────────────────────
ax.text(6.5, 5.55, "Figure 2.X — Top-Level System State Machine",
        ha="center", va="center", fontsize=13.5, fontweight="bold")

# ─── Legend ───────────────────────────────────────────────────────────────────
legend_elems = [
    mpatches.Patch(facecolor=SIMPLE_FILL,    edgecolor="black",
                   label="primitive state"),
    mpatches.Patch(facecolor=COMPOSITE_FILL, edgecolor="black",
                   label="composite state (FSM expanded in cited section)"),
    mpatches.Patch(facecolor=ESTOP_FILL,     edgecolor="black",
                   label="safety sink"),
]
ax.legend(handles=legend_elems, loc="lower left", fontsize=9,
          framealpha=1.0, bbox_to_anchor=(0.01, 0.01))

# ─── Save ─────────────────────────────────────────────────────────────────────
out_path = "/tmp/system_state_diagram.png"
plt.tight_layout()
plt.savefig(out_path, dpi=220, bbox_inches="tight", facecolor="white")
print(f"saved {out_path}")
