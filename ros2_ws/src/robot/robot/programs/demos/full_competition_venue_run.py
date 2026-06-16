"""
full_competition_venue_run.py — four-segment graded venue route
===============================================================
Drives the full competition course in four independently checkpointed segments.
The course can be entered at any segment by setting START_SEGMENT — useful for
restarting at a checkpoint after a minor point deduction.

Segment map (checkpoints are at the midpoints of 90° turn pairs):

  SEG1  Start → CP1   — lane 1 north, first half of top-left U-turn
  SEG2  CP1   → CP2   — complete top-left U-turn, lane 2 south, first half of bottom turn
  SEG3  CP2   → CP3   — complete bottom turn, obstacle zone north (avoidance ON),
                         first half of top-right U-turn
  SEG4  CP3   → Finish — complete top-right U-turn, lane 5 south, stop-sign dwell,
                          manipulation station

Checkpoint coordinates (absolute course frame, mm):
  CP1: (305,    3660)  — top-left  U-turn midpoint  (lane 1 → lane 2)
  CP2: (1068,    460)  — bottom    turn  midpoint   (lane 2 → obstacle zone)
  CP3: (1983,   3660)  — top-right U-turn midpoint  (obstacle zone → lane 5)

For a checkpoint restart: place the robot at CP_N with the correct heading,
set START_SEGMENT = N, then relaunch. Odometry resets to (0, 0) at the robot's
current position, so all segment waypoints are expressed relative to their own
checkpoint (0, 0). The GPS/AprilTag fusion will correct position drift in-flight.

Edit these two lines before each run:
"""
from __future__ import annotations

# ── Run configuration ─────────────────────────────────────────────────────────
START_SEGMENT = 1    # 1–4: segment to begin from; 1 = full course from start
AUTO_START    = False

  # True = 3-second countdown; False = green-light trigger (BTN_1 to override)
# ─────────────────────────────────────────────────────────────────────────────

import math
import time

from robot.hardware_map import (
    Button,
    DCPidLoop,
    DEFAULT_FSM_HZ,
    LED,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    StepMoveType,
    TAG_ID,
    VELOCITY_KD,
    VELOCITY_KI,
    VELOCITY_KP,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot
from robot._manipulator_config import (
    CAMPAN_ACCELERATION,
    CAMPAN_MAX_VELOCITY,
    CAMPAN_SETTLE_S,
    CAMPAN_STEPPER,
    campan_deg_to_steps,
)
from robot.util import densify_polyline

# ── Camera pan for traffic-light detection ────────────────────────────────────
GREEN_LIGHT_CAMPAN_DEG = 30.0  # camera pan angle while watching for green light (left)

# ── Nav tuning ─────────────────────────────────────────────────────────────────
MIN_TRAFFIC_CONF  = 0.50
VISION_STALE_S    = 3.0
STOP_SIGN_DWELL_S = 3.0
# Min stop-sign bbox height as a fraction of the frame height before we
# accept the detection. Apparent size grows ~linearly with 1/distance, so a
# larger value forces the robot to approach further before pausing.
STOP_SIGN_MIN_FRAC = 0.18
STOP_SIGN_MIN_CONF = 0.50
# Don't even look for the stop sign until the rover is south of this absolute
# y (mm). Above this y the detector is ignored entirely — early detections
# of the sign from far up the lane were triggering false stops.
STOP_SIGN_GATE_Y_MM = 1.75 * 610  # 1067.5 mm

# Per-segment tuning — adjust speed (mm/s) and lookahead (mm) here.
# Larger lookahead = wider, earlier turns = less corner overshoot.
#   SEG1  : north straight to CP1
#   SEG2  : top-left U-turn to CP2
#   SEG3A : east approach to obstacle zone
#   SEG3B : north through obstacle zone (avoidance ON)
#   SEG3C : finish north run + 90° right turn + east to CP3 (avoidance OFF)
#   SEG4  : CP3 → 90° right-turn corner → lane 5 south → finish (single path)
SEG1_CFG  = dict(speed=140, lookahead=300, spacing=50)
SEG2_CFG  = dict(speed=130, lookahead=300, spacing=50)
SEG3A_CFG = dict(speed=130, lookahead=300, spacing=50)
# SEG3B tuning — current (2026-06-09): bumped lookahead/safe_dist/alpha_Ld for
# smoother detours, plus depends on the new goal-y obstacle filter in
# path_planner so SEG3B can extend up to OBS_EXIT_OFFSET_MM=400 without the
# north wall triggering avoidance. Revert by swapping with the commented block
# below if behavior regresses on the real course.
SEG3B_CFG = dict(
    speed=75, lookahead=290, spacing=400,
    safe_dist=250, lane_width=451, avoidance_delay=245,
    obstacles_range=570, view_angle=70.0, alpha_Ld=0.70, offset=324,
)
SEG3C_CFG = dict(speed=120, lookahead=300, spacing=50)
SEG4_CFG  = dict(speed=130, lookahead=300, spacing=50)

# ── Course geometry (mm, absolute odometry frame, x=east y=north) ─────────────
X_LANE1   =    0.0    # lane 1 centre (robot start x)
X_LANE2   =  610.0    # lane 2 centre
X_OBS_MID = 1525.0    # obstacle zone centreline
X_LANE5   = 2440.0    # lane 5 centre

Y_START   =    -160.0    # course start y
Y_TOP     = 3450.0    # far (north) end of course — top turns happen here
                      # (lowered 50 mm: SEG1 was stopping 50 mm into the top
                      # wall; this also pulls SEG2's top corners south the
                      # same amount)
Y_BOT     =  460.0    # near (south) end — bottom turns happen here
                      # (lowered 150 mm so the obstacle-zone turn/entry sit further south)

FINISH_X  = 2440.0    # manipulation station x (lane 5) = 4 * 610
FINISH_Y  =  280.0    # manipulation station y

# Checkpoint coordinates — midpoints of the two 90° corners that form each U-turn.
# These are the absolute course positions. When restarting at CP_N the robot is
# placed here and odometry is reset; the segment waypoints below start from (0, 0).
CP1 = (  305.0, 3660.0)
CP2 = ( 1067.5,  460.0)
CP3 = ( 1982.5, 3660.0)

# Wall standoff for the perimeter 90° corners. The corner waypoints used to sit
# on Y_TOP / X_LANE5 / Y_BOT, but pure-pursuit overshoot during the right-hand
# turns drove the robot into the walls. Pull each corner inward so the overshoot
# stays inside the course. Defined here (above the SEG_PATH lists) because the
# path definitions reference these constants.
TOP_WALL_MARGIN_MM    = 200.0  # corner y = Y_TOP - this (SEG3C north→east, SEG4 east→south)
EAST_WALL_MARGIN_MM   = 50.0  # SEG4 east corner x = X_LANE5 - this
BOTTOM_WALL_MARGIN_MM =  75.0  # bottom-row corner y = Y_BOT + this (SEG2 south→east, SEG3A east→north, CP2)

# ── Manual reposition checkpoint (SEG3B → SEG3 exit) ──────────────────────────
# SEG3B's obstacle traversal drifts heading enough that the robot was veering
# into the west wall on the north run. Instead of fighting it autonomously,
# the FSM stops for a fixed window so the operator can physically place the
# robot at a standardized pose; SEG3 exit and SEG4 then plan from a known
# origin instead of inheriting drift from the obstacle zone.
#   • North wall at 6.5 * 610 = 3965 mm; corner sits 305 mm above the handoff
#     (305 mm clearance, well above the 200 mm safety minimum).
#   • East leg spans 915 mm from x=1525 to lane 5 (x=2440); SEG4's existing
#     EAST_WALL_MARGIN_MM keeps the corner waypoint inset to soak pure-pursuit
#     overshoot before the robot drifts back east into lane 5 for the finish.
MANUAL_REPOSITION_HOLD_S = 6.0
MANUAL_REPOS_X     = 2.5 * 610   # 1525 mm — obstacle zone centerline
MANUAL_REPOS_Y     = 5.5 * 610   # 3355 mm — handoff y
MANUAL_REPOS_THETA = 90.0        # robot facing north after manual placement
Y_TOP_POST_REPOS   = MANUAL_REPOS_Y + 305.0   # 3660 mm — post-handoff U-turn corner y

# ── Segment waypoints ─────────────────────────────────────────────────────────
# All coordinates are expressed relative to the segment's own start point
# (= 0, 0 after odometry reset at start / checkpoint).

# Segment 1: lane 1 north to CP1
#   Start facing north (+y). Corner at top-left → turn right (east).
#   Ends at CP1 (top-left U-turn midpoint).
SEG1_PATH = [
    (X_LANE1, Y_START),       # (0, 0) — course start
    (X_LANE1, Y_TOP),         # 90° right-turn corner (now east)
    (X_LANE1 + 305.0, Y_TOP), # CP1: 305 mm east of corner
]

# Segment 2: CP1 → lane 2 south → CP2
#   Robot at CP1 facing east. Corner at lane-2 top → turn right (south).
#   Corner at lane-2 bottom → turn right (east). Ends at CP2.
SEG2_PATH = [
    (X_LANE1 + 305.0, Y_TOP),  # CP1
    (X_LANE2 + 20.0, Y_TOP),           # 90° right-turn corner (now south)
    (X_LANE2 + 20.0, Y_BOT + BOTTOM_WALL_MARGIN_MM),           # 90° right-turn corner (now east), pulled north of bottom wall
    (X_LANE2 + 457.5, Y_BOT + BOTTOM_WALL_MARGIN_MM),  # CP2 (margin-inset)
]

# Obstacle-zone avoidance entry/exit offsets — how far inside the obstacle
# zone the avoidance planner activates and deactivates. Keeps the planner from
# fighting the 90° corner turns or treating the north/south walls as obstacles.
OBS_ENTRY_OFFSET_MM = 400.0  # SEG3A drives the robot this far north of the
                             # corner before SEG3B's avoidance takes over, giving
                             # it time to finish the 90° turn and settle on the
                             # centerline pointing north before any cones are
                             # considered.
OBS_EXIT_OFFSET_MM  = 700.0  # Distance from north wall at which SEG3B avoidance ends.

# Safety net: if pure-pursuit jitters without tripping goal_tolerance after the last
# cone, force-complete SEG3B once the robot crosses this absolute y.
SEG3B_FORCE_COMPLETE_Y = Y_TOP - OBS_EXIT_OFFSET_MM - 100.0

SEG3_APPROACH_PATH = [
    (X_LANE2 + 457.5, Y_BOT + BOTTOM_WALL_MARGIN_MM),              # CP2 (matches SEG2 end)
    (X_OBS_MID, Y_BOT + BOTTOM_WALL_MARGIN_MM),                    # corner (east → north), pulled north of bottom wall
    (X_OBS_MID, Y_BOT + OBS_ENTRY_OFFSET_MM),  # robot heading north, safe to start avoidance
]
SEG3_OBS_PATH = [
    (X_OBS_MID, Y_BOT + OBS_ENTRY_OFFSET_MM),
    (X_OBS_MID, Y_TOP - OBS_EXIT_OFFSET_MM),
]
SEG3_EXIT_PATH = [
    (MANUAL_REPOS_X, MANUAL_REPOS_Y),                          # (1525, 3355) — handoff
    (MANUAL_REPOS_X, Y_TOP_POST_REPOS),                        # (1525, 3660) — corner (north → east)
    (MANUAL_REPOS_X + 457.5, Y_TOP_POST_REPOS),                # (1982.5, 3660) — CP3-equivalent
]

SEG4_PATH = [
    (MANUAL_REPOS_X + 457.5, Y_TOP_POST_REPOS),            # CP3-equiv, matches SEG3 exit end
    (X_LANE5 - EAST_WALL_MARGIN_MM, Y_TOP_POST_REPOS),     # corner (east → south), inset for overshoot
    (FINISH_X, FINISH_Y),                                   # manipulation station
]

# Per-segment start: (course_x_mm, course_y_mm, heading_deg)
# heading_deg follows math convention: east=0°, north=90°
_SEG_START = {
    1: (X_LANE1,           Y_START, 90.0),   # lane 1, facing north
    2: (X_LANE1 + 305.0,   Y_TOP,    0.0),   # CP1, facing east
    3: (X_LANE2 + 457.5,   Y_BOT + BOTTOM_WALL_MARGIN_MM, 0.0),   # CP2 (margin-inset), facing east
    4: (X_OBS_MID + 457.5, Y_TOP_POST_REPOS, 0.0),   # CP3-equivalent (post-reposition), facing east
}


# ── Helpers ───────────────────────────────────────────────────────────────────

def _configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    robot.set_pid_gains(LEFT_WHEEL_MOTOR,  DCPidLoop.VELOCITY, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD)
    robot.set_pid_gains(RIGHT_WHEEL_MOTOR, DCPidLoop.VELOCITY, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD)
    robot.enable_lidar()
    robot.enable_gps()
    robot.enable_vision()
    robot.set_tracked_tag_id(TAG_ID)
    robot.set_orientation_fusion_alpha(0.0)
    robot.set_position_fusion_alpha(0.0)


def _init_firmware(robot: Robot, theta_deg: float) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    time.sleep(0.2)
    robot.set_initial_theta(theta_deg)
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=3.0):
        print("[WARN] Odometry reset not confirmed within 3 s — pose may be stale")
        robot.wait_for_pose_update(timeout=1.0)
    print(f"[CONFIG] Pose after reset: {robot.get_odometry_pose()}")


def _load_path(
    robot: Robot,
    waypoints: list[tuple[float, float]],
    cfg: dict,
    *,
    obstacle_avoidance: bool,
    x_L: float = 0.0,
) -> None:
    path = densify_polyline(waypoints, spacing=float(cfg.get("spacing", 50)))
    if obstacle_avoidance:
        robot._nav_follow_pp_path(
            lookahead_distance=cfg["lookahead"],
            max_linear_speed=cfg["speed"],
            max_angular_speed=1.0,
            goal_tolerance=30.0,
            obstacles_range=float(cfg.get("obstacles_range", 450.0)),
            view_angle=math.radians(float(cfg.get("view_angle", 70.0))),
            safe_dist=float(cfg.get("safe_dist", 181.0)),
            avoidance_delay=int(cfg.get("avoidance_delay", 245)),
            alpha_Ld=float(cfg.get("alpha_Ld", 0.70)),
            offset=float(cfg.get("offset", 324.0)),
            lane_width=float(cfg.get("lane_width", 451.0)),
            obstacle_avoidance=True,
            x_L=x_L,
        )
    else:
        robot._nav_follow_pp_path(
            lookahead_distance=cfg["lookahead"],
            max_linear_speed=cfg["speed"],
            max_angular_speed=1.5,
            goal_tolerance=20.0,
            obstacles_range=450.0,
            view_angle=math.radians(70.0),
            safe_dist=250.0,
            avoidance_delay=150,
            alpha_Ld=0.7,
            offset=305.0,
            lane_width=500.0,
            obstacle_avoidance=False,
            x_L=x_L,
        )
    robot._set_obstacle_avoidance_path(path)


def _traffic_light_color(robot: Robot) -> str | None:
    """Return 'red' or 'green' from the highest-confidence traffic light detection, or None."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_S):
        return None
    best_color, best_conf = None, -1.0
    # NOTE: vision_node publishes class_name="traffic light" (with a space),
    # not "traffic_light" — strict-equality match in get_detections means the
    # underscored form silently returns nothing.
    for det in robot.get_detections("traffic light"):
        conf  = float(det["confidence"])
        color = det.get("attributes", {}).get("color", {}).get("value")
        if conf >= MIN_TRAFFIC_CONF and color in ("red", "green") and conf > best_conf:
            best_conf, best_color = conf, str(color)
    return best_color


def _estop(robot: Robot) -> None:
    robot.stop()
    robot.shutdown()


# ── Main FSM ──────────────────────────────────────────────────────────────────

def run(robot: Robot) -> None:  # noqa: C901
    _configure_robot(robot)
    ox, oy, theta = _SEG_START[START_SEGMENT]
    _init_firmware(robot, theta)

    def _op(path: list[tuple[float, float]]) -> list[tuple[float, float]]:
        """Shift course-frame waypoints into the odometry frame for this run."""
        return [(x - ox, y - oy) for x, y in path]

    x_L_obs = X_OBS_MID - ox   # obstacle-zone centreline in odometry frame

    # Pre-arm the first segment's path before entering IDLE.
    if START_SEGMENT == 1:
        _load_path(robot, _op(SEG1_PATH), SEG1_CFG, obstacle_avoidance=False)
    elif START_SEGMENT == 2:
        _load_path(robot, _op(SEG2_PATH), SEG2_CFG, obstacle_avoidance=False)
    elif START_SEGMENT == 3:
        _load_path(robot, _op(SEG3_APPROACH_PATH), SEG3A_CFG, obstacle_avoidance=False)
    elif START_SEGMENT == 4:
        _load_path(robot, _op(SEG4_PATH), SEG4_CFG, obstacle_avoidance=False)

    if not AUTO_START:
        target_steps = campan_deg_to_steps(GREEN_LIGHT_CAMPAN_DEG)
        robot.step_set_config(CAMPAN_STEPPER, CAMPAN_MAX_VELOCITY, CAMPAN_ACCELERATION)
        time.sleep(0.3)
        moved = False
        for attempt in range(1, 4):
            robot.step_disable(CAMPAN_STEPPER)
            time.sleep(0.2)
            robot.step_enable(CAMPAN_STEPPER)
            time.sleep(0.6)
            if robot.step_move(CAMPAN_STEPPER, target_steps,
                               StepMoveType.ABSOLUTE, timeout=3.0):
                moved = True
                if attempt > 1:
                    print(f"[FSM] CAMPAN move succeeded on attempt {attempt}.")
                break
            print(f"[WARN] CAMPAN move attempt {attempt} timed out — re-arming.")
        if not moved:
            print("[ERROR] CAMPAN failed to move after 3 attempts — continuing anyway.")
        time.sleep(CAMPAN_SETTLE_S)
        print(f"[FSM] Camera panned to {GREEN_LIGHT_CAMPAN_DEG}° for traffic-light detection.")

    print(f"[FSM] Ready. START_SEGMENT={START_SEGMENT}  AUTO_START={AUTO_START}")
    print(f"[FSM] Course offset: ox={ox} oy={oy} theta={theta}°")
    print(f"[FSM] Checkpoints — CP1:{CP1}  CP2:{CP2}  CP3:{CP3}")

    state: str = "IDLE"
    idle_leds_set: bool = False    # latch once — re-flooding LEDs every tick queues commands and stalls the first move
    seg3_phase: str = "approach"   # 'approach' | 'obstacle' | 'exit'
    stop_sign_seen:    bool  = False
    stop_sign_pause_t: float = 0.0
    idle_last_debug_t: float = 0.0   # rate-limit IDLE vision debug print

    def _start_segment() -> str:
        nonlocal idle_leds_set
        if not AUTO_START:
            robot.step_move(CAMPAN_STEPPER, campan_deg_to_steps(0.0),
                            StepMoveType.ABSOLUTE, timeout=3.0)
            time.sleep(CAMPAN_SETTLE_S)
        robot.set_led(LED.RED, 0)
        robot.set_led(LED.GREEN, 255)
        idle_leds_set = False
        return f"SEG{START_SEGMENT}"

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        # ── IDLE ─────────────────────────────────────────────────────────────
        if state == "IDLE":
            if robot.get_button(Button.BTN_2):
                print("[FSM] BTN_2 — shutting down.")
                _estop(robot)
                return

            if AUTO_START:
                if not idle_leds_set:
                    robot.set_led(LED.ORANGE, 255)
                    robot.set_led(LED.GREEN, 0)
                    robot.set_led(LED.RED, 0)
                    idle_leds_set = True
                robot._draw_lidar_obstacles()
                print(f"[FSM] Auto-starting SEG{START_SEGMENT} in 3 s …")
                time.sleep(3)
                robot.set_led(LED.ORANGE, 0)
                state = f"SEG{START_SEGMENT}"
                idle_leds_set = False
                print(f"[FSM] → {state}")
            else:
                if not idle_leds_set:
                    robot.set_led(LED.RED, 255)
                    robot.set_led(LED.GREEN, 0)
                    robot.set_led(LED.ORANGE, 0)
                    idle_leds_set = True
                color = _traffic_light_color(robot)

                now_dbg = time.monotonic()
                if now_dbg - idle_last_debug_t >= 1.0:
                    idle_last_debug_t = now_dbg
                    vision_active = robot.is_vision_active(timeout_s=VISION_STALE_S)
                    all_dets = robot.get_detections()
                    classes = sorted({str(d.get("class_name")) for d in all_dets})
                    tl_dets = robot.get_detections("traffic light")
                    tl_summary = ", ".join(
                        f"{(d.get('attributes', {}).get('color', {}) or {}).get('value','?')}"
                        f"@{float(d.get('confidence', 0.0)):.2f}"
                        for d in tl_dets
                    ) or "(none)"
                    print(f"[IDLE] vision_active={vision_active} "
                          f"classes={classes or '[]'} traffic_light=[{tl_summary}] "
                          f"resolved_color={color}")

                if color == "green":
                    print(f"[FSM] Green light — starting SEG{START_SEGMENT}.")
                    state = _start_segment()
                elif robot.get_button(Button.BTN_1):
                    print(f"[FSM] BTN_1 override — starting SEG{START_SEGMENT}.")
                    state = _start_segment()

        # ── SEG1: Lane 1 north → CP1 (first half of top-left U-turn) ─────────
        elif state == "SEG1":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            if robot.get_button(Button.BTN_2):
                _estop(robot); return
            if robot._nav_follow_pp_path_loop() == "IDLE":
                print("[FSM] SEG1 complete — CP1 reached.")
                _load_path(robot, _op(SEG2_PATH), SEG2_CFG, obstacle_avoidance=False)
                state = "SEG2"

        # ── SEG2: CP1 → complete top-left U-turn → lane 2 south → CP2 ────────
        elif state == "SEG2":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            if robot.get_button(Button.BTN_2):
                _estop(robot); return
            if robot._nav_follow_pp_path_loop() == "IDLE":
                print("[FSM] SEG2 complete — CP2 reached.")
                _load_path(robot, _op(SEG3_APPROACH_PATH), SEG3A_CFG, obstacle_avoidance=False)
                seg3_phase = "approach"
                state = "SEG3"

        # ── SEG3: CP2 → obstacle zone → CP3 ─────────────────────────────────
        elif state == "SEG3":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            if robot.get_button(Button.BTN_2):
                _estop(robot); return
            pp_idle = robot._nav_follow_pp_path_loop() == "IDLE"
            if not pp_idle and seg3_phase == "obstacle":
                _, py_chk, _ = robot.get_odometry_pose()
                if py_chk + oy >= SEG3B_FORCE_COMPLETE_Y:
                    print(f"[FSM] SEG3B force-complete: y={py_chk + oy:.0f} "
                          f">= {SEG3B_FORCE_COMPLETE_Y:.0f} — skipping pp "
                          f"goal_tolerance wait.")
                    robot.stop()
                    pp_idle = True
            if pp_idle:
                px, py, pth = robot.get_odometry_pose()
                abs_x, abs_y = px + ox, py + oy
                if seg3_phase == "approach":
                    print(f"[FSM] SEG3 approach done — entering obstacle zone. "
                          f"abs=({abs_x:.0f},{abs_y:.0f}) θ={pth:.1f}°")
                    robot.stop()
                    time.sleep(3.0)  # hold so operator can verify heading before avoidance starts
                    _load_path(robot, _op(SEG3_OBS_PATH), SEG3B_CFG, obstacle_avoidance=True, x_L=x_L_obs)
                    seg3_phase = "obstacle"
                elif seg3_phase == "obstacle":
                    print(f"[FSM] SEG3 obstacle zone done — pausing {MANUAL_REPOSITION_HOLD_S:.0f}s "
                          f"for manual reposition to ({MANUAL_REPOS_X:.0f},{MANUAL_REPOS_Y:.0f}) "
                          f"θ={MANUAL_REPOS_THETA:.0f}°. abs=({abs_x:.0f},{abs_y:.0f})")
                    robot.stop()
                    n_blinks = max(1, int(round(MANUAL_REPOSITION_HOLD_S)))
                    for _ in range(n_blinks):
                        robot.set_led(LED.RED, 255)
                        robot.set_led(LED.ORANGE, 255)
                        robot.set_led(LED.GREEN, 0)
                        time.sleep(0.5)
                        robot.set_led(LED.RED, 0)
                        robot.set_led(LED.ORANGE, 0)
                        time.sleep(0.5)
                    robot.set_led(LED.GREEN, 255)
                    # Re-anchor odometry to the standardized handoff pose so SEG3 exit / SEG4
                    # plan from a known origin rather than inheriting obstacle-zone drift.
                    robot.set_initial_theta(MANUAL_REPOS_THETA)
                    robot.reset_odometry()
                    if not robot.wait_for_odometry_reset(timeout=3.0):
                        print("[WARN] Odometry reset not confirmed within 3 s after manual reposition — pose may be stale.")
                    ox, oy = MANUAL_REPOS_X, MANUAL_REPOS_Y
                    print(f"[FSM] Resuming SEG3 exit from standardized "
                          f"abs=({ox:.0f},{oy:.0f}) θ={MANUAL_REPOS_THETA:.0f}°.")
                    _load_path(robot, _op(SEG3_EXIT_PATH), SEG3C_CFG, obstacle_avoidance=False)
                    seg3_phase = "exit"
                elif seg3_phase == "exit":
                    print(f"[FSM] SEG3 complete — CP3 reached at abs=({abs_x:.0f},{abs_y:.0f}) "
                          f"θ={pth:.1f}°.")
                    _load_path(robot, _op(SEG4_PATH), SEG4_CFG, obstacle_avoidance=False)
                    state = "SEG4"

        # ── SEG4: CP3 → corner → lane 5 south → Finish ──────────────────────
        elif state == "SEG4":
            if robot.get_button(Button.BTN_2):
                _estop(robot); return

            px, py, _ = robot.get_odometry_pose()
            abs_y = py + oy

            if stop_sign_seen and stop_sign_pause_t > 0.0:
                # Holding at stop sign
                robot.stop()
                robot.set_led(LED.RED, 255)
                robot.set_led(LED.GREEN, 0)
                if time.monotonic() - stop_sign_pause_t >= STOP_SIGN_DWELL_S:
                    abs_x_now = px + ox
                    print(f"[FSM] Stop-sign dwell complete at "
                          f"abs=({abs_x_now:.0f},{abs_y:.0f}) — driving "
                          f"straight to (X_LANE5={X_LANE5:.0f}, y=0).")
                    stop_sign_pause_t = 0.0
                    robot.set_led(LED.RED, 0)
                    robot.set_led(LED.GREEN, 255)
                    _load_path(
                        robot,
                        _op([(abs_x_now, abs_y), (X_LANE5, 0.0)]),
                        SEG4_CFG,
                        obstacle_avoidance=False,
                    )

            elif not stop_sign_seen and abs_y <= STOP_SIGN_GATE_Y_MM:
                _, img_h = robot.get_detection_image_size()
                close_det = None
                if img_h > 0:
                    for det in robot.get_detections("stop sign"):
                        if float(det.get("confidence", 0.0)) < STOP_SIGN_MIN_CONF:
                            continue
                        bbox = det.get("bbox") or {}
                        h_frac = float(bbox.get("height", 0)) / float(img_h)
                        if h_frac >= STOP_SIGN_MIN_FRAC:
                            close_det = (det, h_frac)
                            break

                if close_det is not None:
                    det, h_frac = close_det
                    robot.stop()
                    stop_sign_seen    = True
                    stop_sign_pause_t = time.monotonic()
                    robot.set_led(LED.RED, 255)
                    robot.set_led(LED.GREEN, 0)
                    print(f"[FSM] Stop sign close at abs_y={abs_y:.0f} "
                          f"(bbox h={h_frac:.2f} of frame) "
                          f"— pausing {STOP_SIGN_DWELL_S:.0f} s.")
                else:
                    robot.set_led(LED.GREEN, 255)
                    robot.set_led(LED.ORANGE, 0)
                    if robot._nav_follow_pp_path_loop() == "IDLE":
                        print("[FSM] SEG4 complete — at manipulation station.")
                        robot.stop()
                        state = "DONE"

            else:
                robot.set_led(LED.GREEN, 255)
                robot.set_led(LED.ORANGE, 0)
                if robot._nav_follow_pp_path_loop() == "IDLE":
                    print("[FSM] SEG4 complete — at manipulation station.")
                    robot.stop()
                    state = "DONE"

        # ── DONE ──────────────────────────────────────────────────────────────
        elif state == "DONE":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            robot.set_led(LED.RED, 0)
            robot.stop()

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
