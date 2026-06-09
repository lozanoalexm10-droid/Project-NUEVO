"""
venue_full_course_test.py — four-segment graded venue route
===========================================================
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
from robot.tests.scripts._manipulator_config import (
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
    safe_dist=230, lane_width=451, avoidance_delay=245,
    obstacles_range=570, view_angle=70.0, alpha_Ld=0.90, offset=324,
)
# SEG3B_CFG = dict(   # previous tuning (pre-2026-06-09)
#     speed=75, lookahead=140, spacing=400,
#     safe_dist=190, lane_width=451, avoidance_delay=245,
#     obstacles_range=570, view_angle=70.0, alpha_Ld=0.70, offset=324,
# )
SEG3C_CFG = dict(speed=120, lookahead=300, spacing=50)
SEG4_CFG  = dict(speed=120, lookahead=100, spacing=50)

# ── Course geometry (mm, absolute odometry frame, x=east y=north) ─────────────
X_LANE1   =    0.0    # lane 1 centre (robot start x)
X_LANE2   =  610.0    # lane 2 centre
X_OBS_MID = 1525.0    # obstacle zone centreline
X_LANE5   = 2440.0    # lane 5 centre

Y_START   =    -160.0    # course start y
Y_TOP     = 3500.0    # far (north) end of course — top turns happen here
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
    (X_LANE2 + 20.0, Y_BOT),           # 90° right-turn corner (now east)
    (X_LANE2 + 457.5, Y_BOT),  # CP2: 457.5 mm east of corner  = (1067.5, 610)
]

# Obstacle-zone avoidance entry/exit offsets — how far inside the obstacle
# zone the avoidance planner activates and deactivates. Keeps the planner from
# fighting the 90° corner turns or treating the north/south walls as obstacles.
OBS_ENTRY_OFFSET_MM = 400.0  # SEG3A drives the robot this far north of the
                             # corner before SEG3B's avoidance takes over, giving
                             # it time to finish the 90° turn and settle on the
                             # centerline pointing north before any cones are
                             # considered.
OBS_EXIT_OFFSET_MM  = 400.0  # Distance from north wall at which SEG3B's avoidance
                             # phase ends. Was 900 to keep the wall out of
                             # obstacles_range, but path_planner.gen_obstacle_waypoint
                             # now filters obstacles past goal_y, so the wall can
                             # never trigger avoidance regardless of this offset.
                             # 400 keeps SEG3B's reach (goal at y=3100) north of any
                             # real cone the course can place inside the obstacle zone.
                             # Revert to 900 (with the previous SEG3B_CFG block) if
                             # the goal-y filter misbehaves on the real course:
# OBS_EXIT_OFFSET_MM  = 900.0  # previous value — kept wall out of obstacles_range
                               # by ending SEG3B 900mm short of the north wall.

# Segment 3: three internal sub-paths with different avoidance settings.
#   3a (avoidance OFF): CP2 east → corner → OBS_ENTRY_OFFSET_MM north.
#   The approach carries the robot through the full 90° left turn without
#   avoidance so the planner doesn't fight the turn.
SEG3_APPROACH_PATH = [
    (X_LANE2 + 457.5, Y_BOT),              # CP2
    (X_OBS_MID, Y_BOT),                    # corner (east → north)
    (X_OBS_MID, Y_BOT + OBS_ENTRY_OFFSET_MM),  # robot heading north, safe to start avoidance
]
#   3b (avoidance ON): straight-line north through the obstacle field.
#   Stops OBS_EXIT_OFFSET_MM short of the north wall so the planner doesn't
#   treat the wall as an obstacle and detour into it.
SEG3_OBS_PATH = [
    (X_OBS_MID, Y_BOT + OBS_ENTRY_OFFSET_MM),
    (X_OBS_MID, Y_TOP - OBS_EXIT_OFFSET_MM),
]
#   3c (avoidance OFF): finish north run → 90° right turn → east to CP3.
SEG3_EXIT_PATH = [
    (X_OBS_MID, Y_TOP - OBS_EXIT_OFFSET_MM),
    (X_OBS_MID, Y_TOP),                    # corner (north → east)
    (X_OBS_MID + 457.5, Y_TOP),            # CP3: 457.5 mm east of corner = (1982.5, 3660)
]

# Segment 4: CP3 → corner → lane 5 south → manipulation station.
#   Stop-sign detection active throughout. 3-second dwell on first detection.
#   Single config/speed — bumps are small enough to drive over at normal pace.
#   Path is loaded in two sub-legs internally (east to corner, then south to
#   finish) so PP can't anticipate the 90° turn from a cold-start standalone
#   run — anticipating it collapses linear velocity below the motor stiction
#   floor and the robot jitters in place.
SEG4_EAST_PATH = [
    (X_OBS_MID + 457.5, Y_TOP),  # CP3
    (X_LANE5, Y_TOP),            # 90° right-turn corner
]
SEG4_SOUTH_PATH = [
    (X_LANE5, Y_TOP),            # corner
    (FINISH_X, FINISH_Y),        # manipulation station
]

# Per-segment start: (course_x_mm, course_y_mm, heading_deg)
# heading_deg follows math convention: east=0°, north=90°
_SEG_START = {
    1: (X_LANE1,           Y_START, 90.0),   # lane 1, facing north
    2: (X_LANE1 + 305.0,   Y_TOP,    0.0),   # CP1, facing east
    3: (X_LANE2 + 457.5,   Y_BOT,    0.0),   # CP2, facing east
    4: (X_OBS_MID + 457.5, Y_TOP,    0.0),   # CP3, facing east
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
        _load_path(robot, _op(SEG4_EAST_PATH), SEG4_CFG, obstacle_avoidance=False)

    if not AUTO_START:
        # Stepper API is fire-and-forget (publishes ROS msgs, no ack), so the
        # firmware needs a moment between config/enable and the first move —
        # otherwise pulses arrive before the driver is energized and the motor
        # whirs without rotating. Bounce disable→enable each attempt so the
        # driver state machine starts clean, then retry up to 3 times if the
        # move never reports motion.
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
    idle_leds_set: bool = False    # latch IDLE LEDs once — flooding the bus every tick
                                   # was queuing set_velocity commands and stalling the
                                   # first move after green-light detection.
    seg3_phase: str = "approach"   # 'approach' | 'obstacle' | 'exit'
    seg4_leg:   str = "east"       # 'east' (to corner) | 'south' (to finish)
    stop_sign_seen:    bool  = False
    stop_sign_pause_t: float = 0.0
    idle_last_debug_t: float = 0.0   # rate-limit IDLE vision debug print

    def _start_segment() -> str:
        nonlocal idle_leds_set
        # Pan camera back to 0° (it was at GREEN_LIGHT_CAMPAN_DEG for traffic-light watch)
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
        # AUTO_START=True  → orange LED, 3-second countdown, then go.
        # AUTO_START=False → red LED, watch for green traffic light; BTN_1 to override.
        # BTN_2 at any time → emergency shutdown.
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
                # Traffic-light mode: stay stopped on red, go on green.
                if not idle_leds_set:
                    robot.set_led(LED.RED, 255)
                    robot.set_led(LED.GREEN, 0)
                    robot.set_led(LED.ORANGE, 0)
                    idle_leds_set = True
                color = _traffic_light_color(robot)

                # ── IDLE vision debug (rate-limited to ~1 Hz) ────────────
                # Surfaces what the vision pipeline is actually publishing so a
                # class-name mismatch or stale topic is immediately visible.
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
                # color == 'red' or None → keep red LED on, stay in IDLE

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

        # ── SEG3: CP2 → obstacle zone → CP3 (three internal sub-paths) ────────
        #   approach  (avoidance OFF): CP2 east → obstacle zone entry
        #   obstacle  (avoidance ON):  straight north through obstacle zone
        #   exit      (avoidance OFF): east → CP3
        elif state == "SEG3":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            if robot.get_button(Button.BTN_2):
                _estop(robot); return
            if robot._nav_follow_pp_path_loop() == "IDLE":
                px, py, pth = robot.get_odometry_pose()
                abs_x, abs_y = px + ox, py + oy
                if seg3_phase == "approach":
                    print(f"[FSM] SEG3 approach done at abs=({abs_x:.0f},{abs_y:.0f}) "
                          f"θ={pth:.1f}° — entering obstacle zone "
                          f"(goal y={Y_TOP - OBS_EXIT_OFFSET_MM:.0f}, "
                          f"wall y={Y_TOP:.0f}, buffer={OBS_EXIT_OFFSET_MM:.0f} mm).")
                    _load_path(robot, _op(SEG3_OBS_PATH), SEG3B_CFG, obstacle_avoidance=True, x_L=x_L_obs)
                    seg3_phase = "obstacle"
                elif seg3_phase == "obstacle":
                    # pth is already degrees (see navigation.get_odometry_pose).
                    print(f"[FSM] SEG3 obstacle zone done at abs=({abs_x:.0f},{abs_y:.0f}) "
                          f"θ={pth:.1f}° (expected 90°) — pausing 3 s for manual adjust.")
                    robot.stop()
                    # Blink red+orange so the pause is unmistakable on the rover.
                    for _ in range(3):
                        robot.set_led(LED.RED, 255)
                        robot.set_led(LED.ORANGE, 255)
                        robot.set_led(LED.GREEN, 0)
                        time.sleep(0.5)
                        robot.set_led(LED.RED, 0)
                        robot.set_led(LED.ORANGE, 0)
                        time.sleep(0.5)
                    robot.set_led(LED.GREEN, 255)
                    px2, py2, pth2 = robot.get_odometry_pose()
                    print(f"[FSM] Resuming SEG3 exit at abs=({px2+ox:.0f},{py2+oy:.0f}) θ={pth2:.1f}°.")
                    _load_path(robot, _op(SEG3_EXIT_PATH), SEG3C_CFG, obstacle_avoidance=False)
                    seg3_phase = "exit"
                elif seg3_phase == "exit":
                    print(f"[FSM] SEG3 complete — CP3 reached at abs=({abs_x:.0f},{abs_y:.0f}) "
                          f"θ={pth:.1f}°.")
                    _load_path(robot, _op(SEG4_EAST_PATH), SEG4_CFG, obstacle_avoidance=False)
                    seg4_leg = "east"
                    state = "SEG4"

        # ── SEG4: CP3 → corner → lane 5 south → Finish (single path) ──────────
        #   Stop-sign detection active. First close-enough detection triggers a
        #   3-second hold (red LED). After dwell, nav resumes to the finish.
        elif state == "SEG4":
            if robot.get_button(Button.BTN_2):
                _estop(robot); return

            if stop_sign_seen and stop_sign_pause_t > 0.0:
                # Holding at stop sign
                robot.stop()
                robot.set_led(LED.RED, 255)
                robot.set_led(LED.GREEN, 0)
                if time.monotonic() - stop_sign_pause_t >= STOP_SIGN_DWELL_S:
                    print("[FSM] Stop-sign dwell complete — resuming to finish.")
                    stop_sign_pause_t = 0.0
                    robot.set_led(LED.RED, 0)
                    robot.set_led(LED.GREEN, 255)

            elif not stop_sign_seen:
                # Only trigger once the sign appears large enough in the frame
                # (proxy for being close), not on the very first sighting.
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
                    print(f"[FSM] Stop sign close (bbox h={h_frac:.2f} of frame) "
                          f"— pausing {STOP_SIGN_DWELL_S:.0f} s.")
                else:
                    # Keep driving until the sign is close enough
                    robot.set_led(LED.GREEN, 255)
                    robot.set_led(LED.ORANGE, 0)
                    if robot._nav_follow_pp_path_loop() == "IDLE":
                        if seg4_leg == "east":
                            print("[FSM] SEG4 east leg done — loading south leg.")
                            _load_path(robot, _op(SEG4_SOUTH_PATH), SEG4_CFG,
                                       obstacle_avoidance=False)
                            seg4_leg = "south"
                        else:
                            print("[FSM] SEG4 complete — at manipulation station.")
                            robot.stop()
                            state = "DONE"

            else:
                # Normal driving toward finish
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
