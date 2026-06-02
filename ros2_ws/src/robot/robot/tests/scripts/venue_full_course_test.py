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
  CP2: (1068,    610)  — bottom    turn  midpoint   (lane 2 → obstacle zone)
  CP3: (1983,   3660)  — top-right U-turn midpoint  (obstacle zone → lane 5)

For a checkpoint restart: place the robot at CP_N with the correct heading,
set START_SEGMENT = N, then relaunch. Odometry resets to (0, 0) at the robot's
current position, so all segment waypoints are expressed relative to their own
checkpoint (0, 0). The GPS/AprilTag fusion will correct position drift in-flight.

Edit these two lines before each run:
"""
# ── Run configuration ─────────────────────────────────────────────────────────
START_SEGMENT = 1     # 1–4: segment to begin from; 1 = full course from start
AUTO_START    = True  # True = 3-second countdown; False = green-light trigger (BTN_1 to override)
# ─────────────────────────────────────────────────────────────────────────────

from __future__ import annotations

import math
import time

from robot.hardware_map import (
    Button,
    DCPidLoop,
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LED,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    TAG_ID,
    VELOCITY_KD,
    VELOCITY_KI,
    VELOCITY_KP,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline

# ── Nav tuning ─────────────────────────────────────────────────────────────────
LOOKAHEAD_MM             = 200.0   # larger = wider, smoother 90° corners
LINEAR_SPEED             = 140.0   # mm/s
MIN_TRAFFIC_CONF         = 0.50
VISION_STALE_S           = 3.0
STOP_SIGN_DWELL_S        = 3.0     # seconds to hold at stop sign before resuming

# ── Course geometry (mm, absolute odometry frame, x=east y=north) ─────────────
X_LANE1   =    0.0    # lane 1 centre (robot start x)
X_LANE2   =  610.0    # lane 2 centre
X_OBS_MID = 1525.0    # obstacle zone centreline
X_LANE5   = 2440.0    # lane 5 centre

Y_START   =    0.0    # course start y
Y_TOP     = 3660.0    # far (north) end of course — top turns happen here
Y_BOT     =  610.0    # near (south) end — bottom turns happen here

FINISH_X  = 2440.0    # manipulation station x (lane 5)
FINISH_Y  =  305.0    # manipulation station y

# Checkpoint coordinates — midpoints of the two 90° corners that form each U-turn.
# These are the absolute course positions. When restarting at CP_N the robot is
# placed here and odometry is reset; the segment waypoints below start from (0, 0).
CP1 = (  305.0, 3660.0)
CP2 = ( 1067.5,  610.0)
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
    (X_LANE2, Y_TOP),           # 90° right-turn corner (now south)
    (X_LANE2, Y_BOT),           # 90° right-turn corner (now east)
    (X_LANE2 + 457.5, Y_BOT),  # CP2: 457.5 mm east of corner  = (1067.5, 610)
]

# Segment 3: three internal sub-paths with different avoidance settings.
#   3a (avoidance OFF): CP2 east to obstacle zone entry; 90° left turn (north).
SEG3_APPROACH_PATH = [
    (X_LANE2 + 457.5, Y_BOT),  # CP2
    (X_OBS_MID, Y_BOT),         # obstacle zone entry; 90° left turn (north)
]
#   3b (avoidance ON): straight north through obstacle zone.
SEG3_OBS_PATH = [
    (X_OBS_MID, Y_BOT),
    (X_OBS_MID, Y_TOP),         # top of obstacle zone; 90° right turn (east)
]
#   3c (avoidance OFF): east to CP3.
SEG3_EXIT_PATH = [
    (X_OBS_MID, Y_TOP),
    (X_OBS_MID + 457.5, Y_TOP), # CP3: 457.5 mm east of corner  = (1982.5, 3660)
]

# Segment 4: CP3 → lane 5 south → manipulation station.
#   Stop-sign detection active throughout. 3-second dwell on first detection.
SEG4_PATH = [
    (X_OBS_MID + 457.5, Y_TOP), # CP3
    (X_LANE5, Y_TOP),            # 90° right-turn corner (now south)
    (FINISH_X, FINISH_Y),        # manipulation station
]

# Map START_SEGMENT to its pre-loaded path (loaded before IDLE so the controller
# is armed when the start trigger fires).
_SEG_INITIAL_PATH = {
    1: (SEG1_PATH,         False, 0.0),
    2: (SEG2_PATH,         False, 0.0),
    3: (SEG3_APPROACH_PATH, False, 0.0),
    4: (SEG4_PATH,         False, 0.0),
}


# ── Helpers ───────────────────────────────────────────────────────────────────

def _configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
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


def _init_firmware(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    time.sleep(0.2)
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=3.0):
        print("[WARN] Odometry reset not confirmed within 3 s — pose may be stale")
        robot.wait_for_pose_update(timeout=1.0)
    print(f"[CONFIG] Pose after reset: {robot.get_odometry_pose()}")


def _load_path(
    robot: Robot,
    waypoints: list[tuple[float, float]],
    *,
    obstacle_avoidance: bool,
    x_L: float = 0.0,
) -> None:
    path = densify_polyline(waypoints, spacing=50.0)
    if obstacle_avoidance:
        robot._nav_follow_pp_path(
            lookahead_distance=LOOKAHEAD_MM,
            max_linear_speed=LINEAR_SPEED,
            max_angular_speed=1.0,
            goal_tolerance=30.0,
            obstacles_range=550.0,
            view_angle=math.radians(65.0),
            safe_dist=360.0,
            avoidance_delay=420,
            alpha_Ld=1.3,
            offset=250.0,
            lane_width=400.0,
            obstacle_avoidance=True,
            x_L=x_L,
        )
    else:
        robot._nav_follow_pp_path(
            lookahead_distance=LOOKAHEAD_MM,
            max_linear_speed=LINEAR_SPEED,
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
    _init_firmware(robot)

    # Pre-arm the first segment's path before entering IDLE.
    first_path, first_avoidance, first_xL = _SEG_INITIAL_PATH[START_SEGMENT]
    _load_path(robot, first_path, obstacle_avoidance=first_avoidance, x_L=first_xL)
    print(f"[FSM] Ready. START_SEGMENT={START_SEGMENT}  AUTO_START={AUTO_START}")
    print(f"[FSM] Checkpoints — CP1:{CP1}  CP2:{CP2}  CP3:{CP3}")

    state: str = "IDLE"
    seg3_phase: str = "approach"   # 'approach' | 'obstacle' | 'exit'
    stop_sign_seen:    bool  = False
    stop_sign_pause_t: float = 0.0

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
                robot.set_led(LED.ORANGE, 255)
                robot.set_led(LED.GREEN, 0)
                robot.set_led(LED.RED, 0)
                robot._draw_lidar_obstacles()
                print(f"[FSM] Auto-starting SEG{START_SEGMENT} in 3 s …")
                time.sleep(3)
                robot.set_led(LED.ORANGE, 0)
                state = f"SEG{START_SEGMENT}"
                print(f"[FSM] → {state}")
            else:
                # Traffic-light mode: stay stopped on red, go on green.
                robot.set_led(LED.RED, 255)
                robot.set_led(LED.GREEN, 0)
                robot.set_led(LED.ORANGE, 0)
                color = _traffic_light_color(robot)
                if color == "green":
                    print(f"[FSM] Green light — starting SEG{START_SEGMENT}.")
                    robot.set_led(LED.RED, 0)
                    robot.set_led(LED.GREEN, 255)
                    state = f"SEG{START_SEGMENT}"
                elif robot.get_button(Button.BTN_1):
                    print(f"[FSM] BTN_1 override — starting SEG{START_SEGMENT}.")
                    robot.set_led(LED.RED, 0)
                    robot.set_led(LED.GREEN, 255)
                    state = f"SEG{START_SEGMENT}"
                # color == 'red' or None → keep red LED on, stay in IDLE

        # ── SEG1: Lane 1 north → CP1 (first half of top-left U-turn) ─────────
        elif state == "SEG1":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            if robot.get_button(Button.BTN_2):
                _estop(robot); return
            if robot._nav_follow_pp_path_loop() == "IDLE":
                print("[FSM] SEG1 complete — CP1 reached.")
                _load_path(robot, SEG2_PATH, obstacle_avoidance=False)
                state = "SEG2"

        # ── SEG2: CP1 → complete top-left U-turn → lane 2 south → CP2 ────────
        elif state == "SEG2":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            if robot.get_button(Button.BTN_2):
                _estop(robot); return
            if robot._nav_follow_pp_path_loop() == "IDLE":
                print("[FSM] SEG2 complete — CP2 reached.")
                _load_path(robot, SEG3_APPROACH_PATH, obstacle_avoidance=False)
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
                if seg3_phase == "approach":
                    print("[FSM] SEG3 approach done — entering obstacle zone.")
                    _load_path(robot, SEG3_OBS_PATH, obstacle_avoidance=True, x_L=X_OBS_MID)
                    seg3_phase = "obstacle"
                elif seg3_phase == "obstacle":
                    print("[FSM] SEG3 obstacle zone done — exiting to CP3 (avoidance ON).")
                    _load_path(robot, SEG3_EXIT_PATH, obstacle_avoidance=True, x_L=X_OBS_MID)
                    seg3_phase = "exit"
                elif seg3_phase == "exit":
                    print("[FSM] SEG3 complete — CP3 reached.")
                    _load_path(robot, SEG4_PATH, obstacle_avoidance=False)
                    state = "SEG4"

        # ── SEG4: CP3 → complete top-right U-turn → lane 5 south → Finish ─────
        #   Stop-sign detection active. First detection triggers a 3-second hold
        #   (red LED). After dwell, nav resumes to the manipulation station.
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

            elif not stop_sign_seen and robot.get_detections("stop sign"):
                # First stop-sign sighting — begin dwell
                robot.stop()
                stop_sign_seen    = True
                stop_sign_pause_t = time.monotonic()
                robot.set_led(LED.RED, 255)
                robot.set_led(LED.GREEN, 0)
                print(f"[FSM] Stop sign detected — pausing {STOP_SIGN_DWELL_S:.0f} s.")

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
