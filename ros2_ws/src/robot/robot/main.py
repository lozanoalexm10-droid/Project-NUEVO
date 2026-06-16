# Uncomment exactly one import below to select the active program.

# ── Demos ─────────────────────────────────────────────────────────────────────
#from robot.programs.demos.full_competition_venue_run import run  # noqa: F401
from robot.programs.demos.precision_stack_manipulator_demo import run  # noqa: F401

# ── Navigation ────────────────────────────────────────────────────────────────
#from robot.programs.navigation.straight_line_test import run  # noqa: F401
#from robot.programs.navigation.lane_switch_obstacle_test import run  # noqa: F401
#from robot.programs.navigation.uturn_test import run  # noqa: F401
#from robot.programs.navigation.seg4_standalone_test import run  # noqa: F401
#from robot.programs.navigation.drive_both_wheels_test import run  # noqa: F401

# ── Manipulation (calibration + per-component tests) ──────────────────────────
#from robot.programs.manipulation.arm_ik_calibration_test import run  # noqa: F401
#from robot.programs.manipulation.arm_extend_retract_test import run  # noqa: F401
#from robot.programs.manipulation.elbow_calibration_test import run  # noqa: F401
#from robot.programs.manipulation.servo_calibration import run  # noqa: F401
#from robot.programs.manipulation.servo_range_test import run  # noqa: F401
#from robot.programs.manipulation.elbow_range_test import run  # noqa: F401
#from robot.programs.manipulation.shoulder_range_test import run  # noqa: F401
#from robot.programs.manipulation.gripper_open_close_test import run  # noqa: F401
#from robot.programs.manipulation.campan_range_test import run  # noqa: F401
#from robot.programs.manipulation.turntable_range_test import run  # noqa: F401
#from robot.programs.manipulation.turntable_home_test import run  # noqa: F401
#from robot.programs.manipulation.stepper_basic_move_test import run  # noqa: F401
#from robot.programs.manipulation.disable_servos import run  # noqa: F401
#from robot.programs.manipulation.heating_wire_test import run  # noqa: F401

# ── Perception (vision + lidar) ───────────────────────────────────────────────
#from robot.programs.perception.lidar_snapshot_test import run  # noqa: F401
#from robot.programs.perception.green_light_detect_test import run  # noqa: F401
#from robot.programs.perception.mallow_detection_range_test import run  # noqa: F401
#from robot.programs.perception.detection_vision_test import run  # noqa: F401

# ── Analysis (data recording + plotting) ──────────────────────────────────────
#from robot.programs.analysis.nav_data_recorder import run  # noqa: F401
