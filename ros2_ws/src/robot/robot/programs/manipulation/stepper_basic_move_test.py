from __future__ import annotations

import time

from robot.hardware_map import Stepper, StepMoveType, StepperMotionState
from robot.robot import FirmwareState, Robot

STEPPER_ID = Stepper.STEPPER_1
STEP_COUNT = 200        # steps to move in each direction
MAX_VELOCITY = 1000     # steps/sec
ACCELERATION = 500      # steps/sec^2
MOVE_TIMEOUT_S = 10.0


def _assert_idle(robot: Robot, label: str) -> None:
    state = robot.get_step_state()
    motion = state.states[STEPPER_ID - 1].state
    if motion != StepperMotionState.IDLE:
        print(f"[TEST] FAIL: {label} — stepper not IDLE after move (state={motion})")
        raise RuntimeError("stepper did not reach IDLE")


def run(robot: Robot) -> None:
    print(f"[TEST] stepper_basic_move_test starting (stepper {STEPPER_ID.name})")
    robot.set_state(FirmwareState.RUNNING)

    robot.step_set_config(STEPPER_ID, max_velocity=MAX_VELOCITY, acceleration=ACCELERATION)
    robot.step_enable(STEPPER_ID)
    time.sleep(0.1)

    print(f"[TEST] Moving forward {STEP_COUNT} steps")
    ok = robot.step_move(STEPPER_ID, STEP_COUNT, StepMoveType.RELATIVE, timeout=MOVE_TIMEOUT_S)
    if not ok:
        print("[TEST] FAIL: forward move timed out")
        robot.step_disable(STEPPER_ID)
        return

    _assert_idle(robot, "forward move")
    print("[TEST] Forward move complete")
    time.sleep(0.5)

    print(f"[TEST] Moving back {STEP_COUNT} steps")
    ok = robot.step_move(STEPPER_ID, -STEP_COUNT, StepMoveType.RELATIVE, timeout=MOVE_TIMEOUT_S)
    if not ok:
        print("[TEST] FAIL: return move timed out")
        robot.step_disable(STEPPER_ID)
        return

    _assert_idle(robot, "return move")
    print("[TEST] Return move complete")

    robot.step_disable(STEPPER_ID)
    print("[TEST] PASS — stepper moved forward and back without timeout")
    print("[TEST] Verify visually: shaft returned to original position")
