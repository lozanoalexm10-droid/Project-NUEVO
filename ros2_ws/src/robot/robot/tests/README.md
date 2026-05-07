# Robot Test Framework

Tests for verifying individual subsystems before they are integrated into a full program.
All tests use the `run(robot: Robot) -> None` pattern and print `[TEST] PASS` or `[TEST] FAIL: <reason>`.

---

## Folder structure

```
tests/
  scripts/                   individual subsystem test scripts
    _manipulator_config.py   shared hardware constants for all manipulator tests
    <subsystem>_<purpose>_test.py
  logs/                      test output — write logs here during sessions
  TEST_PLAN.md               full test list with node requirements and pass criteria
  README.md                  this file
```

---

## How tests relate to programs

`tests/scripts/` — isolated, pass/fail diagnostics. One subsystem at a time.
`programs/`      — full operational sequences run via `main.py`. No pass/fail.

Some scripts in `tests/scripts/` (like `lane_switch_obstacle_test.py` and
`venue_no_obstacles_test.py`) can also be run via `main.py` by importing them there.
See `main.py` for the full list of importable programs.

---

## Running a test

All tests run inside the Docker container. At minimum the `robot` node and `bridge` node
must be running. See `TEST_PLAN.md` for exact node requirements per test.

```bash
# Open a shell inside the container
docker compose exec ros2_runtime bash

# Run a test
python3 /ros2_ws/src/robot/robot/tests/scripts/<script>.py

# Run and save log
python3 /ros2_ws/src/robot/robot/tests/scripts/<script>.py | tee tests/logs/<script>.log
```

---

## Naming convention

```
<subsystem>_<purpose>_test.py      e.g. turntable_range_test.py
```

Shared config that is not a test: prefix with underscore — `_manipulator_config.py`.

---

## Recommended order

Drive tests first, then manipulator subsystems, then combined, then heating wire last.
See `TEST_PLAN.md` for the full sequence.
