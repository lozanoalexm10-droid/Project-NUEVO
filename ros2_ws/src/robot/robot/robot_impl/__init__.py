"""
robot.robot_impl
================
Implementation detail of the :class:`robot.Robot` class.

The Robot API is composed of four mixin classes, each in its own module:

  hardware.py   — HardwareMixin: low-level firmware control (motors, servos,
                  steppers, buttons, LEDs, limit switches)
  sensors.py    — SensorsMixin: opt-in sensor subscriptions (lidar, GPS/ArUco,
                  IMU, vision) and their callbacks
  navigation.py — NavigationMixin: odometry, sensor fusion, pose API, drive
                  commands, path following (also defines MotionHandle)
  legacy.py     — LegacyMixin: quarantined pure-pursuit code kept only for the
                  obstacle_avoidance.py example

The mixins are combined in ``robot/robot.py``:

    class Robot(HardwareMixin, SensorsMixin, NavigationMixin, LegacyMixin):
        ...

All instance state lives in ``Robot.__init__`` — the mixins only define methods.

The ``_impl`` suffix is the Python convention for "implementation details":
users of the package see the unified ``Robot`` API, while the source is split
into focused modules so the class doesn't grow into a single 2000-line file.
"""
