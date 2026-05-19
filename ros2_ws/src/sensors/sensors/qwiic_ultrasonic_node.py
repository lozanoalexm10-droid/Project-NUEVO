"""
ROS2 driver for the SparkFun Qwiic Ultrasonic Distance Sensor (TCT40).

Reads distance via I2C (default address 0x2F) and publishes sensor_msgs/Range
on /ultrasonic_range at ~10 Hz.

Protocol: write any byte to trigger a measurement, wait ~80 ms, read 2 bytes
(big-endian) for distance in mm.

Parameters
----------
i2c_bus     : int   (default 1)    — /dev/i2c-<N>
i2c_address : int   (default 0x2F)
rate_hz     : float (default 10.0)
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range

_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

MIN_RANGE_M = 0.02   # 2 cm
MAX_RANGE_M = 4.00   # 400 cm


class QwiicUltrasonicNode(Node):
    def __init__(self) -> None:
        super().__init__("qwiic_ultrasonic")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x2F)
        self.declare_parameter("rate_hz", 10.0)

        self._bus_num = self.get_parameter("i2c_bus").value
        self._addr    = self.get_parameter("i2c_address").value
        self._rate_hz = float(self.get_parameter("rate_hz").value)

        self._pub = self.create_publisher(Range, "/ultrasonic_range", _BEST_EFFORT)
        self._bus = None
        self._open_bus()

        self.create_timer(1.0 / self._rate_hz, self._tick)
        self.get_logger().info(
            f"Qwiic ultrasonic node started "
            f"(I2C bus {self._bus_num}, addr 0x{self._addr:02X}, {self._rate_hz:.0f} Hz)"
        )

    def _open_bus(self) -> None:
        try:
            import smbus2
            self._bus = smbus2.SMBus(self._bus_num)
            self.get_logger().info(f"Opened /dev/i2c-{self._bus_num}")
        except Exception as exc:
            self.get_logger().error(
                f"Cannot open I2C bus {self._bus_num}: {exc} — will retry each tick"
            )
            self._bus = None

    def _read_distance_mm(self) -> int | None:
        """Trigger a measurement and return distance in mm, or None on error."""
        try:
            import smbus2
            if self._bus is None:
                self._open_bus()
                if self._bus is None:
                    return None
            # Write any byte to trigger the measurement, then wait for the echo
            self._bus.write_byte(self._addr, 0x01)
            time.sleep(0.08)
            data = self._bus.read_i2c_block_data(self._addr, 0x01, 2)
            return (data[0] << 8) | data[1]
        except Exception as exc:
            self.get_logger().warn(f"I2C read error: {exc}")
            self._bus = None
            return None

    def _tick(self) -> None:
        dist_mm = self._read_distance_mm()

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ultrasonic_link"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.2618  # ~15 degrees
        msg.min_range = MIN_RANGE_M
        msg.max_range = MAX_RANGE_M

        if dist_mm is None or dist_mm == 0:
            # Sentinel: range > max_range signals invalid/no-echo
            msg.range = MAX_RANGE_M + 1.0
        else:
            msg.range = dist_mm / 1000.0

        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QwiicUltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
