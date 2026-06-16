# sensors

ROS2 package for Raspberry Pi connected sensors outside the Arduino firmware.

## Overview

This package hosts ROS2 nodes for sensors that connect directly to the onboard
Raspberry Pi rather than going through the Arduino firmware/bridge layer.
Sensor data is published using message types from `bridge_interfaces`.

## Nodes

- `sensor_node` — base sensor abstraction
- `robot_gps_node` — GPS integration node
- `qwiic_ultrasonic_node` — SparkFun Qwiic ultrasonic distance sensor
- `mock_lidar_node` — software mock of LiDAR output for testing

## Launch

```bash
ros2 launch sensors sensors.launch.py
```

## Dependencies

- `rclpy`
- `bridge_interfaces`
- `sensor_msgs`
