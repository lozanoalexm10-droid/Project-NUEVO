# bridge

ROS2 wrapper package around the shared nuevo_bridge runtime.

## Overview

The bridge package mediates between the ROS2 graph and the Arduino firmware
over a serial connection. It translates ROS2 messages from `bridge_interfaces`
into firmware commands, and relays firmware state and sensor telemetry back
into the ROS2 ecosystem.

## Nodes / Entrypoints

- `bridge_node` — main bridge node; manages the serial connection lifecycle
- `ros_controller` — maps ROS2 command topics to firmware calls
- `ros_conversions` — helpers for converting between ROS2 and firmware types
- `firmware_state_service` — exposes a service for querying firmware state

## Launch

```bash
ros2 launch bridge bridge.launch.py
```

## Dependencies

- `rclpy`
- `std_msgs`
- `builtin_interfaces`
- `bridge_interfaces`
