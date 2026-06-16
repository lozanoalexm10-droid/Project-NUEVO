# bridge_interfaces

Custom ROS2 interfaces for the raw firmware and bridge boundary.

## Overview

This ament_cmake package defines the message and service types shared across
packages that communicate through the bridge layer (DC motors, servos, steppers,
IMU, LiDAR, vision detections, system state, etc.).

## Messages (`msg/`)

Covers motor control (DC, servo, stepper), IO (LED, Neopixel), sensor
telemetry (IMU, kinematics, magnetometer), vision (tag detections, obstacle
tracking), and system management (config, diagnostics, power, state).

Key message families:
- `DC*` — DC motor commands and state
- `Servo*` / `Step*` — servo and stepper motor commands and state
- `Sensor*` — IMU, kinematics, magnetometer calibration
- `System*` / `Sys*` — system config, diagnostics, odometry
- `Tag*` / `Vision*` / `Tracked*` — vision pipeline outputs

## Services (`srv/`)

- `SetFirmwareState` — set the active firmware operating state

## Dependencies

- `std_msgs`
- `builtin_interfaces`
