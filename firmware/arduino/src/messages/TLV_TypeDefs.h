#pragma once

#include <stdint.h>

// Define TLV type constants here so both server and client can use them:
// A valid TLV type is a non-negative integer.
// This file is auto-generated from TLV_TypeDefs.json - DO NOT EDIT MANUALLY

// ============================================================================
// TLV Type Constants
// ============================================================================

constexpr uint32_t SYS_HEARTBEAT = 1U;
constexpr uint32_t SYS_STATUS = 2U;
constexpr uint32_t SYS_CMD = 3U;
constexpr uint32_t SYS_CONFIG = 4U;
constexpr uint32_t SYS_SET_PID = 5U;
constexpr uint32_t DC_ENABLE = 256U;
constexpr uint32_t DC_SET_POSITION = 257U;
constexpr uint32_t DC_SET_VELOCITY = 258U;
constexpr uint32_t DC_SET_PWM = 259U;
constexpr uint32_t DC_STATUS_ALL = 260U;
constexpr uint32_t STEP_ENABLE = 512U;
constexpr uint32_t STEP_SET_PARAMS = 513U;
constexpr uint32_t STEP_MOVE = 514U;
constexpr uint32_t STEP_HOME = 515U;
constexpr uint32_t STEP_STATUS_ALL = 516U;
constexpr uint32_t SERVO_ENABLE = 768U;
constexpr uint32_t SERVO_SET = 769U;
constexpr uint32_t SERVO_STATUS_ALL = 770U;
constexpr uint32_t SENSOR_IMU = 1024U;
constexpr uint32_t SENSOR_KINEMATICS = 1025U;
constexpr uint32_t SENSOR_VOLTAGE = 1026U;
constexpr uint32_t SENSOR_RANGE = 1027U;
constexpr uint32_t SENSOR_MAG_CAL_CMD = 1028U;
constexpr uint32_t SENSOR_MAG_CAL_STATUS = 1029U;
constexpr uint32_t IO_SET_LED = 1280U;
constexpr uint32_t IO_SET_NEOPIXEL = 1281U;
constexpr uint32_t IO_STATUS = 1282U;

