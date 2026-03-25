"""
TLV payload structures mirrored from firmware/arduino/src/messages/TLV_Payloads.h.

These ctypes definitions are the bridge-side wire-format source for the Arduino
TLV protocol. Keep them byte-exact with the firmware payload header.
"""

from __future__ import annotations

import ctypes


TLV_MAX_DC_MOTORS = 4
TLV_MAX_STEPPERS = 4
TLV_MAX_SERVO_CHANNELS = 16
TLV_MAX_ULTRASONICS = 4
TLV_MAX_USER_LEDS = 5


# ============================================================================
# SYSTEM
# ============================================================================


class PayloadHeartbeat(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp", ctypes.c_uint32),
        ("flags", ctypes.c_uint8),
    ]


class PayloadSysState(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("state", ctypes.c_uint8),
        ("warningFlags", ctypes.c_uint8),
        ("errorFlags", ctypes.c_uint8),
        ("runtimeFlags", ctypes.c_uint8),
        ("uptimeMs", ctypes.c_uint32),
        ("lastRxMs", ctypes.c_uint16),
        ("lastCmdMs", ctypes.c_uint16),
    ]


class PayloadSysCmd(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("command", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
    ]


class PayloadSysInfoReq(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("target", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
    ]


class PayloadSysInfoRsp(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("firmwareMajor", ctypes.c_uint8),
        ("firmwareMinor", ctypes.c_uint8),
        ("firmwarePatch", ctypes.c_uint8),
        ("protocolMajor", ctypes.c_uint8),
        ("protocolMinor", ctypes.c_uint8),
        ("boardRevision", ctypes.c_uint8),
        ("featureMask", ctypes.c_uint8),
        ("sensorCapabilityMask", ctypes.c_uint8),
        ("dcMotorCount", ctypes.c_uint8),
        ("stepperCount", ctypes.c_uint8),
        ("servoChannelCount", ctypes.c_uint8),
        ("ultrasonicMaxCount", ctypes.c_uint8),
        ("userLedCount", ctypes.c_uint8),
        ("maxNeoPixelCount", ctypes.c_uint8),
        ("limitSwitchMask", ctypes.c_uint16),
        ("stepperHomeLimitGpio", ctypes.c_uint8 * TLV_MAX_STEPPERS),
    ]


class PayloadSysConfigReq(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("target", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
    ]


class PayloadSysConfigRsp(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("motorDirMask", ctypes.c_uint8),
        ("configuredSensorMask", ctypes.c_uint8),
        ("neoPixelCount", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
        ("heartbeatTimeoutMs", ctypes.c_uint16),
        ("reserved2", ctypes.c_uint16),
    ]


class PayloadSysConfigSet(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("motorDirMask", ctypes.c_uint8),
        ("motorDirChangeMask", ctypes.c_uint8),
        ("neoPixelCount", ctypes.c_uint8),
        ("configuredSensorMask", ctypes.c_uint8),
        ("heartbeatTimeoutMs", ctypes.c_uint16),
        ("reserved", ctypes.c_uint16),
    ]


class PayloadSysPower(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("batteryMv", ctypes.c_uint16),
        ("rail5vMv", ctypes.c_uint16),
        ("servoRailMv", ctypes.c_uint16),
        # BATTERY_TYPE constant from firmware config.h.
        ("batteryType", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
        ("timestamp", ctypes.c_uint32),
    ]


class PayloadSysDiagReq(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("target", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
    ]


class PayloadSysDiagRsp(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("freeSram", ctypes.c_uint16),
        ("loopTimeAvgUs", ctypes.c_uint16),
        ("loopTimeMaxUs", ctypes.c_uint16),
        ("uartRxErrors", ctypes.c_uint16),
        ("crcErrors", ctypes.c_uint16),
        ("frameErrors", ctypes.c_uint16),
        ("tlvErrors", ctypes.c_uint16),
        ("oversizeErrors", ctypes.c_uint16),
        ("txPendingBytes", ctypes.c_uint16),
        ("reserved", ctypes.c_uint16),
        ("txDroppedFrames", ctypes.c_uint32),
    ]


class PayloadSysOdomReset(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("flags", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
    ]


# ============================================================================
# DC MOTORS
# ============================================================================


class PayloadDCEnable(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("motorId", ctypes.c_uint8),
        ("mode", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 2),
    ]


class PayloadDCSetPosition(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("motorId", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
        ("targetTicks", ctypes.c_int32),
        ("maxVelTicks", ctypes.c_int32),
    ]


class PayloadDCSetVelocity(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("motorId", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
        ("targetTicks", ctypes.c_int32),
    ]


class PayloadDCSetPWM(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("motorId", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
        ("pwm", ctypes.c_int16),
    ]


class DCMotorState(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("mode", ctypes.c_uint8),
        ("faultFlags", ctypes.c_uint8),
        ("position", ctypes.c_int32),
        ("velocity", ctypes.c_int32),
        ("targetPos", ctypes.c_int32),
        ("targetVel", ctypes.c_int32),
        ("pwmOutput", ctypes.c_int16),
        ("currentMa", ctypes.c_int16),
    ]


class PayloadDCStateAll(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("motors", DCMotorState * TLV_MAX_DC_MOTORS),
        ("timestamp", ctypes.c_uint32),
    ]


class PayloadDCPidReq(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("motorId", ctypes.c_uint8),
        ("loopType", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 2),
    ]


class PayloadDCPidRsp(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("motorId", ctypes.c_uint8),
        ("loopType", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 2),
        ("kp", ctypes.c_float),
        ("ki", ctypes.c_float),
        ("kd", ctypes.c_float),
        ("maxOutput", ctypes.c_float),
        ("maxIntegral", ctypes.c_float),
    ]


PayloadDCPidSet = PayloadDCPidRsp


# ============================================================================
# STEPPERS
# ============================================================================


class PayloadStepEnable(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("stepperId", ctypes.c_uint8),
        ("enable", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 2),
    ]


class PayloadStepMove(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("stepperId", ctypes.c_uint8),
        ("moveType", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 2),
        ("target", ctypes.c_int32),
    ]


class PayloadStepHome(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("stepperId", ctypes.c_uint8),
        ("direction", ctypes.c_int8),
        ("reserved", ctypes.c_uint8 * 2),
        ("homeVelocity", ctypes.c_uint32),
        ("backoffSteps", ctypes.c_int32),
    ]


class PayloadStepConfigReq(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("stepperId", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
    ]


class PayloadStepConfigRsp(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("stepperId", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
        ("maxVelocity", ctypes.c_uint32),
        ("acceleration", ctypes.c_uint32),
    ]


PayloadStepConfigSet = PayloadStepConfigRsp


class StepperChannelState(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("enabled", ctypes.c_uint8),
        ("motionState", ctypes.c_uint8),
        ("limitFlags", ctypes.c_uint8),
        ("reserved", ctypes.c_int8),
        ("count", ctypes.c_int32),
        ("targetCount", ctypes.c_int32),
        ("currentSpeed", ctypes.c_int32),
    ]


class PayloadStepStateAll(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("steppers", StepperChannelState * TLV_MAX_STEPPERS),
        ("timestamp", ctypes.c_uint32),
    ]


# ============================================================================
# SERVOS
# ============================================================================


class PayloadServoEnable(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("channel", ctypes.c_uint8),
        ("enable", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 2),
    ]


class PayloadServoSetSingle(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("channel", ctypes.c_uint8),
        ("count", ctypes.c_uint8),
        ("pulseUs", ctypes.c_uint16 * 1),
    ]


class PayloadServoSetBulk(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("startChannel", ctypes.c_uint8),
        ("count", ctypes.c_uint8),
        ("pulseUs", ctypes.c_uint16 * TLV_MAX_SERVO_CHANNELS),
    ]


class PayloadServoStateAll(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("pca9685Connected", ctypes.c_uint8),
        ("pca9685Error", ctypes.c_uint8),
        ("enabledMask", ctypes.c_uint16),
        ("pulseUs", ctypes.c_uint16 * TLV_MAX_SERVO_CHANNELS),
        ("timestamp", ctypes.c_uint32),
    ]


# ============================================================================
# SENSORS
# ============================================================================


class PayloadSensorIMU(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("quatW", ctypes.c_float),
        ("quatX", ctypes.c_float),
        ("quatY", ctypes.c_float),
        ("quatZ", ctypes.c_float),
        ("earthAccX", ctypes.c_float),
        ("earthAccY", ctypes.c_float),
        ("earthAccZ", ctypes.c_float),
        ("rawAccX", ctypes.c_int16),
        ("rawAccY", ctypes.c_int16),
        ("rawAccZ", ctypes.c_int16),
        ("rawGyroX", ctypes.c_int16),
        ("rawGyroY", ctypes.c_int16),
        ("rawGyroZ", ctypes.c_int16),
        ("magX", ctypes.c_int16),
        ("magY", ctypes.c_int16),
        ("magZ", ctypes.c_int16),
        ("magCalibrated", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
        ("timestamp", ctypes.c_uint32),
    ]


class PayloadSensorKinematics(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("theta", ctypes.c_float),
        ("vx", ctypes.c_float),
        ("vy", ctypes.c_float),
        ("vTheta", ctypes.c_float),
        ("timestamp", ctypes.c_uint32),
    ]


class UltrasonicState(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("status", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
        ("distanceMm", ctypes.c_uint16),
    ]


class PayloadSensorUltrasonicAll(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("configuredCount", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
        ("sensors", UltrasonicState * TLV_MAX_ULTRASONICS),
        ("timestamp", ctypes.c_uint32),
    ]


class PayloadMagCalCmd(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("command", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
        ("offsetX", ctypes.c_float),
        ("offsetY", ctypes.c_float),
        ("offsetZ", ctypes.c_float),
        ("softIronMatrix", ctypes.c_float * 9),
    ]


class PayloadMagCalStatus(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("state", ctypes.c_uint8),
        ("sampleCount", ctypes.c_uint16),
        ("reserved", ctypes.c_uint8),
        ("minX", ctypes.c_float),
        ("maxX", ctypes.c_float),
        ("minY", ctypes.c_float),
        ("maxY", ctypes.c_float),
        ("minZ", ctypes.c_float),
        ("maxZ", ctypes.c_float),
        ("offsetX", ctypes.c_float),
        ("offsetY", ctypes.c_float),
        ("offsetZ", ctypes.c_float),
        ("savedToEeprom", ctypes.c_uint8),
        ("reserved2", ctypes.c_uint8 * 3),
    ]


# ============================================================================
# USER IO
# ============================================================================


class PayloadSetLED(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("ledId", ctypes.c_uint8),
        ("mode", ctypes.c_uint8),
        ("brightness", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
        ("periodMs", ctypes.c_uint16),
        ("dutyCycle", ctypes.c_uint16),
    ]


class PayloadSetNeoPixel(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("index", ctypes.c_uint8),
        ("red", ctypes.c_uint8),
        ("green", ctypes.c_uint8),
        ("blue", ctypes.c_uint8),
    ]


class PayloadIOInputState(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("buttonMask", ctypes.c_uint16),
        ("limitMask", ctypes.c_uint16),
        ("timestamp", ctypes.c_uint32),
    ]


class PayloadIOOutputState(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("ledBrightness", ctypes.c_uint8 * TLV_MAX_USER_LEDS),
        ("neoPixelCount", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
        ("timestamp", ctypes.c_uint32),
    ]


# ============================================================================
# SIZE VERIFICATION
# ============================================================================


def verify_payload_sizes() -> None:
    expected_sizes = {
        PayloadHeartbeat: 5,
        PayloadSysState: 12,
        PayloadSysCmd: 4,
        PayloadSysInfoReq: 4,
        PayloadSysInfoRsp: 20,
        PayloadSysConfigReq: 4,
        PayloadSysConfigRsp: 8,
        PayloadSysConfigSet: 8,
        PayloadSysPower: 12,
        PayloadSysDiagReq: 4,
        PayloadSysDiagRsp: 24,
        PayloadSysOdomReset: 4,
        PayloadDCEnable: 4,
        PayloadDCSetPosition: 12,
        PayloadDCSetVelocity: 8,
        PayloadDCSetPWM: 4,
        DCMotorState: 22,
        PayloadDCStateAll: 92,
        PayloadDCPidReq: 4,
        PayloadDCPidRsp: 24,
        PayloadStepEnable: 4,
        PayloadStepMove: 8,
        PayloadStepHome: 12,
        PayloadStepConfigReq: 4,
        PayloadStepConfigRsp: 12,
        StepperChannelState: 16,
        PayloadStepStateAll: 68,
        PayloadServoEnable: 4,
        PayloadServoSetSingle: 4,
        PayloadServoSetBulk: 34,
        PayloadServoStateAll: 40,
        PayloadSensorIMU: 52,
        PayloadSensorKinematics: 28,
        UltrasonicState: 4,
        PayloadSensorUltrasonicAll: 24,
        PayloadMagCalCmd: 52,
        PayloadMagCalStatus: 44,
        PayloadSetLED: 8,
        PayloadSetNeoPixel: 4,
        PayloadIOInputState: 8,
        PayloadIOOutputState: 11,
    }

    errors = []
    for payload_class, expected in expected_sizes.items():
        actual = ctypes.sizeof(payload_class)
        if actual != expected:
            errors.append(f"{payload_class.__name__}: expected {expected}, got {actual}")

    if errors:
        raise AssertionError("Payload size mismatches:\n  " + "\n  ".join(errors))


# ============================================================================
# TRANSITIONAL COMPATIBILITY ALIASES
# ============================================================================
#
# These names let older bridge modules import successfully while the runtime
# migration is completed. New code should use the v4 names above.

PayloadSystemStatus = PayloadSysState
PayloadSysConfig = PayloadSysConfigSet
PayloadSetPID = PayloadDCPidSet
PayloadDCStatusAll = PayloadDCStateAll
PayloadStepSetParams = PayloadStepConfigSet
PayloadStepStatusAll = PayloadStepStateAll
PayloadServoStatusAll = PayloadServoStateAll
PayloadSensorVoltage = PayloadSysPower
DCMotorStatus = DCMotorState
StepperStatus = StepperChannelState


class PayloadSensorRange(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("sensorId", ctypes.c_uint8),
        ("sensorType", ctypes.c_uint8),
        ("status", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
        ("distanceMm", ctypes.c_uint16),
        ("reserved2", ctypes.c_uint16),
        ("timestamp", ctypes.c_uint32),
    ]


class PayloadIOStatus(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("buttonMask", ctypes.c_uint16),
        ("ledBrightness", ctypes.c_uint8 * TLV_MAX_USER_LEDS),
        ("reserved", ctypes.c_uint8),
        ("timestamp", ctypes.c_uint32),
    ]


if __name__ == "__main__":
    verify_payload_sizes()
