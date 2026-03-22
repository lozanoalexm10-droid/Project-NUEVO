"""
Message Router — TLV ↔ JSON Translation

The WebSocket surface now follows the same v4 protocol naming as the wire
protocol. Frontend and ROS adapters can still derive merged views locally, but
the transport topics themselves are no longer renamed into legacy UI aliases.
"""

from __future__ import annotations

import asyncio
import ctypes
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, Union

from .TLV_TypeDefs import *
from .payloads import *
from .mag_calibration import IDENTITY_3X3, MagCalibrationController


DecodedMessages = Optional[List[dict]]


WARN_TO_LEGACY_ERROR = {
    0x01: 0x20,  # WARN_LIVENESS_LOST -> legacy error bit 5
    0x02: 0x40,  # WARN_LOOP_OVERRUN  -> legacy error bit 6
}


def _decode_fixed(payload_class, tlv_data: bytes) -> Optional[dict]:
    expected = ctypes.sizeof(payload_class)
    if len(tlv_data) != expected:
        return None
    return _struct_to_dict(payload_class.from_buffer_copy(tlv_data))


def _struct_to_dict(struct_value: ctypes.Structure) -> dict:
    result = {}
    for field_name, _ in struct_value._fields_:
        if field_name.startswith("reserved"):
            continue
        value = getattr(struct_value, field_name)
        if isinstance(value, ctypes.Structure):
            result[field_name] = _struct_to_dict(value)
        elif hasattr(value, "__len__") and not isinstance(value, (str, bytes)):
            items = []
            for item in value:
                if isinstance(item, ctypes.Structure):
                    items.append(_struct_to_dict(item))
                else:
                    items.append(item)
            result[field_name] = items
        else:
            result[field_name] = value
    return result


def _clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))


class MessageRouter:
    """Bidirectional TLV ↔ JSON router with wire/UI compatibility handling."""

    def __init__(self, ws_manager):
        self.ws_manager = ws_manager
        self._transport_sender: Optional[Callable[[int, ctypes.Structure], None]] = None
        self._mag_cal_controller = MagCalibrationController(sender=self.send_wire_command)

        self._dc_frame_counter = 0
        self._last_uptime_ms: Optional[int] = None

        self._sys_state: Optional[dict] = None
        self._sys_power: Optional[dict] = None
        self._sys_info: Optional[dict] = None
        self._sys_config: Optional[dict] = None
        self._sys_diag: Optional[dict] = None
        self._io_input: Optional[dict] = None
        self._io_output: Optional[dict] = None
        self._dc_pid_cache: Dict[Tuple[int, int], dict] = {}
        self._step_config_cache: Dict[int, dict] = {}
        self._latest_ws_messages: Dict[str, dict] = {}

    # ------------------------------------------------------------------
    # Transport
    # ------------------------------------------------------------------

    def attach_transport_sender(self, sender: Callable[[int, ctypes.Structure], None]) -> None:
        self._transport_sender = sender

    def send_wire_command(self, cmd: str, data: Dict[str, Any]) -> bool:
        result = self.handle_outgoing(cmd, data)
        if result is None or self._transport_sender is None:
            return False
        tlv_type, payload = result
        self._transport_sender(tlv_type, payload)
        return True

    # ------------------------------------------------------------------
    # Bootstrap / reset recovery
    # ------------------------------------------------------------------

    def _invalidate_bootstrap_cache(self) -> None:
        self._sys_info = None
        self._sys_config = None
        self._sys_diag = None
        self._dc_pid_cache.clear()
        self._step_config_cache.clear()
        for topic in ("sys_info_rsp", "sys_config_rsp", "sys_diag_rsp"):
            self._latest_ws_messages.pop(topic, None)
        for topic in list(self._latest_ws_messages.keys()):
            if topic == "dc_pid_rsp" or topic.startswith("dc_pid_rsp:") or \
               topic == "step_config_rsp" or topic.startswith("step_config_rsp:"):
                self._latest_ws_messages.pop(topic, None)

    def _request_bootstrap(self) -> None:
        self._invalidate_bootstrap_cache()
        self.send_wire_command("sys_info_req", {"target": 0xFF})
        self.send_wire_command("sys_config_req", {"target": 0xFF})
        self.send_wire_command("sys_diag_req", {"target": 0xFF})
        for motor_number in range(1, 5):
            self.send_wire_command("dc_pid_req", {"motorNumber": motor_number, "loopType": 0})
            self.send_wire_command("dc_pid_req", {"motorNumber": motor_number, "loopType": 1})
        for stepper_number in range(1, 5):
            self.send_wire_command("step_config_req", {"stepperNumber": stepper_number})

    def _wrap(self, topic: str, data: dict) -> dict:
        self._mag_cal_controller.observe(topic, data)
        message = {"topic": topic, "data": data, "ts": time.time()}
        self._latest_ws_messages[topic] = message
        return message

    def get_cached_ws_messages(self) -> List[dict]:
        ordered_topics = [
            "sys_info_rsp",
            "sys_config_rsp",
            "sys_diag_rsp",
            "sys_power",
            "sys_state",
            "io_input_state",
            "io_output_state",
            "dc_state_all",
            "step_state_all",
            "servo_state_all",
            "sensor_imu",
            "sensor_kinematics",
            "sensor_ultrasonic_all",
            "sensor_mag_cal_status",
        ]
        messages: List[dict] = []
        for topic in ordered_topics:
            message = self._latest_ws_messages.get(topic)
            if message is not None:
                messages.append(message)
        for key in sorted(self._latest_ws_messages.keys()):
            if key.startswith("dc_pid_rsp:") or key.startswith("step_config_rsp:"):
                messages.append(self._latest_ws_messages[key])
        return messages

    def poll_runtime_queries(self) -> None:
        if self._sys_info is None:
            self.send_wire_command("sys_info_req", {"target": 0xFF})
        if self._sys_config is None:
            self.send_wire_command("sys_config_req", {"target": 0xFF})
        self.send_wire_command("sys_diag_req", {"target": 0xFF})

    # ------------------------------------------------------------------
    # Decoders
    # ------------------------------------------------------------------

    def _decode_sys_state(self, tlv_data: bytes) -> DecodedMessages:
        decoded = _decode_fixed(PayloadSysState, tlv_data)
        if decoded is None:
            return None

        uptime_ms = int(decoded.get("uptimeMs", 0))
        if self._last_uptime_ms is None or uptime_ms < self._last_uptime_ms:
            self._request_bootstrap()
        self._last_uptime_ms = uptime_ms

        self._sys_state = decoded
        return [self._wrap("sys_state", decoded)]

    def _decode_sys_power(self, tlv_data: bytes) -> DecodedMessages:
        decoded = _decode_fixed(PayloadSysPower, tlv_data)
        if decoded is None:
            return None
        self._sys_power = decoded
        return [self._wrap("sys_power", decoded)]

    def _decode_sys_info_rsp(self, tlv_data: bytes) -> DecodedMessages:
        decoded = _decode_fixed(PayloadSysInfoRsp, tlv_data)
        if decoded is None:
            return None
        self._sys_info = decoded
        return [self._wrap("sys_info_rsp", decoded)]

    def _decode_sys_config_rsp(self, tlv_data: bytes) -> DecodedMessages:
        decoded = _decode_fixed(PayloadSysConfigRsp, tlv_data)
        if decoded is None:
            return None
        self._sys_config = decoded
        return [self._wrap("sys_config_rsp", decoded)]

    def _decode_sys_diag_rsp(self, tlv_data: bytes) -> DecodedMessages:
        decoded = _decode_fixed(PayloadSysDiagRsp, tlv_data)
        if decoded is None:
            return None
        self._sys_diag = decoded
        return [self._wrap("sys_diag_rsp", decoded)]

    def _decode_dc_state_all(self, tlv_data: bytes) -> DecodedMessages:
        if len(tlv_data) != ctypes.sizeof(PayloadDCStateAll):
            return None

        payload = PayloadDCStateAll.from_buffer_copy(tlv_data)
        frame_index = self._dc_frame_counter
        self._dc_frame_counter += 1

        motors = []
        for idx, motor in enumerate(payload.motors):
            motors.append({
                "motorNumber": idx + 1,
                "frameIndex": frame_index,
                "mode": int(motor.mode),
                "faultFlags": int(motor.faultFlags),
                "position": int(motor.position),
                "velocity": int(motor.velocity),
                "targetPos": int(motor.targetPos),
                "targetVel": int(motor.targetVel),
                "pwmOutput": int(motor.pwmOutput),
                "currentMa": int(motor.currentMa),
                "timestamp": int(payload.timestamp),
            })
        return [self._wrap("dc_state_all", {"motors": motors})]

    def _decode_dc_pid_rsp(self, tlv_data: bytes) -> DecodedMessages:
        decoded = _decode_fixed(PayloadDCPidRsp, tlv_data)
        if decoded is None:
            return None
        motor_id = int(decoded["motorId"])
        loop_type = int(decoded["loopType"])
        self._dc_pid_cache[(motor_id, loop_type)] = decoded
        message = self._wrap("dc_pid_rsp", {
            "motorNumber": motor_id + 1,
            "loopType": loop_type,
            "kp": float(decoded["kp"]),
            "ki": float(decoded["ki"]),
            "kd": float(decoded["kd"]),
            "maxOutput": float(decoded["maxOutput"]),
            "maxIntegral": float(decoded["maxIntegral"]),
        })
        self._latest_ws_messages[f"dc_pid_rsp:{motor_id}:{loop_type}"] = message
        return [message]

    def _decode_step_state_all(self, tlv_data: bytes) -> DecodedMessages:
        if len(tlv_data) != ctypes.sizeof(PayloadStepStateAll):
            return None

        payload = PayloadStepStateAll.from_buffer_copy(tlv_data)
        steppers = []
        for idx, stepper in enumerate(payload.steppers):
            steppers.append({
                "stepperNumber": idx + 1,
                "enabled": int(stepper.enabled),
                "motionState": int(stepper.motionState),
                "limitFlags": int(stepper.limitFlags),
                "count": int(stepper.count),
                "targetCount": int(stepper.targetCount),
                "currentSpeed": int(stepper.currentSpeed),
                "timestamp": int(payload.timestamp),
            })
        return [self._wrap("step_state_all", {"steppers": steppers})]

    def _decode_step_config_rsp(self, tlv_data: bytes) -> DecodedMessages:
        decoded = _decode_fixed(PayloadStepConfigRsp, tlv_data)
        if decoded is None:
            return None
        stepper_id = int(decoded["stepperId"])
        self._step_config_cache[stepper_id] = decoded
        message = self._wrap("step_config_rsp", {
            "stepperNumber": stepper_id + 1,
            "maxVelocity": int(decoded["maxVelocity"]),
            "acceleration": int(decoded["acceleration"]),
        })
        self._latest_ws_messages[f"step_config_rsp:{stepper_id}"] = message
        return [message]

    def _decode_servo_state_all(self, tlv_data: bytes) -> DecodedMessages:
        if len(tlv_data) != ctypes.sizeof(PayloadServoStateAll):
            return None
        payload = PayloadServoStateAll.from_buffer_copy(tlv_data)
        channels = []
        for idx, pulse_us in enumerate(payload.pulseUs):
            channels.append({
                "channelNumber": idx + 1,
                "enabled": bool(payload.enabledMask & (1 << idx)),
                "pulseUs": int(pulse_us),
            })
        return [self._wrap("servo_state_all", {
            "pca9685Connected": int(payload.pca9685Connected),
            "pca9685Error": int(payload.pca9685Error),
            "channels": channels,
            "timestamp": int(payload.timestamp),
        })]

    def _decode_sensor_imu(self, tlv_data: bytes) -> DecodedMessages:
        decoded = _decode_fixed(PayloadSensorIMU, tlv_data)
        if decoded is None:
            return None
        return [self._wrap("sensor_imu", decoded)]

    def _decode_sensor_kinematics(self, tlv_data: bytes) -> DecodedMessages:
        decoded = _decode_fixed(PayloadSensorKinematics, tlv_data)
        if decoded is None:
            return None
        return [self._wrap("sensor_kinematics", decoded)]

    def _decode_sensor_ultrasonic_all(self, tlv_data: bytes) -> DecodedMessages:
        if len(tlv_data) != ctypes.sizeof(PayloadSensorUltrasonicAll):
            return None
        payload = PayloadSensorUltrasonicAll.from_buffer_copy(tlv_data)
        return [self._wrap("sensor_ultrasonic_all", {
            "configuredCount": int(payload.configuredCount),
            "sensors": [
                {
                    "status": int(payload.sensors[idx].status),
                    "distanceMm": int(payload.sensors[idx].distanceMm),
                }
                for idx in range(TLV_MAX_ULTRASONICS)
            ],
            "timestamp": int(payload.timestamp),
        })]

    def _decode_mag_cal_status(self, tlv_data: bytes) -> DecodedMessages:
        decoded = _decode_fixed(PayloadMagCalStatus, tlv_data)
        if decoded is None:
            return None
        return [self._wrap("sensor_mag_cal_status", decoded)]

    def _decode_io_input_state(self, tlv_data: bytes) -> DecodedMessages:
        decoded = _decode_fixed(PayloadIOInputState, tlv_data)
        if decoded is None:
            return None
        self._io_input = decoded
        return [self._wrap("io_input_state", decoded)]

    def _decode_io_output_state(self, tlv_data: bytes) -> DecodedMessages:
        fixed_size = ctypes.sizeof(PayloadIOOutputState)
        if len(tlv_data) < fixed_size:
            return None

        fixed = PayloadIOOutputState.from_buffer_copy(tlv_data[:fixed_size])
        neo_count = int(fixed.neoPixelCount)
        extra = tlv_data[fixed_size:]
        neo_pixels = []
        for idx in range(0, min(len(extra), neo_count * 3), 3):
            if idx + 2 >= len(extra):
                break
            neo_pixels.append({"r": extra[idx], "g": extra[idx + 1], "b": extra[idx + 2]})

        self._io_output = {
            "ledBrightness": list(fixed.ledBrightness),
            "neoPixelCount": neo_count,
            "timestamp": int(fixed.timestamp),
            "neoPixels": neo_pixels,
        }
        return [self._wrap("io_output_state", self._io_output)]

    # ------------------------------------------------------------------
    # Public decode entry points
    # ------------------------------------------------------------------

    def decode_incoming(self, tlv_type: int, tlv_data: bytes) -> Union[None, dict, List[dict]]:
        decode_map = {
            SYS_STATE: self._decode_sys_state,
            SYS_INFO_RSP: self._decode_sys_info_rsp,
            SYS_CONFIG_RSP: self._decode_sys_config_rsp,
            SYS_POWER: self._decode_sys_power,
            SYS_DIAG_RSP: self._decode_sys_diag_rsp,
            DC_STATE_ALL: self._decode_dc_state_all,
            DC_PID_RSP: self._decode_dc_pid_rsp,
            STEP_STATE_ALL: self._decode_step_state_all,
            STEP_CONFIG_RSP: self._decode_step_config_rsp,
            SERVO_STATE_ALL: self._decode_servo_state_all,
            SENSOR_IMU: self._decode_sensor_imu,
            SENSOR_KINEMATICS: self._decode_sensor_kinematics,
            SENSOR_ULTRASONIC_ALL: self._decode_sensor_ultrasonic_all,
            SENSOR_MAG_CAL_STATUS: self._decode_mag_cal_status,
            IO_INPUT_STATE: self._decode_io_input_state,
            IO_OUTPUT_STATE: self._decode_io_output_state,
        }
        decode_fn = decode_map.get(tlv_type)
        if decode_fn is None:
            print(f"[Router] Unknown TLV type: {tlv_type:#04x}")
            return None

        try:
            messages = decode_fn(tlv_data)
        except Exception as exc:
            print(f"[Router] Decode error for TLV {tlv_type:#04x}: {exc}")
            return None

        if messages is None:
            print(f"[Router] Size mismatch for TLV {tlv_type:#04x}: got {len(tlv_data)} bytes")
            return None
        if len(messages) == 1:
            return messages[0]
        return messages

    def handle_incoming(self, tlv_type: int, tlv_data: bytes):
        decoded = self.decode_incoming(tlv_type, tlv_data)
        if decoded is None:
            return
        messages = decoded if isinstance(decoded, list) else [decoded]
        if self.ws_manager.connections:
            for message in messages:
                asyncio.create_task(self.ws_manager.broadcast(message))

    async def flush_to_ws(self, messages: list):
        for msg in messages:
            await self.ws_manager.broadcast(msg)

    # ------------------------------------------------------------------
    # Encoders
    # ------------------------------------------------------------------

    def _encode_sys_cmd(self, data: dict) -> Optional[ctypes.Structure]:
        payload = PayloadSysCmd()
        payload.command = int(data["command"])
        return payload

    def _encode_sys_info_req(self, data: dict) -> Optional[ctypes.Structure]:
        payload = PayloadSysInfoReq()
        payload.target = int(data.get("target", 0xFF))
        return payload

    def _encode_sys_config_req(self, data: dict) -> Optional[ctypes.Structure]:
        payload = PayloadSysConfigReq()
        payload.target = int(data.get("target", 0xFF))
        return payload

    def _encode_sys_diag_req(self, data: dict) -> Optional[ctypes.Structure]:
        payload = PayloadSysDiagReq()
        payload.target = int(data.get("target", 0xFF))
        return payload

    def _encode_sys_config_set(self, data: dict) -> Optional[ctypes.Structure]:
        payload = PayloadSysConfigSet()
        payload.motorDirMask = int(data.get("motorDirMask", 0))
        payload.motorDirChangeMask = int(data.get("motorDirChangeMask", 0))
        payload.neoPixelCount = int(data.get("neoPixelCount", 0))
        payload.configuredSensorMask = int(data.get("configuredSensorMask", data.get("attachedSensors", 0xFF)))
        payload.heartbeatTimeoutMs = int(data.get("heartbeatTimeoutMs", 0))
        return payload

    def _encode_sys_odom_reset(self, data: dict) -> Optional[ctypes.Structure]:
        payload = PayloadSysOdomReset()
        payload.flags = int(data.get("flags", 0))
        return payload

    def _encode_dc_enable(self, data: dict) -> Optional[ctypes.Structure]:
        motor_number = int(data["motorNumber"])
        if not 1 <= motor_number <= TLV_MAX_DC_MOTORS:
            return None
        payload = PayloadDCEnable()
        payload.motorId = motor_number - 1
        payload.mode = int(data.get("mode", 0))
        return payload

    def _encode_dc_set_position(self, data: dict) -> Optional[ctypes.Structure]:
        motor_number = int(data["motorNumber"])
        if not 1 <= motor_number <= TLV_MAX_DC_MOTORS:
            return None
        payload = PayloadDCSetPosition()
        payload.motorId = motor_number - 1
        payload.targetTicks = int(data["targetTicks"])
        payload.maxVelTicks = int(data.get("maxVelTicks", 0))
        return payload

    def _encode_dc_set_velocity(self, data: dict) -> Optional[ctypes.Structure]:
        motor_number = int(data["motorNumber"])
        if not 1 <= motor_number <= TLV_MAX_DC_MOTORS:
            return None
        payload = PayloadDCSetVelocity()
        payload.motorId = motor_number - 1
        payload.targetTicks = int(data["targetTicks"])
        return payload

    def _encode_dc_set_pwm(self, data: dict) -> Optional[ctypes.Structure]:
        motor_number = int(data["motorNumber"])
        if not 1 <= motor_number <= TLV_MAX_DC_MOTORS:
            return None
        payload = PayloadDCSetPWM()
        payload.motorId = motor_number - 1
        payload.pwm = _clamp(int(data["pwm"]), -255, 255)
        return payload

    def _encode_dc_pid_req(self, data: dict) -> Optional[ctypes.Structure]:
        motor_number = int(data["motorNumber"])
        if not 1 <= motor_number <= TLV_MAX_DC_MOTORS:
            return None
        payload = PayloadDCPidReq()
        payload.motorId = motor_number - 1
        payload.loopType = int(data.get("loopType", 1))
        return payload

    def _encode_dc_pid_set(self, data: dict) -> Optional[ctypes.Structure]:
        motor_number = int(data["motorNumber"])
        if not 1 <= motor_number <= TLV_MAX_DC_MOTORS:
            return None
        payload = PayloadDCPidSet()
        payload.motorId = motor_number - 1
        payload.loopType = int(data.get("loopType", 1))
        payload.kp = float(data.get("kp", 0.0))
        payload.ki = float(data.get("ki", 0.0))
        payload.kd = float(data.get("kd", 0.0))
        payload.maxOutput = float(data.get("maxOutput", 255.0))
        payload.maxIntegral = float(data.get("maxIntegral", 255.0))
        return payload

    def _encode_step_enable(self, data: dict) -> Optional[ctypes.Structure]:
        stepper_number = int(data["stepperNumber"])
        if not 1 <= stepper_number <= TLV_MAX_STEPPERS:
            return None
        payload = PayloadStepEnable()
        payload.stepperId = stepper_number - 1
        payload.enable = int(data.get("enable", 0))
        return payload

    def _encode_step_config_req(self, data: dict) -> Optional[ctypes.Structure]:
        stepper_number = int(data["stepperNumber"])
        if not 1 <= stepper_number <= TLV_MAX_STEPPERS:
            return None
        payload = PayloadStepConfigReq()
        payload.stepperId = stepper_number - 1
        return payload

    def _encode_step_config_set(self, data: dict) -> Optional[ctypes.Structure]:
        stepper_number = int(data["stepperNumber"])
        if not 1 <= stepper_number <= TLV_MAX_STEPPERS:
            return None
        payload = PayloadStepConfigSet()
        payload.stepperId = stepper_number - 1
        payload.maxVelocity = int(data.get("maxVelocity", 1000))
        payload.acceleration = int(data.get("acceleration", 500))
        return payload

    def _encode_step_move(self, data: dict) -> Optional[ctypes.Structure]:
        stepper_number = int(data["stepperNumber"])
        if not 1 <= stepper_number <= TLV_MAX_STEPPERS:
            return None
        payload = PayloadStepMove()
        payload.stepperId = stepper_number - 1
        payload.moveType = int(data.get("moveType", 0))
        payload.target = int(data["target"])
        return payload

    def _encode_step_home(self, data: dict) -> Optional[ctypes.Structure]:
        stepper_number = int(data["stepperNumber"])
        if not 1 <= stepper_number <= TLV_MAX_STEPPERS:
            return None
        payload = PayloadStepHome()
        payload.stepperId = stepper_number - 1
        payload.direction = int(data.get("direction", -1))
        payload.homeVelocity = int(data.get("homeVelocity", 200))
        payload.backoffSteps = int(data.get("backoffSteps", 100))
        return payload

    def _encode_servo_enable(self, data: dict) -> Optional[ctypes.Structure]:
        channel = int(data["channel"])
        if channel != 255 and not 1 <= channel <= TLV_MAX_SERVO_CHANNELS:
            return None
        payload = PayloadServoEnable()
        payload.channel = 0xFF if channel == 255 else (channel - 1)
        payload.enable = int(data.get("enable", 0))
        return payload

    def _encode_servo_set(self, data: dict) -> Optional[ctypes.Structure]:
        channel = int(data["channel"])
        if not 1 <= channel <= TLV_MAX_SERVO_CHANNELS:
            return None
        payload = PayloadServoSetSingle()
        payload.channel = channel - 1
        payload.count = 1
        payload.pulseUs[0] = _clamp(int(data["pulseUs"]), 500, 2500)
        return payload

    def _encode_set_led(self, data: dict) -> Optional[ctypes.Structure]:
        payload = PayloadSetLED()
        payload.ledId = int(data.get("ledId", 0))
        payload.mode = int(data.get("mode", 0))
        payload.brightness = _clamp(int(data.get("brightness", 255)), 0, 255)
        payload.periodMs = int(data.get("periodMs", 1000))
        payload.dutyCycle = _clamp(int(data.get("dutyCycle", 500)), 0, 1000)
        return payload

    def _encode_set_neopixel(self, data: dict) -> Optional[ctypes.Structure]:
        payload = PayloadSetNeoPixel()
        payload.index = int(data.get("index", 0))
        payload.red = _clamp(int(data.get("red", 0)), 0, 255)
        payload.green = _clamp(int(data.get("green", 0)), 0, 255)
        payload.blue = _clamp(int(data.get("blue", 0)), 0, 255)
        return payload

    def _encode_mag_cal_cmd(self, data: dict) -> Optional[ctypes.Structure]:
        payload = PayloadMagCalCmd()
        payload.command = int(data["command"])
        payload.offsetX = float(data.get("offsetX", 0.0))
        payload.offsetY = float(data.get("offsetY", 0.0))
        payload.offsetZ = float(data.get("offsetZ", 0.0))
        matrix = data.get("softIronMatrix", IDENTITY_3X3)
        if len(matrix) != 9:
            return None
        for idx, value in enumerate(matrix):
            payload.softIronMatrix[idx] = float(value)
        return payload

    # ------------------------------------------------------------------
    # Outgoing public entry point
    # ------------------------------------------------------------------

    def handle_outgoing(self, cmd: str, data: Dict[str, Any]) -> Optional[Tuple[int, ctypes.Structure]]:
        registry: Dict[str, Tuple[int, Callable[[dict], Optional[ctypes.Structure]]]] = {
            "sys_cmd": (SYS_CMD, self._encode_sys_cmd),
            "sys_info_req": (SYS_INFO_REQ, self._encode_sys_info_req),
            "sys_config_req": (SYS_CONFIG_REQ, self._encode_sys_config_req),
            "sys_diag_req": (SYS_DIAG_REQ, self._encode_sys_diag_req),
            "sys_config_set": (SYS_CONFIG_SET, self._encode_sys_config_set),
            "sys_odom_reset": (SYS_ODOM_RESET, self._encode_sys_odom_reset),
            "dc_enable": (DC_ENABLE, self._encode_dc_enable),
            "dc_set_position": (DC_SET_POSITION, self._encode_dc_set_position),
            "dc_set_velocity": (DC_SET_VELOCITY, self._encode_dc_set_velocity),
            "dc_set_pwm": (DC_SET_PWM, self._encode_dc_set_pwm),
            "dc_pid_req": (DC_PID_REQ, self._encode_dc_pid_req),
            "dc_pid_set": (DC_PID_SET, self._encode_dc_pid_set),
            "step_enable": (STEP_ENABLE, self._encode_step_enable),
            "step_config_req": (STEP_CONFIG_REQ, self._encode_step_config_req),
            "step_config_set": (STEP_CONFIG_SET, self._encode_step_config_set),
            "step_move": (STEP_MOVE, self._encode_step_move),
            "step_home": (STEP_HOME, self._encode_step_home),
            "servo_enable": (SERVO_ENABLE, self._encode_servo_enable),
            "servo_set": (SERVO_SET, self._encode_servo_set),
            "io_set_led": (IO_SET_LED, self._encode_set_led),
            "io_set_neopixel": (IO_SET_NEOPIXEL, self._encode_set_neopixel),
            "sensor_mag_cal_cmd": (SENSOR_MAG_CAL_CMD, self._encode_mag_cal_cmd),
        }

        if cmd not in registry:
            print(f"[Router] Unknown command: {cmd!r}")
            return None

        tlv_type, encode_fn = registry[cmd]
        try:
            payload = encode_fn(data)
        except Exception as exc:
            print(f"[Router] Encode error for {cmd!r}: {exc}")
            return None

        if payload is None:
            print(f"[Router] Rejected command {cmd!r}: validation failed (data={data})")
            return None

        return (tlv_type, payload)
