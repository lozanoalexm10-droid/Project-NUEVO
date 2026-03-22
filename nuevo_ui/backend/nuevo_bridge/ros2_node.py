"""
NuevoBridgeNode — ROS2 integration for nuevo_bridge.

This node lives in the same process as the FastAPI / WebSocket bridge and
shares the SerialManager instance.  It is only instantiated when the
NUEVO_ROS2=1 environment variable is set.

Thread model
────────────
  Serial reader thread  →  publish_decoded()  →  rclpy publisher.publish()
                                                  (rcl layer is thread-safe)
  rclpy spin thread     →  subscriber callbacks  →  SerialManager.send()
                                                  (protected by _write_lock)
  asyncio event loop    →  ws commands / heartbeat → SerialManager.send()
                                                  (also protected by _write_lock)

Topics published
────────────────
  Standard :  /odom  /imu/data  /imu/mag
  Custom   :  /nuevo/voltage  /nuevo/sys_status  /nuevo/dc_status
              /nuevo/step_status  /nuevo/servo_status  /nuevo/io_status
              /nuevo/mag_cal

Topics subscribed
─────────────────
  /nuevo/cmd_vel       (nuevo_msgs/MotorVelocities)
  /nuevo/step_cmd      (nuevo_msgs/StepCommand)
  /nuevo/servo_cmd     (nuevo_msgs/ServoCommand)
  /nuevo/sys_cmd       (nuevo_msgs/SysCommand)
  /nuevo/mag_cal_cmd   (nuevo_msgs/MagCalCmd)
"""
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from nav_msgs.msg import Odometry

from nuevo_msgs.msg import (
    DCStatusAll,
    IOStatus,
    MagCalCmd,
    MagCalStatus,
    MotorVelocities,
    ServoCommand,
    ServoStatusAll,
    StepCommand,
    StepStatusAll,
    SysCommand,
    SystemStatus,
    Voltage,
)

from . import ros2_converters as conv


class NuevoBridgeNode(Node):
    """ROS2 node that bridges Arduino telemetry ↔ ROS2 topics.

    Parameters
    ----------
    serial_manager : SerialManager
        The shared serial manager used to send TLV commands to the Arduino.
    message_router : MessageRouter
        The shared message router used to encode outgoing commands.
    """

    def __init__(self, serial_manager, message_router):
        super().__init__('nuevo_bridge')
        self._serial = serial_manager
        self._router = message_router
        self._clock  = self.get_clock()
        self._sys_state = None
        self._sys_info = None
        self._sys_config = None
        self._sys_power = None
        self._sys_diag = None
        self._io_input = None
        self._io_output = None
        self._dc_pid_cache = {}
        self._step_config_cache = {}

        # ── QoS ───────────────────────────────────────────────────────────────
        # Depth 10 is sufficient for sensor data; no durability/reliability
        # guarantees needed (latest data always wins for control loops).
        _qos = 10

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_imu      = self.create_publisher(Imu,           '/imu/data',           _qos)
        self._pub_mag      = self.create_publisher(MagneticField, '/imu/mag',            _qos)
        self._pub_odom     = self.create_publisher(Odometry,      '/odom',               _qos)
        self._pub_voltage  = self.create_publisher(Voltage,       '/nuevo/voltage',      _qos)
        self._pub_sys      = self.create_publisher(SystemStatus,  '/nuevo/sys_status',   _qos)
        self._pub_dc       = self.create_publisher(DCStatusAll,   '/nuevo/dc_status',    _qos)
        self._pub_step     = self.create_publisher(StepStatusAll, '/nuevo/step_status',  _qos)
        self._pub_servo    = self.create_publisher(ServoStatusAll,'/nuevo/servo_status', _qos)
        self._pub_io       = self.create_publisher(IOStatus,      '/nuevo/io_status',    _qos)
        self._pub_mag_cal  = self.create_publisher(MagCalStatus,  '/nuevo/mag_cal',      _qos)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            MotorVelocities, '/nuevo/cmd_vel', self._on_cmd_vel, _qos)
        self.create_subscription(
            StepCommand, '/nuevo/step_cmd', self._on_step_cmd, _qos)
        self.create_subscription(
            ServoCommand, '/nuevo/servo_cmd', self._on_servo_cmd, _qos)
        self.create_subscription(
            SysCommand, '/nuevo/sys_cmd', self._on_sys_cmd, _qos)
        self.create_subscription(
            MagCalCmd, '/nuevo/mag_cal_cmd', self._on_mag_cal_cmd, _qos)

        # ── Topic → (publisher, converter) dispatch table ─────────────────────
        # Topics with multiple publishers (imu, kinematics) use a callable
        # handler instead of a (pub, conv) tuple.
        self._handlers = {
            'sensor_imu':             self._handle_imu,
            'sensor_kinematics':      self._handle_odom,
            'sys_state':              self._handle_sys_state,
            'sys_info_rsp':           self._handle_sys_info_rsp,
            'sys_config_rsp':         self._handle_sys_config_rsp,
            'sys_power':              self._handle_sys_power,
            'sys_diag_rsp':           self._handle_sys_diag_rsp,
            'dc_state_all':           self._handle_dc_state_all,
            'dc_pid_rsp':             self._handle_dc_pid_rsp,
            'step_state_all':         self._handle_step_state_all,
            'step_config_rsp':        self._handle_step_config_rsp,
            'servo_state_all':        (self._pub_servo,   conv.to_servo_status_all),
            'io_input_state':         self._handle_io_input_state,
            'io_output_state':        self._handle_io_output_state,
            'sensor_mag_cal_status':  (self._pub_mag_cal, conv.to_mag_cal_status),
        }

        self.get_logger().info('NuevoBridgeNode started — publishing to /odom, /imu/*, /nuevo/*')

    # ── Publish dispatcher ────────────────────────────────────────────────────

    def publish_decoded(self, msg_dict: dict) -> None:
        """Publish a decoded TLV dict to the appropriate ROS2 topic(s).

        Called from the blocking serial reader thread.  Safe to call because
        rclpy publisher.publish() is thread-safe at the underlying rcl layer.
        """
        topic   = msg_dict['topic']
        handler = self._handlers.get(topic)
        if handler is None:
            return
        stamp = self._clock.now().to_msg()
        if callable(handler):
            handler(msg_dict['data'], stamp)
        else:
            pub, converter = handler
            pub.publish(converter(msg_dict['data'], stamp))

    def _handle_imu(self, data: dict, stamp) -> None:
        """IMU data → /imu/data AND /imu/mag (two topics from one TLV)."""
        self._pub_imu.publish(conv.to_imu(data, stamp))
        self._pub_mag.publish(conv.to_mag(data, stamp))

    def _handle_odom(self, data: dict, stamp) -> None:
        self._pub_odom.publish(conv.to_odom(data, stamp))

    def _legacy_error_flags(self) -> int:
        if not self._sys_state:
            return 0
        return int(self._sys_state.get('errorFlags', 0))

    def _attached_sensors_mask(self) -> int:
        capability = int((self._sys_info or {}).get('sensorCapabilityMask', 0))
        mask = 0
        if capability & 0x01:
            mask |= 0x01
        if capability & 0x02:
            mask |= 0x04
        return mask

    def _publish_system_status(self, stamp) -> None:
        if self._sys_state is None:
            return
        merged = {
            'firmwareMajor': int((self._sys_info or {}).get('firmwareMajor', 0)),
            'firmwareMinor': int((self._sys_info or {}).get('firmwareMinor', 0)),
            'firmwarePatch': int((self._sys_info or {}).get('firmwarePatch', 0)),
            'state': int(self._sys_state.get('state', 0)),
            'uptimeMs': int(self._sys_state.get('uptimeMs', 0)),
            'lastRxMs': int(self._sys_state.get('lastRxMs', 0)),
            'lastCmdMs': int(self._sys_state.get('lastCmdMs', 0)),
            'batteryMv': int((self._sys_power or {}).get('batteryMv', 0)),
            'rail5vMv': int((self._sys_power or {}).get('rail5vMv', 0)),
            'errorFlags': self._legacy_error_flags(),
            'attachedSensors': self._attached_sensors_mask(),
            'freeSram': int((self._sys_diag or {}).get('freeSram', 0)),
            'loopTimeAvgUs': int((self._sys_diag or {}).get('loopTimeAvgUs', 0)),
            'loopTimeMaxUs': int((self._sys_diag or {}).get('loopTimeMaxUs', 0)),
            'uartRxErrors': int((self._sys_diag or {}).get('uartRxErrors', 0)),
            'motorDirMask': int((self._sys_config or {}).get('motorDirMask', 0)),
            'neoPixelCount': int((self._sys_config or {}).get('neoPixelCount', 0)),
            'heartbeatTimeoutMs': int((self._sys_config or {}).get('heartbeatTimeoutMs', 0)),
            'limitSwitchMask': int((self._sys_info or {}).get('limitSwitchMask', 0)),
            'stepperHomeLimitGpio': list((self._sys_info or {}).get('stepperHomeLimitGpio', [0xFF, 0xFF, 0xFF, 0xFF])),
        }
        self._pub_sys.publish(conv.to_sys_status(merged, stamp))

    def _handle_sys_state(self, data: dict, stamp) -> None:
        self._sys_state = data
        self._publish_system_status(stamp)

    def _handle_sys_info_rsp(self, data: dict, stamp) -> None:
        self._sys_info = data
        self._publish_system_status(stamp)

    def _handle_sys_config_rsp(self, data: dict, stamp) -> None:
        self._sys_config = data
        self._publish_system_status(stamp)

    def _handle_sys_power(self, data: dict, stamp) -> None:
        self._sys_power = data
        self._pub_voltage.publish(conv.to_voltage(data, stamp))
        self._publish_system_status(stamp)

    def _handle_sys_diag_rsp(self, data: dict, stamp) -> None:
        self._sys_diag = data
        self._publish_system_status(stamp)

    def _handle_dc_pid_rsp(self, data: dict, stamp) -> None:
        self._dc_pid_cache[(int(data['motorNumber']), int(data['loopType']))] = data

    def _handle_dc_state_all(self, data: dict, stamp) -> None:
        merged = {'motors': []}
        for motor in data['motors']:
            motor_number = int(motor['motorNumber'])
            pos_pid = self._dc_pid_cache.get((motor_number, 0), {})
            vel_pid = self._dc_pid_cache.get((motor_number, 1), {})
            merged['motors'].append({
                **motor,
                'posKp': float(pos_pid.get('kp', 0.0)),
                'posKi': float(pos_pid.get('ki', 0.0)),
                'posKd': float(pos_pid.get('kd', 0.0)),
                'velKp': float(vel_pid.get('kp', 0.0)),
                'velKi': float(vel_pid.get('ki', 0.0)),
                'velKd': float(vel_pid.get('kd', 0.0)),
            })
        self._pub_dc.publish(conv.to_dc_status_all(merged, stamp))

    def _handle_step_config_rsp(self, data: dict, stamp) -> None:
        self._step_config_cache[int(data['stepperNumber'])] = data

    def _handle_step_state_all(self, data: dict, stamp) -> None:
        merged = {'steppers': []}
        for stepper in data['steppers']:
            stepper_number = int(stepper['stepperNumber'])
            cfg = self._step_config_cache.get(stepper_number, {})
            merged['steppers'].append({
                **stepper,
                'limitHit': int(stepper['limitFlags']),
                'commandedCount': int(stepper['count']),
                'maxSpeed': int(cfg.get('maxVelocity', 0)),
                'acceleration': int(cfg.get('acceleration', 0)),
            })
        self._pub_step.publish(conv.to_step_status_all(merged, stamp))

    def _publish_io_status(self, stamp) -> None:
        if self._io_input is None and self._io_output is None:
            return
        merged = {
          'buttonMask': int((self._io_input or {}).get('buttonMask', 0)),
          'ledBrightness': list((self._io_output or {}).get('ledBrightness', [0, 0, 0, 0, 0])),
          'neoPixels': list((self._io_output or {}).get('neoPixels', [])),
          'timestamp': max(
              int((self._io_input or {}).get('timestamp', 0)),
              int((self._io_output or {}).get('timestamp', 0)),
          ),
        }
        self._pub_io.publish(conv.to_io_status(merged, stamp))

    def _handle_io_input_state(self, data: dict, stamp) -> None:
        self._io_input = data
        self._publish_io_status(stamp)

    def _handle_io_output_state(self, data: dict, stamp) -> None:
        self._io_output = data
        self._publish_io_status(stamp)

    # ── Subscriber callbacks (run in rclpy spin thread) ───────────────────────

    def _send(self, cmd: str, data: dict) -> None:
        """Encode a command and forward to the Arduino via UART."""
        result = self._router.handle_outgoing(cmd, data)
        if result:
            tlv_type, payload = result
            self._serial.send(tlv_type, payload)

    def _on_cmd_vel(self, msg: MotorVelocities) -> None:
        """Forward per-motor velocity / PWM targets to the Arduino.

        The kinematics/trajectory node is responsible for computing individual
        motor commands — the bridge just forwards them verbatim.

        mode[i]:  0 = skip  2 = velocity (ticks/sec)  3 = pwm (-255..+255)
        """
        for i in range(4):
            mode = msg.mode[i]
            if mode == 0:
                continue
            motor_number = i + 1
            val = int(msg.velocity_ticks_per_sec[i])
            if mode == 2:
                self._send('dc_set_velocity', {
                    'motorNumber': motor_number,
                    'targetTicks': val,
                })
            elif mode == 3:
                self._send('dc_set_pwm', {
                    'motorNumber': motor_number,
                    'pwm': val,
                })

    def _on_step_cmd(self, msg: StepCommand) -> None:
        """Dispatch a stepper command by command_type field.

        command_type:
          0 = STEP_MOVE       1 = STEP_HOME
          2 = STEP_ENABLE     3 = STEP_DISABLE    4 = STEP_CONFIG_SET
        """
        n  = msg.stepper_number
        ct = msg.command_type
        if ct == 0:
            self._send('step_move', {
                'stepperNumber': n,
                'moveType':      msg.move_type,
                'target':        msg.target,
            })
        elif ct == 1:
            self._send('step_home', {
                'stepperNumber': n,
                'direction':     msg.direction,
                'homeVelocity':  msg.home_velocity,
                'backoffSteps':  msg.backoff_steps,
            })
        elif ct == 2:
            self._send('step_enable', {'stepperNumber': n, 'enable': 1})
        elif ct == 3:
            self._send('step_enable', {'stepperNumber': n, 'enable': 0})
        elif ct == 4:
            self._send('step_config_set', {
                'stepperNumber': n,
                'maxVelocity':   msg.max_velocity,
                'acceleration':  msg.acceleration,
            })

    def _on_servo_cmd(self, msg: ServoCommand) -> None:
        """Enable/disable a servo channel or set its pulse width.

        If pulse_us > 0: set pulse (implies channel is enabled).
        If pulse_us == 0: send enable/disable command.
        channel == 255 (all channels) is valid for enable/disable only.
        """
        ch = msg.channel
        if msg.pulse_us > 0:
            self._send('servo_set', {'channel': ch, 'pulseUs': msg.pulse_us})
        else:
            self._send('servo_enable', {'channel': ch, 'enable': int(msg.enable)})

    def _on_sys_cmd(self, msg: SysCommand) -> None:
        """Forward system state-machine command (START/STOP/RESET/ESTOP)."""
        self._send('sys_cmd', {'command': msg.command})

    def _on_mag_cal_cmd(self, msg: MagCalCmd) -> None:
        """Forward magnetometer calibration command."""
        self._send('sensor_mag_cal_cmd', {
            'command': msg.command,
            'offsetX': float(msg.offset_x),
            'offsetY': float(msg.offset_y),
            'offsetZ': float(msg.offset_z),
        })

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def spin_in_thread(self) -> threading.Thread:
        """Start rclpy.spin(self) in a daemon thread and return it.

        The thread is started immediately.  Call destroy_node() to stop the
        spin before joining (or let the daemon thread exit on process shutdown).
        """
        t = threading.Thread(
            target=rclpy.spin,
            args=(self,),
            daemon=True,
            name='ros2-spin',
        )
        t.start()
        return t
