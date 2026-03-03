#!/usr/bin/env python3
"""
test_uart_arduino.py - UART Link-Quality Test

Measures communication quality in both directions:
  Arduino → RPi : decode_errors / total_frames  (CRC/framing failures)
  RPi → Arduino : uartRxErrors field in SYS_STATUS (firmware counter)
  Link quality  : EXCELLENT <0.1% | GOOD <1% | MARGINAL <5% | FAIL ≥5%

Usage:
    python3 test_uart_arduino.py [--port /dev/ttyAMA0] [--baud 1000000]

Baud sweep (run once at each rate):
    python3 test_uart_arduino.py --baud 115200
    python3 test_uart_arduino.py --baud 230400
    python3 test_uart_arduino.py --baud 500000
    python3 test_uart_arduino.py --baud 1000000
    python3 test_uart_arduino.py --baud 2000000

Stats auto-print every 5 s. Commands: s=stats  q=quit

Hardware:
    Arduino Serial2 (pin 16 TX, pin 17 RX) → level shifter → RPi /dev/ttyAMA0
    Arduino USB Serial0 shows Arduino-side debug (open in a second terminal)

Requirements:
    pip3 install pyserial
"""

import os
import sys
import time
import argparse
import ctypes
import select
from typing import Optional

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip3 install pyserial")
    sys.exit(1)

# Add the src directory to path so we can import the codec and type defs
sys.path.insert(0, '../src')

from tlvcodec.src.encoder import Encoder
from tlvcodec.src.decoder import Decoder, DecodeErrorCode
from TLV_TypeDefs import SYS_HEARTBEAT, SYS_STATUS, TLV_NAMES

# ============================================================================
# CONFIGURATION
# ============================================================================

DEFAULT_PORT       = '/dev/ttyAMA0'
DEFAULT_BAUD       = 1000000         # Must match RPI_BAUD_RATE in config.h
DEVICE_ID          = 0x01
HEARTBEAT_INTERVAL = 0.2             # seconds; Arduino liveness timeout = 500 ms
STATS_INTERVAL     = 5.0             # auto-print quality report every N seconds
ENABLE_CRC         = True

# ============================================================================
# PAYLOAD STRUCTS  (must match TLV_Payloads.h exactly — same field order/types)
# ============================================================================

class PayloadHeartbeat(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp", ctypes.c_uint32),   # RPi milliseconds since boot
        ("flags",     ctypes.c_uint8),    # Reserved — set 0
    ]
# 5 bytes

class PayloadSystemStatus(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("firmwareMajor",       ctypes.c_uint8),
        ("firmwareMinor",       ctypes.c_uint8),
        ("firmwarePatch",       ctypes.c_uint8),
        ("state",               ctypes.c_uint8),
        ("uptimeMs",            ctypes.c_uint32),
        ("lastRxMs",            ctypes.c_uint32),
        ("lastCmdMs",           ctypes.c_uint32),
        ("batteryMv",           ctypes.c_uint16),
        ("rail5vMv",            ctypes.c_uint16),
        ("errorFlags",          ctypes.c_uint8),
        ("attachedSensors",     ctypes.c_uint8),
        ("freeSram",            ctypes.c_uint16),
        ("loopTimeAvgUs",       ctypes.c_uint16),
        ("loopTimeMaxUs",       ctypes.c_uint16),
        ("uartRxErrors",        ctypes.c_uint16),
        ("wheelDiameterMm",     ctypes.c_float),
        ("wheelBaseMm",         ctypes.c_float),
        ("motorDirMask",        ctypes.c_uint8),
        ("neoPixelCount",       ctypes.c_uint8),
        ("heartbeatTimeoutMs",  ctypes.c_uint16),
        ("limitSwitchMask",     ctypes.c_uint16),
        ("stepperHomeLimitGpio",ctypes.c_uint8 * 4),
    ]
# 48 bytes

assert ctypes.sizeof(PayloadSystemStatus) == 48, \
    f"PayloadSystemStatus size wrong: {ctypes.sizeof(PayloadSystemStatus)}"

SYS_STATE_NAMES = {0: "INIT", 1: "IDLE", 2: "RUNNING", 3: "ERROR", 4: "ESTOP"}

# ============================================================================
# UART TEST CLASS
# ============================================================================

class UARTTest:
    def __init__(self, port: str, baudrate: int):
        self.port     = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.encoder  = Encoder(deviceId=DEVICE_ID, bufferSize=4096, crc=ENABLE_CRC)
        self.decoder  = Decoder(callback=self._decode_callback, crc=ENABLE_CRC)

        self.running             = True
        self.last_heartbeat_time = 0.0
        self.last_stats_time     = 0.0

        self.stats = {
            'heartbeats_sent':                    0,
            'frames_received':                    0,
            'decode_errors':                      0,
            'decode_errors_by_type':              {},
            'arduino_uart_rx_errors_baseline':    None,
            'arduino_uart_rx_errors_current':     0,
        }

    # ---- Connection ----

    def connect(self) -> bool:
        try:
            print(f"[UART] Opening {self.port} @ {self.baudrate} baud...")
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.1
            )
            print("[UART] Connected")
            return True
        except serial.SerialException as e:
            print(f"[UART] ERROR: {e}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[UART] Disconnected")

    # ---- TX ----

    def send_heartbeat(self):
        p = PayloadHeartbeat()
        p.timestamp = int(time.time() * 1000) & 0xFFFFFFFF
        p.flags = 0
        self.encoder.reset()
        self.encoder.addPacket(SYS_HEARTBEAT, ctypes.sizeof(p), p)
        length, buffer = self.encoder.wrapupBuffer()
        self.ser.write(buffer[:length])
        self.stats['heartbeats_sent'] += 1

    # ---- RX ----

    def _process_incoming(self):
        if self.ser.in_waiting > 0:
            self.decoder.decode(self.ser.read(self.ser.in_waiting))

    def _decode_callback(self, error_code, frame_header, tlv_list):
        if error_code != DecodeErrorCode.NoError:
            self.stats['decode_errors'] += 1
            name = error_code.name if hasattr(error_code, 'name') else str(error_code)
            self.stats['decode_errors_by_type'][name] = \
                self.stats['decode_errors_by_type'].get(name, 0) + 1
            return

        self.stats['frames_received'] += 1

        for tlv_type, tlv_len, tlv_data in tlv_list:
            if tlv_type == SYS_STATUS:
                self._handle_sys_status(tlv_len, tlv_data)

    def _handle_sys_status(self, tlv_len: int, tlv_data: bytes):
        expected = ctypes.sizeof(PayloadSystemStatus)
        if tlv_len != expected:
            print(f"  [SYS_STATUS] size mismatch: expected {expected}, got {tlv_len}")
            return
        s = PayloadSystemStatus.from_buffer_copy(tlv_data)

        if self.stats['arduino_uart_rx_errors_baseline'] is None:
            self.stats['arduino_uart_rx_errors_baseline'] = s.uartRxErrors
            if s.uartRxErrors > 0:
                print(f"  [SYS_STATUS] baseline uartRxErrors={s.uartRxErrors} "
                      f"(pre-connection noise — delta will track errors from now on)")
        self.stats['arduino_uart_rx_errors_current'] = s.uartRxErrors

    # ---- Quality report ----

    def _quality_label(self, error_rate_pct: float) -> str:
        if error_rate_pct == 0.0:
            return "EXCELLENT (0%)"
        elif error_rate_pct < 0.1:
            return f"EXCELLENT ({error_rate_pct:.3f}%)"
        elif error_rate_pct < 1.0:
            return f"GOOD      ({error_rate_pct:.2f}%)"
        elif error_rate_pct < 5.0:
            return f"MARGINAL  ({error_rate_pct:.1f}%)"
        else:
            return f"FAIL      ({error_rate_pct:.1f}%)"

    def print_stats(self):
        good     = self.stats['frames_received']
        errors   = self.stats['decode_errors']
        total    = good + errors
        baseline = self.stats['arduino_uart_rx_errors_baseline'] or 0
        current  = self.stats['arduino_uart_rx_errors_current']
        delta    = current - baseline
        hb_sent  = self.stats['heartbeats_sent']

        a2r_rate = (errors / total  * 100) if total    > 0 else 0.0
        r2a_rate = (delta  / hb_sent * 100) if hb_sent > 0 else 0.0

        print()
        print("=" * 58)
        print(f"  LINK QUALITY REPORT  —  {self.baudrate} baud")
        print("=" * 58)
        print(f"  Arduino → RPi  (Python decode errors / frames):")
        print(f"    Good frames : {good}")
        print(f"    Errors      : {errors}", end="")
        if self.stats['decode_errors_by_type']:
            breakdown = "  " + "  ".join(f"{k}={v}"
                         for k, v in self.stats['decode_errors_by_type'].items())
            print(breakdown, end="")
        print()
        print(f"    Quality     : {self._quality_label(a2r_rate)}")
        print()
        print(f"  RPi → Arduino  (firmware uartRxErrors delta since connect):")
        print(f"    Heartbeats sent     : {hb_sent}")
        print(f"    Errors at connect   : {baseline}  (pre-connection noise, excluded)")
        print(f"    New errors (delta)  : {delta}")
        print(f"    Quality             : {self._quality_label(r2a_rate)}")
        if baseline > 0 and delta == 0:
            print(f"    NOTE: link clean since connect — boot noise on RX pin likely")
        print("=" * 58)
        print()

    # ---- Main loop ----

    def run(self):
        if not self.connect():
            return

        print()
        print("=" * 58)
        print(f"  UART Link-Quality Test  —  {self.baudrate} baud")
        print("=" * 58)
        print("Quality report auto-prints every 5 s.")
        print("Commands:  s = print report now   q = quit")
        print("=" * 58)
        print()

        try:
            while self.running:
                now = time.time()

                if now - self.last_heartbeat_time >= HEARTBEAT_INTERVAL:
                    self.send_heartbeat()
                    self.last_heartbeat_time = now

                if now - self.last_stats_time >= STATS_INTERVAL:
                    self.print_stats()
                    self.last_stats_time = now

                self._process_incoming()

                if select.select([sys.stdin], [], [], 0.0)[0]:
                    cmd = sys.stdin.readline().strip()
                    if cmd == 's':
                        self.print_stats()
                    elif cmd == 'q':
                        print("\n[Test] Exiting...")
                        self.running = False
                    elif cmd:
                        print(f"Unknown command: '{cmd}' (s=stats  q=quit)")

                time.sleep(0.01)  # 100 Hz poll loop

        except KeyboardInterrupt:
            print("\n[Test] Interrupted by user")
        finally:
            self.print_stats()
            self.disconnect()

# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='UART Link-Quality Test (pairs with test_uart_tlv.ino)')
    parser.add_argument('--port', default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD,
                        help=f'Baud rate (default: {DEFAULT_BAUD})')
    args = parser.parse_args()

    UARTTest(port=args.port, baudrate=args.baud).run()

if __name__ == '__main__':
    main()
