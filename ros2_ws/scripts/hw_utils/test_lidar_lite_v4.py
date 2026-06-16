#!/usr/bin/env python3
"""
Simple Raspberry Pi I2C test for Garmin / SparkFun LIDAR-Lite v4.

This uses the same basic register sequence as the Arduino library:
  1. write 0x04 to ACQ_COMMANDS (0x00) to trigger a measurement
  2. poll STATUS (0x01) bit0 until BUSY clears
  3. read FULL_DELAY_LOW/HIGH (0x10/0x11) for the distance in cm

Usage:
    python3 test_lidar_lite_v4.py
    python3 test_lidar_lite_v4.py --bus 1 --address 0x62 --rate 10 --count 20

Requirements:
    pip install smbus2
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    from smbus2 import SMBus
except ImportError:
    print("Error: smbus2 not installed")
    print("Install with: pip install smbus2")
    sys.exit(1)


LIDAR_ADDR_DEFAULT = 0x62

REG_ACQ_COMMANDS = 0x00
REG_STATUS = 0x01
REG_FULL_DELAY_LOW = 0x10

CMD_TAKE_RANGE = 0x04
STATUS_BUSY_MASK = 0x01


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read distance from LIDAR-Lite v4 over I2C")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number (default: 1)")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=LIDAR_ADDR_DEFAULT,
                        help="7-bit I2C address, e.g. 0x62")
    parser.add_argument("--rate", type=float, default=5.0,
                        help="Measurement rate in Hz (default: 5)")
    parser.add_argument("--count", type=int, default=0,
                        help="Number of readings, 0 = run forever (default: 0)")
    parser.add_argument("--busy-timeout-ms", type=float, default=20.0,
                        help="Timeout waiting for BUSY to clear (default: 20 ms)")
    return parser.parse_args()


def ping(bus: SMBus, address: int) -> bool:
    try:
        bus.read_byte(address)
        return True
    except OSError:
        return False


def wait_for_not_busy(bus: SMBus, address: int, timeout_s: float) -> bool:
    start = time.monotonic()
    while True:
        status = bus.read_byte_data(address, REG_STATUS)
        if (status & STATUS_BUSY_MASK) == 0:
            return True
        if (time.monotonic() - start) >= timeout_s:
            return False
        time.sleep(0.001)


def read_distance_cm(bus: SMBus, address: int, busy_timeout_s: float) -> int:
    bus.write_byte_data(address, REG_ACQ_COMMANDS, CMD_TAKE_RANGE)
    if not wait_for_not_busy(bus, address, busy_timeout_s):
        raise TimeoutError("Timed out waiting for lidar BUSY bit to clear")

    data = bus.read_i2c_block_data(address, REG_FULL_DELAY_LOW, 2)
    return data[0] | (data[1] << 8)


def main() -> int:
    args = parse_args()

    if args.rate <= 0:
        print("Error: --rate must be > 0")
        return 1

    period_s = 1.0 / args.rate
    busy_timeout_s = args.busy_timeout_ms / 1000.0

    print("=" * 60)
    print("LIDAR-Lite v4 I2C Distance Test")
    print("=" * 60)
    print(f"Bus:      {args.bus}")
    print(f"Address:  0x{args.address:02X}")
    print(f"Rate:     {args.rate:g} Hz")
    print(f"Count:    {'forever' if args.count == 0 else args.count}")
    print(f"Timeout:  {args.busy_timeout_ms:g} ms")
    print("=" * 60)

    try:
        with SMBus(args.bus) as bus:
            if not ping(bus, args.address):
                print(f"Error: no device ACK at 0x{args.address:02X} on bus {args.bus}")
                return 2

            print("Device acknowledged. Starting reads...\n")

            read_index = 0
            while args.count == 0 or read_index < args.count:
                cycle_start = time.monotonic()
                read_index += 1
                try:
                    distance_cm = read_distance_cm(bus, args.address, busy_timeout_s)
                    distance_mm = distance_cm * 10
                    print(f"[{read_index:04d}] {distance_cm:5d} cm | {distance_mm:6d} mm")
                except TimeoutError as exc:
                    print(f"[{read_index:04d}] timeout: {exc}")
                except OSError as exc:
                    print(f"[{read_index:04d}] I2C error: {exc}")

                elapsed = time.monotonic() - cycle_start
                sleep_time = period_s - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    except FileNotFoundError:
        print(f"Error: /dev/i2c-{args.bus} not found")
        return 3
    except PermissionError:
        print(f"Error: permission denied for /dev/i2c-{args.bus}")
        print("Run with sudo or add the user to the i2c group.")
        return 4

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
