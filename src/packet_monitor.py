"""Receive MCU packets without sending commands (Windows/Linux).

Usage: uv run python src/packet_monitor.py --port COM3
"""

import argparse
import sys
import time

import serial
from mcu_protocol import decode_telemetry, extract_packets, format_telemetry


def main() -> int:
    parser = argparse.ArgumentParser(description="Receive MCU packets and print HEX; no commands are sent.")
    parser.add_argument("--port", "-p", required=True, help="Serial port, e.g. COM3")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--decode", action="store_true", help="decode 0x85/86/87 feedback; still receive-only")
    args = parser.parse_args()
    buffer = bytearray()

    try:
        with serial.Serial(args.port, args.baud, timeout=0.1) as port:
            print(f"Monitoring {args.port} at {args.baud} baud. Ctrl+C to stop.", flush=True)
            while True:
                buffer.extend(port.read(min(4096, max(1, port.in_waiting))))
                for packet in extract_packets(buffer):
                    if args.decode:
                        try:
                            data = decode_telemetry(packet)
                        except ValueError as exc:
                            print(f"RX ignored: {exc}", flush=True)
                            continue
                        if data:
                            print(f"[{time.strftime('%H:%M:%S')}] {format_telemetry(data)}", flush=True)
                            continue
                    print(
                        f"[{time.strftime('%H:%M:%S')}] "
                        f"type=0x{packet[3]:02X} len={packet[2]} "
                        f"raw={packet.hex(' ').upper()}",
                        flush=True,
                    )
    except KeyboardInterrupt:
        print("\nMonitor stopped.")
    except (serial.SerialException, OSError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
