"""Receive MCU packets without sending commands (Windows/Linux).

Usage: uv run python src/packet_monitor.py --port COM3
"""

import argparse
import sys
import time

import serial


HEADER = b"\xaa\x55"
MAX_PACKET_LENGTH = 64


def extract_packets(buffer: bytearray) -> list[bytes]:
    """Consume complete, checksum-valid packets; retain incomplete data."""
    packets = []
    while True:
        start = buffer.find(HEADER)
        if start < 0:
            # Preserve a possible header split between serial reads.
            buffer[:] = buffer[-1:] if buffer[-1:] == HEADER[:1] else b""
            break
        if start:
            del buffer[:start]
        if len(buffer) < 3:
            break

        length = buffer[2]
        if not 1 <= length <= MAX_PACKET_LENGTH:
            del buffer[0]
            continue
        size = length + 4  # header + length byte + payload + checksum
        if len(buffer) < size:
            break

        packet = bytes(buffer[:size])
        checksum = 0
        for byte in packet[:-1]:
            checksum ^= byte
        if checksum == packet[-1]:
            packets.append(packet)
            del buffer[:size]
        else:
            # Resynchronize without discarding a possible embedded header.
            del buffer[0]
    return packets


def main() -> int:
    parser = argparse.ArgumentParser(description="Receive MCU packets and print HEX; no commands are sent.")
    parser.add_argument("--port", "-p", required=True, help="Serial port, e.g. COM3")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()
    buffer = bytearray()

    try:
        with serial.Serial(args.port, args.baud, timeout=0.1) as port:
            print(f"Monitoring {args.port} at {args.baud} baud. Ctrl+C to stop.", flush=True)
            while True:
                buffer.extend(port.read(max(1, port.in_waiting)))
                for packet in extract_packets(buffer):
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
