import sys
import time
import argparse
from typing import Optional
import serial
import struct

import serial.tools.list_ports

HEADER = b"\xaa\x55"
COMMAND_LENGTH = 7
COMMAND_TYPE = 0x01
FLAG_ENABLE = 0x01
FLAG_ESTOP = 0x02


def xor_checksum(data: bytes) -> int:
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def build_command_packet(
    v_cmd: int,
    w_cmd: int,
    seq: int = 0,
    enable: bool = True,
    emergency_stop: bool = False,
) -> bytes:
    flags = 0
    if enable:
        flags |= FLAG_ENABLE
    if emergency_stop:
        flags |= FLAG_ESTOP

    packet_without_checksum = (
        HEADER
        + struct.pack(
            "<BBBhhB", # little-endian: unsigned char 3 bytes, then 2x int16, then unsigned char 1 byte
            COMMAND_LENGTH,
            COMMAND_TYPE,
            seq & 0xFF,
            int(v_cmd),
            int(w_cmd),
            flags,
        )
    )
    return packet_without_checksum + bytes([xor_checksum(packet_without_checksum)])


def find_teensy_port() -> Optional[str]:
    """
    Try to auto-detect a Teensy/USB-UART device.
    Looks for "Teensy", "usbmodem", "ttyACM" or common USB serial identifiers.
    """
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        name = (p.device or "").lower()
        if "teensy" in desc or "teensy" in name:
            return p.device
    # fallback heuristics
    for p in serial.tools.list_ports.comports():
        name = (p.device or "").lower()
        if (
            "usbmodem" in name
            or "ttyacm" in name
            or "usbserial" in name
            or "ttyusb" in name
        ):
            return p.device
    return None


def send_command(
    v_cmd: int,
    w_cmd: int,
    seq: int = 0,
    enable: bool = True,
    emergency_stop: bool = False,
    port: Optional[str] = None,
    baud: int = 115200,
    timeout: float = 1.0,
) -> str:
    """
    Open serial port to Teensy and send one ROS-MCU command packet.
    Returns the first line of response (empty string if none).
    """
    if port is None:
        port = find_teensy_port()
        if port is None:
            raise RuntimeError("Teensy port not found. Provide port explicitly.")
    with serial.Serial(port, baud, timeout=timeout) as ser:
        payload = build_command_packet(
            v_cmd=v_cmd,
            w_cmd=w_cmd,
            seq=seq,
            enable=enable,
            emergency_stop=emergency_stop,
        )
        ser.write(payload)
        # give device a short moment to respond
        time.sleep(0.05)
        try:
            resp = ser.readline().decode("utf-8", errors="replace").strip()
        except Exception:
            resp = ""
        return resp


def main(argv):
    parser = argparse.ArgumentParser(
        description="Send one ROS-MCU command packet to Teensy over USB serial."
    )
    parser.add_argument(
        "v_cmd",
        type=int,
        help="Normalized linear command, typically -1000 to +1000.",
    )
    parser.add_argument(
        "w_cmd",
        type=int,
        help="Normalized angular command, typically -1000 to +1000.",
    )
    parser.add_argument(
        "--seq",
        type=int,
        default=0,
        help="Sequence counter byte, default 0.",
    )
    parser.add_argument(
        "--disable",
        action="store_true",
        help="Clear the enable flag.",
    )
    parser.add_argument(
        "--estop",
        action="store_true",
        help="Set the emergency_stop flag.",
    )
    parser.add_argument("--port", "-p", help="Serial port (auto-detected if omitted).")
    parser.add_argument(
        "--baud", "-b", type=int, default=115200, help="Baud rate (default 115200)."
    )
    args = parser.parse_args(argv)

    try:
        payload = build_command_packet(
            v_cmd=args.v_cmd,
            w_cmd=args.w_cmd,
            seq=args.seq,
            enable=not args.disable,
            emergency_stop=args.estop,
        )
        print(
            "Sending "
            f"v_cmd={args.v_cmd}, w_cmd={args.w_cmd}, seq={args.seq & 0xFF}, "
            f"enable={not args.disable}, emergency_stop={args.estop}, "
            f"packet={payload.hex(' ')} "
            f"to port={args.port or 'auto-detected'} at {args.baud} baud..."
        )
        resp = send_command(
            args.v_cmd,
            args.w_cmd,
            seq=args.seq,
            enable=not args.disable,
            emergency_stop=args.estop,
            port=args.port,
            baud=args.baud,
        )
        if resp:
            print(resp)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
