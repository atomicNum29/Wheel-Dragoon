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
RESET_LENGTH = 4
RESET_TYPE = 0x02
RESET_MAGIC = b"\xa5\x5a"
FLAG_ENABLE = 0x01
FLAG_ESTOP = 0x02
LINEAR_MPS_MIN = -2.0
LINEAR_MPS_MAX = 2.0
ANGULAR_RADPS_MIN = -5.0
ANGULAR_RADPS_MAX = 5.0


def xor_checksum(data: bytes) -> int:
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def build_command_packet(
    v_mps: float,
    w_radps: float,
    seq: int = 0,
    enable: bool = True,
    emergency_stop: bool = False,
) -> bytes:
    flags = 0
    if enable:
        flags |= FLAG_ENABLE
    if emergency_stop:
        flags |= FLAG_ESTOP

    if not LINEAR_MPS_MIN <= v_mps <= LINEAR_MPS_MAX:
        raise ValueError(f"v_mps must be between {LINEAR_MPS_MIN} and {LINEAR_MPS_MAX}")
    if not ANGULAR_RADPS_MIN <= w_radps <= ANGULAR_RADPS_MAX:
        raise ValueError(
            f"w_radps must be between {ANGULAR_RADPS_MIN} and {ANGULAR_RADPS_MAX}"
        )

    v_milli_mps = round(v_mps * 1000.0)
    w_milli_radps = round(w_radps * 1000.0)
    if not -32768 <= v_milli_mps <= 32767:
        raise ValueError("v_mps is out of int16 milli-m/s range")
    if not -32768 <= w_milli_radps <= 32767:
        raise ValueError("w_radps is out of int16 milli-rad/s range")

    packet_without_checksum = (
        HEADER
        + struct.pack(
            "<BBBhhB", # little-endian: unsigned char 3 bytes, then 2x int16, then unsigned char 1 byte
            COMMAND_LENGTH,
            COMMAND_TYPE,
            seq & 0xFF,
            int(v_milli_mps),
            int(w_milli_radps),
            flags,
        )
    )
    return packet_without_checksum + bytes([xor_checksum(packet_without_checksum)])


def build_reset_packet(seq: int = 0) -> bytes:
    """Build the protected MCU software-reset command packet."""
    packet_without_checksum = (
        HEADER
        + bytes([RESET_LENGTH, RESET_TYPE, seq & 0xFF])
        + RESET_MAGIC
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


def send_packet(
    payload: bytes,
    port: Optional[str],
    baud: int,
    timeout: float,
    read_response: bool,
) -> str:
    if port is None:
        port = find_teensy_port()
        if port is None:
            raise RuntimeError("Teensy port not found. Provide port explicitly.")

    with serial.Serial(port, baud, timeout=timeout) as ser:
        ser.write(payload)
        ser.flush()
        if not read_response:
            return ""

        # Give the device a short moment to produce a status packet.
        time.sleep(0.05)
        try:
            return ser.readline().decode("utf-8", errors="replace").strip()
        except Exception:
            return ""


def send_command(
    v_mps: float,
    w_radps: float,
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
    payload = build_command_packet(
        v_mps=v_mps,
        w_radps=w_radps,
        seq=seq,
        enable=enable,
        emergency_stop=emergency_stop,
    )
    return send_packet(payload, port, baud, timeout, read_response=True)


def send_reset(
    seq: int = 0,
    port: Optional[str] = None,
    baud: int = 115200,
    timeout: float = 1.0,
) -> None:
    # A reset deliberately has no response: USB disconnects as soon as the MCU
    # completes the two MD200T TQ-OFF transmissions and reboots.
    send_packet(build_reset_packet(seq), port, baud, timeout, read_response=False)


def main(argv):
    parser = argparse.ArgumentParser(
        description="Send one ROS-MCU command packet to Teensy over USB serial."
    )
    parser.add_argument(
        "v_mps",
        type=float,
        nargs="?",
        help="Target linear velocity in m/s.",
    )
    parser.add_argument(
        "w_radps",
        type=float,
        nargs="?",
        help="Target angular velocity in rad/s.",
    )
    parser.add_argument(
        "--reset",
        action="store_true",
        help="TQ-OFF both MD200T drivers and software-reset the MCU.",
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
        if args.reset:
            if args.v_mps is not None or args.w_radps is not None:
                raise ValueError("v_mps and w_radps must be omitted with --reset")
            if args.disable or args.estop:
                raise ValueError("--disable and --estop cannot be used with --reset")

            payload = build_reset_packet(args.seq)
            print(
                f"Sending MCU reset, seq={args.seq & 0xFF}, "
                f"packet={payload.hex(' ')} "
                f"to port={args.port or 'auto-detected'} at {args.baud} baud..."
            )
            send_reset(seq=args.seq, port=args.port, baud=args.baud)
            return 0

        if args.v_mps is None or args.w_radps is None:
            raise ValueError("v_mps and w_radps are required unless --reset is used")

        payload = build_command_packet(
            v_mps=args.v_mps,
            w_radps=args.w_radps,
            seq=args.seq,
            enable=not args.disable,
            emergency_stop=args.estop,
        )
        print(
            "Sending "
            f"v_mps={args.v_mps:.3f}, w_radps={args.w_radps:.3f}, "
            f"seq={args.seq & 0xFF}, "
            f"enable={not args.disable}, emergency_stop={args.estop}, "
            f"packet={payload.hex(' ')} "
            f"to port={args.port or 'auto-detected'} at {args.baud} baud..."
        )
        resp = send_command(
            args.v_mps,
            args.w_radps,
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
