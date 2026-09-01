import argparse
import sys
import time
from typing import Optional

import serial
import serial.tools.list_ports


HEADER = b"\xaa\x55"
CAN_BRIDGE_REQUEST_LENGTH = 15
CAN_BRIDGE_REQUEST_TYPE = 0x20
CAN_BRIDGE_RESPONSE_LENGTH = 14
CAN_BRIDGE_RESPONSE_TYPE = 0xA0
STATUS_PACKET_TYPE = 0x81

BRIDGE_STATUS_NAMES = {
    0: "ok",
    1: "can_tx_failed",
    2: "can_rx_timeout",
    3: "invalid_request",
}


def xor_checksum(data: bytes | bytearray | memoryview) -> int:
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def parse_int(value: str) -> int:
    return int(value, 0)


def parse_byte(value: str) -> int:
    parsed = parse_int(value)
    if not 0 <= parsed <= 0xFF:
        raise argparse.ArgumentTypeError(f"byte out of range: {value}")
    return parsed


def parse_can_id(value: str) -> int:
    parsed = parse_int(value)
    if not 0 <= parsed <= 0x7FF:
        raise argparse.ArgumentTypeError(f"standard CAN ID out of range: {value}")
    return parsed


def find_teensy_port() -> Optional[str]:
    for port in serial.tools.list_ports.comports():
        desc = (port.description or "").lower()
        name = (port.device or "").lower()
        if "teensy" in desc or "teensy" in name:
            return port.device

    for port in serial.tools.list_ports.comports():
        name = (port.device or "").lower()
        if (
            "usbmodem" in name
            or "ttyacm" in name
            or "usbserial" in name
            or "ttyusb" in name
        ):
            return port.device
    return None


def build_bridge_request(
    can_id: int,
    data: list[int],
    dlc: int,
    seq: int,
    timeout_ms: int,
) -> bytes:
    if not 0 <= can_id <= 0x7FF:
        raise ValueError("can_id must be a standard 11-bit CAN ID")
    if not 0 <= dlc <= 8:
        raise ValueError("dlc must be between 0 and 8")
    if len(data) > 8:
        raise ValueError("CAN data may contain at most 8 bytes")
    if len(data) > dlc:
        raise ValueError("data byte count may not exceed dlc")
    if not 0 <= timeout_ms <= 0xFFFF:
        raise ValueError("timeout_ms must be between 0 and 65535")

    padded_data = bytes(data + [0] * (8 - len(data)))
    packet_without_checksum = (
        HEADER
        + bytes(
            [
                CAN_BRIDGE_REQUEST_LENGTH,
                CAN_BRIDGE_REQUEST_TYPE,
                seq & 0xFF,
                can_id & 0xFF,
                (can_id >> 8) & 0xFF,
                dlc,
            ]
        )
        + padded_data
        + bytes([timeout_ms & 0xFF, (timeout_ms >> 8) & 0xFF])
    )
    return packet_without_checksum + bytes([xor_checksum(packet_without_checksum)])


def read_one(ser: serial.Serial, deadline: float) -> int:
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        ser.timeout = max(0.001, min(0.05, remaining))
        value = ser.read(1)
        if value:
            return value[0]
    raise TimeoutError("serial response timeout")


def read_packet(ser: serial.Serial, deadline: float) -> bytes:
    state = 0
    packet = bytearray()
    while True:
        byte = read_one(ser, deadline)
        if state == 0:
            if byte == HEADER[0]:
                packet = bytearray([byte])
                state = 1
        elif state == 1:
            if byte == HEADER[1]:
                packet.append(byte)
                state = 2
            elif byte == HEADER[0]:
                packet = bytearray([byte])
            else:
                packet.clear()
                state = 0
        else:
            packet.append(byte)
            if len(packet) == 3:
                length = packet[2]
                if length == 0 or length > 32:
                    packet.clear()
                    state = 0
            elif len(packet) >= 4 and len(packet) == 2 + 1 + packet[2] + 1:
                if xor_checksum(packet[:-1]) == packet[-1]:
                    return bytes(packet)
                packet.clear()
                state = 0


def read_bridge_response(
    ser: serial.Serial,
    seq: int,
    timeout_s: float,
) -> tuple[int, int, int, list[int]]:
    deadline = time.monotonic() + timeout_s
    while True:
        packet = read_packet(ser, deadline)
        packet_type = packet[3]
        if packet_type == STATUS_PACKET_TYPE:
            continue
        if packet_type != CAN_BRIDGE_RESPONSE_TYPE:
            continue
        if packet[2] != CAN_BRIDGE_RESPONSE_LENGTH:
            continue
        if packet[4] != (seq & 0xFF):
            continue

        status = packet[5]
        can_id = packet[6] | (packet[7] << 8)
        dlc = min(packet[8], 8)
        data = list(packet[9 : 9 + dlc])
        return status, can_id, dlc, data


def send_request(
    port: Optional[str],
    baud: int,
    request: bytes,
    seq: int,
    timeout_s: float,
    settle_s: float,
) -> tuple[int, int, int, list[int]]:
    if port is None:
        port = find_teensy_port()
        if port is None:
            raise RuntimeError("Teensy port not found. Provide --port explicitly.")

    with serial.Serial(port, baud, timeout=0.05) as ser:
        if settle_s > 0.0:
            time.sleep(settle_s)
        ser.reset_input_buffer()
        ser.write(request)
        ser.flush()
        return read_bridge_response(ser, seq, timeout_s)


def format_data(data: list[int], dlc: int) -> str:
    padded = data[:dlc] + [0] * max(0, dlc - len(data))
    return " ".join(f"{byte:02X}" for byte in padded)


def print_transaction(
    tx_id: int,
    tx_dlc: int,
    tx_data: list[int],
    status: int,
    rx_id: int,
    rx_dlc: int,
    rx_data: list[int],
) -> None:
    print(f"TX id=0x{tx_id:03X} dlc={tx_dlc} data={format_data(tx_data, tx_dlc)}")
    status_name = BRIDGE_STATUS_NAMES.get(status, f"unknown_{status}")
    print(
        f"RX status={status_name} id=0x{rx_id:03X} "
        f"dlc={rx_dlc} data={format_data(rx_data, rx_dlc)}"
    )
    if status == 0 and rx_dlc > 0:
        pid = rx_data[0]
        payload = rx_data[1:rx_dlc]
        print(f"MDROBOT pid={pid} payload={format_data(payload, len(payload))}")


def add_common_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", "-p", help="Serial port. Auto-detected if omitted.")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="USB Serial baud rate.")
    parser.add_argument("--seq", type=int, default=0, help="Bridge sequence byte.")
    parser.add_argument("--timeout-ms", type=int, default=200, help="CAN response timeout in ms.")
    parser.add_argument(
        "--serial-timeout-s",
        type=float,
        default=None,
        help="Overall USB response timeout. Default is derived from --timeout-ms.",
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=0.1,
        help="Delay after opening the USB serial port before sending.",
    )


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Send MD200T CAN PID/data frames through the Teensy CAN bridge."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    pid_parser = subparsers.add_parser("pid", help="Send an MDROBOT PID frame.")
    pid_parser.add_argument("driver_id", type=parse_byte, help="MD200T driver ID, for example 1 or 2.")
    pid_parser.add_argument("pid", type=parse_byte, help="MDROBOT PID byte.")
    pid_parser.add_argument("data", nargs="*", type=parse_byte, help="PID data bytes, max 7.")
    pid_parser.add_argument("--mid", type=parse_byte, default=0, help="CAN ID MID field, default 0.")
    add_common_options(pid_parser)

    frame_parser = subparsers.add_parser("frame", help="Send a raw standard CAN data frame.")
    frame_parser.add_argument("can_id", type=parse_can_id, help="Standard 11-bit CAN ID.")
    frame_parser.add_argument("data", nargs="*", type=parse_byte, help="CAN data bytes, max 8.")
    frame_parser.add_argument("--dlc", type=int, default=None, help="DLC. Default is len(data).")
    add_common_options(frame_parser)

    args = parser.parse_args(argv)

    if args.timeout_ms < 0 or args.timeout_ms > 0xFFFF:
        parser.error("--timeout-ms must be between 0 and 65535")

    if args.command == "pid":
        if len(args.data) > 7:
            parser.error("pid data may contain at most 7 bytes")
        if args.mid > 0x07:
            parser.error("--mid must be between 0 and 7")
        can_id = ((args.mid & 0x07) << 8) | args.driver_id
        tx_data = [args.pid] + args.data
        tx_dlc = 8
    else:
        if len(args.data) > 8:
            parser.error("frame data may contain at most 8 bytes")
        can_id = args.can_id
        tx_data = args.data
        tx_dlc = len(tx_data) if args.dlc is None else args.dlc
        if not 0 <= tx_dlc <= 8:
            parser.error("--dlc must be between 0 and 8")
        if len(tx_data) > tx_dlc:
            parser.error("data byte count may not exceed --dlc")

    request = build_bridge_request(can_id, tx_data, tx_dlc, args.seq, args.timeout_ms)
    serial_timeout_s = args.serial_timeout_s
    if serial_timeout_s is None:
        serial_timeout_s = max(1.0, (args.timeout_ms / 1000.0) + 0.5)

    try:
        status, rx_id, rx_dlc, rx_data = send_request(
            args.port,
            args.baud,
            request,
            args.seq,
            serial_timeout_s,
            args.settle_s,
        )
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print_transaction(can_id, tx_dlc, tx_data, status, rx_id, rx_dlc, rx_data)
    return 0 if status == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
