import argparse
import os
import select
import shlex
import sys
import termios
import time
from dataclasses import dataclass
from typing import Optional

import serial
import serial.tools.list_ports


HEADER = b"\xaa\x55"
CAN_BRIDGE_REQUEST_LENGTH = 15
CAN_BRIDGE_REQUEST_TYPE = 0x20
CAN_BRIDGE_RESPONSE_LENGTH = 14
CAN_BRIDGE_RESPONSE_TYPE = 0xA0
MAX_PACKET_LENGTH = 64

BRIDGE_STATUS_NAMES = {
    0: "ok",
    1: "can_tx_failed",
    2: "can_rx_timeout",
    3: "invalid_request",
}

EXIT_COMMANDS = {"q", "quit", "exit"}
HELP_COMMANDS = {"?", "h", "help"}


@dataclass(frozen=True)
class BridgeCommand:
    can_id: int
    data: list[int]
    dlc: int
    timeout_ms: int


class CommandError(ValueError):
    pass


class ExitRequested(Exception):
    pass


class RaisingArgumentParser(argparse.ArgumentParser):
    def error(self, message: str) -> None:
        raise CommandError(message)


class PacketStreamParser:
    """Incrementally split the MCU byte stream into checksum-valid packets."""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        self.buffer.extend(data)
        packets: list[bytes] = []

        while True:
            header_index = self.buffer.find(HEADER)
            if header_index < 0:
                # Keep a trailing 0xAA because it may be the first header byte.
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == HEADER[:1] else b""
                break
            if header_index > 0:
                del self.buffer[:header_index]
            if len(self.buffer) < 3:
                break

            payload_length = self.buffer[2]
            if payload_length == 0 or payload_length > MAX_PACKET_LENGTH:
                del self.buffer[0]
                continue

            packet_size = payload_length + 4
            if len(self.buffer) < packet_size:
                break

            candidate = bytes(self.buffer[:packet_size])
            if xor_checksum(candidate[:-1]) == candidate[-1]:
                packets.append(candidate)
                del self.buffer[:packet_size]
            else:
                # Discard only the first header byte so an embedded header can
                # be found without losing the following valid packet.
                del self.buffer[0]

        return packets


class TerminalUi:
    PROMPT = "can> "

    def __init__(self) -> None:
        self.fd = sys.stdin.fileno()
        self.saved_attributes: Optional[list] = None
        self.input_buffer = ""
        self.escape_state = 0

    def __enter__(self) -> "TerminalUi":
        if not sys.stdin.isatty() or not sys.stdout.isatty():
            raise RuntimeError("interactive CLI requires a terminal")

        self.saved_attributes = termios.tcgetattr(self.fd)
        attributes = termios.tcgetattr(self.fd)
        attributes[3] &= ~(termios.ICANON | termios.ECHO)
        attributes[6][termios.VMIN] = 0
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attributes)
        self.redraw()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        if self.saved_attributes is not None:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.saved_attributes)
        sys.stdout.write("\r\x1b[2K")
        sys.stdout.flush()

    def redraw(self) -> None:
        sys.stdout.write(f"\r\x1b[2K{self.PROMPT}{self.input_buffer}")
        sys.stdout.flush()

    def log(self, message: str) -> None:
        sys.stdout.write(f"\r\x1b[2K{message}\n")
        self.redraw()

    def read_lines(self) -> list[str]:
        lines: list[str] = []
        for byte in os.read(self.fd, 64):
            if self.escape_state == 1:
                self.escape_state = 2 if byte == ord("[") else 0
                continue
            if self.escape_state == 2:
                if 0x40 <= byte <= 0x7E:
                    self.escape_state = 0
                continue
            if byte == 0x1B:
                self.escape_state = 1
            elif byte in (0x0A, 0x0D):
                line = self.input_buffer.strip()
                self.input_buffer = ""
                if line:
                    lines.append(line)
            elif byte in (0x08, 0x7F):
                self.input_buffer = self.input_buffer[:-1]
            elif byte == 0x15:  # Ctrl-U
                self.input_buffer = ""
            elif byte == 0x04:  # Ctrl-D
                raise ExitRequested
            elif 0x20 <= byte <= 0x7E:
                self.input_buffer += chr(byte)
        self.redraw()
        return lines


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


def build_command_parser(prog: str, add_help: bool) -> RaisingArgumentParser:
    parser = RaisingArgumentParser(prog=prog, add_help=add_help)
    subparsers = parser.add_subparsers(dest="command")

    pid_parser = subparsers.add_parser("pid", add_help=add_help, help="send an MDROBOT PID frame")
    pid_parser.add_argument("driver_id", type=parse_byte)
    pid_parser.add_argument("pid", type=parse_byte)
    pid_parser.add_argument("data", nargs="*", type=parse_byte)
    pid_parser.add_argument("--mid", type=parse_byte, default=0)
    pid_parser.add_argument("--timeout-ms", type=int, default=None)

    frame_parser = subparsers.add_parser("frame", add_help=add_help, help="send a raw CAN frame")
    frame_parser.add_argument("can_id", type=parse_can_id)
    frame_parser.add_argument("data", nargs="*", type=parse_byte)
    frame_parser.add_argument("--dlc", type=int, default=None)
    frame_parser.add_argument("--timeout-ms", type=int, default=None)
    return parser


def command_from_namespace(args: argparse.Namespace, default_timeout_ms: int) -> BridgeCommand:
    timeout_ms = default_timeout_ms if args.timeout_ms is None else args.timeout_ms
    if not 0 <= timeout_ms <= 0xFFFF:
        raise CommandError("--timeout-ms must be between 0 and 65535")

    if args.command == "pid":
        if len(args.data) > 7:
            raise CommandError("pid data may contain at most 7 bytes")
        if args.mid > 0x07:
            raise CommandError("--mid must be between 0 and 7")
        return BridgeCommand(
            can_id=((args.mid & 0x07) << 8) | args.driver_id,
            data=[args.pid] + args.data,
            dlc=8,
            timeout_ms=timeout_ms,
        )

    if args.command == "frame":
        if len(args.data) > 8:
            raise CommandError("frame data may contain at most 8 bytes")
        dlc = len(args.data) if args.dlc is None else args.dlc
        if not 0 <= dlc <= 8:
            raise CommandError("--dlc must be between 0 and 8")
        if len(args.data) > dlc:
            raise CommandError("data byte count may not exceed --dlc")
        return BridgeCommand(args.can_id, args.data, dlc, timeout_ms)

    raise CommandError("enter 'pid ...', 'frame ...', 'help', or 'quit'")


def parse_interactive_command(line: str, default_timeout_ms: int) -> BridgeCommand:
    try:
        tokens = shlex.split(line)
    except ValueError as exc:
        raise CommandError(str(exc)) from exc
    if not tokens:
        raise CommandError("empty command")

    keyword = tokens[0].lower()
    if keyword in EXIT_COMMANDS:
        if len(tokens) != 1:
            raise CommandError(f"{keyword} does not take arguments")
        raise ExitRequested
    if keyword in HELP_COMMANDS:
        raise CommandError(interactive_help())

    parser = build_command_parser("", add_help=False)
    return command_from_namespace(parser.parse_args(tokens), default_timeout_ms)


def interactive_help() -> str:
    return (
        "commands:\n"
        "  pid DRIVER_ID PID [DATA_BYTE ...] [--mid MID] [--timeout-ms N]\n"
        "  frame CAN_ID [DATA_BYTE ...] [--dlc N] [--timeout-ms N]\n"
        "  help\n"
        "  quit | exit | q"
    )


def format_data(data: list[int], dlc: int) -> str:
    padded = data[:dlc] + [0] * max(0, dlc - len(data))
    return " ".join(f"{byte:02X}" for byte in padded)


def format_packet(packet: bytes) -> str:
    timestamp = time.strftime("%H:%M:%S")
    if packet[3] != CAN_BRIDGE_RESPONSE_TYPE or packet[2] != CAN_BRIDGE_RESPONSE_LENGTH:
        return f"[{timestamp}] MCU type=0x{packet[3]:02X} raw={format_data(list(packet), len(packet))}"

    seq = packet[4]
    status = packet[5]
    status_name = BRIDGE_STATUS_NAMES.get(status, f"unknown_{status}")
    can_id = packet[6] | (packet[7] << 8)
    dlc = min(packet[8], 8)
    data = list(packet[9 : 9 + dlc])
    if status != 0:
        return f"[{timestamp}] BRIDGE seq={seq} status={status_name}"

    pid_text = f" pid={data[0]}" if data else ""
    return (
        f"[{timestamp}] RX seq={seq} id=0x{can_id:03X} dlc={dlc} "
        f"data={format_data(data, dlc)}{pid_text}"
    )


def send_command(ser: serial.Serial, command: BridgeCommand, seq: int, ui: TerminalUi) -> int:
    request = build_bridge_request(
        command.can_id,
        command.data,
        command.dlc,
        seq,
        command.timeout_ms,
    )
    ser.write(request)
    ser.flush()
    ui.log(
        f"TX seq={seq} id=0x{command.can_id:03X} dlc={command.dlc} "
        f"data={format_data(command.data, command.dlc)}"
    )
    return (seq + 1) & 0xFF


def monitor(
    port: str,
    baud: int,
    seq: int,
    default_timeout_ms: int,
    settle_s: float,
    initial_command: Optional[BridgeCommand],
) -> None:
    stream_parser = PacketStreamParser()
    with serial.Serial(port, baud, timeout=0) as ser, TerminalUi() as ui:
        if settle_s > 0.0:
            time.sleep(settle_s)
        ui.log(f"Connected to {port} at {baud} baud. Type 'help' for commands.")
        if initial_command is not None:
            seq = send_command(ser, initial_command, seq, ui)

        while True:
            readable, _, _ = select.select([ser.fileno(), sys.stdin.fileno()], [], [], 0.1)
            if ser.fileno() in readable:
                received = ser.read(max(1, ser.in_waiting))
                for packet in stream_parser.feed(received):
                    ui.log(format_packet(packet))

            if sys.stdin.fileno() in readable:
                for line in ui.read_lines():
                    try:
                        command = parse_interactive_command(line, default_timeout_ms)
                        seq = send_command(ser, command, seq, ui)
                    except CommandError as exc:
                        for message_line in str(exc).splitlines():
                            ui.log(message_line)


def add_connection_options(parser: argparse.ArgumentParser, suppress_defaults: bool) -> None:
    default = argparse.SUPPRESS if suppress_defaults else None
    parser.add_argument("--port", "-p", default=default, help="serial port; auto-detected if omitted")
    parser.add_argument("--baud", "-b", type=int, default=argparse.SUPPRESS if suppress_defaults else 115200)
    parser.add_argument("--seq", type=parse_byte, default=argparse.SUPPRESS if suppress_defaults else 0)
    parser.add_argument(
        "--timeout-ms",
        type=int,
        default=argparse.SUPPRESS if suppress_defaults else 100,
        help="default request wait time; MCU caps it at 100 ms",
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=argparse.SUPPRESS if suppress_defaults else 0.1,
        help="delay after opening the serial port",
    )


def build_main_parser() -> RaisingArgumentParser:
    parser = RaisingArgumentParser(
        description="Continuously monitor and transmit CAN frames through the Teensy 0x20 bridge."
    )
    add_connection_options(parser, suppress_defaults=False)
    subparsers = parser.add_subparsers(dest="command")

    pid_parser = subparsers.add_parser("pid", help="send an initial MDROBOT PID frame")
    pid_parser.add_argument("driver_id", type=parse_byte)
    pid_parser.add_argument("pid", type=parse_byte)
    pid_parser.add_argument("data", nargs="*", type=parse_byte)
    pid_parser.add_argument("--mid", type=parse_byte, default=0)
    add_connection_options(pid_parser, suppress_defaults=True)

    frame_parser = subparsers.add_parser("frame", help="send an initial raw CAN frame")
    frame_parser.add_argument("can_id", type=parse_can_id)
    frame_parser.add_argument("data", nargs="*", type=parse_byte)
    frame_parser.add_argument("--dlc", type=int, default=None)
    add_connection_options(frame_parser, suppress_defaults=True)
    return parser


def main(argv: list[str]) -> int:
    parser = build_main_parser()
    try:
        args = parser.parse_args(argv)
        if not 0 <= args.timeout_ms <= 0xFFFF:
            raise CommandError("--timeout-ms must be between 0 and 65535")
        if args.settle_s < 0.0:
            raise CommandError("--settle-s must not be negative")
        initial_command = (
            command_from_namespace(args, args.timeout_ms) if args.command is not None else None
        )
    except CommandError as exc:
        parser.print_usage(sys.stderr)
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    port = args.port or find_teensy_port()
    if port is None:
        print("Error: Teensy port not found. Provide --port explicitly.", file=sys.stderr)
        return 1

    try:
        monitor(port, args.baud, args.seq, args.timeout_ms, args.settle_s, initial_command)
    except (ExitRequested, KeyboardInterrupt):
        print("CAN bridge monitor stopped.")
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
