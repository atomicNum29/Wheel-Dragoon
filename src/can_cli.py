import argparse
import os
import select
import shlex
import statistics
import sys
import termios
import time
from dataclasses import dataclass, field
from typing import Optional

import serial
import serial.tools.list_ports


HEADER = b"\xaa\x55"
CAN_BRIDGE_REQUEST_LENGTH = 15
CAN_BRIDGE_REQUEST_TYPE = 0x20
CAN_BRIDGE_RESPONSE_LENGTH = 14
CAN_BRIDGE_RESPONSE_TYPE = 0xA0
CAN_BENCHMARK_REQUEST_LENGTH = 24
CAN_BENCHMARK_REQUEST_TYPE = 0x21
CAN_BENCHMARK_RESULT_LENGTH = 52
CAN_BENCHMARK_RESULT_TYPE = 0xA1
MAX_PACKET_LENGTH = 64

BRIDGE_STATUS_NAMES = {
    0: "ok",
    1: "can_tx_failed",
    2: "can_rx_timeout",
    3: "invalid_request",
}

BENCHMARK_STATUS_NAMES = {
    0: "response",
    1: "can_tx_failed",
    2: "response_timeout",
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


@dataclass(frozen=True)
class BenchmarkCommand:
    request_can_id: int
    request_data: list[int]
    request_dlc: int
    response_can_id: int
    response_pids: tuple[int, ...]
    repeat_count: int
    period_ms: int
    timeout_ms: int


@dataclass
class BenchmarkStatistics:
    """Collect per-response and whole-round timing for one benchmark session."""

    seq: Optional[int] = None
    expected_count: int = 0
    last_iteration: int = 0
    timeout_count: int = 0
    tx_failure_count: int = 0
    response_pids: tuple[int, ...] = ()
    rtt_samples_us: list[list[int]] = field(default_factory=lambda: [[], []])
    tx_interval_samples_us: list[int] = field(default_factory=list)
    rx_interval_samples_us: list[list[int]] = field(default_factory=lambda: [[], []])
    round_rtt_samples_us: list[int] = field(default_factory=list)
    response_gap_samples_us: list[int] = field(default_factory=list)
    round_rx_interval_samples_us: list[int] = field(default_factory=list)
    previous_tx_timestamp_us: Optional[int] = None
    previous_rx_timestamps_us: list[Optional[int]] = field(default_factory=lambda: [None, None])
    previous_round_rx_timestamp_us: Optional[int] = None
    partial_round_count: int = 0

    def start(self, seq: int, expected_count: int, response_pids: tuple[int, ...]) -> None:
        self.seq = seq & 0xFF
        self.expected_count = expected_count
        self.response_pids = response_pids
        self.last_iteration = 0
        self.timeout_count = 0
        self.tx_failure_count = 0
        for samples in self.rtt_samples_us:
            samples.clear()
        self.tx_interval_samples_us.clear()
        for samples in self.rx_interval_samples_us:
            samples.clear()
        self.round_rtt_samples_us.clear()
        self.response_gap_samples_us.clear()
        self.round_rx_interval_samples_us.clear()
        self.previous_tx_timestamp_us = None
        self.previous_rx_timestamps_us = [None, None]
        self.previous_round_rx_timestamp_us = None
        self.partial_round_count = 0

    def cancel(self) -> None:
        self.seq = None
        self.expected_count = 0
        self.last_iteration = 0
        self.timeout_count = 0
        self.tx_failure_count = 0
        self.response_pids = ()
        for samples in self.rtt_samples_us:
            samples.clear()
        self.tx_interval_samples_us.clear()
        for samples in self.rx_interval_samples_us:
            samples.clear()
        self.round_rtt_samples_us.clear()
        self.response_gap_samples_us.clear()
        self.round_rx_interval_samples_us.clear()
        self.previous_tx_timestamp_us = None
        self.previous_rx_timestamps_us = [None, None]
        self.previous_round_rx_timestamp_us = None
        self.partial_round_count = 0

    @staticmethod
    def _percentile(sorted_samples: list[int], percentile: float) -> float:
        if len(sorted_samples) == 1:
            return float(sorted_samples[0])

        position = (len(sorted_samples) - 1) * percentile
        lower = int(position)
        upper = min(lower + 1, len(sorted_samples) - 1)
        fraction = position - lower
        return sorted_samples[lower] + (sorted_samples[upper] - sorted_samples[lower]) * fraction

    @classmethod
    def _format_distribution(cls, name: str, samples: list[int]) -> str:
        if not samples:
            return f"{name}[n=0]"

        sorted_samples = sorted(samples)
        return (
            f"{name}[n={len(sorted_samples)} min={sorted_samples[0]} "
            f"avg={statistics.fmean(sorted_samples):.1f} "
            f"p50={cls._percentile(sorted_samples, 0.50):.1f} "
            f"p95={cls._percentile(sorted_samples, 0.95):.1f} "
            f"p99={cls._percentile(sorted_samples, 0.99):.1f} "
            f"max={sorted_samples[-1]} stddev={statistics.pstdev(sorted_samples):.1f}]"
        )

    def consume(self, packet: bytes) -> Optional[str]:
        if (
            self.seq is None
            or packet[2] != CAN_BENCHMARK_RESULT_LENGTH
            or packet[3] != CAN_BENCHMARK_RESULT_TYPE
            or packet[4] != self.seq
        ):
            return None

        iteration = int.from_bytes(packet[5:7], "little")
        status = packet[7]
        final = bool(packet[8] & 0x01)
        response_count = min(packet[36], 2)
        received_mask = packet[37] & ((1 << response_count) - 1)

        # Serial is ordered, but ignore a duplicate result if one is ever
        # observed so it cannot bias any timing distribution.
        if iteration > self.last_iteration:
            self.last_iteration = iteration
            tx_timestamp_us = int.from_bytes(packet[9:13], "little")
            if self.previous_tx_timestamp_us is not None:
                self.tx_interval_samples_us.append(
                    (tx_timestamp_us - self.previous_tx_timestamp_us) & 0xFFFFFFFF
                )
            self.previous_tx_timestamp_us = tx_timestamp_us

            rx_offsets = (13, 38)
            rtt_offsets = (17, 42)
            received_rtts: list[int] = []
            received_timestamps: list[int] = []
            for response_index in range(response_count):
                if (received_mask & (1 << response_index)) == 0:
                    continue
                rx_timestamp_us = int.from_bytes(
                    packet[rx_offsets[response_index] : rx_offsets[response_index] + 4],
                    "little",
                )
                rtt_us = int.from_bytes(
                    packet[rtt_offsets[response_index] : rtt_offsets[response_index] + 4],
                    "little",
                )
                previous_rx = self.previous_rx_timestamps_us[response_index]
                if previous_rx is not None:
                    self.rx_interval_samples_us[response_index].append(
                        (rx_timestamp_us - previous_rx) & 0xFFFFFFFF
                    )
                self.previous_rx_timestamps_us[response_index] = rx_timestamp_us
                self.rtt_samples_us[response_index].append(rtt_us)
                received_rtts.append(rtt_us)
                received_timestamps.append(rx_timestamp_us)

            complete_mask = (1 << response_count) - 1
            if response_count > 0 and received_mask == complete_mask and status == 0:
                round_rtt_us = max(received_rtts)
                completion_index = received_rtts.index(round_rtt_us)
                round_rx_timestamp_us = received_timestamps[completion_index]
                self.round_rtt_samples_us.append(round_rtt_us)
                if self.previous_round_rx_timestamp_us is not None:
                    self.round_rx_interval_samples_us.append(
                        (round_rx_timestamp_us - self.previous_round_rx_timestamp_us) & 0xFFFFFFFF
                    )
                self.previous_round_rx_timestamp_us = round_rx_timestamp_us
                if response_count == 2:
                    self.response_gap_samples_us.append(abs(received_rtts[1] - received_rtts[0]))
            elif status == 2 and received_mask != 0:
                self.partial_round_count += 1

        self.timeout_count = int.from_bytes(packet[21:23], "little")
        self.tx_failure_count = int.from_bytes(packet[23:25], "little")
        if not final:
            return None

        attempts = iteration
        complete_rounds = max(0, attempts - self.timeout_count - self.tx_failure_count)
        success_rate = (100.0 * complete_rounds / attempts) if attempts else 0.0
        prefix = (
            f"BENCH SUMMARY seq={self.seq} attempts={attempts}/{self.expected_count} "
            f"complete_rounds={complete_rounds} partial_rounds={self.partial_round_count} "
            f"timeouts={self.timeout_count} "
            f"tx_failures={self.tx_failure_count} success_rate={success_rate:.2f}%"
        )
        distributions = [prefix]
        for response_index, pid in enumerate(self.response_pids):
            distributions.append(
                self._format_distribution(f"pid{pid}_rtt_us", self.rtt_samples_us[response_index])
            )
        distributions.append(self._format_distribution("round_rtt_us", self.round_rtt_samples_us))
        if len(self.response_pids) == 2:
            distributions.append(
                self._format_distribution("response_gap_us", self.response_gap_samples_us)
            )
        distributions.append(
            self._format_distribution("tx_interval_us", self.tx_interval_samples_us)
        )
        for response_index, pid in enumerate(self.response_pids):
            distributions.append(
                self._format_distribution(
                    f"pid{pid}_rx_interval_us", self.rx_interval_samples_us[response_index]
                )
            )
        distributions.append(
            self._format_distribution("round_rx_interval_us", self.round_rx_interval_samples_us)
        )
        summary = "\n  ".join(distributions)

        self.cancel()
        return summary


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


def build_benchmark_request(command: BenchmarkCommand, seq: int) -> bytes:
    if not 0 <= command.request_can_id <= 0x7FF:
        raise ValueError("request_can_id must be a standard 11-bit CAN ID")
    if not 0 <= command.request_dlc <= 8:
        raise ValueError("request_dlc must be between 0 and 8")
    if len(command.request_data) > 8 or len(command.request_data) > command.request_dlc:
        raise ValueError("request data byte count must fit the DLC and 8-byte CAN limit")
    if not 0 <= command.response_can_id <= 0x7FF:
        raise ValueError("response_can_id must be a standard 11-bit CAN ID")
    if not 1 <= len(command.response_pids) <= 2:
        raise ValueError("response_pids must contain one or two PIDs")
    if any(not 0 <= pid <= 0xFF for pid in command.response_pids):
        raise ValueError("each response PID must be a byte")
    if len(set(command.response_pids)) != len(command.response_pids):
        raise ValueError("response PIDs must be distinct")
    if not 1 <= command.repeat_count <= 0xFFFF:
        raise ValueError("repeat_count must be between 1 and 65535")
    if not 1 <= command.period_ms <= 0xFFFF:
        raise ValueError("period_ms must be between 1 and 65535")
    if not 1 <= command.timeout_ms <= 100:
        raise ValueError("timeout_ms must be between 1 and 100")
    if command.timeout_ms > command.period_ms:
        raise ValueError("timeout_ms must not exceed period_ms")

    padded_data = bytes(command.request_data + [0] * (8 - len(command.request_data)))
    packet_without_checksum = (
        HEADER
        + bytes(
            [
                CAN_BENCHMARK_REQUEST_LENGTH,
                CAN_BENCHMARK_REQUEST_TYPE,
                seq & 0xFF,
                command.request_can_id & 0xFF,
                (command.request_can_id >> 8) & 0xFF,
                command.request_dlc,
            ]
        )
        + padded_data
        + bytes(
            [
                command.response_can_id & 0xFF,
                (command.response_can_id >> 8) & 0xFF,
                command.response_pids[0],
                command.repeat_count & 0xFF,
                (command.repeat_count >> 8) & 0xFF,
                command.period_ms & 0xFF,
                (command.period_ms >> 8) & 0xFF,
                command.timeout_ms & 0xFF,
                (command.timeout_ms >> 8) & 0xFF,
                len(command.response_pids),
                command.response_pids[1] if len(command.response_pids) == 2 else 0,
            ]
        )
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

    benchmark_parser = subparsers.add_parser(
        "benchmark",
        add_help=add_help,
        help="repeat a CAN request and measure one or two matching response RTTs",
    )
    benchmark_parser.add_argument("can_id", type=parse_can_id)
    benchmark_parser.add_argument("data", nargs="*", type=parse_byte)
    benchmark_parser.add_argument("--dlc", type=int, default=None)
    benchmark_parser.add_argument("--response-id", type=parse_can_id, default=None)
    benchmark_parser.add_argument("--response-pid", type=parse_byte, default=None)
    benchmark_parser.add_argument("--response-pid2", type=parse_byte, default=None)
    benchmark_parser.add_argument("--count", type=int, default=100)
    benchmark_parser.add_argument("--period-ms", type=int, default=10)
    benchmark_parser.add_argument("--timeout-ms", type=int, default=None)
    return parser


def command_from_namespace(
    args: argparse.Namespace, default_timeout_ms: int
) -> BridgeCommand | BenchmarkCommand:
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

    if args.command == "benchmark":
        timeout_ms = (
            min(default_timeout_ms, args.period_ms)
            if args.timeout_ms is None
            else args.timeout_ms
        )
        if len(args.data) > 8:
            raise CommandError("benchmark request data may contain at most 8 bytes")
        dlc = len(args.data) if args.dlc is None else args.dlc
        if not 0 <= dlc <= 8 or len(args.data) > dlc:
            raise CommandError("benchmark request data must fit --dlc in range 0..8")
        if not 1 <= args.count <= 0xFFFF:
            raise CommandError("--count must be between 1 and 65535")
        if not 1 <= args.period_ms <= 0xFFFF:
            raise CommandError("--period-ms must be between 1 and 65535")
        if not 1 <= timeout_ms <= 100 or timeout_ms > args.period_ms:
            raise CommandError("--timeout-ms must be 1..100 and no greater than --period-ms")

        response_can_id = (
            ((7 << 8) | (args.can_id & 0xFF))
            if args.response_id is None
            else args.response_id
        )
        if args.response_pid2 is not None and args.response_pid is None:
            raise CommandError("--response-pid2 requires --response-pid")

        if args.response_pid is not None:
            response_pids = (
                (args.response_pid, args.response_pid2)
                if args.response_pid2 is not None
                else (args.response_pid,)
            )
        else:
            if len(args.data) >= 2 and args.data[0] == 4:
                response_pids = (args.data[1],)
            elif args.data and args.data[0] in (174, 207):
                response_pids = (193, 200)
            elif args.data:
                response_pids = (args.data[0],)
            else:
                raise CommandError("--response-pid is required when request data is empty")

        return BenchmarkCommand(
            args.can_id,
            args.data,
            dlc,
            response_can_id,
            response_pids,
            args.count,
            args.period_ms,
            timeout_ms,
        )

    raise CommandError("enter 'pid ...', 'frame ...', 'benchmark ...', 'help', or 'quit'")


def parse_interactive_command(
    line: str, default_timeout_ms: int
) -> BridgeCommand | BenchmarkCommand:
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
        "  benchmark CAN_ID [DATA_BYTE ...] [--dlc N] [--response-id ID]\n"
        "            [--response-pid PID] [--response-pid2 PID] [--count N]\n"
        "            [--period-ms N] [--timeout-ms N]\n"
        "  help\n"
        "  quit | exit | q"
    )


def format_data(data: list[int], dlc: int) -> str:
    padded = data[:dlc] + [0] * max(0, dlc - len(data))
    return " ".join(f"{byte:02X}" for byte in padded)


def format_packet(packet: bytes) -> str:
    timestamp = time.strftime("%H:%M:%S")
    if packet[3] == CAN_BENCHMARK_RESULT_TYPE and packet[2] == CAN_BENCHMARK_RESULT_LENGTH:
        seq = packet[4]
        iteration = int.from_bytes(packet[5:7], "little")
        status = packet[7]
        status_name = BENCHMARK_STATUS_NAMES.get(status, f"unknown_{status}")
        final = bool(packet[8] & 0x01)
        tx_us = int.from_bytes(packet[9:13], "little")
        timeout_count = int.from_bytes(packet[21:23], "little")
        tx_failure_count = int.from_bytes(packet[23:25], "little")
        can_id = int.from_bytes(packet[25:27], "little")
        response_count = min(packet[36], 2)
        received_mask = packet[37] & ((1 << response_count) - 1)
        final_text = " final" if final else ""
        response_parts: list[str] = []
        rtts: list[int] = []
        for response_index, (rx_offset, rtt_offset, dlc_offset, data_offset) in enumerate(
            ((13, 17, 27, 28), (38, 42, 46, 47))
        ):
            if response_index >= response_count or (received_mask & (1 << response_index)) == 0:
                continue
            rx_us = int.from_bytes(packet[rx_offset : rx_offset + 4], "little")
            rtt_us = int.from_bytes(packet[rtt_offset : rtt_offset + 4], "little")
            dlc = min(packet[dlc_offset], 8)
            data = list(packet[data_offset : data_offset + dlc])
            pid = data[0] if data else None
            pid_text = f"pid={pid}" if pid is not None else "pid=?"
            response_parts.append(
                f"rsp{response_index + 1}[{pid_text} rx_us={rx_us} rtt_us={rtt_us} "
                f"dlc={dlc} data={format_data(data, dlc)}]"
            )
            rtts.append(rtt_us)
        timing_text = ""
        if len(rtts) == response_count and response_count > 0:
            timing_text = f" round_rtt_us={max(rtts)}"
            if response_count == 2:
                timing_text += f" response_gap_us={abs(rtts[1] - rtts[0])}"
        response_text = " " + " ".join(response_parts) if response_parts else ""
        return (
            f"[{timestamp}] BENCH seq={seq} iter={iteration} status={status_name}{final_text} "
            f"tx_us={tx_us} received={received_mask.bit_count()}/{response_count} "
            f"timeouts={timeout_count} tx_failures={tx_failure_count} id=0x{can_id:03X}"
            f"{timing_text}{response_text}"
        )

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


def send_command(
    ser: serial.Serial, command: BridgeCommand | BenchmarkCommand, seq: int, ui: TerminalUi
) -> int:
    if isinstance(command, BenchmarkCommand):
        request = build_benchmark_request(command, seq)
        ser.write(request)
        ser.flush()
        ui.log(
            f"BENCH TX seq={seq} id=0x{command.request_can_id:03X} "
            f"dlc={command.request_dlc} data={format_data(command.request_data, command.request_dlc)} "
            f"response_id=0x{command.response_can_id:03X} "
            f"response_pids={','.join(str(pid) for pid in command.response_pids)} "
            f"count={command.repeat_count} period_ms={command.period_ms} timeout_ms={command.timeout_ms}"
        )
        return (seq + 1) & 0xFF

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
    initial_command: Optional[BridgeCommand | BenchmarkCommand],
) -> None:
    stream_parser = PacketStreamParser()
    benchmark_statistics = BenchmarkStatistics()

    def send_and_track(
        ser: serial.Serial,
        command: BridgeCommand | BenchmarkCommand,
        current_seq: int,
        ui: TerminalUi,
    ) -> int:
        if isinstance(command, BenchmarkCommand):
            benchmark_statistics.start(current_seq, command.repeat_count, command.response_pids)
        else:
            # A raw 0x20 request cancels the MCU benchmark session.
            benchmark_statistics.cancel()
        return send_command(ser, command, current_seq, ui)

    with serial.Serial(port, baud, timeout=0) as ser, TerminalUi() as ui:
        if settle_s > 0.0:
            time.sleep(settle_s)
        ui.log(f"Connected to {port} at {baud} baud. Type 'help' for commands.")
        if initial_command is not None:
            seq = send_and_track(ser, initial_command, seq, ui)

        while True:
            readable, _, _ = select.select([ser.fileno(), sys.stdin.fileno()], [], [], 0.1)
            if ser.fileno() in readable:
                received = ser.read(max(1, ser.in_waiting))
                for packet in stream_parser.feed(received):
                    ui.log(format_packet(packet))
                    summary = benchmark_statistics.consume(packet)
                    if summary is not None:
                        ui.log(summary)

            if sys.stdin.fileno() in readable:
                for line in ui.read_lines():
                    try:
                        command = parse_interactive_command(line, default_timeout_ms)
                        seq = send_and_track(ser, command, seq, ui)
                    except CommandError as exc:
                        for message_line in str(exc).splitlines():
                            ui.log(message_line)


def add_connection_options(
    parser: argparse.ArgumentParser,
    suppress_defaults: bool,
    include_timeout: bool = True,
) -> None:
    default = argparse.SUPPRESS if suppress_defaults else None
    parser.add_argument("--port", "-p", default=default, help="serial port; auto-detected if omitted")
    parser.add_argument("--baud", "-b", type=int, default=argparse.SUPPRESS if suppress_defaults else 115200)
    parser.add_argument("--seq", type=parse_byte, default=argparse.SUPPRESS if suppress_defaults else 0)
    if include_timeout:
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

    benchmark_parser = subparsers.add_parser(
        "benchmark", help="start a one- or two-response CAN RTT benchmark"
    )
    benchmark_parser.add_argument("can_id", type=parse_can_id)
    benchmark_parser.add_argument("data", nargs="*", type=parse_byte)
    benchmark_parser.add_argument("--dlc", type=int, default=None)
    benchmark_parser.add_argument("--response-id", type=parse_can_id, default=None)
    benchmark_parser.add_argument("--response-pid", type=parse_byte, default=None)
    benchmark_parser.add_argument("--response-pid2", type=parse_byte, default=None)
    benchmark_parser.add_argument("--count", type=int, default=100)
    benchmark_parser.add_argument("--period-ms", type=int, default=10)
    benchmark_parser.add_argument(
        "--timeout-ms", type=int, default=10, help="per-sample response timeout; maximum 100 ms"
    )
    add_connection_options(benchmark_parser, suppress_defaults=True, include_timeout=False)
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
