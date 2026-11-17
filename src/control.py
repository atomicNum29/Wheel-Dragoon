"""Line-command motion CLI for Windows/Linux, using MCU protocol 0x03."""

import argparse
import math
import shlex
import sys
import time
from dataclasses import replace

import serial
import serial.tools.list_ports

from can_cli import ExitRequested, TerminalUi, find_teensy_port, wait_for_activity
from mcu_protocol import (MotionCommand, PacketStreamParser, WHEELS, build_reset_packet,
                          decode_telemetry, format_telemetry)

HELP = """Commands (Enter to apply; targets repeat at --rate-hz):
  drive V THETA_DEG   normal drive: m/s, virtual center angle in degrees
  drive-rad V THETA   normal drive: m/s, angle in radians
  speed V            change normal-drive speed; starts from zero if no target
  steer THETA_DEG    change normal-drive angle; starts from zero if no target
  pivot W            rear-axle-center rotation, rad/s
  stop               scheduled stop, preserving normal steering / PIVOT pose
  disable            torque-off request; does not clear an active estop
  estop              torque-off + opposing-PIVOT steering request (AUTO only)
  clear-estop        clear estop, keep drive disabled; then enter a new target
  pause              stop periodic TX to observe the MCU command timeout
  status             show latest feedback, host sample ages and local target
  reset              send protected MCU reset and exit
  help / quit        show help / exit (best-effort disable, preserving estop)
RC must be AUTO for USB motion. Startup is receive-only until a command is entered.
Input/feedback angles are virtual-center angles, not individual servo angles."""


class ControlUi(TerminalUi):
    PROMPT = "motion> "


class PlainUi:
    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def log(self, message: str) -> None:
        print(message, flush=True)


class Feedback:
    def __init__(self):
        self.samples: dict[tuple[int, int], tuple[dict, float]] = {}

    def update(self, data: dict, now: float) -> None:
        if data["type"] == 0x87:
            old = self.samples.get((0x87, 0))
            if old and old[0]["session"] != data["session"]:
                self.samples.clear()
        key = (data["type"], data.get("id", 0))
        self.samples[key] = (data, now)

    def estop_active(self) -> bool:
        state = self.samples.get((0x87, 0))
        # Retain an observed estop even if its feedback is stale.
        return bool(state and state[0]["flags"] & (1 << 5))

    def describe(self, now: float) -> str:
        lines = []
        state = self.samples.get((0x87, 0))
        state_fresh = state is not None and now - state[1] <= 1.0
        if state:
            age = now - state[1]
            lines.append(f"{'STALE ' if not state_fresh else ''}age={age:.2f}s {format_telemetry(state[0])}")
        else:
            lines.append("STATE unknown (no 0x87 received)")
        for kind, names in ((0x85, WHEELS), (0x86, ("A", "B"))):
            parts = []
            for idx, name in enumerate(names):
                sample = self.samples.get((kind, idx))
                if sample is None:
                    parts.append(f"{name}:unknown")
                    continue
                data, received = sample
                validity_bit = idx if kind == 0x85 else idx + 4
                validity = ("valid" if state[0]["validity"] & (1 << validity_bit) else "invalid") if state_fresh else "validity?"
                value = (f"rpm={data['rpm']} enc={data['encoder']} status=0x{data['status']:02X}"
                         if kind == 0x85 else f"{data['voltage']:.2f}V")
                parts.append(f"{name}:{value} {validity} age={now-received:.2f}s")
            lines.append(" | ".join(parts))
        return "\n".join(lines)


def parse_command(line: str, current: MotionCommand | None, observed_estop: bool = False):
    words = shlex.split(line)
    if not words:
        return "none", current
    name, *args = words
    name = name.lower()
    arity = {"drive": 2, "drive-rad": 2, "speed": 1, "steer": 1, "pivot": 1,
             "stop": 0, "disable": 0, "estop": 0, "clear-estop": 0,
             "pause": 0, "status": 0, "reset": 0, "help": 0, "quit": 0}
    name = {"q": "quit", "exit": "quit", "?": "help"}.get(name, name)
    if name not in arity or len(args) != arity[name]:
        raise ValueError("unknown command or wrong argument count; type help")
    if name in ("pause", "status", "reset", "help", "quit"):
        return name, current
    active_estop = observed_estop or bool(current and current.estop)
    if active_estop and name not in ("estop", "clear-estop", "disable"):
        raise ValueError("estop is active: use clear-estop, then wait for feedback before driving")
    target = current or MotionCommand()
    if name in ("drive", "drive-rad"):
        theta = float(args[1])
        target = MotionCommand(v=float(args[0]), theta=math.radians(theta) if name == "drive" else theta, enable=True)
    elif name in ("speed", "steer"):
        if target.pivot:
            raise ValueError("use drive V THETA_DEG to switch from PIVOT to normal drive")
        target = replace(target, enable=True, **({"v": float(args[0])} if name == "speed" else {"theta": math.radians(float(args[0]))}))
    elif name == "pivot":
        target = MotionCommand(w=float(args[0]), pivot=True, enable=True)
    elif name == "stop":
        target = replace(target.stopped(), enable=True)
    elif name == "disable":
        target = MotionCommand(estop=active_estop)
    elif name == "estop":
        target = MotionCommand(estop=True)
    elif name == "clear-estop":
        target = MotionCommand()
    target.packet(0)  # Validate before changing the target or opening a port.
    return "target", target


class Writer:
    def __init__(self, port, seq: int):
        self.port, self.seq = port, seq
        self.broken = False
        self.sent_motion = False

    def write(self, packet: bytes) -> None:
        try:
            count = self.port.write(packet)
            if count != len(packet):
                raise OSError(f"partial serial write ({count}/{len(packet)}); TX stopped")
        except (serial.SerialException, OSError, KeyboardInterrupt):
            self.broken = True
            raise

    def motion(self, target: MotionCommand) -> None:
        self.write(target.packet(self.seq))
        self.sent_motion = True
        self.seq = (self.seq + 1) & 0xffff


def run(args, initial: MotionCommand | None, interactive: bool) -> None:
    port_name = args.port or find_teensy_port()
    if not port_name:
        raise ValueError("no unique serial device; use --list-ports and --port")
    if interactive and (not sys.stdin.isatty() or not sys.stdout.isatty()):
        raise ValueError("interactive mode needs a terminal; use monitor or an initial command with --duration")
    parser, feedback = PacketStreamParser(), Feedback()
    with serial.Serial(port_name, args.baud, timeout=0, write_timeout=0.2) as port:
        with (ControlUi() if interactive else PlainUi()) as ui:
            writer = Writer(port, args.seq)
            target = initial
            tx_paused = target is None
            allow_clear_estop = args.command == "clear-estop"
            # Let the MCU's previous sender/sequence window expire; read feedback
            # throughout, without sending a command just to open the CLI.
            started = time.monotonic()
            ready_at = started + 0.6
            next_tx, next_status = ready_at, started
            end_at = ((ready_at if initial else started) + args.duration) if args.duration else math.inf
            resetting = args.command == "reset" or args.reset
            ui.log(f"Connected to {port_name}. Commands use 0x03; TX rate={args.rate_hz:g}Hz. Initial TX quiet time=0.6s.")
            if interactive:
                ui.log(HELP)
            try:
                if resetting:
                    writer.write(build_reset_packet(args.seq))
                    time.sleep(0.03)
                    ui.log("Reset frame written; no MCU acknowledgement is provided.")
                    return
                while True:
                    now = time.monotonic()
                    if now >= end_at:
                        break
                    if port.in_waiting:
                        for packet in parser.feed(port.read(min(port.in_waiting, 4096))):
                            try:
                                data = decode_telemetry(packet)
                            except ValueError as exc:
                                ui.log(f"RX ignored: {exc}")
                                continue
                            if data:
                                feedback.update(data, time.monotonic())
                    if interactive:
                        _, input_ready = wait_for_activity(port, ui, timeout_s=0.005)
                        if input_ready:
                            for line in ui.read_lines():
                                try:
                                    action, candidate = parse_command(line, target, feedback.estop_active())
                                    if action == "quit":
                                        return
                                    if action == "reset":
                                        resetting = True
                                        writer.write(build_reset_packet(writer.seq))
                                        time.sleep(0.03)
                                        ui.log("Reset frame written; no MCU acknowledgement is provided.")
                                        return
                                    if action == "help":
                                        ui.log(HELP)
                                    elif action == "status":
                                        ui.log(feedback.describe(time.monotonic()))
                                        ui.log("TX " + (target.describe() if target else "no target") + (" [paused]" if tx_paused else ""))
                                    elif action == "pause":
                                        tx_paused = True
                                        ui.log("Periodic TX paused. MCU timeout is measured from its last accepted command.")
                                    elif action == "target":
                                        target = candidate
                                        tx_paused = False
                                        allow_clear_estop = shlex.split(line)[0].lower() == "clear-estop"
                                        next_tx = max(ready_at, time.monotonic())
                                        ui.log("TX target " + target.describe())
                                except ValueError as exc:
                                    ui.log(f"Input rejected: {exc}. Previous target retained.")
                    else:
                        time.sleep(0.005)
                    now = time.monotonic()
                    if target is not None and not tx_paused and now >= next_tx:
                        if feedback.estop_active() and not target.estop and not allow_clear_estop:
                            if target.enable:
                                raise ValueError("MCU reports estop; use clear-estop before starting a drive command")
                            target = replace(target, estop=True)
                        writer.motion(target)
                        next_tx = now + 1.0 / args.rate_hz  # Never replay missed TX slots.
                        if args.once:
                            time.sleep(0.03)
                            ui.log("One frame written; no ACK. No exit command is sent in --once mode.")
                            return
                    if args.status_hz and now >= next_status:
                        ui.log(feedback.describe(now))
                        next_status = now + 1.0 / args.status_hz
            except (KeyboardInterrupt, ExitRequested):
                pass
            finally:
                if writer.sent_motion and not writer.broken and not resetting and not args.once:
                    try:
                        keep_estop = bool(target and target.estop) or feedback.estop_active()
                        writer.motion(MotionCommand(estop=keep_estop))
                        time.sleep(0.03)
                        ui.log(f"Exit request written: enable=0 estop={int(keep_estop)} (no MCU ACK).")
                    except (serial.SerialException, OSError, KeyboardInterrupt) as exc:
                        ui.log(f"Exit request could not be completed: {exc}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Windows/Linux MCU motion CLI. No command: interactive receive-only startup.")
    parser.add_argument("--port", "-p", help="e.g. COM3 or /dev/ttyACM0; unique-device detection if omitted")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seq", type=int, default=0, help="initial u16 sequence (default 0)")
    parser.add_argument("--rate-hz", type=float, default=50, help="periodic command rate, 5..100Hz (default 50)")
    parser.add_argument("--status-hz", type=float, default=2, help="feedback display rate, 0..10Hz; 0 disables periodic display")
    parser.add_argument("--duration", type=float, help="run time in seconds; initial-command duration starts after quiet time")
    parser.add_argument("--once", action="store_true", help="write one command only; no exit disable (MCU timeout still applies)")
    parser.add_argument("--dry-run", action="store_true", help="encode initial command/reset and exit without opening a port")
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--reset", action="store_true", help="alias for the reset command")
    parser.add_argument("command", nargs="?", choices=("drive", "drive-rad", "pivot", "stop", "disable", "estop", "clear-estop", "reset", "monitor"))
    parser.add_argument("values", nargs="*")
    args = parser.parse_args(argv)
    try:
        if args.list_ports:
            for port in serial.tools.list_ports.comports():
                print(f"{port.device}: {port.description} [{port.hwid}]")
            return 0
        if not 0 <= args.seq <= 65535 or args.baud <= 0:
            raise ValueError("--seq must be 0..65535 and --baud must be positive")
        if not math.isfinite(args.rate_hz) or not 5 <= args.rate_hz <= 100:
            raise ValueError("--rate-hz must be finite and 5..100")
        if not math.isfinite(args.status_hz) or not 0 <= args.status_hz <= 10:
            raise ValueError("--status-hz must be finite and 0..10")
        if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
            raise ValueError("--duration must be finite and positive")
        if args.reset and args.command is not None:
            raise ValueError("use either --reset or a command")
        if args.reset and args.values:
            raise ValueError("--reset does not take motion values")
        command = "reset" if args.reset else args.command
        initial = None
        if command not in (None, "monitor"):
            _, initial = parse_command(" ".join([command, *args.values]), None)
        elif args.values:
            raise ValueError("unexpected command values")
        if args.once and (initial is None or args.duration is not None):
            raise ValueError("--once needs a motion command and cannot be combined with --duration")
        if args.dry_run:
            if initial is None and command != "reset":
                raise ValueError("--dry-run needs a motion command or reset")
            packet = build_reset_packet(args.seq) if command == "reset" else initial.packet(args.seq)
            print(packet.hex(" ").upper())
            return 0
        run(args, initial, interactive=command is None)
        return 0
    except KeyboardInterrupt:
        return 0
    except (ValueError, OSError, serial.SerialException) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
