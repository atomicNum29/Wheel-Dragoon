"""USB wire codec shared by the host CLIs; no device I/O."""

import math
import struct
from dataclasses import dataclass, replace

HEADER = b"\xaa\x55"
SIZES = {0x03: 15, 0x85: 20, 0x86: 15, 0x87: 29, 0x84: 63, 0xA0: 18, 0xA1: 56}
MODES = ("STOP", "MANUAL", "AUTO", "POWERDOWN", "PIVOT", "FAILSAFE")
TRANSITIONS = ("STARTUP", "DRIVING", "BRAKING", "STEERING", "SETTLING", "EMERGENCY_STOP")
STATES = {0: "DISABLED", 1: "ENABLED", 2: "TIMEOUT_STOP", 3: "ESTOP", 4: "FAULT", 5: "BOOTING", 7: "POWERDOWN"}
WHEELS = ("LF", "RF", "LR", "RR")
# Current firmware geometry: L=.420m, Tf=.439m, inner-wheel limit=70 degrees.
THETA_LIMIT_RAD = 0.845


def xor_checksum(data: bytes | bytearray) -> int:
    result = 0
    for byte in data:
        result ^= byte
    return result


def extract_packets(buffer: bytearray) -> list[bytes]:
    packets = []
    while True:
        start = buffer.find(HEADER)
        if start < 0:
            buffer[:] = b"\xaa" if buffer.endswith(b"\xaa") else b""
            break
        del buffer[:start]
        if len(buffer) < 4:
            break
        size = buffer[2] + 4
        if not 1 <= buffer[2] <= 64 or (buffer[3] in SIZES and SIZES[buffer[3]] != size):
            del buffer[0]
            continue
        if len(buffer) < size:
            break
        packet = bytes(buffer[:size])
        if xor_checksum(packet) != 0:
            del buffer[0]
            continue
        packets.append(packet)
        del buffer[:size]
    return packets


class PacketStreamParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        self.buffer.extend(data)
        return extract_packets(self.buffer)


@dataclass(frozen=True)
class MotionCommand:
    v: float = 0.0
    theta: float = 0.0  # radians, virtual front-center wheel
    w: float = 0.0
    pivot: bool = False
    enable: bool = False
    estop: bool = False

    def packet(self, seq: int) -> bytes:
        if not 0 <= seq <= 0xffff:
            raise ValueError("sequence must be 0..65535")
        if not all(math.isfinite(value) for value in (self.v, self.theta, self.w)):
            raise ValueError("motion values must be finite")
        if not (-2 <= self.v <= 2 and abs(self.theta) <= THETA_LIMIT_RAD and abs(self.w) <= 5):
            raise ValueError(f"limits: |v|<=2 m/s, |theta|<={THETA_LIMIT_RAD} rad ({math.degrees(THETA_LIMIT_RAD):.4f} deg), |w|<=5 rad/s")
        if (self.pivot and (self.v != 0 or self.theta != 0)) or (not self.pivot and self.w != 0):
            raise ValueError("PIVOT requires v=theta=0; normal drive requires w=0")
        flags = int(self.enable) | (int(self.estop) << 1) | (int(self.pivot) << 2)
        packet = HEADER + struct.pack("<BBBHBhhh", 11, 0x03, 1, seq, flags,
                                      round(self.v * 1000), round(self.theta * 1000), round(self.w * 1000))
        return packet + bytes([xor_checksum(packet)])

    def stopped(self) -> "MotionCommand":
        return replace(self, v=0.0, w=0.0)

    def describe(self) -> str:
        motion = f"PIVOT w={self.w:.3f} rad/s" if self.pivot else f"DRIVE v={self.v:.3f} m/s theta={math.degrees(self.theta):.3f} deg"
        return f"{motion} enable={int(self.enable)} estop={int(self.estop)}"


def build_reset_packet(seq: int = 0) -> bytes:
    packet = HEADER + bytes([4, 0x02, seq & 0xff, 0xa5, 0x5a])
    return packet + bytes([xor_checksum(packet)])


def decode_telemetry(packet: bytes) -> dict | None:
    if len(packet) < 5 or packet[:2] != HEADER or len(packet) != packet[2] + 4 or xor_checksum(packet):
        raise ValueError("invalid frame")
    kind = packet[3]
    if kind not in (0x85, 0x86, 0x87):
        return None
    if len(packet) != SIZES[kind] or packet[4] != 1:
        raise ValueError(f"unsupported telemetry layout/version: 0x{kind:02X}")
    u16 = lambda offset: struct.unpack_from("<H", packet, offset)[0]
    result = dict(type=kind, seq=u16(5))
    if kind in (0x85, 0x86):
        if packet[7] >= (4 if kind == 0x85 else 2):
            raise ValueError("invalid telemetry ID")
        result.update(id=packet[7], rx_us=struct.unpack_from("<I", packet, 8)[0])
        if kind == 0x85:
            result.update(status=packet[12], rpm=struct.unpack_from("<h", packet, 13)[0],
                          encoder=struct.unpack_from("<i", packet, 15)[0])
        else:
            result.update(voltage=u16(12) / 1000)
        return result
    mode, transition = packet[8] & 15, packet[8] >> 4
    if mode >= len(MODES) or transition >= len(TRANSITIONS):
        raise ValueError("unsupported mode/transition")
    result.update(state=packet[7], mode=mode, transition=transition, flags=u16(9),
                  errors=u16(11), validity=packet[13], session=packet[27], applied_mode=packet[26],
                  requested=tuple(value / 1000 for value in struct.unpack_from("<hhh", packet, 14)),
                  applied=tuple(value / 1000 for value in struct.unpack_from("<hhh", packet, 20)))
    return result


def describe_motion(values: tuple, pivot: bool) -> str:
    v, theta, w = values
    return f"PIVOT w={w:.3f}rad/s" if pivot else f"v={v:.3f}m/s theta={math.degrees(theta):.2f}deg"


def format_telemetry(data: dict) -> str:
    if data["type"] == 0x85:
        return (f"WHEEL {WHEELS[data['id']]} seq={data['seq']} rx_us={data['rx_us']} "
                f"rpm={data['rpm']} encoder={data['encoder']} status=0x{data['status']:02X}")
    if data["type"] == 0x86:
        return f"VOLTAGE {'AB'[data['id']]} seq={data['seq']} rx_us={data['rx_us']} V={data['voltage']:.2f}"
    flags = data["flags"]
    requested = describe_motion(data["requested"], bool(flags & (1 << 12))) if flags & (1 << 4) else "invalid"
    applied = describe_motion(data["applied"], data["applied_mode"] == 4) if flags & (1 << 11) else "invalid"
    return (f"STATE session={data['session']} seq={data['seq']} mode={MODES[data['mode']]} "
            f"state={STATES.get(data['state'], 'UNKNOWN')} motion={TRANSITIONS[data['transition']]} "
            f"gate={int(bool(flags & (1 << 9)))} estop={int(bool(flags & (1 << 5)))} "
            f"timeout={int(bool(flags & (1 << 6)))} rc_signal_lost={int(bool(flags & (1 << 13)))} errors=0x{data['errors']:04X} "
            f"flags=0x{flags:04X} valid=0x{data['validity']:02X} request[{requested}] applied[{applied}]")
