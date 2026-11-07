"""Decode the actual C++ profiler fixture; no serial connection is opened."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import can_cli


def check(path: Path) -> None:
    raw = path.read_bytes()
    parser = can_cli.PacketStreamParser()
    packets = []
    # Deliberately cut across record boundaries like fragmented USB reads.
    for offset in range(0, len(raw), 17):
        packets.extend(parser.feed(raw[offset:offset + 17]))
    record_count = 1 + len(can_cli.TIMING_STAGES) + 4 + 2
    assert len(packets) == record_count
    assert len(raw) == record_count * 63
    lines = [can_cli.format_packet(packet) for packet in packets]
    assert "rx_poll_gap_us[avg=100.00 max=100 n=1]" in lines[0]
    assert "overrun_observations=7" in lines[0]
    assert "stage=motion n=1 avg_us=2.000 max_us=2.000 total_ms=0.002" in lines[8]
    wheel_start = 1 + len(can_cli.TIMING_STAGES)
    assert "A/PID201 received=2 accepted=0 outside_window=1 short_frame=1" in lines[wheel_start + 1]
    assert "driver=A timeouts=1 pending_at_timeout=1 last_missing_mask=0x08" in lines[wheel_start + 4]
    assert "last_age_us=3010" in lines[wheel_start + 4]
    assert "snapshot=1 at_ms=1007" in lines[-1]
    assert not parser.buffer
    print(f"C++ to Python timing fixture: {record_count} records passed")


if __name__ == "__main__":
    check(Path(sys.argv[1]))
