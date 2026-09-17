import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import can_cli


def make_packet(payload: bytes) -> bytes:
    packet = can_cli.HEADER + bytes([len(payload)]) + payload
    return packet + bytes([can_cli.xor_checksum(packet)])


def make_benchmark_result(
    seq: int,
    iteration: int,
    status: int,
    final: bool,
    rtt_us: int,
    timeout_count: int = 0,
    tx_failure_count: int = 0,
    tx_timestamp_us: int = 100,
    response_pids: tuple[int, ...] = (193,),
    rtt2_us: int = 0,
    received_mask: int | None = None,
) -> bytes:
    response_count = len(response_pids)
    if received_mask is None:
        received_mask = (1 << response_count) - 1 if status == 0 else 0
    rx_timestamp_us = (
        (tx_timestamp_us + rtt_us) & 0xFFFFFFFF if received_mask & 0x01 else 0
    )
    rx_timestamp2_us = (
        (tx_timestamp_us + rtt2_us) & 0xFFFFFFFF if received_mask & 0x02 else 0
    )
    response1 = bytes(
        [8 if received_mask & 0x01 else 0]
        + ([response_pids[0], 0, 0, 0, 0, 0, 0, 0] if received_mask & 0x01 else [0] * 8)
    )
    response2_pid = response_pids[1] if response_count == 2 else 0
    response2 = bytes(
        [8 if received_mask & 0x02 else 0]
        + ([response2_pid, 0, 0, 0, 0, 0, 0, 0] if received_mask & 0x02 else [0] * 8)
    )
    payload = (
        bytes([can_cli.CAN_BENCHMARK_RESULT_TYPE, seq])
        + iteration.to_bytes(2, "little")
        + bytes([status, int(final)])
        + tx_timestamp_us.to_bytes(4, "little")
        + rx_timestamp_us.to_bytes(4, "little")
        + (rtt_us if received_mask & 0x01 else 0).to_bytes(4, "little")
        + timeout_count.to_bytes(2, "little")
        + tx_failure_count.to_bytes(2, "little")
        + (0x701 if received_mask else 0).to_bytes(2, "little")
        + response1
        + bytes([response_count, received_mask])
        + rx_timestamp2_us.to_bytes(4, "little")
        + (rtt2_us if received_mask & 0x02 else 0).to_bytes(4, "little")
        + response2
    )
    return make_packet(payload)


class BridgeRequestTests(unittest.TestCase):
    def test_existing_0x20_layout_is_preserved(self) -> None:
        request = can_cli.build_bridge_request(
            0x001,
            [0xCF, 0x01, 0x64, 0x00, 0x01, 0xB0, 0xFF, 0x00],
            8,
            3,
            100,
        )

        self.assertEqual(len(request), 19)
        self.assertEqual(
            request[:-1],
            bytes.fromhex(
                "AA 55 0F 20 03 01 00 08 CF 01 64 00 01 B0 FF 00 64 00"
            ),
        )
        self.assertEqual(request[-1], can_cli.xor_checksum(request[:-1]))


class BenchmarkRequestTests(unittest.TestCase):
    def test_benchmark_layout_and_md200t_matcher(self) -> None:
        command = can_cli.parse_interactive_command(
            "benchmark 1 4 143 --dlc 8 --count 3 --period-ms 10 --timeout-ms 5", 100
        )

        self.assertIsInstance(command, can_cli.BenchmarkCommand)
        self.assertEqual(command.response_can_id, 0x701)
        self.assertEqual(command.response_pids, (143,))
        request = can_cli.build_benchmark_request(command, 7)
        self.assertEqual(len(request), 28)
        self.assertEqual(
            request[:-1],
            bytes.fromhex(
                "AA 55 18 21 07 01 00 08 04 8F 00 00 00 00 00 00 "
                "01 07 8F 03 00 0A 00 05 00 01 00"
            ),
        )
        self.assertEqual(request[-1], can_cli.xor_checksum(request[:-1]))

    def test_timeout_must_fit_period(self) -> None:
        with self.assertRaisesRegex(can_cli.CommandError, "no greater than"):
            can_cli.parse_interactive_command(
                "benchmark 1 4 193 --period-ms 5 --timeout-ms 6", 100
            )

    def test_explicit_matcher_supports_raw_request(self) -> None:
        command = can_cli.parse_interactive_command(
            "benchmark 0x123 --response-id 0x456 --response-pid 9 --count 1", 10
        )

        self.assertEqual(command.response_can_id, 0x456)
        self.assertEqual(command.response_pids, (9,))

    def test_velocity_command_defaults_to_two_automatic_responses(self) -> None:
        command = can_cli.parse_interactive_command(
            "benchmark 1 207 1 100 0 1 176 255 0 --count 2", 10
        )

        self.assertEqual(command.response_can_id, 0x701)
        self.assertEqual(command.response_pids, (193, 200))
        request = can_cli.build_benchmark_request(command, 9)
        self.assertEqual(request[18], 193)
        self.assertEqual(request[25], 2)
        self.assertEqual(request[26], 200)

    def test_interactive_default_timeout_is_clamped_to_period(self) -> None:
        command = can_cli.parse_interactive_command("benchmark 1 4 193", 100)

        self.assertEqual(command.period_ms, 10)
        self.assertEqual(command.timeout_ms, 10)

    def test_initial_benchmark_has_usable_timeout_default(self) -> None:
        args = can_cli.build_main_parser().parse_args(
            ["benchmark", "1", "4", "193", "--port", "/dev/ttyACM0"]
        )
        command = can_cli.command_from_namespace(args, args.timeout_ms)

        self.assertEqual(command.timeout_ms, 10)


class BenchmarkStatisticsTests(unittest.TestCase):
    def test_final_result_prints_session_rtt_statistics(self) -> None:
        stats = can_cli.BenchmarkStatistics()
        stats.start(seq=7, expected_count=3, response_pids=(193,))

        self.assertIsNone(
            stats.consume(make_benchmark_result(7, 1, 0, False, 100, tx_timestamp_us=100))
        )
        self.assertIsNone(
            stats.consume(
                make_benchmark_result(
                    7, 2, 2, False, 0, timeout_count=1, tx_timestamp_us=1100
                )
            )
        )
        summary = stats.consume(
            make_benchmark_result(
                7, 3, 0, True, 300, timeout_count=1, tx_timestamp_us=2100
            )
        )

        self.assertIsNotNone(summary)
        self.assertIn(
            "attempts=3/3 complete_rounds=2 partial_rounds=0 timeouts=1 tx_failures=0",
            summary,
        )
        self.assertIn("success_rate=66.67%", summary)
        self.assertIn("pid193_rtt_us[n=2 min=100 avg=200.0 p50=200.0", summary)
        self.assertIn("p95=290.0 p99=298.0 max=300 stddev=100.0]", summary)
        self.assertIn(
            "tx_interval_us[n=2 min=1000 avg=1000.0 p50=1000.0", summary
        )
        self.assertIn(
            "pid193_rx_interval_us[n=1 min=2200 avg=2200.0 p50=2200.0", summary
        )
        self.assertIsNone(stats.seq)

    def test_new_session_resets_samples_and_other_sequence_is_ignored(self) -> None:
        stats = can_cli.BenchmarkStatistics()
        stats.start(seq=1, expected_count=2, response_pids=(193,))
        stats.consume(make_benchmark_result(1, 1, 0, False, 500))

        stats.start(seq=2, expected_count=1, response_pids=(193,))
        self.assertEqual(stats.rtt_samples_us, [[], []])
        self.assertIsNone(stats.consume(make_benchmark_result(1, 2, 0, True, 900)))
        summary = stats.consume(make_benchmark_result(2, 1, 1, True, 0, tx_failure_count=1))

        self.assertIn("attempts=1/1 complete_rounds=0", summary)
        self.assertIn("pid193_rtt_us[n=0]", summary)
        self.assertIn("tx_interval_us[n=0]", summary)
        self.assertIn("pid193_rx_interval_us[n=0]", summary)

    def test_timestamp_intervals_handle_micros_wrap(self) -> None:
        stats = can_cli.BenchmarkStatistics()
        stats.start(seq=3, expected_count=2, response_pids=(193,))
        stats.consume(
            make_benchmark_result(3, 1, 0, False, 100, tx_timestamp_us=0xFFFFFF00)
        )
        summary = stats.consume(
            make_benchmark_result(3, 2, 0, True, 200, tx_timestamp_us=0x000002E8)
        )

        self.assertIn("tx_interval_us[n=1 min=1000", summary)
        self.assertIn("pid193_rx_interval_us[n=1 min=1100", summary)

    def test_dual_response_statistics_include_round_and_partial_results(self) -> None:
        stats = can_cli.BenchmarkStatistics()
        stats.start(seq=4, expected_count=2, response_pids=(193, 200))
        stats.consume(
            make_benchmark_result(
                4,
                1,
                0,
                False,
                200,
                tx_timestamp_us=1000,
                response_pids=(193, 200),
                rtt2_us=450,
            )
        )
        summary = stats.consume(
            make_benchmark_result(
                4,
                2,
                2,
                True,
                300,
                timeout_count=1,
                tx_timestamp_us=11000,
                response_pids=(193, 200),
                received_mask=1,
            )
        )

        self.assertIn("complete_rounds=1 partial_rounds=1", summary)
        self.assertIn("pid193_rtt_us[n=2", summary)
        self.assertIn("pid200_rtt_us[n=1", summary)
        self.assertIn("round_rtt_us[n=1 min=450", summary)
        self.assertIn("response_gap_us[n=1 min=250", summary)

class PacketStreamParserTests(unittest.TestCase):
    def test_fragmented_and_back_to_back_packets(self) -> None:
        first = make_packet(bytes.fromhex("A0 01 00 01 07 02 C1 00 00 00 00 00 00 00"))
        second = make_packet(bytes.fromhex("82 01"))
        parser = can_cli.PacketStreamParser()

        self.assertEqual(parser.feed(first[:5]), [])
        self.assertEqual(parser.feed(first[5:] + second), [first, second])

    def test_noise_and_bad_checksum_do_not_hide_next_packet(self) -> None:
        bad = bytearray(make_packet(bytes.fromhex("82 01")))
        bad[-1] ^= 0xFF
        good = make_packet(bytes.fromhex("A0 02 02 00 00 00 00 00 00 00 00 00 00 00"))
        parser = can_cli.PacketStreamParser()

        self.assertEqual(parser.feed(b"noise" + bad + good), [good])

    def test_bridge_frames_are_not_filtered_by_sequence_or_can_id(self) -> None:
        first = make_packet(bytes.fromhex("A0 11 00 01 07 01 C1 00 00 00 00 00 00 00"))
        second = make_packet(bytes.fromhex("A0 FE 00 23 01 01 8F 00 00 00 00 00 00 00"))
        parser = can_cli.PacketStreamParser()

        self.assertEqual(parser.feed(first + second), [first, second])
        self.assertIn("seq=17 id=0x701", can_cli.format_packet(first))
        self.assertIn("seq=254 id=0x123", can_cli.format_packet(second))

    def test_benchmark_result_format(self) -> None:
        payload = (
            bytes([0xA1, 7])
            + (2).to_bytes(2, "little")
            + bytes([0, 1])
            + (100).to_bytes(4, "little")
            + (145).to_bytes(4, "little")
            + (45).to_bytes(4, "little")
            + (1).to_bytes(2, "little")
            + (0).to_bytes(2, "little")
            + (0x701).to_bytes(2, "little")
            + bytes([2, 0x8F, 0x12, 0, 0, 0, 0, 0, 0])
            + bytes([1, 1])
            + (0).to_bytes(4, "little")
            + (0).to_bytes(4, "little")
            + bytes(9)
        )
        packet = make_packet(payload)

        formatted = can_cli.format_packet(packet)
        self.assertIn("BENCH seq=7 iter=2 status=response final", formatted)
        self.assertIn("received=1/1 timeouts=1 tx_failures=0 id=0x701", formatted)
        self.assertIn("round_rtt_us=45", formatted)
        self.assertIn("rsp1[pid=143 rx_us=145 rtt_us=45 dlc=2 data=8F 12]", formatted)


class InteractiveCommandTests(unittest.TestCase):
    def test_pid_command(self) -> None:
        command = can_cli.parse_interactive_command(
            "pid 1 207 1 0x64 0 1 0xB0 0xFF 0", 100
        )

        self.assertEqual(command.can_id, 0x001)
        self.assertEqual(command.data, [0xCF, 1, 0x64, 0, 1, 0xB0, 0xFF, 0])
        self.assertEqual(command.dlc, 8)
        self.assertEqual(command.timeout_ms, 100)

    def test_raw_frame_command_and_timeout(self) -> None:
        command = can_cli.parse_interactive_command(
            "frame 0x701 0xC1 --dlc 8 --timeout-ms 25", 100
        )

        self.assertEqual(command.can_id, 0x701)
        self.assertEqual(command.data, [0xC1])
        self.assertEqual(command.dlc, 8)
        self.assertEqual(command.timeout_ms, 25)

    def test_exit_commands(self) -> None:
        for command in ("q", "quit", "exit"):
            with self.subTest(command=command), self.assertRaises(can_cli.ExitRequested):
                can_cli.parse_interactive_command(command, 100)

    def test_initial_command_accepts_port_after_subcommand(self) -> None:
        args = can_cli.build_main_parser().parse_args(
            ["pid", "1", "143", "--port", "/dev/ttyACM0"]
        )

        self.assertEqual(args.port, "/dev/ttyACM0")
        self.assertEqual(can_cli.command_from_namespace(args, args.timeout_ms).data[0], 143)


if __name__ == "__main__":
    unittest.main()
