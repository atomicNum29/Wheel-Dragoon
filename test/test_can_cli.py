import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import can_cli


def make_packet(payload: bytes) -> bytes:
    packet = can_cli.HEADER + bytes([len(payload)]) + payload
    return packet + bytes([can_cli.xor_checksum(packet)])


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
