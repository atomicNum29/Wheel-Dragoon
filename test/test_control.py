import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import control


class ResetPacketTests(unittest.TestCase):
    def test_reset_packet_layout_and_checksum(self) -> None:
        packet = control.build_reset_packet(seq=0)

        self.assertEqual(packet, bytes.fromhex("AA 55 04 02 00 A5 5A 06"))
        self.assertEqual(packet[-1], control.xor_checksum(packet[:-1]))

    def test_reset_sequence_wraps_to_one_byte(self) -> None:
        packet = control.build_reset_packet(seq=0x1FF)

        self.assertEqual(packet[4], 0xFF)
        self.assertEqual(packet[-1], control.xor_checksum(packet[:-1]))


if __name__ == "__main__":
    unittest.main()
