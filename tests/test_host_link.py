"""PC codec checks; same golden command is checked by actual firmware C tests."""
from pathlib import Path
import struct
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parent))
from host_protocol_reference import Parser, encode, velocity, decode_status, ARM, STATUS


class HostLinkTests(unittest.TestCase):
    def test_shared_golden_and_fragmentation(self):
        golden = bytes.fromhex("a55a01010c00ffffffffceffffff19000000d4feffff4791")
        self.assertEqual(velocity(0xFFFFFFFF, -.05, .025, -.3), golden)
        parser = Parser()
        frames = []
        for byte in b"noise\xa5" + golden + encode(ARM, 0):
            frames.extend(parser.feed(bytes([byte])))
        self.assertEqual(frames, [(1, 0xFFFFFFFF, struct.pack("<iii", -50, 25, -300)), (ARM, 0, b"")])

    def test_corruption_noise_and_limits(self):
        corrupt = bytearray(velocity(1, .05))
        corrupt[10] ^= 1
        parser = Parser()
        self.assertEqual(parser.feed(bytes(corrupt) + b"\xa5\x5a\x01\x01\xff\xff" + encode(ARM, 2)), [(ARM, 2, b"")])
        self.assertEqual(parser.crc_errors, 1)
        for args in ((2147483.648, 0, 0), (0, -2147483.649, 0), (0, 0, float("inf")), (float("nan"), 0, 0)):
            with self.assertRaises(ValueError):
                velocity(1, *args)

    def test_large_velocity_wire_range(self):
        parser = Parser()
        frames = parser.feed(velocity(0, .5, -.3, 1.0))
        self.assertEqual(frames, [(1, 0, struct.pack("<iii", 500, -300, 1000))])
        frames = parser.feed(velocity(0, -2147483.648, 2147483.647, 0))
        self.assertEqual(frames[0][2], struct.pack("<iii", -2147483648, 2147483647, 0))

    def test_status_layout_and_unavailable(self):
        payload = bytearray(196)
        struct.pack_into("<I4B", payload, 0, 1234, 2, 0, 1, 1)
        struct.pack_into("<iii", payload, 64, -2147483648, 25, -300)
        struct.pack_into("<qiIII", payload, 100, -21636, 900, 1200, 0b11111, 1190)
        parser = Parser()
        frame = encode(STATUS, 0x12345678, payload)
        self.assertEqual(len(frame), 208)
        self.assertEqual(parser.feed(frame[:64]), [])
        frames = parser.feed(frame[64:] * 1)
        status = decode_status(frames[0][2])
        self.assertEqual(status["state_name"], "ACTIVE")
        self.assertEqual(status["measured"], [None, .025, -.3])
        self.assertEqual(status["wheels"][0]["position_raw"], -21636)
        self.assertEqual(status["wheels"][0]["speed_rad_s"], .9)
        self.assertTrue(status["wheels"][0]["enabled"])
        with self.assertRaises(ValueError):
            decode_status(payload[:-1])


if __name__ == "__main__":
    unittest.main()
