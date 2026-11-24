"""Odometry must not integrate a rounded display speed."""
import struct
import unittest

from core.decoder import RealTeslaSEIDecoder


class DecoderSpeedPrecisionTests(unittest.TestCase):
    def test_sei_float_speed_survives_display_rounding(self):
        decoder = RealTeslaSEIDecoder.__new__(RealTeslaSEIDecoder)
        value = 0.08888864
        payload = bytes([4 << 3 | 5]) + struct.pack('<f', value)
        result = decoder.parse_protobuf_wire(payload)
        self.assertEqual(result['speed_kmh'], 0)
        self.assertAlmostEqual(result['speed_mps'], value, places=7)

    def test_missing_speed_is_stationary(self):
        decoder = RealTeslaSEIDecoder.__new__(RealTeslaSEIDecoder)
        self.assertEqual(decoder.parse_protobuf_wire(b'')['speed_mps'], 0.)

    def test_sequence_is_preserved_and_missing_is_not_zero(self):
        decoder = RealTeslaSEIDecoder.__new__(RealTeslaSEIDecoder)
        self.assertIsNone(decoder.parse_protobuf_wire(b'')['frame_seq_no'])
        self.assertEqual(decoder.parse_protobuf_wire(bytes([24, 129, 1]))['frame_seq_no'], 129)
