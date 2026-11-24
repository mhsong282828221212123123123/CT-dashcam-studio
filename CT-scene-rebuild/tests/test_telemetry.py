import struct
import unittest
from ct_scene.telemetry import decode_metadata, sei_messages, nal_units


class TelemetryTests(unittest.TestCase):
    def test_float_and_schema_default_are_not_missing_payload(self):
        row = decode_metadata(b'\x08\x01\x25'+struct.pack('<f', .08888864))
        self.assertAlmostEqual(row['fields']['vehicle_speed_mps']['value'], .08888864, places=7)
        self.assertEqual(row['fields']['brake_applied'], {'value':False,'valid':True,'presence':'schema_default'})
        self.assertIsNone(row['steering_wheel_rad'])

    def test_unsupported_schema_never_defaults_to_valid(self):
        self.assertEqual(decode_metadata(b'\x08\x02')['fields'], {})
        self.assertEqual(decode_metadata(b'')['status'], 'unsupported_version')

    def test_negative_finite_speed_is_retained_not_missing_or_clamped(self):
        row=decode_metadata(b'\x08\x01\x25'+struct.pack('<f',-2.5e-7))
        self.assertTrue(row['fields']['vehicle_speed_mps']['valid'])
        self.assertLess(row['fields']['vehicle_speed_mps']['value'],0)
        self.assertIn('negative_speed_requires_motion_interpretation',row['quality_flags'])

    def test_truncation_wrong_wire_and_invalid_values(self):
        for bad in (b'\x08\x80', b'\x08\x01\x25\x00', b'\x08\x01\x21'+b'\x00'*8):
            with self.assertRaises(ValueError):
                decode_metadata(bad)
        row = decode_metadata(b'\x08\x01\x25'+struct.pack('<f', float('nan')))
        self.assertIsNone(row['fields']['vehicle_speed_mps']['value'])
        self.assertFalse(row['fields']['vehicle_speed_mps']['valid'])

    def test_emulation_prevention_and_vendor_prefix(self):
        raw = b'BBBi\x08\x01\x18\x01\x25'+struct.pack('<f', 0)
        escaped = bytearray()
        zero = 0
        for b in raw:
            if zero >= 2 and b <= 3:
                escaped.append(3)
                zero = 0
            escaped.append(b)
            zero = zero+1 if b == 0 else 0
        nal = bytes([6,5,len(raw)])+bytes(escaped)+b'\x80'
        row = sei_messages(nal)[0]
        self.assertEqual(row['fields']['vehicle_speed_mps']['value'], 0)
        self.assertEqual(sei_messages(b'\x06\x05\x03ABC\x80'), [])

    def test_nal_length_is_not_always_four(self):
        self.assertEqual(list(nal_units(b'\x00\x02AB\x00\x01C',2)), [b'AB',b'C'])
        with self.assertRaises(ValueError):
            list(nal_units(b'\x00\x04AB',2))
