import unittest
from ct_scene.wheel_scale import SpeedTrace,unique_motion_rows


def row(t,speed):
    return {'camera_time_s':t,'fields':{'vehicle_speed_mps':{'valid':True,'value':speed}}}


class WheelScaleTests(unittest.TestCase):
    def test_consistent_same_pts_messages_are_one_sample_without_mutating_raw(self):
        raw=[row(0,19.38),row(0,19.36),row(.1,19.37)];trace=SpeedTrace(raw)
        self.assertEqual(len(trace.times),2);self.assertEqual(raw[0]['fields']['vehicle_speed_mps']['value'],19.38)
        self.assertAlmostEqual(trace.distance(0,.1),1.937)

    def test_conflicting_same_pts_motion_is_not_averaged_away(self):
        with self.assertRaisesRegex(ValueError,'Conflicting duplicate'):SpeedTrace([row(0,20),row(0,-20),row(.1,20)])
        a=row(0,1);b=row(0,1);a['fields']['gear_state']=dict(valid=True,value=1);b['fields']['gear_state']=dict(valid=True,value=2)
        with self.assertRaisesRegex(ValueError,'gear_state'):unique_motion_rows([a,b])

    def test_sub_kmh_travel_not_rounded_to_zero(self):
        trace=SpeedTrace([row(i/10,.08888864) for i in range(61)])
        self.assertAlmostEqual(trace.distance(0,6),.53333184)
        self.assertAlmostEqual(trace.distance(.05,1.55),.08888864*1.5)

    def test_missing_intervals_and_extrapolation_rejected(self):
        trace=SpeedTrace([row(0,1),row(.1,1),row(.9,1),row(1,1)])
        with self.assertRaises(ValueError):trace.distance(0,1)
        with self.assertRaises(ValueError):trace.distance(-.1,.1)

    def test_negative_noise_is_not_modified_by_parser_or_integrator(self):
        trace=SpeedTrace([row(0,-2.5e-7),row(.1,-2.5e-7)])
        self.assertAlmostEqual(trace.distance(0,.1),-2.5e-8)
