import unittest
import json,numpy as np
from ct_scene.ego_lamps import LampTrace,role


def row(t,left=False,right=False,brake=False,gear=1):
    return dict(camera_time_s=t,status='decoded_v1',fields={k:dict(value=v,valid=v is not None) for k,v in [('blinker_on_left',left),('blinker_on_right',right),('brake_applied',brake),('gear_state',gear)]})


class LampTests(unittest.TestCase):
    def test_numpy_render_clock_serializes_as_normal_json(self):
        state=LampTrace([row(0,left=True)]).at(np.float64(.1))
        self.assertIs(type(state['left_lit']),bool);json.dumps(state,allow_nan=False)

    def test_no_future_boolean_interpolation_and_stale_is_unknown(self):
        trace=LampTrace([row(0),row(.1,left=True)])
        self.assertFalse(trace.at(.05)['left']);self.assertTrue(trace.at(.1)['left'])
        self.assertIsNone(trace.at(-.01)['brake']);self.assertIsNone(trace.at(.5)['left'])

    def test_flash_is_separate_from_recorded_indicator_state(self):
        trace=LampTrace([row(i*.1,left=True) for i in range(15)])
        self.assertTrue(trace.at(.1)['left_lit']);self.assertFalse(trace.at(.4)['left_lit'])
        self.assertTrue(trace.at(.4)['left']);self.assertEqual(trace.at(.4)['flash_phase'],'illustrative_not_measured')

    def test_invalid_brake_and_gear_are_not_fabricated_false(self):
        trace=LampTrace([row(0,brake=None,gear=None)])
        self.assertIsNone(trace.at(0)['brake']);self.assertIsNone(trace.at(0)['reverse'])
        self.assertTrue(LampTrace([row(0,brake=True,gear=2)]).at(0)['brake'])
        self.assertTrue(LampTrace([row(0,gear=2)]).at(0)['reverse'])

    def test_both_indicators_and_material_sides_remain_independent(self):
        state=LampTrace([row(0,left=True,right=True)]).at(0)
        self.assertTrue(state['left_lit'] and state['right_lit'])
        self.assertEqual(role('TurnSignal_RL','TurnSignalAmberLED'),'left')
        self.assertEqual(role('TurnSignal_RR','TurnSignalAmberLED'),'right')
        self.assertIsNone(role('Wheel_RL','AeroWheelCharcoal'))
