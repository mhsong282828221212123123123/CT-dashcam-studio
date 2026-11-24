import unittest
from ct_scene.surrounding_lamps import confirmed_samples,RearAppearanceTrace,rear_material


class ObservedRearLampTests(unittest.TestCase):
    def test_repeated_distinct_source_frames_required(self):
        rows=[dict(time_s=i*.2,frame_id=str(i)) for i in range(3)]
        self.assertEqual(len(confirmed_samples(rows)),3)
        self.assertEqual(confirmed_samples(rows[:2]),[])
        self.assertEqual(confirmed_samples([rows[0]]*3),[])
        self.assertEqual(confirmed_samples([dict(time_s=i*2,frame_id=str(i)) for i in range(3)]),[])

    def test_no_brake_or_blink_state_fabricated_no_ghost_or_gap_hold(self):
        rows=confirmed_samples([dict(time_s=i*.2,frame_id=str(i)) for i in range(3)])
        trace=RearAppearanceTrace({'a':rows})
        state=trace.at('a',.25)
        self.assertTrue(state['rear_red_visible'])
        self.assertTrue(all(state[k] is None for k in ('brake','left','right')))
        for key,t,ghost in [('a',1,False),('a',.25,True),('b',.25,False)]:
            self.assertIsNone(trace.at(key,t,ghost)['rear_red_visible'])

    def test_only_audited_rear_materials_selected(self):
        self.assertTrue(rear_material('FSDTaillight'));self.assertTrue(rear_material('FSDRedSignal'))
        self.assertFalse(rear_material('FSDBodyMatte'));self.assertFalse(rear_material('FSDHeadlight'))
