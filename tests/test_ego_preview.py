import unittest
import numpy as np
from core.scene_preview import EgoTimeline, model, render_ego


class EgoPreviewTests(unittest.TestCase):
    def test_distance_does_not_bridge_missing_data(self):
        rows=[{'time_s':i*.2,'ego':{'data':{'speed_kmh':36}} if i!=2 else None} for i in range(5)]
        states=EgoTimeline(rows).states
        self.assertAlmostEqual(states[1][2],2/.343)
        self.assertEqual(states[1][2],states[3][2])
        self.assertAlmostEqual(states[4][2],4/.343)

    def test_gap_is_not_interpolated(self):
        rows=[{'time_s':t,'ego':{'data':{'speed_kmh':36}}} for t in [0,5]]
        self.assertEqual(EgoTimeline(rows).states[1][2],0)

    def test_wheels_rotate_without_rotating_calipers(self):
        asset=model(); still,moving=asset.transforms(0),asset.transforms(1)
        for i,node in enumerate(asset.g['nodes']):
            if node['name'].startswith('Wheel_'):
                self.assertFalse(np.array_equal(still[i],moving[i]))
                np.testing.assert_array_equal(still[i][:3,3],moving[i][:3,3])
            else: np.testing.assert_array_equal(still[i],moving[i])

    def test_seek_replays_identical_frame_and_blink_uses_video_time(self):
        data={'speed_kmh':12,'left_blinker':True}
        first=render_ego(data,time_s=.1,spin=1.)
        off=render_ego(data,time_s=.4,spin=1.)
        self.assertFalse(np.array_equal(first,off))
        np.testing.assert_array_equal(first,render_ego(data,time_s=.1,spin=1.))
        np.testing.assert_array_equal(render_ego(data,time_s=.4,spin=1.),render_ego({'speed_kmh':12},time_s=.4,spin=1.))
