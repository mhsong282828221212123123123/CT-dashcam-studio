import unittest
import numpy as np

from core.scene_telemetry import odometry_speed_kmh
from core.scene_world_tracks import WorldTracks
from core.scene_preview import EgoTimeline


class OdometryPrecisionTests(unittest.TestCase):
    def test_creep_travel_is_not_lost_to_zero_display_speed(self):
        data=dict(speed_mps=.08888864,speed_kmh=0,heading=0.)
        tracker=WorldTracks()
        rows=[]
        for i in range(61):
            t=i*.1
            tracker.update([],t,data)
            rows.append(dict(time_s=t,ego=dict(data=data)))
        self.assertAlmostEqual(tracker.pose[1],6*.08888864,places=9)
        self.assertAlmostEqual(EgoTimeline(rows).states[-1][2]*.343,tracker.pose[1],places=9)

    def test_old_records_keep_kmh_fallback(self):
        self.assertEqual(odometry_speed_kmh(dict(speed_kmh=36)),36.)

    def test_invalid_raw_speed_does_not_silently_fall_back_to_display(self):
        for value in [None,float('nan'),float('inf'),-1,100,True]:
            self.assertIsNone(odometry_speed_kmh(dict(speed_mps=value,speed_kmh=36)))
