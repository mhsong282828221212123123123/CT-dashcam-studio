import unittest
import numpy as np
from ct_scene.camera_road_support import below_body_score,summarize


class CameraRoadSupportTests(unittest.TestCase):
    def test_below_body_region_avoids_vehicle_and_clipped_ground(self):
        mask=np.zeros((100,100),np.uint8);mask[54:74,35:65]=1
        self.assertEqual(below_body_score(mask,[20,10,80,50]),1.)
        self.assertIsNone(below_body_score(mask,[20,10,80,90]))
    def test_positive_road_evidence_vetoes_whole_track_exclusion(self):
        rows=[dict(time_s=i*.2,road_fraction=0.) for i in range(10)]
        self.assertEqual(summarize(rows)['recommendation'],'review_off_carriageway')
        rows.append(dict(time_s=2.,road_fraction=.8))
        r=summarize(rows);self.assertEqual(r['recommendation'],'insufficient_or_conflicting_evidence');self.assertFalse(r['automatically_exclude'])
    def test_brief_partial_observations_are_not_road_membership_proof(self):
        self.assertEqual(summarize([dict(time_s=i*.2,road_fraction=0.) for i in range(3)])['recommendation'],'insufficient_or_conflicting_evidence')

    def test_low_resolution_negative_is_unknown_but_positive_still_vetoes(self):
        mask=np.zeros((960,1280),np.uint8);box=[600,450,640,490]
        self.assertIsNone(below_body_score(mask,box))
        mask[:]=1
        self.assertEqual(below_body_score(mask,box),1.)
