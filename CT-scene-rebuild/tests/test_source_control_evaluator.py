import importlib.util
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('source_controls',Path(__file__).parents[1]/'tools/evaluate_source_controls.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class SourceControlTests(unittest.TestCase):
    def test_visible_detection_without_global_geometry_is_not_coverage(self):
        anchor=dict(time_s=5.,bbox_xyxy=[10,10,30,40])
        obs=[dict(camera_time_s=5.,support='detector_observation',local_track_id='front-1',bbox_xyxy=[10,10,30,40])]
        gates=dict(anchor_time_tolerance_s=.3,anchor_iou_min=.4,anchor_match_margin_min=.1)
        match=module.match_anchor(anchor,obs,{},gates)
        self.assertEqual(match['status'],'MATCH');self.assertIsNone(match['global_id'])

    def test_gap_fill_cannot_establish_identity_and_ambiguous_cars_fail(self):
        anchor=dict(time_s=5.,bbox_xyxy=[10,10,30,40])
        gates=dict(anchor_time_tolerance_s=.3,anchor_iou_min=.4,anchor_match_margin_min=.1)
        obs=[dict(camera_time_s=5.,support='offline_gap_fill',local_track_id='fake',bbox_xyxy=anchor['bbox_xyxy'])]
        self.assertEqual(module.match_anchor(anchor,obs,{'fake':'fake'},gates)['status'],'NO_OBSERVATION')
        obs=[dict(camera_time_s=5.,support='detector_observation',local_track_id=k,bbox_xyxy=anchor['bbox_xyxy']) for k in ['a','b']]
        self.assertIsNone(module.match_anchor(anchor,obs,{'a':'a','b':'b'},gates)['global_id'])

    def test_dropout_includes_terminal_missing_frame(self):
        self.assertAlmostEqual(module.longest_absence([0,.2,.4,.6,.8],[True,True,False,False,False]),.6)

    def test_relative_lane_is_independent_of_absolute_index_and_abstains_on_unknown(self):
        self.assertTrue(module.relative_lane_matches(dict(actor_lane=3,ego_lane=2),1))
        self.assertTrue(module.relative_lane_matches(dict(actor_lane=0,ego_lane=0),0))
        for row in [dict(actor_lane=None,ego_lane=None),dict(actor_lane=-1,ego_lane=0),dict(actor_lane=1,ego_lane=0)]:
            self.assertFalse(module.relative_lane_matches(row,0))
