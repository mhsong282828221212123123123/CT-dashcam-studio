import copy,unittest
import numpy as np
from ct_scene.right_edge_profile import refine
from ct_scene.road_geometry import boundaries_at


class RightEdgePrefixTests(unittest.TestCase):
    def fixture(self):
        road=dict(boundaries_y=[8.75,5.25,1.75,-1.75,-5.25],junctions=[],reference_line=dict(points_W_xy=[[0,0],[400,0]],supported_extent_m=[-40,440]))
        ego=np.c_[np.arange(0,401,2),np.zeros(201),np.zeros(201)]
        paint=[dict(kind='white',ground_length_m=15.,color_continuity=.99,frame_id=str(s),points_W_xy=[[s,-1.75],[s+15,-1.75]]) for s in range(0,201,4)]
        return road,paint,[],ego
    def test_supported_prefix_closes_only_outer_band_without_moving_poses(self):
        args=self.fixture();before=copy.deepcopy(args);sections,proof=refine(*args);self.assertTrue(proof['changes'])
        road=dict(args[0],lane_sections=sections);self.assertEqual(len(boundaries_at(road,60))-1,3);self.assertEqual(len(boundaries_at(road,400))-1,4)
        self.assertEqual(args[0],before[0]);self.assertEqual(args[1],before[1]);np.testing.assert_array_equal(args[3],before[3])
    def test_persistent_clear_forward_traffic_beyond_line_vetoes_closure(self):
        road,paint,actors,ego=self.fixture();rows=[dict(support='detector_observation',time_s=s/20,position_W=[s,-3.5,0],bbox_xyxy=[100,100,300,400],image_width=1280) for s in range(0,251,4)]
        actors=[dict(camera_id='front',samples=rows)]
        self.assertFalse(refine(road,paint,actors,ego)[1]['changes'])
    def test_dash_or_brief_or_late_observations_cannot_close_prefix(self):
        for mode in ['dash','brief','late']:
            road,paint,actors,ego=self.fixture()
            if mode=='dash':
                for r in paint:r['ground_length_m']=3.
            if mode=='brief':paint=paint[:4]
            if mode=='late':
                for r in paint:
                    for p in r['points_W_xy']:p[0]+=180
            self.assertFalse(refine(road,paint,actors,ego)[1]['changes'],mode)
    def test_existing_junction_or_section_is_not_overridden(self):
        for key in ['junctions','lane_sections']:
            road,paint,actors,ego=self.fixture();road[key]=[dict(existing=True)]
            self.assertFalse(refine(road,paint,actors,ego)[1]['changes'])
