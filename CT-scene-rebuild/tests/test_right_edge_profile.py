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
        road=dict(args[0],lane_sections=sections);self.assertEqual(len(boundaries_at(road,60))-1,3);self.assertEqual(len(boundaries_at(road,400))-1,3)
        self.assertEqual(args[0],before[0]);self.assertEqual(args[1],before[1]);np.testing.assert_array_equal(args[3],before[3])
    def test_vehicle_beyond_unbroken_edge_cannot_create_a_lane(self):
        road,paint,actors,ego=self.fixture();rows=[dict(support='detector_observation',time_s=s/20,position_W=[s,-3.5,0],bbox_xyxy=[100,100,300,400],image_width=1280) for s in range(0,251,4)]
        actors=[dict(camera_id='front',samples=rows)]
        sections,proof=refine(road,paint,actors,ego);self.assertTrue(proof['changes']);self.assertEqual(len(boundaries_at(dict(road,lane_sections=sections),100))-1,3)
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

    def test_confirmed_new_outer_edge_and_dashed_separator_can_open_lane(self):
        road,paint,actors,ego=self.fixture()
        for s in range(250,401,4):
            paint.append(dict(kind='white',ground_length_m=15.,color_continuity=.99,frame_id='outer'+str(s),points_W_xy=[[s,-5.25],[s+15,-5.25]]))
            paint.append(dict(kind='white',ground_length_m=3.,color_continuity=.99,frame_id='dash'+str(s),points_W_xy=[[s,-1.75],[s+3,-1.75]]))
        sections,proof=refine(road,paint,actors,ego);self.assertIsNotNone(proof['changes'][0]['confirmed_expansion_station_m'])
        self.assertEqual(len(boundaries_at(dict(road,lane_sections=sections),390))-1,4)
    def test_constant_camera_clearance_does_not_turn_reference_drift_into_lane_change(self):
        from ct_scene.lane_state import timeline
        road,paint,actors,ego=self.fixture();ego[:,1]=-3.5*np.clip((ego[:,0]-150)/100,0,1);paint=[]
        for s in range(0,401,4):
            y=-1.75-3.5*np.clip((s-150)/100,0,1)
            paint.append(dict(kind='white',ground_length_m=15.,color_continuity=.99,frame_id=str(s),points_W_xy=[[s,y],[s+12,y]]))
        sections,proof=refine(road,paint,actors,ego);self.assertEqual(proof['status'],'camera_right_edge_clearance_profile')
        road=dict(road,lane_sections=sections);t=np.arange(len(ego))*.2;state=timeline(t,ego,np.zeros(len(ego)),road,4.7,2.)
        self.assertEqual(state['events'],[]);self.assertTrue(all(r['lane_id']==2 for r in state['records']))
