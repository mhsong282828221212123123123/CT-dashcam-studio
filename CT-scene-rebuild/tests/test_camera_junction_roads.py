import copy,unittest
import numpy as np
from ct_scene.camera_junction_roads import supplement


class CameraCrossRoadTests(unittest.TestCase):
    def fixture(self):
        road=dict(reference_line=dict(points_W_xy=[[0,0],[20,0],[40,-20],[40,-100]]),boundaries_y=[3.5,0],lane_sections=[],junctions=[dict(id='turn',turn_deg=-90.,start_s=2.,end_s=8.,crossing_support_frames=12,road_station_range_m=[15,90],polygon_W_xy=[[0,-40],[60,-40],[60,10],[0,10]],branch_display_hints=[])])
        rows=[dict(frame_id=str(i),time_s=float(t),support='detector_observation',bbox_xyxy=[200,200,400,500],position_W=[35,-20+5*(t-3),0]) for i,t in enumerate(np.arange(3,5.21,.2))]
        return road,[dict(global_id='crossing',lane_index=-2,frozen_lead=False,samples=rows)]

    def test_actual_crossing_fills_missing_map_arm_and_preserves_geometry(self):
        road,actors=self.fixture();old=copy.deepcopy((road,actors));junctions,review=supplement(road,actors)
        self.assertEqual((road,actors),old);self.assertEqual(len(review),1);self.assertFalse(review[0]['map_used']);self.assertFalse(review[0]['actor_or_ego_modified'])
        self.assertEqual(len(junctions[0]['branch_display_hints']),1)
        self.assertGreaterEqual(review[0]['width_m'],6.)

    def test_stationary_following_or_noisy_objects_do_not_invent_a_cross_street(self):
        for mode in ['stationary','following','jitter']:
            road,actors=self.fixture()
            for i,r in enumerate(actors[0]['samples']):
                if mode=='stationary':r['position_W']=[35,-20,0]
                if mode=='following':r['position_W'][1]=-20-5*(r['time_s']-3)
                if mode=='jitter':r['position_W'][0]+=3*(-1)**i
            self.assertEqual(supplement(road,actors)[1],[],mode)

    def test_gentle_curve_known_map_or_unconfirmed_scope_is_not_extended(self):
        for mode in ['curve','map','scope']:
            road,actors=self.fixture()
            if mode=='curve':road['junctions'][0]['turn_deg']=20
            if mode=='map':road['junctions'][0]['branch_display_hints']=[dict(source='osm_existing')]
            if mode=='scope':actors[0]['lane_index']=0
            self.assertEqual(supplement(road,actors)[1],[],mode)

    def test_brief_crossing_has_insufficient_repeated_support(self):
        road,actors=self.fixture();actors[0]['samples']=actors[0]['samples'][:5];self.assertEqual(supplement(road,actors)[1],[])
