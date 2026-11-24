import copy,unittest
import numpy as np
from ct_scene.road_geometry import boundaries_at,lane_at
from ct_scene.local_road_extents import refine
from ct_scene.lane_state import timeline


class LocalExtentTests(unittest.TestCase):
    def road(self):
        return dict(boundaries_y=[3.5,0,-3.5],left_pockets=[],junctions=[],reference_line=dict(points_W_xy=[[0,0],[200,0]]),
                    lane_sections=[dict(id='local',station_range_m=[0,200],boundaries_y=[3.5,0,-3.5],left_added_lane=True,
                    boundary_profile=[dict(station_m=0,boundaries_y=[3.5,3.5,0,-3.5]),
                                      dict(station_m=30,boundaries_y=[7,3.5,0,-3.5]),
                                      dict(station_m=100,boundaries_y=[3.5,0,-3.5,-3.5]),
                                      dict(station_m=200,boundaries_y=[3.5,0,-3.5,-3.5])])])

    def test_zero_width_taper_is_not_reported_as_a_lane(self):
        road=self.road()
        self.assertEqual(len(boundaries_at(road,0)),3)
        self.assertEqual(len(boundaries_at(road,30)),4)
        self.assertEqual(len(boundaries_at(road,100)),3)
        self.assertTrue(all(a>b for a,b in zip(boundaries_at(road,95),boundaries_at(road,95)[1:])))

    def test_left_layout_change_is_not_a_vehicle_lane_change(self):
        road=self.road();t=np.arange(0,4,.1);poses=np.c_[t*10,t*0+1.75,t*0]
        result=timeline(t,poses,t*0,road,4.,1.8)
        self.assertEqual(result['records'][0]['lane_id'],0)
        self.assertEqual(result['records'][-1]['lane_id'],1)
        self.assertEqual(result['events'],[])

    def test_missing_paint_does_not_change_layout_or_input_poses(self):
        road=self.road();road['lane_sections'][0].pop('boundary_profile');before=copy.deepcopy(road)
        ego=np.c_[np.arange(20)*4,np.zeros(20),np.zeros(20)];old=ego.copy()
        sections,review=refine(road,[],[],ego,3.5)
        self.assertEqual(road,before);np.testing.assert_array_equal(ego,old)
        self.assertEqual(sections,road['lane_sections']);self.assertFalse(review['changes'])

    def test_outer_lane_closure_does_not_create_a_remaining_lane_change(self):
        road=self.road();s=road['lane_sections'][0];s['left_added_lane']=False
        s['boundary_profile']=[dict(station_m=0,boundaries_y=[3.5,0,-3.5,-7]),dict(station_m=100,boundaries_y=[3.5,0,-3.5,-3.5])]
        t=np.arange(0,12,.1);poses=np.c_[t*10,t*0-1.75,t*0]
        result=timeline(t,poses,t*0,road,4.,1.8)
        self.assertEqual(result['events'],[])
        self.assertTrue(all(r['lane_id']==1 for r in result['records']))
