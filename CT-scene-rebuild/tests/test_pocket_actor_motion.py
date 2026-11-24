import copy
import unittest
import numpy as np
from ct_scene.road_frame import frame_for
from ct_scene.pocket_actor_motion import priors
from ct_scene.surrounding import actor_lane_priors


class AuxiliaryMotionTests(unittest.TestCase):
    def setUp(self):
        self.road=dict(boundaries_y=[5.25,1.75,-1.75],reference_line=dict(points_W_xy=[[0,0],[100,100]],supported_extent_m=[0,140]),
                       left_pockets=[dict(width_knots=[[20,0],[30,3.5],[100,3.5]])],junctions=[])
        self.frame=frame_for(self.road)

    def rows(self,t,s,d):return [dict(time_s=float(T),position_W=[*self.frame.world(S,D),0],quality=1.) for T,S,D in zip(t,s,d)]

    def test_pocket_appearance_never_renumbers_through_lane(self):
        t=np.arange(0,24,.2);rows=self.rows(t,t*5,t*0)
        lane,target,review=priors(rows,self.road,1)
        self.assertTrue(all(lane==1));self.assertFalse(review['events'])

    def test_actual_motion_between_main_and_pocket_preserves_all_observations(self):
        t=np.arange(0,12,.2);s=30+t*5;d=3.5+np.clip((t-3)/3,0,1)*3.5;rows=self.rows(t,s,d);original=copy.deepcopy(rows)
        lane,target,review=priors(rows,self.road,0)
        self.assertEqual(lane[0],0);self.assertEqual(lane[-1],-1)
        self.assertEqual(len(lane),len(rows));self.assertEqual(rows,original)
        self.assertEqual([(e['from_lane'],e['to_lane']) for e in review['events']],[(0,-1)])

    def test_unknown_junction_cannot_generate_parallel_lane_event_or_target(self):
        self.road['junctions']=[dict(road_station_range_m=[40,90])]
        t=np.arange(0,8,.2);s=45+t*4;d=3.5+np.clip(t/3,0,1)*3.5;rows=self.rows(t,s,d)
        _,target,review=priors(rows,self.road,0)
        self.assertFalse(review['events']);self.assertTrue(np.allclose(target,d))

    def test_added_pocket_cannot_reset_other_cars_offline_side_anchor(self):
        road=copy.deepcopy(self.road);road['boundaries_y']=[5.25,1.75,-1.75,-5.25]
        road['lane_sections']=[dict(id='section',station_range_m=[0,140],boundaries_y=road['boundaries_y'])]
        t=np.arange(0,12,.2);rows=[]
        for i,now in enumerate(t):
            s=10+now*8;clear=now<2
            rows.append(dict(time_s=float(now),position_W=[*self.frame.world(s,0),0],unrefined_position_W=[*self.frame.world(s,0 if clear else -9),0],
                             camera_id='right_repeater',support='detector_observation',image_width=1280,image_height=960,
                             bbox_xyxy=[100,200,400 if clear else 180,600]))
        baseline=copy.deepcopy(road);baseline['left_pockets']=[]
        before=actor_lane_priors(rows,road['boundaries_y'],0,2,road=baseline)
        after=actor_lane_priors(rows,road['boundaries_y'],0,2,road=road)
        self.assertTrue(np.array_equal(before[0],after[0]));self.assertTrue(np.array_equal(before[1],after[1]));self.assertTrue(all(after[0]==1))
