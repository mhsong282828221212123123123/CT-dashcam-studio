import copy,unittest
from ct_scene.surrounding import actor_corridor_lane,unresolved_front_border_fragment


class RoadsideScopeTests(unittest.TestCase):
    def test_never_resolved_small_border_body_does_not_create_a_second_full_vehicle(self):
        rows=[dict(support='detector_observation',bbox_xyxy=[0,400,100,800]) for _ in range(20)]
        self.assertTrue(unresolved_front_border_fragment(rows,1280))
        for r in rows[-4:]:r['bbox_xyxy']=[100,400,300,600]
        self.assertFalse(unresolved_front_border_fragment(rows,1280))
        for r in rows:r['bbox_xyxy']=[0,100,1000,900]
        self.assertFalse(unresolved_front_border_fragment(rows,1280))

    def fixture(self):
        road=dict(boundaries_y=[1.75,-1.75],lane_width_m_prior=3.5,reference_line=dict(points_W_xy=[[0,0],[100,0]],supported_extent_m=[0,100]))
        actor=dict(camera_id='front',samples=[dict(time_s=i*.2,position_W=[20+i*.2,-3,0],support='detector_observation',bbox_xyxy=[700,300,1000,550]) for i in range(12)])
        return road,actor

    def test_curb_vehicle_is_visible_without_creating_lane_or_moving_it(self):
        road,a=self.fixture();before=copy.deepcopy((road,a));lane,support,y=actor_corridor_lane(a,road['boundaries_y'],0,road)
        self.assertEqual(lane,-2);self.assertIn('roadside_vehicle',support);self.assertEqual(before,(road,a))

    def test_left_opposing_side_far_background_and_short_sliver_are_not_admitted(self):
        for variant in ['left','far','small','short']:
            road,a=self.fixture();road['left_boundary_role']='camera_observed_yellow_divider_estimated_geometry'
            for r in a['samples']:
                if variant=='left':r['position_W'][1]=3
                if variant=='far':r['position_W'][1]=-10
                if variant=='small':r['bbox_xyxy']=[500,400,550,430]
            if variant=='short':a['samples']=a['samples'][:3]
            self.assertIsNone(actor_corridor_lane(a,road['boundaries_y'],0,road)[0])
