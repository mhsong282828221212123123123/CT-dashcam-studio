import copy,unittest
from ct_scene.driving_road_scope import point_scope,mark_raw_scope,filter_plan

class DrivingRoadScopeTests(unittest.TestCase):
    def setUp(self):
        self.road=dict(boundaries_y=[5.25,1.75,-1.75],reference_line=dict(points_W_xy=[[0,0],[100,0]],supported_extent_m=[0,100]),junctions=[],left_pockets=[])

    def actor(self,y):
        return dict(global_id='car',samples=[dict(time_s=i*.2,camera_id='front',support='detector_observation',bbox_xyxy=[400,400,550,550],position_W=[20+i,0,0],unrefined_position_W=[20+i,y,0]) for i in range(5)])

    def test_shoulder_and_opposite_road_are_outside_without_snapping(self):
        self.assertEqual(point_scope(self.road,[20,-4,0]),'outside')
        self.assertEqual(point_scope(self.road,[20,8,0]),'outside')
        self.assertEqual(point_scope(self.road,[20,-2,0]),'inside')
        self.assertEqual(point_scope(self.road,[20,3,0]),'inside')

    def test_unknown_geometry_abstains_and_pocket_is_kept(self):
        self.assertEqual(point_scope({},[20,-4,0]),'unknown')
        self.assertEqual(point_scope(self.road,[120,-4,0]),'unknown')
        self.road['left_pockets']=[dict(width_knots=[[0,3],[100,3]])]
        self.assertEqual(point_scope(self.road,[20,7,0]),'inside')

    def test_actual_crossing_in_junction_polygon_is_kept(self):
        self.road['junctions']=[dict(polygon_W_xy=[[40,-10],[60,-10],[60,10],[40,10]],road_station_range_m=[40,60])]
        self.assertEqual(point_scope(self.road,[50,8,0]),'junction')
        frames=[dict(time_s=1,actors=[dict(id='cross',position_W=[50,8,0])])]
        self.assertEqual(filter_plan(frames,[],self.road),[])
        self.assertEqual(len(frames[0]['actors']),1)

    def test_raw_outside_actor_cannot_be_hidden_by_lane_clamp_even_for_frozen_lead(self):
        actor=self.actor(-8);actor['frozen_lead']=True;before=copy.deepcopy(actor)
        mark_raw_scope(actor,self.road)
        self.assertTrue(all(r['driving_road_scope']=='outside' for r in actor['samples']))
        frames=[dict(time_s=.4,actors=[dict(id='car',position_W=[22,0,0],source_visible=True)])]
        removed=filter_plan(frames,[actor],self.road)
        self.assertEqual(len(removed),1);self.assertFalse(frames[0]['actors'])
        self.assertEqual([r['unrefined_position_W'] for r in actor['samples']],[r['unrefined_position_W'] for r in before['samples']])

    def test_both_legitimate_lanes_and_short_noisy_contact_remain(self):
        actor=self.actor(0);actor['samples'][2]['unrefined_position_W'][1]=-8
        mark_raw_scope(actor,self.road)
        self.assertFalse(any(r['driving_road_scope']=='outside' for r in actor['samples']))
        frames=[dict(time_s=.4,actors=[dict(id='car',position_W=[22,0,0])])]
        self.assertEqual(filter_plan(frames,[actor],self.road),[])

    def test_final_estimation_error_alone_cannot_prove_object_is_outside(self):
        frames=[dict(time_s=.4,actors=[dict(id='car',position_W=[22,-6,0])])]
        removed=filter_plan(frames,[],self.road)
        self.assertEqual(removed,[])
        self.assertEqual(len(frames[0]['actors']),1)

    def test_normal_parallel_front_traffic_is_not_hidden_by_bad_lateral_geometry(self):
        actor=self.actor(-8)
        for i,r in enumerate(actor['samples']):r['unrefined_position_W']=[20+15*r['time_s'],-8,0]
        actor['samples'].append(dict(actor['samples'][-1],time_s=1.2,unrefined_position_W=[38,-8,0]))
        mark_raw_scope(actor,self.road)
        self.assertTrue(actor['driving_road_scope_parallel_front_motion'])
        self.assertFalse(any(r['driving_road_scope']=='outside' for r in actor['samples']))

    def test_long_junction_side_arms_do_not_admit_external_traffic(self):
        self.road['junctions']=[dict(polygon_W_xy=[[40,-30],[60,-30],[60,30],[40,30]],road_station_range_m=[40,60])]
        self.assertEqual(point_scope(self.road,[50,4,0]),'junction')
        self.assertEqual(point_scope(self.road,[50,20,0]),'outside')

    def test_unlocalized_side_car_in_supported_extent_gap_is_not_drawn_offroad(self):
        self.road['reference_line']['unsupported_intervals_m']=[[15,30]]
        actor=self.actor(-12)
        for r in actor['samples']:r['camera_id']='right_repeater';r['bbox_xyxy']=[0,400,200,600]
        mark_raw_scope(actor,self.road)
        self.assertTrue(all(r['driving_road_scope']=='unknown' for r in actor['samples']))
        plan=[dict(time_s=.4,actors=[dict(id='car',position_W=[22,-9,0])])]
        removed=filter_plan(plan,[actor],self.road)
        self.assertEqual(len(removed),1)
        self.assertEqual(removed[0]['reason'],'side_position_unlocalized_far_beyond_ego_carriageway')

    def test_unknown_side_position_inside_road_is_retained(self):
        self.road['reference_line']['unsupported_intervals_m']=[[15,30]]
        actor=self.actor(0)
        for r in actor['samples']:r['camera_id']='right_repeater'
        mark_raw_scope(actor,self.road)
        plan=[dict(time_s=.4,actors=[dict(id='car',position_W=[22,0,0])])]
        self.assertFalse(filter_plan(plan,[actor],self.road))
