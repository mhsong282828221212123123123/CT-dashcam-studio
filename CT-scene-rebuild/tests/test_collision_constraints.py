import copy,math,unittest
import numpy as np
from ct_scene.collision_constraints import overlap,audit,constrain,omit_conflicting_predictions


def actor(key,p,yaw=0,cap=.75):
    return dict(id=key,position_W=list(p),yaw_rad=yaw,model='car',collision_cap_m=cap,ghost=False,source_visible=True)


class CollisionTests(unittest.TestCase):
    dims={'car':[4.6,2.,1.5],'ego':[4.6,2.,1.5]}
    road={'boundaries_y':[4.,0.,-4.]}

    def test_overlapping_near_prediction_yields_to_visible_body(self):
        ghost=dict(actor('prediction',[8,2,0]),ghost=True,source_visible=False)
        observed=actor('seen',[8.5,2,0])
        raw=[dict(time_s=0,ego_position_W=[0,0,0],ego_yaw_rad=0,actors=[ghost,observed])]
        before=copy.deepcopy(raw);fixed,r=omit_conflicting_predictions(raw,self.dims)
        self.assertEqual(raw,before)
        self.assertEqual(fixed[0]['actors'],[observed])
        self.assertEqual(r['omitted_samples'][0]['observed_ids'],['seen'])

    def test_visible_conflicts_and_small_errors_are_never_hidden(self):
        frames=[dict(time_s=0,ego_position_W=[0,0,0],ego_yaw_rad=0,
                     actors=[actor('a',[8,2,0]),actor('b',[8.5,2,0])]),
                dict(time_s=.05,ego_position_W=[0,0,0],ego_yaw_rad=0,
                     actors=[actor('a',[8,2,0]),dict(actor('b',[12.4,2,0]),ghost=True,source_visible=False)])]
        fixed,r=omit_conflicting_predictions(frames,self.dims)
        self.assertEqual(fixed,frames);self.assertEqual(r['omitted_count'],0)

    def test_far_predictions_and_uncontested_predictions_remain(self):
        frames=[dict(time_s=0,ego_position_W=[0,0,0],ego_yaw_rad=0,
                     actors=[actor('seen',[80,2,0]),dict(actor('far',[80,2,0]),ghost=True,source_visible=False),
                             dict(actor('clear',[10,5,0]),ghost=True,source_visible=False)])]
        fixed,r=omit_conflicting_predictions(frames,self.dims)
        self.assertEqual(fixed,frames);self.assertEqual(r['omitted_count'],0)

    def test_depth_only_front_estimate_cannot_be_pushed_to_another_lane(self):
        a=actor('front',[0,2,0],cap=2.);a['collision_axis_W']=[1.,0.]
        f=[dict(time_s=i*.05,actors=[dict(a,position_W=[i,2,0]),actor('fixed',[i+3.5,2,0],cap=0)]) for i in range(20)]
        fixed,r=constrain(f,self.road,self.dims)
        self.assertEqual(r['after']['gate'],'PASS')
        for state in fixed:self.assertEqual(state['actors'][0]['position_W'][1],2.)

    def test_separation_cannot_introduce_impossible_speed(self):
        fs=[]
        for i in range(40):
            t=i*.05;a=actor('uncertain',[40*t-8,2,0],cap=2.)
            a['collision_axis_W']=[1.,0.]
            fs.append(dict(time_s=t,actors=[a,actor('anchor',[22*t,2,0],cap=0)]))
        fixed,r=constrain(fs,self.road,self.dims)
        speeds=[np.linalg.norm(b['actors'][0]['position_W']-a['actors'][0]['position_W'])/.05 for a,b in zip(fixed,fixed[1:])]
        self.assertLessEqual(max(speeds),65.)
        self.assertTrue(r['rejected_correction_intervals'])

    def test_rotated_boxes_separate_when_aabbs_overlap(self):
        a=actor('a',[0,0,0],math.pi/4);b=actor('b',[0,2.9,0],math.pi/4)
        self.assertIsNone(overlap(a,b,self.dims))
        self.assertIsNotNone(overlap(a,actor('c',[0,2.,0],math.pi/4),self.dims))

    def test_touch_and_adjacent_lanes_are_not_penetration(self):
        self.assertIsNone(overlap(actor('a',[0,0,0]),actor('b',[0,2.,0]),self.dims))
        self.assertIsNone(overlap(actor('a',[0,0,0]),actor('b',[0,3.5,0]),self.dims))

    def test_small_overlap_resolves_without_moving_frozen_body(self):
        f=[dict(time_s=i*.05,actors=[actor('locked',[i,1.,0],cap=0),actor('side',[i,2.8,0])]) for i in range(20)]
        raw=copy.deepcopy(f);fixed,r=constrain(f,self.road,self.dims)
        self.assertEqual(r['before']['gate'],'FAIL');self.assertEqual(r['after']['gate'],'PASS')
        self.assertEqual(f,raw)
        for orig,now in zip(f,fixed):
            np.testing.assert_array_equal(now['actors'][0]['position_W'],orig['actors'][0]['position_W'])
            self.assertLessEqual(np.linalg.norm(now['actors'][1]['position_W']-orig['actors'][1]['position_W']),.750001)
            self.assertTrue(now['actors'][1]['source_visible'])
            self.assertFalse(now['actors'][1]['ghost'])
        self.assertFalse(r['identity_merge'])

    def test_coincident_ids_are_flagged_never_merged_or_hidden(self):
        f=[dict(time_s=0,actors=[actor('a',[0,2.,0]),actor('b',[0,2.,0])])]
        fixed,r=constrain(f,self.road,self.dims)
        self.assertEqual(r['after']['gate'],'FAIL');self.assertEqual(len(fixed[0]['actors']),2)

    def test_impossible_locked_conflict_stays_a_failure(self):
        f=[dict(time_s=0,actors=[actor('a',[0,1.,0],cap=0),actor('b',[0,1.5,0],cap=0)])]
        fixed,r=constrain(f,self.road,self.dims)
        self.assertEqual(r['after']['gate'],'FAIL');self.assertEqual(r['max_correction_m'],0)

    def test_lane_boundary_and_unknown_road_are_not_crossed(self):
        f=[dict(time_s=0,actors=[actor('a',[0,3.9,0]),actor('b',[0,3.7,0],cap=0)])]
        fixed,r=constrain(f,self.road,self.dims)
        self.assertLess(fixed[0]['actors'][0]['position_W'][1],4.)
        f=[dict(time_s=0,actors=[actor('a',[0,10.,0]),actor('b',[0,10.5,0],cap=0)])]
        fixed,r=constrain(f,self.road,self.dims)
        self.assertEqual(r['max_correction_m'],0)

    def test_ego_uses_mesh_center_offset_and_is_locked(self):
        f=dict(time_s=0,ego_position_W=[0,0,0],ego_yaw_rad=0,actors=[actor('side',[-1.2,1.8,0])])
        self.assertEqual(audit([f],self.dims)['gate'],'FAIL')
        fixed,r=constrain([f],{'boundaries_y':[4,-4]},self.dims)
        self.assertEqual(fixed[0]['ego_position_W'],[0,0,0]);self.assertEqual(r['after']['gate'],'PASS')

    def test_real_contact_is_not_a_physics_bounce(self):
        f=[dict(time_s=0,actors=[actor('a',[0,0,0],cap=0),actor('b',[4.6,0,0],cap=0)])]
        fixed,r=constrain(f,{'boundaries_y':[4,-4]},self.dims)
        self.assertEqual(r['after']['gate'],'PASS');self.assertEqual(r['max_correction_m'],0)
