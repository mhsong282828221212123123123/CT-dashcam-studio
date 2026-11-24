import unittest
import numpy as np
from core.driving_scene import Actor,Scene,road_display_scene
from core.scene_road import add_travel_corridors
from core.scene_world_tracks import rotation


class VehicleRoadTests(unittest.TestCase):
    def test_image_supported_reconstructed_subtypes_establish_real_adjacent_lane(self):
        scenes,poses,audits=self.sequence(False)
        for s in scenes:
            s.lanes=[]
            s.actors=[Actor('lead','van',0,12,tracking_state='reconstructed',image_observed=True,sources=('front:1',)),
                      Actor('beside','suv',-3.4,9,tracking_state='reconstructed',image_observed=True,sources=('front:2',)),
                      Actor('behind','small_truck',-3.6,-5,tracking_state='reconstructed',image_observed=True,sources=('right_repeater:1',))]
        add_travel_corridors(scenes,poses,audits)
        self.assertEqual(audits[0]['travel_corridors']['occupied_lane_indices'],[-1,0])
        self.assertEqual([p['dashed'] for p in scenes[0].lanes],[False,True,False])

    def test_cross_traffic_and_unverified_reconstructions_do_not_create_lanes(self):
        scenes,poses,audits=self.sequence(False)
        for s in scenes:
            s.lanes=[]
            s.actors.extend([Actor('cross','small_truck',7,10,yaw=np.pi/2,tracking_state='reconstructed',image_observed=True,sources=('right_repeater:1',)),
                            Actor('hypothesis','suv',-7,10,tracking_state='reconstructed',image_observed=True,position_hypothesis=True,sources=('front:4',))])
        add_travel_corridors(scenes,poses,audits)
        self.assertEqual(audits[0]['travel_corridors']['occupied_lane_indices'],[0])

    def test_one_source_drift_does_not_create_another_lane_without_paint(self):
        scenes,poses,audits=self.sequence(False)
        for i,scene in enumerate(scenes):
            scene.lanes=[]
            scene.actors=[Actor('generation'+str(i//20),'car',-4.7-i*.04,10,sources=('front:1',)),
                          Actor('truck','truck',-5.1,-5,sources=('right_repeater:1',))]
        add_travel_corridors(scenes,poses,audits)
        self.assertTrue(all(a['travel_corridors']['occupied_lane_indices']==[-1,0] for a in audits))

    def test_three_same_direction_lanes_are_not_collapsed_to_two(self):
        scenes,poses,audits=self.sequence(False)
        for scene in scenes:
            scene.lanes=[]
            scene.actors=[Actor('a','car',-3.5,10),Actor('b','car',-7.,15)]
        add_travel_corridors(scenes,poses,audits)
        self.assertEqual(audits[-1]['travel_corridors']['occupied_lane_indices'],[-2,-1,0])

    def test_other_vehicle_motion_cannot_change_host_pose_in_paint_gap(self):
        from core.scene_vehicle_road import _road_frames
        scenes,poses,_=self.sequence()
        for i,(scene,(origin,R)) in enumerate(zip(scenes,poses)):
            scene.lanes=[] if 15<i<48 else [
                {'points':((np.array([[x,origin[1]+5],[x,origin[1]+20]])-origin)@R).tolist()}
                for x in (-1.75,1.75,5.25)]
        before=_road_frames(scenes,poses,0,len(scenes),3.5)
        for i,scene in enumerate(scenes):
            scene.actors=[Actor('a','car',i*.7,12),Actor('b','car',i*.7+3.5,20)]
        after=_road_frames(scenes,poses,0,len(scenes),3.5)
        np.testing.assert_allclose([v[1] for v in before],[v[1] for v in after])
        self.assertAlmostEqual(after[45][1],3.5,places=5)

    def sequence(self,lane_change=True):
        scenes=[];poses=[]
        for i in range(61):
            t=i*.1
            x=3.5*np.clip((t-2)/2,0,1) if lane_change else 0.
            yaw=-.08*np.sin(np.pi*(t-2)/2) if 2<t<4 and lane_change else 0.
            origin=np.array([x,10*t]);R=rotation(yaw)
            local=R.T@(np.array([0.,10*t+20])-origin)
            scenes.append(Scene(t,{'speed_kmh':36},actors=[Actor('lead','car',*local)]))
            scenes[-1].lanes=[{'points':((np.array([[edge,origin[1]+5],[edge,origin[1]+20]])-origin)@R).tolist()}
                             for edge in (-1.75,1.75,5.25)]
            poses.append((origin,R))
        return scenes,poses,[{} for _ in scenes]

    def test_host_lane_change_does_not_create_false_opposite_lane(self):
        scenes,poses,audits=self.sequence()
        add_travel_corridors(scenes,poses,audits)
        # Host goes left from lane 0 to lane +1. The lead vehicle remains in 0.
        self.assertTrue(all(a['travel_corridors']['occupied_lane_indices']==[0,1] for a in audits))
        for scene in scenes:
            display=road_display_scene(scene)
            self.assertAlmostEqual(display.actors[0].x,0.,places=6)
            boundaries=[line['points'][0][0] for line in display.lanes]
            np.testing.assert_allclose(boundaries,[-1.75,1.75,5.25],atol=1e-6)
        self.assertAlmostEqual(road_display_scene(scenes[-1]).ego_display[0],3.5)

    def test_vehicle_evidence_builds_road_without_paint(self):
        scenes,poses,audits=self.sequence(False)
        for scene in scenes:
            scene.lanes=[];scene.actors.append(Actor('adjacent','car',-3.5,12))
        add_travel_corridors(scenes,poses,audits)
        self.assertTrue(all(a['travel_corridors']['occupied_lane_indices']==[-1,0] for a in audits))
        self.assertEqual([p['dashed'] for p in scenes[0].lanes],[False,True,False])
        self.assertTrue(scenes[0].road_surface)

    def test_missing_road_after_curve_does_not_send_host_off_screen(self):
        scenes,poses,audits=self.sequence(False)
        for i,scene in enumerate(scenes):
            if i>10:scene.lanes=[]
            poses[i]=(np.array([.1*i*i,i*2.]),rotation(i*.04))
        add_travel_corridors(scenes,poses,audits)
        self.assertLess(abs(scenes[-1].ego_display_override[0]),.1)
        self.assertEqual(scenes[-1].ego_position_state,'held_uncertain')
        self.assertEqual(audits[-1]['travel_corridors']['phase_source'],'held_without_two_sided_road_support')

    def test_single_curved_fragment_cannot_invent_host_lane_change(self):
        scenes,poses,audits=self.sequence(False)
        for i,s in enumerate(scenes):
            if i>10:
                x=1.75-i*.08
                s.lanes=[{'points':[[x,5],[x+.2,20]]}]
            poses[i]=(np.array([i*i*.05,i*2.]),rotation(i*.02))
        add_travel_corridors(scenes,poses,audits)
        self.assertLess(abs(scenes[-1].ego_display_override[0]),.1)
        self.assertEqual(audits[-1]['travel_corridors']['occupied_lane_indices'],[0])

    def test_distant_vehicle_on_unobserved_bend_does_not_create_third_lane(self):
        scenes,poses,audits=self.sequence(False)
        for i,s in enumerate(scenes):
            s.actors.append(Actor('left','car',3.5,10))
            if i>10:
                s.lanes=[];s.actors.append(Actor('far_bend','car',7,45))
        add_travel_corridors(scenes,poses,audits)
        self.assertTrue(all(a['travel_corridors']['occupied_lane_indices']==[0,1] for a in audits))

    def test_supported_third_lane_is_not_hard_capped_to_two(self):
        scenes,poses,audits=self.sequence(False)
        for s in scenes:s.actors.extend([Actor('left','car',3.5,10),Actor('right','car',-3.5,12)])
        add_travel_corridors(scenes,poses,audits)
        self.assertEqual(audits[0]['travel_corridors']['occupied_lane_indices'],[-1,0,1])

    def test_no_road_observation_never_integrates_hundreds_of_metres_sideways(self):
        scenes,poses,audits=self.sequence(False)
        for i,s in enumerate(scenes):
            s.lanes=[];poses[i]=(np.array([i*i*.3,i*2.]),rotation(i*.04))
        add_travel_corridors(scenes,poses,audits)
        self.assertTrue(all(abs(s.ego_display_override[0])<1e-6 for s in scenes))
        self.assertTrue(all(s.ego_position_state=='held_uncertain' for s in scenes))

    def test_paint_phase_unwraps_host_crossing_without_extra_opposite_lane(self):
        scenes,poses,audits=self.sequence()
        for scene,(origin,R) in zip(scenes,poses):
            scene.lanes=[{'points':((np.array([[x,origin[1]+5],[x,origin[1]+20]])-origin)@R).tolist()}
                         for x in (-1.75,1.75,5.25)]
        add_travel_corridors(scenes,poses,audits)
        self.assertEqual(audits[-1]['travel_corridors']['occupied_lane_indices'],[0,1])
        self.assertAlmostEqual(road_display_scene(scenes[-1]).ego_display[0],3.5,places=5)
        self.assertTrue(all(abs(road_display_scene(s).actors[0].x)<.05 for s in scenes))

    def test_curved_world_motion_does_not_accumulate_false_lanes(self):
        scenes,poses,audits=self.sequence(False)
        for i,scene in enumerate(scenes):
            poses[i]=(np.array([.02*i*i,10*i*.1]),rotation(.01*i))
            scene.lanes=[{'points':[[x,5],[x,20]]} for x in (-1.75,1.75)]
        add_travel_corridors(scenes,poses,audits)
        self.assertTrue(all(a['travel_corridors']['display_lane_count']==1 for a in audits))
        self.assertAlmostEqual(road_display_scene(scenes[-1]).ego_display[0],0.)

    def test_one_frame_outlier_and_ghost_do_not_create_lane(self):
        scenes,poses,audits=self.sequence(False)
        scenes[20].actors.append(Actor('outlier','car',10.5,12))
        for scene in scenes:scene.actors.append(Actor('ghost','car',-7,12,tracking_state='predicted'))
        add_travel_corridors(scenes,poses,audits)
        self.assertTrue(all(a['travel_corridors']['display_lane_count']==1 for a in audits))

    def test_occlusion_does_not_remove_established_road(self):
        scenes,poses,audits=self.sequence(False)
        for i,scene in enumerate(scenes):
            if i<20:scene.actors.append(Actor('adjacent','car',3.5,12))
            if i>30:scene.actors=[]
        add_travel_corridors(scenes,poses,audits)
        self.assertEqual(audits[-1]['travel_corridors']['display_lane_count'],2)

    def test_odometry_reset_starts_a_separate_road(self):
        scenes,poses,audits=self.sequence(False)
        for scene in scenes[:25]:scene.actors.append(Actor('adjacent','car',3.5,12))
        audits[30]['world_track_events']=[{'reason':'odometry unavailable; tracks reset'}]
        add_travel_corridors(scenes,poses,audits)
        self.assertEqual(audits[20]['travel_corridors']['display_lane_count'],2)
        self.assertEqual(audits[40]['travel_corridors']['display_lane_count'],1)
