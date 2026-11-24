import unittest
from dataclasses import replace
import numpy as np
from core.driving_scene import Actor
from core.scene_world_tracks import WorldTracks,bridge_motion_consistent


class WorldTrackTests(unittest.TestCase):
    def test_curved_vehicle_motion_keeps_observed_identity(self):
        import math
        tracker=WorldTracks()
        for i in range(60):
            t=i*.1;angle=.6*t
            actor=Actor('turn','car',8/.6*(1-math.cos(angle)),20+8/.6*math.sin(angle),
                        yaw=angle,sources=('front:turn',))
            out,events=tracker.update([actor],t,{'speed_kmh':0})
            self.assertEqual(len(out),1)
            self.assertEqual(out[0].key,'turn')
            self.assertEqual(out[0].tracking_state,'observed')
            self.assertFalse(events)

    def image_support(self,source='front:1',z=15):
        from core.scene_fusion import CameraGeometry,corners,dimensions
        camera=CameraGeometry()
        bounds=camera.visible_bounds(corners(3.5,z,dimensions('car'),kind='car'))
        return {source:dict(camera=camera,bbox=np.r_[bounds[:2],bounds[2:]-bounds[:2]].tolist())}

    def seed_image_track(self,tracker):
        for t in (0,.1,.2):tracker.update([self.actor()],t,{'speed_kmh':0},self.image_support())

    def test_visible_2d_identity_survives_metric_fit_gap_and_reacquires_same_id(self):
        tracker=WorldTracks();self.seed_image_track(tracker)
        for i in range(3,31):
            out,_=tracker.update([],i*.1,{'speed_kmh':0},self.image_support(z=15+(i-2)*.1))
            self.assertEqual(len(out),1);self.assertEqual(out[0].key,'one')
            self.assertEqual(out[0].tracking_state,'image_supported')
            self.assertEqual(out[0].sources,('front:1',))
        out,_=tracker.update([self.actor(z=17.9)],3.1,{'speed_kmh':0},self.image_support(z=17.9))
        self.assertEqual(out[0].key,'one');self.assertEqual(out[0].tracking_state,'observed')

    def test_short_detection_dropout_after_metric_gap_is_bounded_then_expires(self):
        tracker=WorldTracks();self.seed_image_track(tracker)
        for i in range(3,21):tracker.update([],i*.1,{'speed_kmh':0},self.image_support())
        out,_=tracker.update([],2.1,{'speed_kmh':0})
        self.assertEqual(out[0].key,'one');self.assertEqual(out[0].tracking_state,'predicted')
        for i in range(22,42):out,_=tracker.update([],i*.1,{'speed_kmh':0})
        self.assertEqual(out,[])

    def test_image_only_observation_cannot_seed_a_metric_vehicle(self):
        tracker=WorldTracks()
        for i in range(20):out,_=tracker.update([],i*.1,{'speed_kmh':0},self.image_support())
        self.assertEqual(out,[]);self.assertEqual(tracker.tracks,{})

    def test_rear_presence_cannot_lease_front_identity(self):
        tracker=WorldTracks();self.seed_image_track(tracker)
        for i in range(3,31):out,_=tracker.update([],i*.1,{'speed_kmh':0},self.image_support(source='back:1'))
        self.assertEqual(out,[])

    def test_narrow_occlusion_sliver_cannot_create_size_based_flying_ghost(self):
        tracker=WorldTracks();self.seed_image_track(tracker)
        support=self.image_support();support['front:1']['bbox'][2]*=.35
        for i in range(3,25):out,_=tracker.update([],i*.1,{'speed_kmh':0},support)
        self.assertFalse(any(a.tracking_state=='image_supported' for a in out))
        self.assertIn('one',tracker.tracks) # Identity retained, invalid pose not drawn.

    def test_box_shrink_cannot_override_an_established_velocity(self):
        tracker=WorldTracks();self.seed_image_track(tracker)
        support=self.image_support()
        # Similar width/height change passes aspect and old absolute speed
        # gates, but implies an impossible instantaneous range change.
        x,y,w,h=support['front:1']['bbox']
        support['front:1']['bbox']=[x+w*.1,y+h*.1,w*.8,h*.8]
        out,_=tracker.update([],.3,{'speed_kmh':0},support)
        self.assertEqual(len(out),1)
        self.assertEqual(out[0].tracking_state,'predicted')
        self.assertAlmostEqual(out[0].z,15,delta=.1)

    def test_new_image_continuation_does_not_change_side_or_far_track_memory(self):
        for source,z in [('left:1',15),('front:1',55)]:
            tracker=WorldTracks()
            for t in (0,.1,.2):
                tracker.update([self.actor(z=z,source=source)],t,{'speed_kmh':0},self.image_support(source,z))
            for i in range(3,31):
                out,_=tracker.update([],i*.1,{'speed_kmh':0},self.image_support(source,z))
                self.assertFalse(any(a.tracking_state=='image_supported' for a in out))
            self.assertEqual(out,[])

    def test_known_local_id_cannot_steal_stranger_after_its_own_gate_fails(self):
        tracker=WorldTracks()
        a=Actor('a','car',3.5,20,sources=('front:1',))
        b=Actor('b','car',3.5,40,sources=('front:2',))
        for t in (0,.1,.2):tracker.update([a,b],t,{'speed_kmh':0})
        out,events=tracker.update([replace(a,z=40)],.3,{'speed_kmh':0})
        self.assertFalse(any(v.key=='b' and v.tracking_state=='observed' for v in out))
        self.assertEqual(tracker.tracks['b']['sources'],{'front:2'})
        self.assertTrue(any(e.get('id')=='a' and 'outside' in e['reason'] for e in events))

    def test_distinct_live_ids_from_same_camera_cannot_merge_between_frames(self):
        tracker=WorldTracks();self.seed(tracker)
        out,_=tracker.update([self.actor(z=15.5,source='front:2',key='two')],.3,{'speed_kmh':0})
        self.assertEqual(next(a for a in out if a.tracking_state=='observed').key,'two')
        self.assertEqual(tracker.tracks['one']['sources'],{'front:1'})

    def test_filter_overlap_restores_disjoint_camera_measurements(self):
        tr=WorldTracks()
        a=Actor('a','car',0,10,sources=('front:1',))
        b=Actor('b','car',0,16,sources=('front:2',))
        tr.update([a,b],0,{'speed_kmh':0,'heading':0})
        tr.tracks['b']['state'][1]=14.5
        tr.tracks['b']['cov']*=.001
        out,events=tr.update([a,b],.1,{'speed_kmh':0,'heading':0})
        self.assertEqual([(v.x,v.z) for v in out],[(0,10),(0,16)])
        self.assertTrue(any('filter overlap' in e['reason'] for e in events))

    def test_abrupt_stop_or_turn_is_not_smoothed_into_an_offline_bridge(self):
        self.assertFalse(bridge_motion_consistent([0,20],[0,0],.6))
        self.assertFalse(bridge_motion_consistent([0,10],[10,0],.6))
        self.assertTrue(bridge_motion_consistent([0,20],[1,19],.6))
    def actor(self,x=3.5,z=15,source='front:1',key='one'):
        return Actor(key,'car',x,z,sources=(source,))

    def seed(self,tracker,actor=None):
        for t in (0,.1,.2): tracker.update([actor or self.actor()],t,{'speed_kmh':0,'heading':0})

    def test_occlusion_is_predicted_not_observed_and_expires(self):
        tr=WorldTracks();self.seed(tr)
        out,_=tr.update([],.3,{'speed_kmh':0,'heading':0})
        self.assertEqual(len(out),1);self.assertEqual(out[0].tracking_state,'predicted')
        self.assertEqual(out[0].sources,());self.assertAlmostEqual(out[0].prediction_age,.1)
        first=out[0].position_uncertainty
        for t in (.4,.5):out,_=tr.update([],t,{'speed_kmh':0,'heading':0})
        self.assertGreater(out[0].position_uncertainty,first)
        for t in np.arange(.6,1.71,.1):tr.update([],float(t),{'speed_kmh':0,'heading':0})
        out,_=tr.update([],1.8,{'speed_kmh':0,'heading':0});self.assertEqual(out,[])

    def test_single_frame_false_positive_has_no_memory(self):
        tr=WorldTracks();tr.update([self.actor()],0,{'speed_kmh':0})
        self.assertEqual(tr.update([],.1,{'speed_kmh':0})[0],[])

    def test_reappearance_with_new_camera_retains_identity(self):
        tr=WorldTracks();self.seed(tr);tr.update([],.3,{'speed_kmh':0})
        out,_=tr.update([self.actor(source='left:9',key='new')],.4,{'speed_kmh':0})
        self.assertEqual(len(out),1);self.assertEqual(out[0].key,'one')
        self.assertEqual(out[0].tracking_state,'observed')

    def test_adjacent_lane_cannot_steal_hidden_track(self):
        tr=WorldTracks();self.seed(tr)
        out,_=tr.update([self.actor(x=0,source='front:2',key='two')],.3,{'speed_kmh':0})
        self.assertEqual(len(out),2)
        self.assertEqual({a.key:a.tracking_state for a in out},{'two':'observed','one':'predicted'})

    def test_ego_translation_and_heading_transform_stationary_world(self):
        tr=WorldTracks()
        for i in range(3):
            tr.update([self.actor(x=0,z=20-i)],i*.1,{'speed_kmh':36,'heading':0})
        out,_=tr.update([],.3,{'speed_kmh':36,'heading':0})
        self.assertAlmostEqual(out[0].z,17,places=4)
        tr=WorldTracks();self.seed(tr,self.actor(x=0,z=20))
        out,_=tr.update([],.3,{'speed_kmh':36,'heading':90})
        self.assertAlmostEqual(out[0].x,20-.5/np.sqrt(2),places=4)
        self.assertAlmostEqual(out[0].z,-.5/np.sqrt(2),places=4)

    def test_stopped_heading_noise_does_not_rotate_the_scene(self):
        tr=WorldTracks();self.seed(tr)
        out,_=tr.update([],.3,{'speed_kmh':0,'heading':20})
        self.assertAlmostEqual(out[0].x,3.5);self.assertAlmostEqual(out[0].z,15)

    def test_highway_vehicle_initial_velocity_does_not_reject_repeated_observations(self):
        tr=WorldTracks()
        for i in range(10):
            out,events=tr.update([self.actor(z=20)],i*.2,{'speed_kmh':73,'heading':0})
            self.assertEqual(out[0].tracking_state,'observed')
            self.assertFalse(events)
        out,_=tr.update([],2.,{'speed_kmh':73,'heading':0})
        self.assertEqual(out[0].tracking_state,'predicted')
        self.assertAlmostEqual(out[0].z,20,delta=.5)

    def test_telemetry_loss_and_timestamp_gap_disable_prediction(self):
        for t,data in ((.3,None),(2.,{'speed_kmh':0}),(.3,{'speed_kmh':20})):
            tr=WorldTracks();self.seed(tr)
            self.assertEqual(tr.update([],t,data)[0],[])

    def test_jump_rejected_without_duplicate_and_prediction_conflict_suppressed(self):
        tr=WorldTracks();self.seed(tr)
        out,events=tr.update([self.actor(x=25)],.3,{'speed_kmh':0})
        self.assertEqual(len(out),1);self.assertEqual(out[0].tracking_state,'predicted')
        self.assertTrue(any('outside' in e['reason'] for e in events))

    def test_world_covariance_stays_positive(self):
        tr=WorldTracks()
        for i in range(100):
            tr.update([self.actor(x=3.5+.03*np.sin(i))],i*.1,{'speed_kmh':0})
            for track in tr.tracks.values():
                self.assertTrue(np.linalg.eigvalsh(track['cov']).min()>0)
