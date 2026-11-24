import math,unittest
import numpy as np
from core.scene_fusion import CameraGeometry,corners,dimensions
from core.scene_motion_bundle import fit_motion_bundle


class MotionBundleTests(unittest.TestCase):
    def fixture(self,stationary=False,jump=False):
        camera=CameraGeometry(horizon=500);observations=[]
        for i,t in enumerate(np.arange(0,2.1,.1)):
            yaw=math.pi if stationary else math.pi/2-.2*t
            point=np.array([0.,30.]) if stationary else np.array([-4+4*t,30+.4*t*t])
            if jump and i>=10:point[0]+=15
            p=camera.visible_bounds(corners(*point,dimensions('car'),yaw=yaw,kind='car'))
            observations.append(dict(index=i,time=float(t),camera=camera,bbox=np.r_[p[:2],p[2:]-p[:2]],
                ego_origin=np.zeros(2),ego_rotation=np.eye(2),stationary=stationary,
                reviewed_world_yaw=math.pi if stationary else None))
        return observations

    def test_whole_track_has_smooth_motion_and_withheld_image_agreement(self):
        observations=self.fixture();fit=fit_motion_bundle(observations)
        self.assertIsNotNone(fit);self.assertTrue(fit['accepted'])
        self.assertGreater(fit['validation_iou_median'],.95)
        self.assertLess(fit['lateral_speed_p95_mps'],.2)
        positions=np.array([p['position'] for p in fit['positions']])
        self.assertLess(np.linalg.norm(np.diff(positions,n=2,axis=0),axis=1).max(),.1)
        self.assertTrue(all(np.array_equal(o['ego_origin'],[0,0]) for o in observations))

    def test_reviewed_stopped_front_direction_does_not_rotate_in_place(self):
        fit=fit_motion_bundle(self.fixture(stationary=True))
        self.assertTrue(fit['accepted']);self.assertLess(fit['max_speed_mps'],.05)
        self.assertLess(max(abs((p['yaw']-math.pi+math.pi)%(2*math.pi)-math.pi) for p in fit['positions']),.03)

    def test_an_identity_teleport_is_not_a_valid_vehicle_trajectory(self):
        fit=fit_motion_bundle(self.fixture(jump=True))
        self.assertTrue(fit is None or not fit['accepted'])

    def test_valid_fit_is_invariant_to_large_world_origin_translation(self):
        original=self.fixture();translated=self.fixture()
        for o in translated:o['ego_origin']=np.array([2000.,5000.])
        a=fit_motion_bundle(original);b=fit_motion_bundle(translated)
        self.assertTrue(a['accepted']);self.assertTrue(b['accepted'])
        np.testing.assert_allclose([p['position'] for p in a['positions']],
                                   [p['position'] for p in b['positions']],atol=.02)

    def test_highway_travel_beyond_fixed_world_bounds_remains_camera_relative(self):
        camera=CameraGeometry(horizon=500);observations=[]
        bounds=camera.visible_bounds(corners(0,25,dimensions('car'),kind='car'))
        box=np.r_[bounds[:2],bounds[2:]-bounds[:2]]
        for i,t in enumerate(np.arange(0,35.1,.7)):
            observations.append(dict(index=i,time=float(t),camera=camera,bbox=box,
                ego_origin=np.array([0.,31*t]),ego_rotation=np.eye(2),initial_yaw=0.))
        fit=fit_motion_bundle(observations,max_speed=40.)
        self.assertIsNotNone(fit);self.assertTrue(fit['accepted'])
        self.assertGreater(fit['positions'][-1]['world_position'][1],1000.)
        self.assertLess(max(abs(p['position'][1]-25) for p in fit['positions']),.2)

    def test_stationary_box_mismatch_cannot_rotate_a_held_input_direction(self):
        observations=self.fixture(stationary=True)
        c,s=math.cos(.7),math.sin(.7);R=np.array([[c,-s],[s,c]])
        for o in observations:
            o['reviewed_world_yaw']=None;o['initial_yaw']=math.pi;o['ego_rotation']=R
            p=o['camera'].visible_bounds(corners(0,30,dimensions('car'),yaw=math.pi+.18,kind='car'))
            o['bbox']=np.r_[p[:2],p[2:]-p[:2]]
        fit=fit_motion_bundle(observations)
        self.assertIsNotNone(fit)
        self.assertEqual(fit['stationary_heading_prior_frames'],len(observations))
        self.assertLess(max(abs((p['yaw']-math.pi+math.pi)%(2*math.pi)-math.pi) for p in fit['positions']),.03)
        self.assertIn('not independently measured',fit['heading_accuracy'])

    def test_unverified_road_height_remains_an_explicit_ghost_hypothesis(self):
        observations=self.fixture()
        for o in observations:
            o['bbox'][1]-=80 # A different road level must not become a false tyre plane.
        fit=fit_motion_bundle(observations,ground_contact=False)
        self.assertTrue(fit['projection_hypothesis']);self.assertFalse(fit['accepted'])
        self.assertGreater(fit['validation_iou_median'],.95)
        self.assertGreater(fit['contact_error_p95'],.15)
        self.assertIn('unverified',fit['validation_alignment'])
