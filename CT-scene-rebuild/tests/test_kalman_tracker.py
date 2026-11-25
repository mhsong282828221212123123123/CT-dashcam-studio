import unittest
import numpy as np
from ct_scene.kalman_tracker import smooth_actor_rts


class KalmanTrackerTests(unittest.TestCase):
    def setUp(self):
        self.road = {
            'boundaries_y': [3.5, 0.0, -3.5],
            'reference_line': {'points_W_xy': [[0.0, 0.0], [300.0, 0.0]], 'supported_extent_m': [0.0, 300.0]},
            'junctions': [],
            'left_pockets': [],
        }
        self.ego_t = np.linspace(0.0, 10.0, 51)
        self.ego_xyz = np.c_[self.ego_t * 15.0, np.zeros(51), np.zeros(51)]

    def test_simultaneous_front_and_side_observations_fuse_by_inverse_covariance(self):
        # Front has tight lateral (y=1.75) but noisy x; side has tight longitudinal x but noisier y
        raw_rows = []
        samples = []
        for i in range(10):
            t = i * 0.2
            true_x = 20.0 + 18.0 * t
            f_row = {
                'time_s': t, 'camera_id': 'front', 'support': 'detector_observation',
                'position_W': [true_x + (3.5 if i % 2 == 0 else -3.5), 1.75, 0.0],
                'unrefined_position_W': [true_x + (3.5 if i % 2 == 0 else -3.5), 1.75, 0.0],
                'position_sigma_m': 3.5, 'bbox_xyxy': [500, 440, 620, 560], 'image_width': 1280,
            }
            s_row = {
                'time_s': t, 'camera_id': 'left_repeater', 'support': 'detector_observation',
                'projection_support': 'gated_side_ground_contact_fusion',
                'position_W': [true_x, 1.75 + (1.2 if i % 2 == 0 else -1.2), 0.0],
                'unrefined_position_W': [true_x, 1.75 + (1.2 if i % 2 == 0 else -1.2), 0.0],
                'position_sigma_m': 0.55, 'bbox_xyxy': [300, 440, 520, 600], 'image_width': 1280,
            }
            raw_rows.extend([f_row, s_row])
            samples.append(dict(f_row, lane_index=0))

        actor = {'global_id': 'fused-1', 'lane_index': 0, 'samples': samples}
        res = smooth_actor_rts(actor, self.road, self.ego_t, self.ego_xyz, raw_member_rows=raw_rows)
        self.assertTrue(res['applied'])
        self.assertEqual(res['multi_camera_fused_steps'], 10)
        xs = np.array([s['position_W'][0] for s in actor['samples']])
        ys = np.array([s['position_W'][1] for s in actor['samples']])
        expected_x = 20.0 + 18.0 * np.arange(10) * 0.2
        self.assertLess(np.max(np.abs(xs - expected_x)), 0.75)
        self.assertLess(np.max(np.abs(ys - 1.75)), 0.45)
        self.assertTrue(all('kalman_sigma_m' in s for s in actor['samples']))

    def test_single_frame_far_outlier_is_rejected_by_rts_smoother(self):
        samples = []
        for i in range(12):
            t = i * 0.2
            tx = 50.0 + 20.0 * t
            # Frame 6 has a +35m occlusion glitch
            noisy_x = tx + (35.0 if i == 6 else 0.0)
            samples.append({
                'time_s': t, 'camera_id': 'front', 'support': 'detector_observation',
                'position_W': [noisy_x, 1.75, 0.0], 'unrefined_position_W': [noisy_x, 1.75, 0.0],
                'position_sigma_m': 2.0, 'lane_index': 0, 'bbox_xyxy': [580, 460, 660, 520], 'image_width': 1280,
            })
        actor = {'global_id': 'car-outlier', 'lane_index': 0, 'samples': samples}
        res = smooth_actor_rts(actor, self.road, self.ego_t, self.ego_xyz)
        self.assertTrue(res['applied'])
        true_x6 = 50.0 + 20.0 * (6 * 0.2)
        self.assertLess(abs(actor['samples'][6]['position_W'][0] - true_x6), 2.5)

    def test_cross_street_vehicle_is_not_dragged_into_corridor_lane(self):
        samples = []
        for i in range(14):
            t = i * 0.2
            # Observed laterally 40..52 m away from the corridor, nearly stationary
            y = 40.0 + 0.9 * t
            samples.append({
                'time_s': t, 'camera_id': 'front', 'support': 'detector_observation',
                'position_W': [120.0, y, 0.0], 'unrefined_position_W': [120.0, y, 0.0],
                'position_sigma_m': 1.5, 'lane_index': 0, 'bbox_xyxy': [500, 470, 600, 520], 'image_width': 1280,
            })
        actor = {'global_id': 'cross', 'lane_index': 0, 'samples': samples}
        smooth_actor_rts(actor, self.road, self.ego_t, self.ego_xyz)
        ys = np.array([s['position_W'][1] for s in actor['samples']])
        self.assertGreater(ys.min(), 30.0)
        steps = np.linalg.norm(np.diff([s['position_W'][:2] for s in actor['samples']], axis=0), axis=1) / 0.2
        self.assertLess(steps.max(), 14.0)

    def test_yaw_snap_is_rate_limited(self):
        from ct_scene.kalman_tracker import limit_yaw_rate
        rows = [{'time_s': i * 0.2, 'yaw_rad': 0.7 if i < 8 else 1.57} for i in range(16)]
        actor = {'samples': rows}
        self.assertGreater(limit_yaw_rate(actor, 1.4), 0)
        y = np.array([r['yaw_rad'] for r in rows])
        self.assertLessEqual(np.max(np.abs(np.diff(y)) / 0.2), 1.4 + 1e-6)
        lead = {'frozen_lead': True, 'samples': [dict(r) for r in rows]}
        self.assertEqual(limit_yaw_rate(lead), 0)

    def test_frozen_lead_is_never_modified(self):
        actor = {
            'global_id': 'lead', 'frozen_lead': True,
            'samples': [{'time_s': float(i), 'position_W': [10.0 * i, 0.0, 0.0]} for i in range(5)],
        }
        res = smooth_actor_rts(actor, self.road, self.ego_t, self.ego_xyz)
        self.assertFalse(res['applied'])
        self.assertEqual(actor['samples'][2]['position_W'], [20.0, 0.0, 0.0])

    def test_end_of_track_backward_glitch_does_not_pull_trajectory_back(self):
        samples = []
        for i in range(10):
            t = i * 0.2
            tx = 290.0 + 25.0 * t
            # Final frame has a -28m backward ID switch glitch past supported_extent_m=300m
            noisy_x = tx - (28.0 if i == 9 else 0.0)
            samples.append({
                'time_s': t, 'camera_id': 'front', 'support': 'detector_observation',
                'position_W': [noisy_x, 1.75, 0.0], 'unrefined_position_W': [noisy_x, 1.75, 0.0],
                'position_sigma_m': 3.5, 'lane_index': 0, 'bbox_xyxy': [590, 470, 650, 515], 'image_width': 1280,
            })
        actor = {'global_id': 'car-end-glitch', 'lane_index': 0, 'samples': samples}
        res = smooth_actor_rts(actor, self.road, self.ego_t, self.ego_xyz)
        self.assertTrue(res['applied'])
        true_x8 = 290.0 + 25.0 * (8 * 0.2)
        self.assertLess(abs(actor['samples'][8]['position_W'][0] - true_x8), 2.5)


class CrossingWorldFrameTests(unittest.TestCase):
    def scene(self,angle=0):
        c,s=np.cos(angle),np.sin(angle);R=np.array([[c,-s],[s,c]])
        ts=np.arange(0,4.001,.2);ego=np.c_[ts*3,np.zeros(len(ts)),np.zeros(len(ts))];ego[:,:2]=ego[:,:2]@R.T
        samples=[]
        for i,t in enumerate(ts):
            xy=R@np.array([30+.25*np.sin(i),-12+6*t])
            samples.append(dict(time_s=float(t),camera_id='front',position_W=[*xy,0],unrefined_position_W=[*xy,0],position_sigma_m=3.,support='detector_observation',yaw_rad=angle+np.pi/2,bbox_xyxy=[500,440,550,480]))
        return ts,ego,dict(global_id='cross',lane_index=-2,samples=samples)

    def test_crossing_motion_does_not_depend_on_ego_route_shape(self):
        import copy
        ts,ego,a=self.scene();b=copy.deepcopy(a)
        straight=dict(reference_line=dict(points_W_xy=[[0,0],[100,0]],supported_extent_m=[0,100]),junctions=[])
        turn=dict(reference_line=dict(points_W_xy=[[0,0],[20,0],[20,60]],supported_extent_m=[0,80]),junctions=[])
        smooth_actor_rts(a,straight,ts,ego);smooth_actor_rts(b,turn,ts,ego)
        np.testing.assert_allclose([r['position_W'] for r in a['samples']],[r['position_W'] for r in b['samples']],atol=1e-8)
        self.assertLess(np.max(np.linalg.norm(np.array([r['position_W'][:2] for r in a['samples']])-np.array([[30,-12+6*t] for t in ts]),axis=1)),1.)

    def test_crossing_covariance_and_dynamics_rotate_with_scene(self):
        angle=1.1;c,s=np.cos(angle),np.sin(angle);R=np.array([[c,-s],[s,c]])
        ts,ego,a=self.scene();ts2,ego2,b=self.scene(angle)
        road=dict(reference_line=dict(points_W_xy=[[0,0],[100,0]],supported_extent_m=[0,100]),junctions=[])
        r1=smooth_actor_rts(a,road,ts,ego);r2=smooth_actor_rts(b,road,ts2,ego2)
        self.assertEqual(r1['coordinate_frame'],'world_xy_crossing')
        expected=np.array([r['position_W'][:2] for r in a['samples']])@R.T
        np.testing.assert_allclose(expected,[r['position_W'][:2] for r in b['samples']],atol=1e-7)


    def test_first_range_glitch_does_not_become_persistent_crossing_velocity(self):
        ts,ego,a=self.scene()
        a['samples'][0]['position_W'][0]+=25
        a['samples'][0]['unrefined_position_W'][0]+=25
        smooth_actor_rts(a,{},ts,ego)
        p=np.array([r['position_W'][:2] for r in a['samples']]);expected=np.array([[30,-12+6*t] for t in ts])
        self.assertLess(np.max(np.linalg.norm(p[3:]-expected[3:],axis=1)),2.)
        self.assertLess(np.max(np.linalg.norm(np.diff(p,axis=0)/.2,axis=1)),12.)


    def test_bearing_guard_does_not_confuse_depth_uncertainty_with_sideways_motion(self):
        from ct_scene.crossing_smoother import bearing_conflict
        ts=np.arange(8)*.2
        obs=[[(np.array([60.,0.]),100.,.25,'front',np.diag([100.,.25]))] for t in ts]
        self.assertFalse(bearing_conflict(np.array([[80.,0.] for t in ts]),obs,ts)['trigger'])
        self.assertTrue(bearing_conflict(np.array([[60.,3.] for t in ts]),obs,ts)['trigger'])
        self.assertFalse(bearing_conflict(np.array([[60.,3.] for t in ts[:4]]),obs[:4],ts[:4])['trigger'])
