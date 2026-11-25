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
