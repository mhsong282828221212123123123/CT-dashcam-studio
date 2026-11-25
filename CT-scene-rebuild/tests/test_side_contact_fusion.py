import unittest
import numpy as np
from pathlib import Path
import json
from ct_scene.side_contact_fusion import fuse_side_sample, calibrate_side_height_tails
from ct_scene.surrounding import project_contact, project_vehicle_height, rotation


class SideContactFusionTests(unittest.TestCase):
    def setUp(self):
        repo = Path(__file__).resolve().parents[1]
        self.lens = json.loads((repo / 'configs/multicam-optical-candidates.json').read_text())['cameras']
        self.proto = json.loads((repo / 'configs/surrounding-prototype.json').read_text())['cameras']
        self.dims = [4.6, 1.9, 1.48]

    def test_clear_side_box_keeps_uncalibrated_mount_uncertainty(self):
        row = {
            'bbox_xyxy': [280.0, 440.0, 480.0, 580.0],
            'support': 'detector_observation',
            'quality_flags': [],
        }
        out = fuse_side_sample(
            row, 1280, 960, self.lens['left_repeater'], self.proto['left_repeater'],
            self.dims, project_contact, project_vehicle_height,
            contact_uv=[0.52, 0.97], camera='left_repeater',
        )
        self.assertEqual(out['projection_support'], 'gated_side_ground_contact_fusion')
        self.assertGreater(out['contact_fusion_weight'], .5)
        self.assertLess(out['contact_fusion_weight'], .95)
        self.assertGreater(out['position_sigma_m'], 1.2)
        self.assertGreater(out['diagnostics']['pitch_sigma_m'],0)

    def test_truncated_or_tiny_box_falls_back_to_height_prior_with_large_uncertainty(self):
        for row in [
            {'bbox_xyxy': [800.0, 300.0, 1278.0, 720.0], 'support': 'detector_observation', 'quality_flags': ['truncated']},
            {'bbox_xyxy': [180.0, 492.0, 204.0, 520.0], 'support': 'detector_observation', 'quality_flags': []},
        ]:
            out = fuse_side_sample(
                row, 1280, 960, self.lens['left_repeater'], self.proto['left_repeater'],
                self.dims, project_contact, project_vehicle_height,
                contact_uv=[0.5, 0.98], camera='left_repeater',
            )
            self.assertEqual(out['projection_support'], 'apparent_vehicle_height_prior')
            self.assertEqual(out['contact_fusion_weight'], 0.0)
            self.assertGreater(out['position_sigma_m'], 1.5)

    def test_elevated_curb_outlier_ray_beyond_height_range_is_rejected(self):
        # Wide box near horizon: apparent height says ~16m, but bottom y=482 shoots ground ray >35m
        row = {
            'bbox_xyxy': [500.0, 380.0, 720.0, 482.0],
            'support': 'detector_observation',
            'quality_flags': [],
        }
        out = fuse_side_sample(
            row, 1280, 960, self.lens['right_repeater'], self.proto['right_repeater'],
            self.dims, project_contact, project_vehicle_height,
            contact_uv=[0.5, 0.99], camera='right_repeater',
        )
        self.assertEqual(out['projection_support'], 'apparent_vehicle_height_prior')
        self.assertEqual(out['contact_fusion_weight'], 0.0)

    def test_tail_scale_calibration_aligns_ungated_tail_without_jump(self):
        mount = dict(self.proto['left_repeater'],pitch_calibration_accepted=True)
        times = np.arange(8, dtype=float)
        ego = np.zeros((8, 3), float)
        yaws = np.zeros(8, float)
        samples = []
        # 6 gated samples where true contact range is 0.9x apparent height range, plus 2 ungated tail samples
        for i in range(8):
            rel_h = np.array([-15.0 - i, 9.0, 0.0])
            rel_f = mount['offset_W_ego_m'] + 0.9 * (rel_h - mount['offset_W_ego_m'])
            rel_f[2] = 0.0
            gated = i < 6
            p_init = rel_f if gated else rel_h
            samples.append({
                'time_s': float(i),
                'position_W': p_init.tolist(),
                'unrefined_position_W': p_init.tolist(),
                'contact_fusion_weight': 0.92 if gated else 0.0,
                '_rel_fused_ego': rel_f.tolist(),
                '_rel_height_ego': rel_h.tolist(),
            })
        scale = calibrate_side_height_tails(samples, times, ego, yaws, mount, rotation)
        self.assertAlmostEqual(scale, 0.9, places=2)
        # Tail sample 7 should be scaled near ~0.9 instead of jumping back to unscaled y=9.0
        self.assertLess(samples[7]['position_W'][1],8.5)


    def test_ground_projection_that_is_much_too_near_cannot_dominate(self):
        row=dict(bbox_xyxy=[300,400,450,550],support='detector_observation',quality_flags=[])
        mount=dict(self.proto['left_repeater'])
        off=np.array(mount['offset_W_ego_m'])
        out=fuse_side_sample(row,1280,960,self.lens['left_repeater'],mount,self.dims,
            lambda *args,**kwargs:off+np.array([5,0,0]),lambda *args:off+np.array([25,0,0]))
        self.assertEqual(out['contact_fusion_weight'],0.)
        np.testing.assert_allclose(out['relative_ego_m'],off+np.array([25,0,0]))

    def test_unknown_pitch_does_not_calibrate_tail_from_compressed_contacts(self):
        samples=[]
        for i in range(8):
            samples.append(dict(time_s=float(i),position_W=[-15.,9.,0.],unrefined_position_W=[-15.,9.,0.],contact_fusion_weight=.95 if i<6 else 0.,_rel_fused_ego=[-8.,4.,0.],_rel_height_ego=[-15.,9.,0.],_is_truncated=False))
        result=calibrate_side_height_tails(samples,np.arange(8),np.zeros((8,3)),np.zeros(8),self.proto['left_repeater'],rotation)
        self.assertIsNone(result);self.assertEqual(samples[-1]['position_W'],[-15.,9.,0.])
