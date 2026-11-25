import json
import tempfile
import unittest
from pathlib import Path
import cv2
import numpy as np
from ct_scene.front_pitch_fusion import (
    estimate_front_pitch_profile,
    project_front_ray_and_depths,
    fuse_front_track_depths,
)
from ct_scene.surrounding import rotation

PROFILE = {'model': 'SIMPLE_RADIAL', 'resolution': [1280, 960], 'params': [1653.0, 640.0, 480.0, 0.0]}
MOUNT = {'height_m': 1.3, 'yaw_deg': 0.0, 'offset_W_ego_m': [0.0, 0.0, 0.0]}


class FrontPitchFusionTests(unittest.TestCase):
    def test_pitch_compensation_counteracts_uphill_horizon_shift_at_far_range(self):
        # Distant 65m car: nominal bottom y = 480 + 1653 * (1.3 / 65) = 513.06 px
        # Uphill road rises by 8.2 px (delta_pitch_rad = +0.005 rad), shifting bottom up to 504.8 px
        row_uphill = {'bbox_xyxy': [618.0, 468.0, 662.0, 504.8], 'support': 'detector_observation', 'quality_flags': []}
        uncomp = project_front_ray_and_depths(row_uphill, PROFILE, MOUNT, 1.5, delta_pitch_rad=0.0)
        comp = project_front_ray_and_depths(row_uphill, PROFILE, MOUNT, 1.5, delta_pitch_rad=0.005)
        # Uncompensated blows up to ~86m; compensated stays close to true ~65m!
        self.assertGreater(uncomp['z_contact'], 82.0)
        self.assertAlmostEqual(comp['z_contact'], 65.0, delta=2.0)

    def test_near_field_contact_ignores_pitch_ramp_and_size_bias(self):
        # Close 12m car: bottom y = 480 + 1653 * (1.3 / 12) = 659.1 px
        row_near = {'bbox_xyxy': [530.0, 440.0, 750.0, 659.1], 'support': 'detector_observation', 'quality_flags': []}
        info = project_front_ray_and_depths(row_near, PROFILE, MOUNT, 1.5, delta_pitch_rad=0.008)
        self.assertAlmostEqual(info['z_contact'], 12.0, delta=0.05)

    def test_occluded_or_border_clipped_depth_blowup_is_suppressed_by_size_fusion(self):
        times = np.arange(6, dtype=float)
        ego = np.zeros((6, 3), float)
        yaws = np.zeros(6, float)
        entries = []
        # 5 clear mid-range samples at z=25m, plus 1 occluded-bottom spike where y2=496 -> z_contact=134m!
        for i in range(6):
            y2 = 480.0 + 1653.0 * (1.3 / 25.0) if i < 5 else 496.0
            h_px = 1653.0 * (1.5 / 25.0)
            row = {'bbox_xyxy': [590.0, y2 - h_px, 690.0, y2], 'support': 'detector_observation', 'quality_flags': []}
            info = project_front_ray_and_depths(row, PROFILE, MOUNT, 1.5)
            sample = {'time_s': float(i), 'position_W': [0.0, 0.0, 0.0], 'unrefined_position_W': [0.0, 0.0, 0.0]}
            entries.append({'ray_info': info, 'sample': sample})
        kappa = fuse_front_track_depths(entries, MOUNT, 2.3, rotation, times, ego, yaws)
        self.assertAlmostEqual(kappa, 1.0, delta=0.05)
        # Spike frame 5 must stay near 25m + 2.3m half-length (~27.3m), NOT blow up to 136m!
        self.assertLess(entries[5]['sample']['position_W'][0], 34.0)

    def test_road_mask_pitch_estimator_masks_horizon_blocking_vehicles(self):
        with tempfile.TemporaryDirectory() as tmp:
            rm = Path(tmp)
            frames_meta = []
            front_frames = []
            obs = {}
            for i in range(10):
                fid = f'f{i}'
                mask = np.zeros((960, 1280), np.uint8)
                # First 5 frames horizon at y=506; last 5 frames uphill horizon rises to y=498
                hy = 506 if i < 5 else 498
                mask[hy:900, 200:1080] = 255
                # Place a vehicle blocking central columns [580..700] from y=450..620 on frame 7
                if i == 7:
                    mask[450:620, 580:700] = 0
                    obs[fid] = [{'bbox_xyxy': [580.0, 450.0, 700.0, 620.0]}]
                cv2.imwrite(str(rm / f'{fid}.png'), mask)
                frames_meta.append({'frame_id': fid, 'road': f'{fid}.png', 'time_s': float(i)})
                front_frames.append({'frame_id': fid, 'camera_time_s': float(i)})
            (rm / 'frames.json').write_text(json.dumps(frames_meta), encoding='utf-8')
            prof = estimate_front_pitch_profile(rm, front_frames, obs, PROFILE)
            self.assertEqual(prof['status'], 'estimated_from_unoccluded_road_horizon')
            # Later uphill frames should have positive delta_pitch relative to earlier frames
            self.assertGreater(prof['by_frame_id']['f9'], prof['by_frame_id']['f0'])
