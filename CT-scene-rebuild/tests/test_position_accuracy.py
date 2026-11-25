import json
import tempfile
import unittest
from pathlib import Path
from PIL import Image
from ct_scene.position_accuracy import distance_bin_name, evaluate_contact_labels, evaluate_scene_surrogates


class PositionAccuracyTests(unittest.TestCase):
    def test_distance_bin_partition(self):
        self.assertEqual(distance_bin_name(5.0), '0_20m')
        self.assertEqual(distance_bin_name(19.99), '0_20m')
        self.assertEqual(distance_bin_name(20.0), '20_50m')
        self.assertEqual(distance_bin_name(49.9), '20_50m')
        self.assertEqual(distance_bin_name(50.0), '50m_plus')
        self.assertEqual(distance_bin_name(120.0), '50m_plus')

    def test_contact_label_benchmark_stratifies_by_camera_and_distance(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            img_path = tmp_path / 'src.jpg'
            Image.new('RGB', (1280, 960), (80, 90, 100)).save(img_path)
            ann = {
                'records': [
                    {
                        'id': 'rec-front-near',
                        'split': 'test',
                        'session': 's1',
                        'camera': 'front',
                        'source_image': str(img_path),
                        'crop_rect_xyxy': [540, 500, 740, 720],
                        'bbox_xyxy': [560.0, 520.0, 720.0, 700.0],
                        'tile_transform': {'scale': 1.0, 'dx': 0, 'dy': 0},
                        # True contact slightly inside bbox bottom (v < 1.0)
                        'contact_tile_xy': [100.0, 192.0],
                        'annotation_status': 'visual_reviewed',
                    },
                    {
                        'id': 'rec-side-near',
                        'split': 'validation',
                        'session': 's2',
                        'camera': 'left_repeater',
                        'source_image': str(img_path),
                        'crop_rect_xyxy': [300, 440, 620, 720],
                        'bbox_xyxy': [320.0, 460.0, 600.0, 700.0],
                        'tile_transform': {'scale': 1.0, 'dx': 0, 'dy': 0},
                        'contact_tile_xy': [160.0, 250.0],
                        'annotation_status': 'visual_reviewed',
                    },
                ]
            }
            ann_path = tmp_path / 'ann.json'
            ann_path.write_text(json.dumps(ann), encoding='utf-8')
            repo_root = Path(__file__).resolve().parents[1]
            res = evaluate_contact_labels(
                ann_path,
                repo_root / 'configs' / 'multicam-optical-candidates.json',
                repo_root / 'configs' / 'surrounding-prototype.json',
            )
            self.assertEqual(res['visible_evaluated_records'], 2)
            holdout = res['subsets']['holdout_val_test']
            self.assertEqual(holdout['total_records'], 2)
            pipe = holdout['methods']['current_pipeline']
            self.assertEqual(pipe['overall']['count'], 2)
            self.assertEqual(pipe['by_camera']['front']['count'], 1)
            self.assertEqual(pipe['by_camera']['left_repeater']['count'], 1)
            self.assertGreater(pipe['overall']['mean_m'], 0.0)

    def test_scene_surrogates_detects_raw_oob_jitter_and_disagreement(self):
        with tempfile.TemporaryDirectory() as tmp:
            sc = Path(tmp)
            road = {
                'boundaries_y': [3.5, 0.0, -3.5],
                'reference_line': {'points_W_xy': [[0.0, 0.0], [200.0, 0.0]], 'supported_extent_m': [0.0, 200.0]},
                'junctions': [],
                'left_pockets': [],
            }
            report = {
                'road': road,
                'collision_constraints': {
                    'before': {'overlapping_pair_frames': 5},
                    'after': {'overlapping_pair_frames': 0},
                },
            }
            # Actor with noisy unrefined positions (one outside road y=5.2m) clamped to y=1.75m
            samples = [
                {'time_s': 0.0, 'camera_id': 'left_repeater', 'position_W': [10.0, 1.75, 0.0], 'unrefined_position_W': [10.0, 1.8, 0.0]},
                {'time_s': 0.2, 'camera_id': 'left_repeater', 'position_W': [14.0, 1.75, 0.0], 'unrefined_position_W': [14.0, 5.2, 0.0]},
                {'time_s': 0.4, 'camera_id': 'front', 'position_W': [18.0, 1.75, 0.0], 'unrefined_position_W': [20.5, 1.7, 0.0]},
            ]
            actors = [{'global_id': 'a1', 'lane_index': 0, 'samples': samples}]
            states = [
                {'time_s': 0.0, 'ego_position_W': [0.0, 0.0, 0.0], 'actors': [{'id': 'a1', 'position_W': [10.0, 1.75, 0.0], 'yaw_rad': 0.0, 'ghost': False}]},
                {'time_s': 0.05, 'ego_position_W': [1.0, 0.0, 0.0], 'actors': [{'id': 'a1', 'position_W': [11.0, 1.75, 0.0], 'yaw_rad': 0.0, 'ghost': False, 'collision_correction_W': [0.4, 0.0, 0.0]}]},
                {'time_s': 0.10, 'ego_position_W': [2.0, 0.0, 0.0], 'actors': [{'id': 'a1', 'position_W': [12.0, 1.75, 0.0], 'yaw_rad': 0.0, 'ghost': False}]},
            ]
            edges = [{'a': 'l1', 'b': 'f1', 'kind': 'future_reappearance_with_fov_boundary', 'selected': True, 'distance_m_prior': 2.4}]
            (sc / 'report.json').write_text(json.dumps(report), encoding='utf-8')
            (sc / 'actors.json').write_text(json.dumps(actors), encoding='utf-8')
            (sc / 'render-states.jsonl').write_text('\n'.join(json.dumps(s) for s in states) + '\n', encoding='utf-8')
            (sc / 'association-review.json').write_text(json.dumps(edges), encoding='utf-8')

            out = evaluate_scene_surrogates(sc)
            self.assertEqual(out['carriageway_bounds']['raw_unrefined_out_of_bounds_samples'], 1)
            self.assertEqual(out['carriageway_bounds']['final_center_out_of_bounds_samples'], 0)
            self.assertGreater(out['trajectory_jitter']['raw_projection_jitter']['mean_m'], 1.0)
            self.assertAlmostEqual(out['cross_camera_disagreement']['selected_handoff_edge_error']['mean_m'], 2.4)
            self.assertEqual(out['artificial_displacement']['collision_shifted_actor_frames'], 1)
