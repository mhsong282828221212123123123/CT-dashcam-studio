import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from core.scene_analysis import CameraReader, Detection, PixelTracker, MotionTracker, SceneDetector, discover_clip, letterbox, optimal_pairs
from tools.analyze_scene import analyze


class SceneAnalysisTests(unittest.TestCase):
    def test_similar_area_does_not_join_distant_car_to_tall_entering_sliver(self):
        tracker=MotionTracker('front')
        for i,x in enumerate([180,140,100]):
            previous=tracker.update([Detection('car',.9,[x,500,100,50])],i*.1)[0]
        current=tracker.update([Detection('truck',.9,[60,400,35,250])],.3)[0]
        self.assertNotEqual(previous['track_id'],current['track_id'])
        self.assertEqual(current['track_status'],'tentative')

    def test_global_assignment_beats_greedy(self):
        self.assertEqual(set(optimal_pairs([[.9,.8],[.85,0]])), {(0,1),(1,0)})
        self.assertEqual(optimal_pairs(np.zeros((2,3))), [])

    def test_vehicle_subclass_duplicates_are_suppressed(self):
        detector=SceneDetector.__new__(SceneDetector); detector.threshold=.35
        output=np.zeros((1,84,2),np.float32)
        output[0,:4,:]=np.array([160,160,80,80])[:,None]
        output[0,6,0]=.9; output[0,11,1]=.8
        results=detector.decode(output,(320,320),(1,1,0,0))
        self.assertEqual(len(results),1); self.assertEqual(results[0].kind,'car')

    def test_invalid_metadata_releases_camera(self):
        from unittest.mock import Mock,patch
        cap=Mock(); cap.get.side_effect=[30,float('nan')]
        with patch('core.scene_analysis.cv2.VideoCapture',return_value=cap):
            with self.assertRaises(ValueError): CameraReader('broken.mp4')
        cap.release.assert_called_once()

    def test_motion_and_weak_detections_preserve_identity_without_seeding_noise(self):
        tracker=MotionTracker('front')
        self.assertEqual(tracker.update([Detection('car',.3,[0,0,20,20])],0),[])
        first=tracker.update([Detection('car',.9,[0,0,20,20])],.2)[0]
        second=tracker.update([Detection('car',.9,[12,0,20,20])],.4)[0]
        third=tracker.update([Detection('truck',.9,[30,0,20,20])],.6)[0]
        fourth=tracker.update([Detection('car',.3,[48,0,20,20])],.8)[0]
        self.assertEqual({o['track_id'] for o in [first,second,third,fourth]}, {'front:1'})
        self.assertEqual(third['kind'],'car')
        self.assertEqual(fourth['track_status'],'confirmed')
        self.assertEqual(tracker.update([],1.),[])
        new=tracker.update([Detection('car',.9,[48,0,20,20])],2.)[0]
        self.assertNotEqual(new['track_id'],first['track_id'])

    def test_cancellation_never_publishes_partial_results(self):
        from core.scene_pipeline import AnalysisCancelled
        class FakeDetector:
            model_path=__file__
            def detect(self,frame): return []
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); front=root/'2026-01-01_00-00-00-front.mp4'
            self.make_video(front,10,2)
            with self.assertRaises(AnalysisCancelled):
                analyze(front,root/'result',detector=FakeDetector(),cancelled=lambda:True)
            self.assertFalse((root/'result').exists())
            self.assertEqual(list(root.glob('.ct-scene-*')),[])

    def test_letterbox_decoding_preserves_coordinates_and_class_nms(self):
        frame = np.zeros((100, 200, 3), np.uint8)
        _, transform = letterbox(frame)
        output = np.zeros((1, 84, 3), np.float32)
        # Same original box (50,25,100,50); two cars and an overlapping person.
        output[0, :4, :] = np.array([160, 160, 160, 80])[:, None]
        output[0, 6, :2] = [.9, .8]
        output[0, 4, 2] = .85
        detector = SceneDetector.__new__(SceneDetector)
        detector.threshold = .35
        results = detector.decode(output, frame.shape[:2], transform)
        self.assertEqual({r.kind for r in results}, {'car', 'person'})
        self.assertEqual(len(results), 2)
        for result in results:
            np.testing.assert_allclose(result.bbox, [50, 25, 100, 50])

    def test_tracker_is_one_to_one_expires_and_does_not_invent_positions(self):
        tracker = PixelTracker('front')
        detection = Detection('car', .9, [10, 10, 20, 20])
        first = tracker.update([detection], 0)[0]
        second = tracker.update([detection, detection], .2)
        self.assertEqual(second[0]['track_id'], first['track_id'])
        self.assertNotEqual(second[0]['track_id'], second[1]['track_id'])
        self.assertIsNone(second[0]['position_m'])
        self.assertEqual(tracker.update([], .4), [])
        later = tracker.update([detection], 2)[0]
        self.assertNotEqual(later['track_id'], first['track_id'])
        with self.assertRaises(ValueError):
            tracker.update([], 2)

    @staticmethod
    def make_video(path, fps, count):
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'), fps, (64, 48))
        if not writer.isOpened():
            raise RuntimeError('Test video encoder unavailable')
        for index in range(count):
            writer.write(np.full((48, 64, 3), index*10, np.uint8))
        writer.release()

    def test_multicamera_sampling_missing_data_and_no_overwrite(self):
        class FakeDetector:
            model_path = __file__
            def detect(self, frame):
                return [Detection('car', .9, [5, 5, 20, 20])]

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            front = root / '2026-01-01_00-00-00-front.mp4'
            back = root / '2026-01-01_00-00-00-back.mp4'
            self.make_video(front, 10, 10)
            self.make_video(back, 20, 10)
            self.assertEqual(set(discover_clip(front)), {'front', 'back'})
            output = root / 'analysis'
            analyze(front, output, sample_hz=4, duration=1, detector=FakeDetector())
            rows = [json.loads(line) for line in (output/'observations.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows), 4)
            self.assertEqual(rows[1]['cameras']['front']['frame_index'], 3)
            self.assertEqual(rows[1]['cameras']['back']['frame_index'], 5)
            self.assertFalse(rows[2]['cameras']['back']['available'])
            self.assertEqual(rows[2]['cameras']['back']['objects'], [])
            self.assertTrue(all(row['ego'] is None for row in rows))
            self.assertEqual(len(list(output.glob('frame_*.jpg'))), 4)
            with self.assertRaises(FileExistsError):
                analyze(front, output, detector=FakeDetector())
            reader = CameraReader(front)
            try:
                _, first = reader.read_at(0)
                _, repeated = reader.read_at(.01)
                self.assertIs(first, repeated)
                self.assertIsNone(reader.read_at(1)[1])
            finally:
                reader.close()

    def test_failed_analysis_does_not_publish_partial_result(self):
        class BrokenDetector:
            model_path = __file__
            def detect(self, frame):
                raise RuntimeError('inference failure')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            front = root/'2026-01-01_00-00-00-front.mp4'
            self.make_video(front, 10, 2)
            with self.assertRaisesRegex(RuntimeError, 'inference failure'):
                analyze(front, root/'result', detector=BrokenDetector())
            self.assertFalse((root/'result').exists())
            self.assertEqual(list(root.glob('.ct-scene-*')), [])

    def test_sequence_diagnostic_cannot_override_container_timestamps(self):
        from unittest.mock import patch
        from types import SimpleNamespace
        class FakeDetector:
            model_path=__file__
            def detect(self,frame):return []
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);front=root/'2026-01-01_00-00-00-front.mp4'
            left=root/'2026-01-01_00-00-00-left_repeater.mp4'
            self.make_video(front,10,10);self.make_video(left,10,10)
            def decoder(path):
                start=101 if 'left_repeater' in str(path) else 100
                return SimpleNamespace(telemetry_cache={i:dict(frame_seq_no=start+i,speed_mps=0.,speed_kmh=0,
                    heading=0.,steering_deg=0,brake_applied=False) for i in range(10)})
            with patch('core.scene_pipeline.RealTeslaSEIDecoder',side_effect=decoder):
                analyze(front,root/'result',sample_hz=10,duration=.3,detector=FakeDetector())
            rows=[json.loads(s) for s in (root/'result'/'observations.jsonl').read_text().splitlines()]
            self.assertTrue(rows[0]['cameras']['left_repeater']['available'])
            self.assertEqual(rows[1]['cameras']['front']['frame_index'],1)
            self.assertEqual(rows[1]['cameras']['left_repeater']['frame_index'],1)
            self.assertEqual(rows[1]['cameras']['left_repeater']['synchronization'],'container_pts')
            self.assertAlmostEqual(rows[1]['cameras']['left_repeater']['source_time_s'],.1)

    def test_front_lens_is_rectified_before_fresh_detection(self):
        class CentroidDetector:
            model_path=__file__
            def detect(self,frame):
                gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
                y,x=np.nonzero(gray>100)
                return [Detection('car',.9,[float(x.mean())-2,float(y.mean())-2,4.,4.])]
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);front=root/'2026-01-01_00-00-00-front.mp4'
            writer=cv2.VideoWriter(str(front),cv2.VideoWriter_fourcc(*'mp4v'),10,(64,48))
            self.assertTrue(writer.isOpened())
            image=np.zeros((48,64,3),np.uint8);cv2.circle(image,(48,30),4,(255,255,255),-1)
            for _ in range(3):writer.write(image)
            writer.release()
            profile=dict(model='opencv_pinhole',image_size=[64,48],
                camera_matrix=[[40,0,32],[0,40,24],[0,0,1]],distortion_coefficients=[0,0,0,0],
                rectified_camera_matrix=[[20,0,32],[0,20,24],[0,0,1]])
            lens=root/'lens.json';lens.write_text(json.dumps(profile))
            analyze(front,root/'result',duration=.1,detector=CentroidDetector(),camera_calibrations={'front':lens})
            row=json.loads((root/'result'/'observations.jsonl').read_text().splitlines()[0])
            box=row['cameras']['front']['raw_detections'][0]['bbox']
            self.assertAlmostEqual(box[0]+box[2]/2,40.,delta=1.)
            self.assertAlmostEqual(box[1]+box[3]/2,27.,delta=1.)
            meta=json.loads((root/'result'/'manifest.json').read_text())
            self.assertEqual(meta['cameras']['front']['rectification']['coordinate_space'],'rectified_pixels')
