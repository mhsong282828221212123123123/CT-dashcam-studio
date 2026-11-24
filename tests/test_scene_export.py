import tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import cv2,numpy as np
from core.driving_scene import Scene
from core.scene_export import export_scene_video,OriginalCameraStrip
from core.scene_pipeline import AnalysisCancelled


class Timeline:
    def __init__(self):self.scenes=[Scene(0,{}),Scene(.1,{})]
    def sample(self,t):return Scene(t,{})


class SceneExportTests(unittest.TestCase):
    def test_four_camera_strip_order_and_timestamp(self):
        class Reader:
            def __init__(self,path):self.index=['front','back','left_repeater','right_repeater'].index(path.stem);self.time=None
            def read_at(self,t):self.time=t;return 0,np.full((180,240,3),[20+50*self.index,0,0],np.uint8)
            def close(self):pass
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);manifest={'cameras':{name:{'file':name+'.mp4'} for name in OriginalCameraStrip.names}}
            for name in OriginalCameraStrip.names:(root/(name+'.mp4')).touch()
            with patch('core.scene_export.CameraReader',Reader):
                strip=OriginalCameraStrip(manifest,root,root/'out.mp4');image=strip.render(12.3)
                self.assertEqual(image.shape,(180,960,3))
                for i,reader in enumerate(strip.readers.values()):
                    self.assertEqual(reader.time,12.3);self.assertEqual(image[100,i*240+100,0],20+50*i)
                strip.close()

    def test_real_encoder_outputs_expected_fps_duration_and_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'scene.mp4'
            with patch('core.scene_export.DrivingRenderer') as render:
                render.return_value.render.return_value=np.full((640,960,3),120,np.uint8)
                export_scene_video(Timeline(),p)
            cap=cv2.VideoCapture(str(p));frames=0
            self.assertEqual(cap.get(cv2.CAP_PROP_FPS),30)
            while cap.read()[0]:frames+=1
            cap.release();self.assertEqual(frames,6)
            with self.assertRaises(FileExistsError):export_scene_video(Timeline(),p)

    def test_encoded_video_contains_four_original_views_below_scene(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);manifest={'cameras':{name:{'file':name+'.mp4'} for name in OriginalCameraStrip.names}}
            for i,name in enumerate(OriginalCameraStrip.names):
                writer=cv2.VideoWriter(str(root/(name+'.mp4')),cv2.VideoWriter_fourcc(*'mp4v'),10,(240,180))
                self.assertTrue(writer.isOpened())
                for _ in range(3):writer.write(np.full((180,240,3),[30+50*i,0,0],np.uint8))
                writer.release()
            with patch('core.scene_export.DrivingRenderer') as render:
                render.return_value.render.return_value=np.full((640,960,3),120,np.uint8)
                export_scene_video(Timeline(),root/'out.mp4',manifest=manifest,source_directory=root)
            cap=cv2.VideoCapture(str(root/'out.mp4'));ok,image=cap.read();cap.release()
            self.assertTrue(ok);self.assertEqual(image.shape,(820,960,3))
            for i in range(4):self.assertAlmostEqual(int(image[740,i*240+100,0]),30+50*i,delta=12)

    def test_cancel_keeps_existing_file_and_removes_partial_output(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'scene.mp4';p.write_bytes(b'previous')
            progress=[]
            with patch('core.scene_export.DrivingRenderer') as render:
                render.return_value.render.return_value=np.zeros((640,960,3),np.uint8)
                with self.assertRaises(AnalysisCancelled):
                    export_scene_video(Timeline(),p,overwrite=True,progress=progress.append,cancelled=lambda:bool(progress))
            self.assertEqual(p.read_bytes(),b'previous')
            self.assertEqual(list(Path(directory).iterdir()),[p])
