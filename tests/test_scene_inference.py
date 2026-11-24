import unittest
from unittest.mock import patch,Mock
import numpy as np
from core.scene_inference import InferenceEngine
from core.scene_analysis import SceneDetector,Detection

class InferenceTests(unittest.TestCase):
    def test_missing_directml_is_explicit_cpu_fallback(self):
        ort=Mock();ort.get_available_providers.return_value=['CPUExecutionProvider']
        with patch.dict('sys.modules',{'onnxruntime':ort}),patch('core.scene_inference.cv2.dnn.readNetFromONNX'):
            engine=InferenceEngine('test.onnx',640,'directml')
        self.assertEqual(engine.info['effective'],'opencv_cpu')
        self.assertIn('not installed',engine.info['fallback_reason'])

    def test_runtime_failure_switches_and_records_cpu(self):
        engine=InferenceEngine.__new__(InferenceEngine)
        engine.session=Mock();engine.session.run.side_effect=RuntimeError('device removed')
        engine.input='images';engine.model='test.onnx';engine.info={}
        net=Mock();net.forward.return_value=np.ones((1,84,1))
        with patch('core.scene_inference.cv2.dnn.readNetFromONNX',return_value=net):
            result=engine.run(np.zeros((1,3,32,32),np.float32))
        self.assertEqual(engine.info['effective'],'opencv_cpu')
        self.assertIn('device removed',engine.info['fallback_reason'])
        self.assertIsNone(engine.session);self.assertEqual(result.shape,(1,84,1))

    def test_cuda_not_accepted(self):
        with self.assertRaises(ValueError):InferenceEngine('unused',640,'cuda')

    def test_cropped_edge_fragments_do_not_become_new_objects(self):
        detector=SceneDetector.__new__(SceneDetector);detector.detail='tiled';detector.threshold=.25
        detector._detect_frame=Mock(side_effect=[[],[Detection('car',.9,[55,10,5,20])],[],[],[]])
        self.assertEqual(detector.detect(np.zeros((100,100,3),np.uint8)),[])
