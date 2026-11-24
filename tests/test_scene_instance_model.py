import unittest,json
import numpy as np
from core.scene_analysis import CLASSES
from core.scene_instance_model import InstanceSceneDetector


class InstanceModelTests(unittest.TestCase):
    def detector(self):
        d=InstanceSceneDetector.__new__(InstanceSceneDetector)
        d.classes=CLASSES;d.threshold=.25;d.input_size=640
        return d

    def test_mask_returns_to_original_pixels_after_letterbox(self):
        raw=np.zeros((1,116,1),np.float32);raw[0,:4,0]=[320,320,160,80]
        raw[0,6,0]=.9;raw[0,84,0]=5
        proto=np.zeros((1,32,160,160),np.float32);proto[0,0]=1
        found=self.detector().decode_masks(raw,proto,(960,1280),(.5,.5,0,80))
        self.assertEqual(len(found),1)
        np.testing.assert_allclose(found[0].bbox,[480,400,320,160])
        polygon=np.array(found[0].mask_polygon)
        np.testing.assert_equal(polygon.min(0),[480,400]);np.testing.assert_equal(polygon.max(0),[799,559])
        json.dumps(vars(found[0]))

    def test_suppressed_identical_box_does_not_supply_wrong_mask(self):
        raw=np.zeros((1,116,2),np.float32);raw[0,:4,:]=np.array([[320,320,160,80]]*2).T
        raw[0,6,:]=[.4,.9];raw[0,84,:]=[-5,5]
        proto=np.zeros((1,32,160,160),np.float32);proto[0,0]=1
        found=self.detector().decode_masks(raw,proto,(960,1280),(.5,.5,0,80))
        self.assertEqual(len(found),1);self.assertTrue(found[0].mask_polygon)
