import unittest
import cv2
import numpy as np
from ct_scene.label_assist import flow_box


class LabelAssistTests(unittest.TestCase):
    def test_known_pixel_translation(self):
        rng=np.random.default_rng(23)
        first=np.zeros((240,320),np.uint8)
        first[60:180,90:210]=rng.integers(0,256,(120,120),dtype=np.uint8)
        M=np.array([[1.,0.,4.],[0.,1.,-3.]])
        second=cv2.warpAffine(first,M,(320,240))
        box,report=flow_box(first,second,[90,60,210,180])
        self.assertIsNotNone(box,report)
        np.testing.assert_allclose(box,[94,57,214,177],atol=.5)

    def test_no_texture_is_not_stationary_measurement(self):
        blank=np.zeros((240,320),np.uint8)
        box,report=flow_box(blank,blank,[90,60,210,180])
        self.assertIsNone(box)
        self.assertEqual(report['reason'],'insufficient_features')
