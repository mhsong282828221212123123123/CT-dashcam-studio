import unittest
import cv2
import numpy as np
from core.scene_image_track import propagate_box


class ImageTrackTests(unittest.TestCase):
    def test_translation_round_trip_preserves_box(self):
        rng=np.random.default_rng(5)
        image=np.zeros((160,200,3),np.uint8)
        image[40:100,60:120]=rng.integers(0,255,(60,60,3),dtype=np.uint8)
        moved=cv2.warpAffine(image,np.float32([[1,0,8],[0,1,-4]]),(200,160))
        forward=propagate_box(image,moved,[60,40,60,60])
        self.assertIsNotNone(forward)
        np.testing.assert_allclose(forward['bbox'],[68,36,60,60],atol=.5)
        backward=propagate_box(moved,image,forward['bbox'])
        self.assertIsNotNone(backward)
        np.testing.assert_allclose(backward['bbox'],[60,40,60,60],atol=.5)

    def test_blank_occlusion_does_not_create_supported_proposal(self):
        rng=np.random.default_rng(6)
        image=rng.integers(0,255,(160,200,3),dtype=np.uint8)
        self.assertIsNone(propagate_box(image,np.zeros_like(image),[60,40,60,60]))
