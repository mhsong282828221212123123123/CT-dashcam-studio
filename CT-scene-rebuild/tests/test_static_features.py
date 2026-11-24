import unittest
import cv2
import numpy as np
from ct_scene.static_features import correspondences,feature_roi,optical_feature_roi


class StaticFeatureTests(unittest.TestCase):
    def test_optical_roi_retains_upper_facade_and_excludes_mirror(self):
        mask=np.full((960,1280),255,np.uint8);mask[310:350,700:740]=0
        self.assertEqual(feature_roi(mask)[220,700],0)
        self.assertEqual(optical_feature_roi(mask,'front')[220,700],255)
        for camera in ('front','left_repeater','right_repeater'):
            self.assertEqual(optical_feature_roi(mask,camera)[320,720],0)
        self.assertEqual(optical_feature_roi(mask,'left_repeater')[120,200],0)
        self.assertEqual(optical_feature_roi(mask,'right_repeater')[120,1080],0)
        self.assertEqual(optical_feature_roi(mask,'left_repeater')[220,800],255)

    def test_known_translation_and_both_frame_exclusion(self):
        rng=np.random.default_rng(31)
        a=rng.integers(0,256,(240,320),dtype=np.uint8)
        b=cv2.warpAffine(a,np.array([[1.,0.,2.],[0.,1.,1.]]),(320,240))
        mask=np.full_like(a,255);mask[90:160,100:200]=0
        pairs,diag=correspondences(a,b,mask,mask)
        self.assertGreater(len(pairs),20)
        np.testing.assert_allclose(np.median(pairs[:,2:]-pairs[:,:2],axis=0),[2,1],atol=.2)
        roi=feature_roi(mask)
        for points in [pairs[:,:2],pairs[:,2:]]:
            points=np.rint(points).astype(int)
            self.assertTrue(np.all(roi[points[:,1],points[:,0]]>0))
        self.assertFalse(diag['low_motion'])

    def test_still_image_is_low_motion_not_estimated_pose(self):
        a=np.random.default_rng(32).integers(0,256,(240,320),dtype=np.uint8)
        mask=np.full_like(a,255)
        pairs,diag=correspondences(a,a,mask,mask)
        self.assertGreater(len(pairs),20)
        self.assertTrue(diag['low_motion'])
        self.assertNotIn('pose',diag)
