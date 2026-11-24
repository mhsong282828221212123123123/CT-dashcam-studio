import unittest
import numpy as np
from ct_scene.pixels import CropResize,RadialTangentialLens


class PixelTests(unittest.TestCase):
    def test_distortion_crop_letterbox_roundtrip(self):
        lens=RadialTangentialLens(1200,1190,640,480,(-.24,.08,.001,-.002,.01))
        points=np.array([[0,0],[-.45,-.32],[.42,.3],[.1,-.3]])
        raw=lens.raw_pixels(points)
        xy,valid=lens.rays(raw)
        self.assertTrue(valid.all())
        np.testing.assert_allclose(xy,points,atol=1e-9)
        rectified=xy*[1100,1100]+[640,480]
        mapping=CropResize(50,20,1180,900,640,488,pad_y=76)
        restored=mapping.backward(mapping.forward(rectified))
        np.testing.assert_allclose(restored,rectified,atol=1e-10)
        raw_restored=lens.raw_pixels((restored-[640,480])/[1100,1100])
        np.testing.assert_allclose(raw_restored,raw,atol=.01)

    def test_bad_geometry_rejected(self):
        with self.assertRaises(ValueError):RadialTangentialLens(-1,100,0,0)
        with self.assertRaises(ValueError):CropResize(0,0,0,100,640,640)
