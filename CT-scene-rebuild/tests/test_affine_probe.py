import unittest
import numpy as np
from ct_scene.affine_probe import camera_matrix,match_affine


class AffineProbeTests(unittest.TestCase):
    def test_colmap_radial_coefficients_map_to_opencv(self):
        K,D=camera_matrix({'model':'RADIAL','params':[1000,640,480,-.3,.12]})
        np.testing.assert_array_equal(K,[[1000,0,640],[0,1000,480],[0,0,1]])
        np.testing.assert_array_equal(D,[-.3,.12,0,0,0])
        with self.assertRaises(ValueError):
            camera_matrix({'model':'OPENCV','params':[1000,1000,640,480]})

    def test_affine_duplicates_cannot_inflate_physical_support(self):
        # Many view-synthesized descriptors of one physical point are one match.
        descriptors=np.eye(32,dtype=np.float32)
        points=np.tile([100,100],(32,1)).astype(np.float32)
        image=np.zeros((2,2),np.uint8)
        _,stats=match_affine((image,points,descriptors),(image,points,descriptors))
        self.assertEqual(stats['unique_matches'],1)
        self.assertEqual(stats['inliers'],0)
