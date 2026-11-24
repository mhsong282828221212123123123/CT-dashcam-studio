import math,unittest
import numpy as np
from ct_scene.image_vehicle_heading import decode,camera_heading,preprocess,foreground_occlusion


class ImageHeadingMathTests(unittest.TestCase):
    def test_kitti_body_front_and_world_axes(self):
        rear,_=decode([[1,0],[1,0]],[3,0]);front,_=decode([[1,0],[1,0]],[0,3])
        self.assertAlmostEqual(camera_heading(rear,0),0)
        self.assertAlmostEqual(abs(camera_heading(front,0)),math.pi)
        right,_=decode([[0,1],[0,1]],[3,0])
        self.assertAlmostEqual(camera_heading(right,0),-math.pi/2)

    def test_bearing_correction_keeps_parallel_car_parallel_off_centre(self):
        self.assertAlmostEqual(camera_heading(-.3,.3),0)

    def test_rgb_normalization_and_partial_body_rejection(self):
        rgb=np.zeros((150,200,3),np.uint8);rgb[:,:,0]=255
        t=preprocess(rgb,[10,10,150,130]);self.assertEqual(t.shape,(1,3,224,224));self.assertEqual(t.dtype,np.float32)
        np.testing.assert_allclose(t[0,:,0,0],[(1-.485)/.229,-.456/.224,-.406/.225],atol=1e-6)
        with self.assertRaises(ValueError):preprocess(rgb,[0,10,150,130])

    def test_nearer_body_occludes_but_a_background_box_does_not(self):
        box=[100,100,300,200]
        self.assertGreater(foreground_occlusion(box,[[150,150,280,250]]),.15)
        self.assertEqual(foreground_occlusion(box,[[150,110,280,190]]),0.)
