import unittest
import numpy as np
import cv2
from ct_scene.manual_rig_probe import triangulate,project,flow_step


class ManualRigProbeTests(unittest.TestCase):
    profile={'model':'SIMPLE_RADIAL','params':[1000,640,480,-.1]}

    def test_known_multiview_point_and_unused_check_camera(self):
        X=np.array([1,2,20.]);poses=np.array([np.column_stack([np.eye(3),[-x,0,0]]) for x in (0,1,2)])
        pixels=np.array([project(X[None],P,self.profile)[0] for P in poses])
        recovered,diag=triangulate(pixels,poses,self.profile)
        np.testing.assert_allclose(recovered,X,atol=1e-6)
        self.assertGreater(diag['max_ray_angle_deg'],5)
        check=np.column_stack([np.eye(3),[-3,0,0]])
        np.testing.assert_allclose(project(recovered[None],check,self.profile),project(X[None],check,self.profile),atol=1e-6)

    def test_pure_rotation_and_invalid_camera_cannot_produce_depth(self):
        P=np.column_stack([np.eye(3),[0,0,0]])
        with self.assertRaises(ValueError):triangulate([[640,480]]*3,[P]*3,self.profile)
        bad=P.copy();bad[0,0]=2
        with self.assertRaises(ValueError):triangulate([[640,480]]*3,[bad]*3,self.profile)

    def test_lost_point_never_reappears_as_measured(self):
        a=np.random.default_rng(22).integers(0,256,(180,240),dtype=np.uint8)
        b=cv2.warpAffine(a,np.array([[1.,0,2],[0,1,1]]),(240,180))
        points=np.array([[80,70],[140,120]],np.float32);mask=np.full_like(a,255)
        q,active=flow_step(a,b,points,np.array([True,False]),mask)
        self.assertTrue(active[0]);self.assertFalse(active[1])
        np.testing.assert_allclose(q[0]-points[0],[2,1],atol=.2)
        _,active=flow_step(a,b,points,np.array([True,False]),np.zeros_like(mask))
        self.assertFalse(active.any())
