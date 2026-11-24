import unittest
import cv2
import numpy as np
from ct_scene.planar_warp_probe import refine


class PlanarWarpTests(unittest.TestCase):
    def test_known_plane_warp_and_paired_physical_pixels(self):
        cv2.setNumThreads(2)
        a=np.random.default_rng(20).integers(0,256,(360,480),dtype=np.uint8)
        a=cv2.GaussianBlur(a,(3,3),0)
        H=np.array([[.8,.03,25],[.02,.9,15],[.0001,.0002,1.]])
        b=cv2.warpPerspective(a,H,(480,360));mask=np.full_like(a,255)
        p=np.array([(x,y) for y in (60,140,220,280) for x in (60,140,220,300)],np.float32)
        q=cv2.perspectiveTransform(p.reshape(-1,1,2),H).reshape(-1,2)
        pairs,stats=refine(a,b,mask,mask,np.column_stack([p,q]))
        self.assertGreater(stats['refined_inliers'],25)
        predicted=cv2.perspectiveTransform(pairs[:,:2].copy().reshape(-1,1,2),H).reshape(-1,2)
        self.assertLess(np.percentile(np.linalg.norm(predicted-pairs[:,2:],axis=1),95),1.5)
        self.assertEqual(len(set(map(tuple,np.floor(pairs[:,:2]/4)))),len(pairs))
        self.assertEqual(len(set(map(tuple,np.floor(pairs[:,2:]/4)))),len(pairs))

    def test_unsupported_plane_is_rejected(self):
        image=np.zeros((100,100),np.uint8);mask=np.full_like(image,255)
        with self.assertRaises(ValueError):refine(image,image,mask,mask,np.zeros((8,4),np.float32))
