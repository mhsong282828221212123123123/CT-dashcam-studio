import unittest,math
import numpy as np
from tools.camera_reprojection import cube_bbox,bbox_metrics
from ct_scene.surrounding import project_contact

P=dict(model='SIMPLE_RADIAL',params=[1000,640,480,0])
M=dict(height_m=.85,yaw_deg=0,offset_W_ego_m=[0,0,0])

class CameraReprojectionTests(unittest.TestCase):
    def test_projected_front_surface_and_body_centre_are_distinct(self):
        box=cube_bbox([22,0,0],0,[4,2,1.5],[0,0,0],0,M,P)
        np.testing.assert_allclose(box,[590,447.5,690,522.5],atol=1e-7)
        self.assertAlmostEqual(project_contact(box,P,M)[0],20.)
        self.assertEqual(bbox_metrics(box,box)['bbox_iou'],1.)

    def test_pitch_is_inverted_when_projecting_into_raw_image(self):
        mount=dict(M,pitch_correction_rad=-.035)
        box=cube_bbox([22,0,0],0,[4,2,1.5],[0,0,0],0,mount,P)
        self.assertAlmostEqual(project_contact(box,P,mount)[0],20.,places=5)

    def test_rotating_world_and_camera_keeps_reprojection_identical(self):
        a=cube_bbox([22,0,0],0,[4,2,1.5],[0,0,0],0,M,P)
        b=cube_bbox([0,22,0],math.pi/2,[4,2,1.5],[0,0,0],math.pi/2,M,P)
        np.testing.assert_allclose(a,b,atol=1e-7)
        self.assertIsNone(cube_bbox([0,0,0],0,[4,2,1.5],[0,0,0],0,M,P))
