import math,unittest
import cv2,numpy as np
from core.scene_background_pitch import reference_points,estimate_pitch
from core.scene_fusion import CameraGeometry
from core.scene_bundle_refinement import occluded


class BackgroundPitchTests(unittest.TestCase):
    def test_zoom_about_horizon_is_not_mistaken_for_pitch(self):
        rng=np.random.default_rng(19);image=rng.integers(0,256,(480,640),dtype=np.uint8)
        image=cv2.GaussianBlur(image,(3,3),0)
        # Forward motion magnifies the facade around the vanishing point.
        horizon=250.;scale=1.03
        T=np.array([[scale,0,320*(1-scale)],[0,scale,horizon*(1-scale)+4]],np.float32)
        moved=cv2.warpAffine(image,T,(640,480))
        result=estimate_pitch(image,moved,reference_points(image),horizon)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result['horizon_px'],254,delta=.5)

    def test_rig_pitch_corrects_repeater_ground_geometry(self):
        camera=CameraGeometry(yaw=math.radians(146),horizon=504,camera_height=.9,x=.9,z=.8,rig_pitch=.02)
        world=np.array([[3.5,0.,-8.]])
        pixel=camera.pixels(world)[0]
        np.testing.assert_allclose(camera.ground(*pixel),world[0,[0,2]],atol=1e-8)

    def test_side_occlusion_is_not_only_a_bottom_centre_test(self):
        target={'bbox':[100,100,50,30]};foreground={'bbox':[130,90,30,60]}
        self.assertTrue(occluded(target,[target,foreground]))
        self.assertFalse(occluded(target,[target,{'bbox':[170,90,30,60]}]))
