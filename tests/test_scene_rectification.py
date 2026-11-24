import unittest
import cv2,numpy as np
from core.scene_rectification import CameraRectifier
from core.scene_fusion import cameras_for,default_profile


class RectificationTests(unittest.TestCase):
    def profile(self,model='opencv_pinhole'):
        return dict(model=model,image_size=[128,96],camera_matrix=[[90,0,64],[0,90,48],[0,0,1]],
            distortion_coefficients=[0,0,0,0],rectified_camera_matrix=[[90,0,64],[0,90,48],[0,0,1]])

    def test_identity_pinhole_preserves_image_and_does_not_mutate_original(self):
        image=np.random.default_rng(2).integers(0,256,(96,128,3),dtype=np.uint8);original=image.copy()
        rectifier=CameraRectifier(self.profile(),128,96)
        np.testing.assert_array_equal(rectifier.apply(image),image)
        np.testing.assert_array_equal(image,original)

    def test_fisheye_ray_maps_to_expected_distorted_pixel(self):
        r=CameraRectifier(self.profile('opencv_fisheye'),128,96)
        expected=cv2.fisheye.distortPoints(np.array([[[.5,0.]]]),r.K,np.zeros(4))[0,0]
        np.testing.assert_allclose([r.mapx[48,109],r.mapy[48,109]],expected,atol=1e-4)

    def test_size_mismatch_and_nonfinite_parameters_rejected(self):
        with self.assertRaises(ValueError):CameraRectifier(self.profile(),128,80)
        p=self.profile();p['distortion_coefficients'][0]=float('nan')
        with self.assertRaises(ValueError):CameraRectifier(p,128,96)

    def test_division_model_inverse_mapping_round_trip(self):
        p=self.profile();p['model']='division1';p['distortion_coefficients']=[-.2]
        r=CameraRectifier(p,128,96)
        distorted=(np.array([r.mapx[48,109],r.mapy[48,109]])-[64,48])/90
        undistorted=distorted/(1-.2*np.dot(distorted,distorted))
        np.testing.assert_allclose(undistorted,[.5,0.],atol=1e-6)

    def test_fusion_uses_rectified_focal_and_preserves_pitch_assumption(self):
        p=self.profile();p['rectified_camera_matrix']=[[70,0,64],[0,70,48],[0,0,1]]
        meta=CameraRectifier(p,128,96).metadata()
        cameras=cameras_for({'cameras':{'back':dict(width=128,height=96,rectification=meta)}},default_profile())
        camera=cameras['back']
        self.assertEqual(camera.focal,70)
        self.assertAlmostEqual((48-camera.horizon)/camera.focal,(48-30)/65)
