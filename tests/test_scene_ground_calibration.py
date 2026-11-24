import unittest
import numpy as np
from core.scene_ground_calibration import transfer,fit_ground_camera

class GroundCalibrationTests(unittest.TestCase):
    def fixture(self):
        pairs=[];rng=np.random.default_rng(41)
        for i,t in enumerate(np.arange(0,3,.2)):
            a=np.c_[rng.uniform(150,1130,70),rng.uniform(600,800,70)]
            oa=np.array([0.,t*4]);ob=np.array([0.,(t+.15)*4])
            b=transfer(a,oa,np.eye(2),ob,np.eye(2),1600,485+2*t,485+2*(t+.15))
            pairs.append(dict(a=a,b=b,origin_a=oa,origin_b=ob,R_a=np.eye(2),R_b=np.eye(2),time_a=t,time_b=t+.15))
        return pairs
    def test_ground_motion_recovers_focal_and_pitch_without_vehicle_sizes(self):
        fit=fit_ground_camera(self.fixture(),np.arange(0,3.1,.5))
        self.assertLess(abs(fit['focal_px']-1600),3)
        self.assertLess(fit['withheld_p95_error_px'],.03)
        self.assertFalse(fit['surveyed'])
    def test_wrong_focal_cannot_be_hidden_by_refitting_horizon(self):
        fit=fit_ground_camera(self.fixture(),np.arange(0,3.1,.5),fixed_focal=850)
        self.assertGreater(fit['withheld_median_error_px'],1)
    def test_height_prior_is_not_reported_as_a_surveyed_measurement(self):
        fit=fit_ground_camera(self.fixture(),np.arange(0,3.1,.5))
        self.assertAlmostEqual(fit['focal_height_product_px_m'],fit['focal_px']*fit['height_prior_m'])
        self.assertIn('not measured',fit['height_scale_warning'])
