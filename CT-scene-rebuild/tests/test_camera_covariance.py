import unittest
import numpy as np
from ct_scene.camera_covariance import ray_covariance,update_position,initialize_offline


class CameraCovarianceTests(unittest.TestCase):
    def test_world_rotations_and_road_heading_preserve_road_covariance(self):
        p=np.array([20.,10.]);angle=.9;c,s=np.cos(angle),np.sin(angle);R=np.array([[c,-s],[s,c]])
        w,a=ray_covariance(p,[0,0],8.,.3,.2)
        wr,b=ray_covariance(R@p,[0,0],8.,.3,.2+angle)
        np.testing.assert_allclose(a,b,atol=1e-10)
        np.testing.assert_allclose(wr,R@w@R.T,atol=1e-10)
        self.assertGreater(abs(a[0,1]),1.)
    def test_depth_prior_cannot_drag_observed_camera_bearing_sideways(self):
        p=np.array([20.,10.]);ray=p/np.linalg.norm(p);normal=np.array([-ray[1],ray[0]])
        x=np.r_[p+ray*8+normal*2,0.,0.];P=np.eye(4)*5
        W,_=ray_covariance(p,[0,0],10.,.2)
        fit,cov=update_position(x,P,p,W)
        self.assertLess(abs((fit[:2]-p)@normal),.02)
        self.assertGreater(abs((fit[:2]-p)@ray),7.)
        self.assertGreater(np.linalg.eigvalsh(cov).min(),0.)
    def test_initial_glitch_is_not_persistent_velocity(self):
        ts=np.arange(7)*.2;rows=[dict(time_s=t) for t in ts]
        obs=[[(np.array([30+8*t+(25 if i==0 else 0),2.]),4.,.2,'front',None)] for i,t in enumerate(ts)]
        z,v,var=initialize_offline(rows,obs,0,36.)
        self.assertLess(abs(v[0]-8),2.)
        self.assertLess(abs(z[0]-30),2.)
    def test_initializer_does_not_invent_velocity_for_identical_timestamp(self):
        z,v,var=initialize_offline([dict(time_s=0.)],[[(np.array([30.,2.]),4.,1.,'front',None)]],0,36.)
        np.testing.assert_array_equal(v,[0.,0.])
