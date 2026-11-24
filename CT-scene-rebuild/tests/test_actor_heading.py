import unittest
import numpy as np
from ct_scene.actor_heading import estimate,resolve_axis_flips


class HeadingTests(unittest.TestCase):
    def test_continuously_visible_identity_can_bridge_failed_ground_contacts(self):
        t=[0.,.2,7.,7.2];yaw=[-.2,-.2,np.pi-.6,np.pi-.6]
        fixed=resolve_axis_flips(t,yaw,visible_times=np.arange(0,7.21,.2))
        np.testing.assert_allclose(fixed,[-.2,-.2,-.6,-.6])
        missing=np.r_[np.arange(0,2,.2),np.arange(6,7.21,.2)]
        np.testing.assert_allclose(resolve_axis_flips(t,yaw,visible_times=missing),yaw)
        np.testing.assert_allclose(resolve_axis_flips([0.,10.],[0.,np.pi],visible_times=np.arange(0,10.01,.2)),[0.,np.pi])

    def test_stop_fit_sign_flip_does_not_reverse_body_front(self):
        t=np.arange(0,2,.2);yaw=np.array([.2,.2,.2,.2,.2+np.pi,.2+np.pi,.2,.2,.2,.2])
        np.testing.assert_allclose(resolve_axis_flips(t,yaw),.2,atol=1e-12)

    def test_gradual_uturn_and_long_unknown_reappearance_are_retained(self):
        t=np.arange(0,8,.2);yaw=np.linspace(0,np.pi,len(t))
        np.testing.assert_allclose(resolve_axis_flips(t,yaw),yaw,atol=1e-12)
        np.testing.assert_allclose(resolve_axis_flips([0,5],[0,np.pi]),[0,np.pi])

    def test_initial_stop_uses_later_motion_not_observer_rotation(self):
        t=np.arange(0,8,.2);p=np.c_[np.maximum(0,t-3)*-4,t*0,t*0]
        angles,support=estimate(t,p,fallback=np.pi/2)
        self.assertLess(np.max(abs(np.unwrap(angles)-np.pi)),1e-8)
        self.assertFalse(support[0])

    def test_sustained_real_turn_is_retained(self):
        t=np.arange(0,8,.1);a=t/8*np.pi/2;p=np.c_[30*np.sin(a),30*(1-np.cos(a)),t*0]
        angles,support=estimate(t,p)
        self.assertGreater(angles[-1]-angles[0],1.3)
        self.assertLess(np.max(abs(np.diff(angles))),.03)

    def test_stopped_contact_jitter_does_not_create_heading(self):
        t=np.arange(0,8,.2);p=np.c_[np.sin(t*20)*.1,np.cos(t*20)*.1,t*0]
        angles,support=estimate(t,p,fallback=.3)
        np.testing.assert_array_equal(angles,np.full(len(t),.3));self.assertFalse(support.any())
