import math,unittest
import numpy as np
from core.scene_vehicle_road import stabilize_road_frames


class EgoStabilizationTests(unittest.TestCase):
    def frames(self,x,angles):
        return [(np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]]),v,'test')
                for v,a in zip(x,angles)]

    def test_position_and_heading_jitter_removed_while_lane_change_remains(self):
        t=np.arange(0,8,.1);truth=3.5*(1+np.tanh((t-4)/.8))/2
        noise=.14*np.cos(2*np.pi*4*t)
        result=stabilize_road_frames(self.frames(truth+noise,.025*np.cos(2*np.pi*4*t)),t)
        x=np.array([f[1] for f in result]);angle=np.array([math.atan2(f[0][1,0],f[0][0,0]) for f in result])
        self.assertLess(np.sqrt(np.mean((x-truth)**2)),.035)
        self.assertLess(np.max(abs(angle[10:-10])),.003)
        self.assertLess(abs(t[np.argmin(abs(x-1.75))]-4),.11)
        self.assertGreater(x[-1]-x[0],3.4)

    def test_constant_velocity_has_no_phase_lag_including_endpoints(self):
        t=np.arange(0,4,.1);x=.4*t
        result=stabilize_road_frames(self.frames(x,np.zeros(len(t))),t)
        np.testing.assert_allclose([f[1] for f in result],x,atol=1e-8)
