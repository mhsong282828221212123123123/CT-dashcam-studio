import copy,unittest
import numpy as np
from ct_scene.curved_road import continuous_paint_excludes_lane_jump


class ContinuousPaintPhaseTests(unittest.TestCase):
    def fixture(self):
        t=np.linspace(0,10,101);ego=np.c_[t*22,t*0,t*0];yaw=np.zeros(len(t));window=dict(start_s=1.,end_s=8.)
        ref=dict(station_support=[dict(s=float(s),phase=.2,frames=4,concentration=.99) for s in range(20,181,10)])
        return window,t,ego,yaw,ref

    def test_dense_stationary_stripes_refute_synthetic_straight_frame_crossing(self):
        args=self.fixture();old=copy.deepcopy(args)
        self.assertTrue(continuous_paint_excludes_lane_jump(*args))
        np.testing.assert_array_equal(args[2],old[2]);self.assertEqual(args[-1],old[-1])

    def test_paint_gap_cannot_hide_a_real_lane_change(self):
        args=list(self.fixture());args[-1]['station_support']=[r for r in args[-1]['station_support'] if not 70<=r['s']<=120]
        self.assertFalse(continuous_paint_excludes_lane_jump(*args))

    def test_full_lane_phase_motion_remains_protected(self):
        args=list(self.fixture())
        for r in args[-1]['station_support']:r['phase']=float(np.angle(np.exp(2j*np.pi*(r['s']-20)/160)))
        self.assertFalse(continuous_paint_excludes_lane_jump(*args))

    def test_alias_or_weak_camera_evidence_cannot_veto_ego_change(self):
        for mode in ['alias','weak','few_frames','short']:
            args=list(self.fixture())
            if mode=='alias':args[-1]['station_support'][7]['phase']=2.
            if mode=='weak':args[-1]['station_support'][7]['concentration']=.8
            if mode=='few_frames':args[-1]['station_support'][7]['frames']=2
            if mode=='short':args[0]['end_s']=2.
            self.assertFalse(continuous_paint_excludes_lane_jump(*args),mode)
