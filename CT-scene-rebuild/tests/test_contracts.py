import unittest
from ct_scene.contracts import FrameRecord,ActorEvidenceState
from ct_scene.geometry import Clock


class ContractTests(unittest.TestCase):
    def test_unverified_time_cannot_be_promoted(self):
        f=FrameRecord('a'*64,'front',0,123,1,10000)
        self.assertAlmostEqual(f.camera_time_s,.0123)
        self.assertIsNone(f.session_time_s)
        with self.assertRaises(ValueError):FrameRecord('a'*64,'front',0,123,1,10000,session_time_s=1)
        with self.assertRaises(ValueError):FrameRecord('a'*64,'front',0,123,1,0)

    def test_clock_extrapolation_and_false_confidence_rejected(self):
        with self.assertRaises(ValueError):Clock(status='verified')
        clock=Clock(status='estimated',valid_from_camera_s=2,valid_to_camera_s=5,uncertainty_s=.02)
        self.assertEqual(clock.to_session(3),3)
        for t in (1,6,float('nan')):
            with self.assertRaises(ValueError):clock.to_session(t)

    def test_position_failure_is_not_occlusion_or_solid_pose(self):
        s=ActorEvidenceState('visible','unresolved','unlocalized','unestablished')
        self.assertEqual(s.image_presence,'visible')
        for support in ('predicted','interpolated','unresolved'):
            with self.assertRaises(ValueError):ActorEvidenceState('visible',support,'solid','estimated_scale')
        with self.assertRaises(ValueError):ActorEvidenceState('occluded','single_view','solid','estimated_scale')
        ActorEvidenceState('occluded','interpolated','ghost','estimated_scale')
