import copy,unittest
from test_far_lead_stability import fixture,PROFILE,DIMS,MOUNT
from ct_scene.image_motion_audit import audit

class ImageMotionAuditTests(unittest.TestCase):
    def test_consistent_size_approach_is_not_flagged(self):
        s,a=fixture(lambda t:90-5*t,lambda t:90-5*t)
        r=audit(s,a,PROFILE,DIMS,MOUNT)
        self.assertGreater(r['direct_samples'],10);self.assertEqual(r['range_size_event_count'],0)
        self.assertEqual(r['metric_accuracy'],'NOT_EVALUATED')

    def test_depth_oscillation_is_flagged_without_changing_scene(self):
        s,a=fixture();before=copy.deepcopy((s,a));r=audit(s,a,PROFILE,DIMS,MOUNT)
        self.assertGreater(r['range_size_event_count'],0);self.assertEqual((s,a),before)

    def test_ghosts_and_side_observations_are_not_direct_evidence(self):
        for mode in ['ghost','side']:
            s,a=fixture()
            if mode=='side':a[0]['camera_id']='right_repeater'
            else:
                for row in s:row['actors'][0]['ghost']=True
            self.assertEqual(audit(s,a,PROFILE,DIMS,MOUNT)['direct_samples'],0)
