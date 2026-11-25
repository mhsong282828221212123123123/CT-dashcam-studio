import unittest
from ct_scene.quality_review import stored_overlap_failure

class StoredFailureTests(unittest.TestCase):
    def test_recorded_failure_and_nonzero_overlap_are_flagged(self):
        self.assertTrue(stored_overlap_failure(dict(collision_constraints=dict(after=dict(gate='FAIL',overlapping_pair_frames=82)))))
        self.assertTrue(stored_overlap_failure(dict(collision_constraints=dict(after=dict(gate='PASS',overlapping_pair_frames=1)))))

    def test_passed_or_unknown_results_do_not_invent_failures(self):
        self.assertFalse(stored_overlap_failure(dict(collision_constraints=dict(after=dict(gate='PASS',overlapping_pair_frames=0)))))
        self.assertFalse(stored_overlap_failure({}))
        self.assertFalse(stored_overlap_failure(dict(collision_constraints=dict(after=dict(overlapping_pair_frames=None)))))
