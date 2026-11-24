import unittest
from ct_scene.quality_review import source_presence_screen


class SourcePresenceScreenTests(unittest.TestCase):
    def report(self,n,covered):return dict(observation_world_coverage=dict(front=dict(observed_samples=n,covered_in_world_state=covered)))
    def states(self,n):return [dict(actors=[dict(id='visible')])]*n+[dict(actors=[])]*(100-n)
    def test_many_observations_and_empty_world_is_rejected(self):
        r=source_presence_screen(self.report(1700,28),self.states(9));self.assertEqual(r['gate'],'REJECTED');self.assertEqual(r['source_accuracy'],'NOT_EVALUATED')
    def test_empty_source_does_not_require_inventing_vehicles(self):
        self.assertEqual(source_presence_screen(self.report(0,0),self.states(0))['gate'],'NOT_FLAGGED')
    def test_distant_observations_need_not_all_be_localized(self):
        self.assertEqual(source_presence_screen(self.report(1700,30),self.states(70))['gate'],'NOT_FLAGGED')
    def test_legacy_missing_evidence_is_not_a_claimed_pass(self):
        self.assertEqual(source_presence_screen({},self.states(0))['gate'],'NOT_EVALUATED')
