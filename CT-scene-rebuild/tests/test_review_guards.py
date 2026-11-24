import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from review_guards import require_cases,require_window,require_full_clock,source_exit_code,require_same_base

class ReviewGuardTests(unittest.TestCase):
    def test_base_replacement_requires_explicit_opt_in(self):
        scene={'inputs':[{'role':'frozen_ego_lead','artifact_id':'corrected'}]}
        require_same_base(scene,{'artifact_id':'corrected'})
        with self.assertRaises(ValueError):require_same_base(scene,{'artifact_id':'old'})
        require_same_base(scene,{'artifact_id':'old'},allow_change=True)

    def test_failure_or_empty_results_reject(self):
        self.assertEqual(source_exit_code([]),2)
        self.assertEqual(source_exit_code([{'gate':'PASS'},{'gate':'FAIL'}]),2)
        self.assertEqual(source_exit_code([{'gate':'PASS'}]),0)

    def test_missing_duplicate_cases_reject(self):
        for actual in ([],['a'],['a','a']):
            with self.assertRaises(ValueError):require_cases(actual,['a','b'])
        require_cases(['b','a'],['a','b'])

    def test_empty_and_gapped_source_window_reject(self):
        for states in ([],[{'time_s':0},{'time_s':1}]):
            with self.assertRaises(ValueError):require_window(states,0,1)

    def test_truncated_output_cannot_self_certify_its_tail(self):
        states=[{'time_s':i/20} for i in range(200)]
        with self.assertRaises(ValueError):require_full_clock(states,{'source_duration_s':20,'source_first_pts_s':0},200)
        r=require_full_clock(states,{'source_duration_s':10,'source_first_pts_s':0},200)
        self.assertGreater(r['last_five_seconds_frames'],90)
