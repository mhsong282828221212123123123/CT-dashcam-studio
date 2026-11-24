import unittest
from tools.scene_acceptance_gate import evaluate,REQUIRED


class AcceptanceGateTests(unittest.TestCase):
    def test_empty_or_machine_only_results_cannot_pass(self):
        self.assertEqual(evaluate([])['status'],'FAIL')
        self.assertEqual(evaluate([{'id':'one','split':'held_out','duration_s':35,
                                   'ground_truth_evaluated':False,'coverage':1.}])['status'],'FAIL')

    def test_single_critical_dropout_cannot_be_hidden_by_other_good_clips(self):
        metrics=dict(critical_vehicle_display_recall=1.,longest_visible_vehicle_dropout_s=0.,
                     wrong_lane_events=0,duplicate_vehicle_events=0,wrong_lane_count_events=0,
                     ego_pinned_road_events=0,invented_intersection_lane_events=0,
                     lane_reprojection_error_p95_image_width=.005,unresolved_critical_events=0)
        reviews=[dict(id=str(i),event_id=str(i),split='held_out',duration_s=35,
                      independent_manual_annotation=True,full_sequence_reviewed=True,
                      verified_scenarios=list(REQUIRED),metrics=dict(metrics)) for i in range(3)]
        self.assertEqual(evaluate(reviews)['status'],'PASS')
        reviews[1]['metrics']['longest_visible_vehicle_dropout_s']=.6
        self.assertEqual(evaluate(reviews)['status'],'FAIL')

    def test_duplicate_event_does_not_count_as_independent_coverage(self):
        reviews=[dict(id=str(i),event_id='same',split='held_out',duration_s=35,
                      independent_manual_annotation=True,full_sequence_reviewed=True,
                      verified_scenarios=['day'],metrics={}) for i in range(4)]
        self.assertEqual(evaluate(reviews)['reviewed_events_by_scenario']['day'],1)
