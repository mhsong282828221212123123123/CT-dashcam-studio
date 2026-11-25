import unittest
from review_guards import require_input_artifact


class SourceReviewGuardTests(unittest.TestCase):
    def test_same_date_different_clip_is_rejected(self):
        scene=dict(inputs=[dict(role='source',artifact_id='morning_hash')])
        with self.assertRaises(ValueError):require_input_artifact(scene,dict(artifact_id='afternoon_hash'),'source')
        require_input_artifact(scene,dict(artifact_id='morning_hash'),'source')
    def test_missing_or_ambiguous_role_is_rejected(self):
        for ids in [[],[dict(role='source',artifact_id='a'),dict(role='source',artifact_id='a')]]:
            with self.assertRaises(ValueError):require_input_artifact(dict(inputs=ids),dict(artifact_id='a'),'source')


class IntervalSourceControlTests(unittest.TestCase):
    def test_good_endpoints_cannot_hide_bad_middle(self):
        from evaluate_position_source_controls import evaluate
        actors=[dict(id=k,global_id=k,local_ids=[k]) for k in ['a','b']]
        body=lambda k,x:dict(id=k,model='sedan',position_W=[x,0,0],yaw_rad=0.)
        states=[dict(time_s=t,actors=[body('a',0),body('b',x)]) for t,x in [(0.,10.),(.05,0.),(.1,10.)]]
        c=dict(kind='distinct_nonoverlapping_bodies',ids=['a','b'],time_s=0.,interval_s=[0.,.1])
        r=evaluate(c,states,actors,dict(sedan=[4.5,1.8,1.5]))
        self.assertFalse(r['passed']);self.assertEqual(r['failed_frames'],1);self.assertEqual(r['frames'],3)
