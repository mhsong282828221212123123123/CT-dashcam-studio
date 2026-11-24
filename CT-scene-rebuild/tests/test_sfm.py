import unittest
import numpy as np
from ct_scene.sfm import select_frames,consensus_matches


class SfmTests(unittest.TestCase):
    def test_query_frames_never_enter_training(self):
        frames=[{'camera_id':'front','relative_request_s':i/5,'frame_id':str(i)} for i in range(175)]
        rows=select_frames(frames,'front',(4,12,20),sample_hz=2)
        self.assertEqual(len(rows),70)
        held={r['frame_id'] for r in rows if r['heldout']}
        fit={r['frame_id'] for r in rows if not r['heldout']}
        self.assertEqual(held,{'20','60','100'})
        self.assertFalse(held&fit)
        self.assertEqual(len({r['sfm_name'] for r in rows}),70)

    def test_single_view_descriptor_match_not_enough(self):
        desc=np.random.default_rng(51).uniform(0,255,(5,128)).astype(np.float32)
        points={i:np.array([i,0,10]) for i in range(5)}
        ids,xyz=consensus_matches(desc,[(1,desc,list(range(5)))],points)
        self.assertEqual(len(ids),0)
        ids,xyz=consensus_matches(desc,[(1,desc,list(range(5))),(2,desc,list(range(5)))],points)
        self.assertEqual(set(ids),set(range(5)))
        self.assertEqual(xyz.shape,(5,3))

    def test_ambiguous_landmark_votes_are_not_resolved_by_pose_error(self):
        desc=np.random.default_rng(52).uniform(0,255,(5,128)).astype(np.float32)
        points={i:np.array([i,0,10]) for i in range(10)}
        ids,_=consensus_matches(desc,[(1,desc,list(range(5))),(2,desc,list(range(5,10)))],points)
        self.assertEqual(len(ids),0)

    def test_temporal_bracketing_rejects_past_only_repeated_marks(self):
        desc=np.random.default_rng(53).uniform(0,255,(5,128)).astype(np.float32)
        points={i:np.array([i,0,10]) for i in range(5)}
        views=[(1,desc,list(range(5)),8.),(2,desc,list(range(5)),9.),(3,desc,[-1]*5,11.)]
        ids,_=consensus_matches(desc,views,points,query_time=10.)
        self.assertEqual(len(ids),0)
        views[-1]=(3,desc,list(range(5)),11.)
        ids,_=consensus_matches(desc,views,points,query_time=10.)
        self.assertEqual(len(ids),5)
