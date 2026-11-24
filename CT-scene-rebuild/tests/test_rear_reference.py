import unittest
from unittest.mock import patch
import numpy as np
from ct_scene.rear_reference import select_reference_intervals,reference_presence,feature_agreement


def evidence(t,rear='rear1',gid='car1',score=10):
    return dict(time_s=t,rear_id=rear,global_id=gid,score=score)


class RearReferenceTests(unittest.TestCase):
    def test_finite_ransac_matrix_with_no_consensus_is_rejected_before_hull(self):
        rng=np.random.default_rng(13);points=rng.uniform(10,180,(20,2)).astype('float32');descriptors=rng.normal(0,1,(20,128)).astype('float32')
        first=(points,descriptors,(200,200));second=(points+[2,3],descriptors.copy(),(200,200))
        for n in [0,2]:
            mask=np.zeros((20,1),'uint8');mask[:n]=1
            with patch('ct_scene.rear_reference.cv2.findHomography',return_value=(np.eye(3),mask)):
                self.assertIsNone(feature_agreement(first,second))
    def test_three_repeated_exclusive_matches_required(self):
        self.assertEqual(select_reference_intervals([evidence(0),evidence(.2)]),[])
        out=select_reference_intervals([evidence(0),evidence(.2),evidence(.4)])
        self.assertEqual(len(out),1);self.assertEqual(out[0]['end_s'],.4)

    def test_ambiguous_similar_cars_do_not_attach(self):
        rows=[evidence(t,gid=gid,score=score) for t in [0,.2,.4] for gid,score in [('car1',10),('car2',9)]]
        self.assertEqual(select_reference_intervals(rows),[])

    def test_rear_id_switch_is_not_transitive_global_identity(self):
        rows=[evidence(t) for t in [0,.2,.4]]+[evidence(t,gid='car2') for t in [1,1.2,1.4]]
        report={'intervals':select_reference_intervals(rows)}
        obs=[dict(camera_id='back',support='detector_observation',local_track_id='rear1',camera_time_s=t) for t in [-1,0,.2,.4,.6,1,1.2,1.4,2]]
        presence=reference_presence(report,obs)
        self.assertEqual(presence,{'car1':[0,.2,.4],'car2':[1,1.2,1.4]})

    def test_interleaved_identity_switch_and_ambiguous_frame_break_support(self):
        rows=[evidence(t,gid='car1') for t in [0,.4,.8]]+[evidence(t,gid='car2') for t in [.2,.6,1]]
        self.assertEqual(select_reference_intervals(rows),[])
        rows=[evidence(t) for t in [0,.4,.6]]+[evidence(.2,score=10),evidence(.2,gid='car2',score=9)]
        self.assertEqual(select_reference_intervals(rows),[])

    def test_evidence_gap_does_not_fill_unseen_interval(self):
        rows=[evidence(t) for t in [0,.2,.4,2,2.2,2.4]]
        intervals=select_reference_intervals(rows)
        self.assertEqual(len(intervals),2)
        self.assertEqual(intervals[0]['end_s'],.4)
        self.assertEqual(intervals[1]['start_s'],2)

    def test_unique_spread_geometric_matches_and_duplicate_points(self):
        rng=np.random.default_rng(20261114);p=rng.uniform(10,180,(20,2)).astype('float32');d=rng.normal(0,1,(20,128)).astype('float32')
        first=(p,d,(200,200));second=(p+[3,7],d.copy(),(200,200))
        result=feature_agreement(first,second)
        self.assertIsNotNone(result);self.assertGreaterEqual(result['inliers'],15)
        self.assertIsNone(feature_agreement((np.ones_like(p),d,(200,200)),second))

    def test_repeated_wheel_design_with_background_outlier_is_not_body_identity(self):
        rng=np.random.default_rng(4)
        points=rng.uniform(80,100,(20,2)).astype('float32');points[0]=[190,190]
        descriptors=rng.normal(0,1,(20,128)).astype('float32')
        self.assertIsNone(feature_agreement((points,descriptors,(200,200)),(points+[2,3],descriptors.copy(),(200,200))))


if __name__=='__main__':unittest.main()
