import unittest
from ct_scene.surrounding import consolidate_camera_fragments


class FragmentDivergenceTests(unittest.TestCase):
    def test_early_shared_boxes_do_not_merge_later_distinct_overlapping_cars(self):
        # Adjacent distant cars overlap at IoU .25. A stolen image track once
        # shared the same detection, but they are not one physical trajectory.
        def actor(key,other):
            return dict(id=key,camera_id='front',frozen_lead=False,start_s=0,end_s=1.,appearance=[1.,0.],
                        samples=[dict(time_s=i*.2,support='detector_observation',bbox_xyxy=([0,0,100,80] if not other or i<3 else [60,0,160,80])) for i in range(6)])
        actors,edges=consolidate_camera_fragments([actor('A',False),actor('B',True)])
        self.assertEqual(len(actors),2);self.assertEqual(edges,[])

    def test_genuine_repeated_duplicate_boxes_still_merge(self):
        def actor(key):return dict(id=key,camera_id='front',frozen_lead=False,start_s=0,end_s=1.,appearance=[1.,0.],
                                 samples=[dict(time_s=i*.2,support='detector_observation',bbox_xyxy=[0,0,100,80]) for i in range(6)])
        actors,edges=consolidate_camera_fragments([actor('A'),actor('B')]);self.assertEqual(len(actors),1);self.assertEqual(len(edges),1)
