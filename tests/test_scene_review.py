import unittest
import numpy as np
from core.scene_review import compare_targets,check_relations
from core.scene_analysis import SceneDetector,CLASSES


class SceneReviewTests(unittest.TestCase):
    def test_one_detection_cannot_satisfy_two_reference_targets(self):
        targets=[{'id':str(i),'kind':'vehicle','bbox':[0,0,20,20],'status':'confirmed'} for i in range(2)]
        result=compare_targets(targets,[{'kind':'car','bbox':[0,0,20,20]}])
        self.assertEqual(result['matched'],1)
        self.assertEqual(result['missed'],1)

    def test_uncertain_signal_is_not_a_success_or_a_recall_denominator(self):
        result=compare_targets([{'id':'signal','kind':'traffic_light','bbox':[0,0,20,20],'status':'uncertain'}],
                               [{'kind':'traffic_light','bbox':[0,0,20,20]}])
        self.assertEqual(result['targets'],0)
        self.assertEqual(result['matched'],0)
        self.assertEqual(result['uncertain_targets'],1)
        self.assertEqual(result['unmatched_detections'],[0])

    def test_vehicle_family_match_does_not_hide_subtype(self):
        result=compare_targets([{'id':'SUV','kind':'vehicle','subtype':'SUV','bbox':[0,0,20,20],'status':'confirmed'}],
                               [{'kind':'truck','bbox':[0,0,20,20]}])
        self.assertEqual(result['matched'],1)
        self.assertEqual(result['results'][0]['detected_kind'],'truck')
        self.assertEqual(result['results'][0]['reference_subtype'],'SUV')

    def test_missing_3d_actor_is_unassessable_not_a_pass(self):
        r=check_relations([{'a':'missing','b':'ego','type':'left_of'}],{})
        self.assertEqual(r[0]['outcome'],'unassessable')

    def test_light_class_is_opt_in_and_has_no_color_claim(self):
        d=SceneDetector.__new__(SceneDetector); d.threshold=.25; d.classes=dict(CLASSES)
        output=np.zeros((1,84,1),np.float32);output[0,:4,0]=[50,50,20,20];output[0,13,0]=.9
        self.assertEqual(d.decode(output,(100,100),(1,1,0,0)),[])
        d.classes[9]='traffic_light'
        detections=d.decode(output,(100,100),(1,1,0,0))
        self.assertEqual(len(detections),1)
        self.assertEqual(detections[0].kind,'traffic_light')
        self.assertNotIn('state',vars(detections[0]))
