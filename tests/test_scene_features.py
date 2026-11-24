import json
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np
from core.scene_analysis import Detection
from core.scene_features import detect_lane_segments,signal_appearance,SignalStateTracker
from core.scene_review_assist import ReviewedObservationAssist


class SceneFeatureTests(unittest.TestCase):
    def test_lane_segments_require_visible_paint_and_respect_objects(self):
        image=np.full((480,640,3),65,np.uint8)
        self.assertEqual(detect_lane_segments(image),[])
        cv2.line(image,(80,445),(280,270),(230,230,230),8)
        cv2.line(image,(560,445),(360,270),(230,230,230),8)
        lanes=detect_lane_segments(image)
        self.assertEqual({l['id'] for l in lanes},{'lane-left','lane-right'})
        self.assertTrue(all(not l['extrapolated'] for l in lanes))
        covered=[{'bbox':[0,250,640,230]}]
        self.assertEqual(detect_lane_segments(image,covered),[])

    def test_signal_requires_temporal_support_and_keeps_vertical_heads_unknown(self):
        image=np.zeros((200,300,3),np.uint8)
        cv2.circle(image,(70,30),5,(0,220,0),-1)
        tracker=SignalStateTracker();d=Detection('traffic_light',.8,[30,20,60,20])
        self.assertEqual(tracker.update(image,[d],0)[0]['state'],'unknown')
        self.assertEqual(tracker.update(image,[d],.1)[0]['state'],'unknown')
        self.assertEqual(tracker.update(image,[d],.2)[0]['state'],'green')
        tracker=SignalStateTracker();vertical=Detection('traffic_light',.9,[60,15,20,50])
        for t in (0,.1,.2):result=tracker.update(image,[vertical],t)
        self.assertEqual(result[0]['state'],'unknown')

    def test_saturation_does_not_become_a_color(self):
        image=np.full((40,100,3),255,np.uint8)
        self.assertEqual(signal_appearance(image,[0,0,100,40])['state'],'unknown')

    def test_review_corrections_are_explicit_clip_bound_and_expire(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'reference.json'
            reference={'front_file':'one-front.mp4','image_size':[100,100],
                'frames':[{'camera':'front','time_s':0,'targets':[{'id':'car','kind':'vehicle','status':'confirmed','bbox':[20,20,40,40]}]}]}
            p.write_text(json.dumps(reference))
            with self.assertRaises(ValueError): ReviewedObservationAssist(p,'another-front.mp4')
            assist=ReviewedObservationAssist(p,'one-front.mp4')
            image=np.random.default_rng(1).integers(0,255,(100,100,3),dtype=np.uint8)
            objects,_,_=assist.update('front',image,0,[])
            self.assertEqual(objects[0]['source'],'visual_review_flow')
            self.assertEqual(objects[0]['confidence_type'],'review_status_not_model_probability')
            self.assertEqual(assist.update('front',image,1,[])[0],[])
            with self.assertRaises(ValueError):assist.update('front',np.zeros((50,50,3),np.uint8),0,[])

    def test_review_flow_does_not_hold_an_object_without_visual_support(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'reference.json'
            target={'id':'car','kind':'vehicle','status':'confirmed','bbox':[20,20,40,40]}
            p.write_text(json.dumps({'front_file':'one-front.mp4','image_size':[100,100],'frames':[
                {'camera':'front','time_s':0,'targets':[target]}, {'camera':'front','time_s':1,'targets':[target]}]}))
            assist=ReviewedObservationAssist(p,'one-front.mp4')
            image=np.zeros((100,100,3),np.uint8)
            self.assertEqual(len(assist.update('front',image,0,[])[0]),1)
            self.assertEqual(assist.update('front',image,.1,[])[0],[])
