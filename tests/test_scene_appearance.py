import unittest
import numpy as np
from core.scene_analysis import MotionTracker,Detection
from core.scene_appearance import signature,agreement


class AppearanceTests(unittest.TestCase):
    def test_white_vehicle_identity_does_not_jump_to_dark_vehicle(self):
        tracker=MotionTracker('front');box=[40,40,100,60]
        frame=np.full((180,240,3),180,np.uint8)
        for i in range(4):first=tracker.update([Detection('car',.9,box)],i*.1,frame)[0]
        frame[40:100,40:140]=25
        other=tracker.update([Detection('car',.9,box)],.4,frame)[0]
        self.assertNotEqual(first['track_id'],other['track_id'])

    def test_small_colour_variation_retains_motion_identity(self):
        tracker=MotionTracker('front');ids=[]
        for i,value in enumerate([180,177,170,174,181]):
            frame=np.full((180,240,3),value,np.uint8)
            ids.append(tracker.update([Detection('car',.9,[40+i,40,100,60])],i*.1,frame)[0]['track_id'])
        self.assertEqual(len(set(ids)),1)

    def test_tiny_or_clipped_boxes_do_not_supply_colour_identity(self):
        frame=np.full((180,240,3),180,np.uint8)
        self.assertIsNone(signature(frame,[0,40,100,60]))
        self.assertIsNone(signature(frame,[40,40,15,15]))
        self.assertTrue(agreement(None,signature(frame,[40,40,100,60]))[0])
