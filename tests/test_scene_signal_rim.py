import unittest
import cv2
import numpy as np
from core.scene_features import signal_appearance


class SignalRimTests(unittest.TestCase):
    def head(self):return np.zeros((40,160,3),np.uint8)

    def test_yellow_core_inside_red_lamp_is_red(self):
        image=self.head();cv2.circle(image,(20,20),12,(0,0,230),-1)
        cv2.circle(image,(20,20),9,(0,220,220),-1)
        self.assertEqual(signal_appearance(image,[0,0,160,40])['state'],'red')

    def test_separately_lit_amber_and_red_are_not_collapsed_to_red(self):
        image=self.head();cv2.circle(image,(20,20),8,(0,0,230),-1)
        cv2.circle(image,(65,20),12,(0,220,220),-1)
        self.assertEqual(signal_appearance(image,[0,0,160,40])['state'],'unknown')

    def test_unambiguous_amber_stays_amber(self):
        image=self.head();cv2.circle(image,(65,20),10,(0,220,220),-1)
        self.assertEqual(signal_appearance(image,[0,0,160,40])['state'],'amber')

    def test_white_glare_is_not_promoted_to_red(self):
        image=self.head();cv2.circle(image,(20,20),17,(0,0,230),-1)
        cv2.circle(image,(20,20),15,(255,255,255),-1)
        self.assertEqual(signal_appearance(image,[0,0,160,40])['state'],'unknown')
