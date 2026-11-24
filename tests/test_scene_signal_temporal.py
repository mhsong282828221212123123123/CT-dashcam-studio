import unittest
from unittest.mock import patch
import numpy as np
from core.scene_analysis import Detection
from core.scene_features import SignalStateTracker


class SignalTemporalTests(unittest.TestCase):
    def run_colors(self,colors,times=None):
        tracker=SignalStateTracker();frame=np.zeros((200,300,3),np.uint8)
        head=Detection('traffic_light',.9,[30,20,60,20]);result=[]
        for i,color in enumerate(colors):
            with patch('core.scene_features.signal_appearance',return_value={'state':color,'reason':'test'}):
                result.append(tracker.update(frame,[head],times[i] if times else i*.1)[0]['state'])
        return result

    def test_uncertain_frame_does_not_display_old_color_or_erase_support(self):
        self.assertEqual(self.run_colors(['red','red','unknown','red']),
                         ['unknown','unknown','unknown','red'])

    def test_green_transition_never_keeps_red_from_history(self):
        self.assertEqual(self.run_colors(['red']*3+['green']*3),
                         ['unknown','unknown','red','unknown','unknown','green'])

    def test_timestamp_gap_and_duplicate_do_not_accumulate_support(self):
        self.assertEqual(self.run_colors(['red']*3,[0,.1,1.]),['unknown']*3)
        self.assertEqual(self.run_colors(['red']*3,[0,0,0]),['unknown']*3)
