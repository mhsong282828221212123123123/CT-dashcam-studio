import unittest
from ct_scene.multicam_sfm import pairing_schedule,connected_camera_graph


class MultiCameraTests(unittest.TestCase):
    def test_disconnected_side_pair_does_not_create_front_rig(self):
        self.assertFalse(connected_camera_graph([('left_repeater','right_repeater',100)]))
        self.assertFalse(connected_camera_graph([('front','left_repeater',29),('left_repeater','right_repeater',100)]))
        self.assertTrue(connected_camera_graph([('front','left_repeater',30),('left_repeater','right_repeater',100)]))

    def test_pair_schedule_respects_holdout_and_does_not_require_same_time(self):
        frames=[{'sfm_name':'front/0.jpg','camera_id':'front','selection_time_s':0,'heldout':False},
                {'sfm_name':'left/6.jpg','camera_id':'left_repeater','selection_time_s':6,'heldout':False},
                {'sfm_name':'left/12.jpg','camera_id':'left_repeater','selection_time_s':12,'heldout':True},
                {'sfm_name':'front/20.jpg','camera_id':'front','selection_time_s':20,'heldout':False}]
        pairs=pairing_schedule(frames)
        self.assertEqual(pairs,[('front/0.jpg','left/6.jpg')])
