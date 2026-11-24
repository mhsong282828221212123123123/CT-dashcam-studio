import unittest
import numpy as np
from ct_scene.rear_view_cue import paired_red_lamps


class RearAppearanceTests(unittest.TestCase):
    def test_paired_red_corners_abstain_on_body_one_lamp_and_headlamps(self):
        im=np.zeros((300,400,3),'uint8');box=[100,100,300,250]
        im[150:165,115:130]=[0,0,240];im[150:165,270:285]=[0,0,240]
        self.assertTrue(paired_red_lamps(im,box)['rear_cue'])
        im[150:165,270:285]=0;self.assertFalse(paired_red_lamps(im,box)['rear_cue'])
        im[100:250,100:300]=[0,0,200];self.assertFalse(paired_red_lamps(im,box)['rear_cue'])
        im[100:250,100:300]=0;im[150:165,115:130]=255;im[150:165,270:285]=255
        self.assertFalse(paired_red_lamps(im,box)['rear_cue'])

    def test_cut_off_body_is_not_certified_by_one_corner(self):
        im=np.zeros((300,400,3),'uint8');self.assertFalse(paired_red_lamps(im,[0,100,150,250])['rear_cue'])
