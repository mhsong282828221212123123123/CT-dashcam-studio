import unittest
import numpy as np
from ct_scene.curved_road import supported_boundaries


class YellowReferenceGaugeTests(unittest.TestCase):
    def test_reference_translation_cannot_turn_left_median_into_right_curb(self):
        paint=[dict(kind=kind,frame_id=str(i),points_W_xy=[[x,y],[x+3,y]]) for i,x in enumerate(np.linspace(5,85,12)) for kind,y in [('yellow',1.75),('white',-1.75)]]
        ego=np.c_[np.arange(0,101,2),np.zeros((51,2))]
        original,_=supported_boundaries(paint,dict(points_W_xy=[[0,0],[100,0]]),3.5,[],ego_positions=ego)
        shifted,review=supported_boundaries(paint,dict(points_W_xy=[[0,7],[100,7]]),3.5,[],ego_positions=ego)
        self.assertIsNotNone(shifted)
        np.testing.assert_allclose(np.asarray(shifted)+7,original,atol=1e-8)
        self.assertEqual(review['peaks'][0]['kind'],'yellow_divider')
