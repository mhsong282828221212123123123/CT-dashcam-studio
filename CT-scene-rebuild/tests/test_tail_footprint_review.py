import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from review_right_tail_source import footprint_review


class TailFootprintTests(unittest.TestCase):
    def states(self,y):return [dict(time_s=i*.05,actors=[dict(id='car',position_W=[i,y,0],yaw_rad=0)]) for i in range(21)]

    def test_body_outside_fails_even_if_center_is_inside(self):
        r=footprint_review(self.states(-.8),'car',{'boundaries_y':[9.4,5.85,2.28,-1.29]},4.7,2.1,0,1,2)
        self.assertEqual(r['gate'],'FAIL');self.assertGreater(r['worst_body_outside_m'],.5)

    def test_lane_center_passes_and_missing_tail_cannot_hide(self):
        states=self.states(.5);road={'boundaries_y':[9.4,5.85,2.28,-1.29]}
        self.assertEqual(footprint_review(states,'car',road,4.7,2.1,0,1,2)['gate'],'PASS')
        states[-1]['actors']=[]
        self.assertEqual(footprint_review(states,'car',road,4.7,2.1,0,1,2)['gate'],'FAIL')
