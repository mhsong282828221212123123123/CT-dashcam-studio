import copy,math,unittest
from tools.review_source_maneuver import evaluate


class ManeuverReviewTests(unittest.TestCase):
    def fixture(self):
        states=[dict(time_s=i*.05,ego_position_W=[i*.5-5,-1,0],
                     actors=[dict(id='car',position_W=[i*.5,1-i*.03,0],yaw_rad=math.atan2(-.6,10),ghost=False)]) for i in range(91)]
        events=[dict(direction='right',from_lane=0,to_lane=1,crossing_s=1.7,completion_s=3.5)]
        control=dict(required_world_interval_s=[0,4.5],crossing_interval_s=[1.4,2.],completion_interval_s=[3.,4.],
                     manual_source_anchors=[dict(time_s=0,vehicle_lane=0,ego_lane=1),dict(time_s=3,vehicle_lane=1,ego_lane=1)])
        return states,events,dict(boundaries_y=[2,0,-2]),control

    def test_wrong_direction_or_missing_observations_cannot_pass(self):
        s,e,r,c=self.fixture();self.assertEqual(evaluate(s,'car',e,r,c)['motion_gate'],'PASS')
        e[0]['direction']='left';self.assertEqual(evaluate(s,'car',e,r,c)['motion_gate'],'FAIL')
        s[30]['actors']=[];self.assertEqual(evaluate(s,'car',e,r,c)['motion_gate'],'FAIL')

    def test_motion_pass_does_not_hide_source_lane_failure(self):
        s,e,r,c=self.fixture();c['manual_source_anchors'][0]['vehicle_lane']=1
        result=evaluate(s,'car',e,r,c)
        self.assertEqual(result['motion_gate'],'PASS');self.assertEqual(result['source_lane_gate'],'FAIL')
        self.assertEqual(result['overall_source_gate'],'FAIL')

    def test_truncated_window_is_rejected(self):
        s,e,r,c=self.fixture()
        with self.assertRaises(ValueError):evaluate(s[:30],'car',e,r,c)
