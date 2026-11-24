import unittest
import numpy as np
from ct_scene.unresolved_road import route_display
from ct_scene.road_geometry import supported,lane_at
from ct_scene.road_frame import coordinates
from ct_scene.surrounding import actor_corridor_lane


class UnresolvedRoadTests(unittest.TestCase):
    def test_turning_route_retains_car_without_claiming_a_lane_or_changing_ego(self):
        p=np.r_[np.c_[np.arange(0,51,5),np.zeros(11),np.zeros(11)],np.c_[np.full(10,50),np.arange(5,51,5),np.zeros(10)]]
        original=p.copy();line=route_display(p,np.r_[np.zeros(11),np.full(10,np.pi/2)])
        road=dict(reference_line=line,boundaries_y=[1.75,-1.75],lane_geometry_status='unknown_route_display_only')
        np.testing.assert_array_equal(p,original)
        station=coordinates(road,[50,45,0])[0]
        self.assertFalse(supported(road,station));self.assertIsNone(lane_at(road,station,0))
        rows=[dict(position_W=[50.,40.+i*.2,0.],support='detector_observation',bbox_xyxy=[500,400,800,700]) for i in range(8)]
        lane,reason,y=actor_corridor_lane(dict(camera_id='front',samples=rows),road['boundaries_y'],0.,road)
        self.assertEqual(lane,-2);self.assertIn('unresolved',reason)

    def test_straight_lane_change_and_stationary_heading_do_not_create_route_lane(self):
        p=np.c_[np.arange(0,100,5),np.linspace(0,3.5,20),np.zeros(20)]
        self.assertIsNone(route_display(p,np.linspace(0,.07,20)))
        self.assertIsNone(route_display(np.zeros((20,3)),np.linspace(0,2,20)))
