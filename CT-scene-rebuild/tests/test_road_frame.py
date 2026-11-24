import unittest
import numpy as np
from ct_scene.road_frame import RoadFrame,coordinates,world
from ct_scene.lane_state import timeline


class RoadFrameTests(unittest.TestCase):
    def test_turn_has_local_tangent_and_roundtrip(self):
        f=RoadFrame([[0,0],[30,0],[30,30]])
        p=f.world([10,45],[1,1]);c=f.project(p)
        np.testing.assert_allclose(c[:,:2],[[10,1],[45,1]],atol=1e-9)
        np.testing.assert_allclose(c[:,2],[0,np.pi/2])

    def test_finite_corner_does_not_project_onto_infinite_branch(self):
        f=RoadFrame([[0,0],[10,0],[10,10],[20,10]])
        c=f.project([11,8]);self.assertAlmostEqual(c[0],18);self.assertAlmostEqual(c[1],-1)

    def test_endpoint_extrapolation_does_not_clamp_vehicle(self):
        f=RoadFrame([[0,0],[10,0]])
        np.testing.assert_allclose(f.project([[-3,2],[15,2]])[:,:2],[[-3,2],[15,2]])

    def test_invalid_lines(self):
        for p in ([[0,0]],[[0,0],[0,0]],[[0,0],[float('nan'),0]]):
            with self.assertRaises(ValueError):RoadFrame(p)

    def test_legacy_straight_units_are_unchanged(self):
        road={'heading_evidence':{'slope':.04}}
        np.testing.assert_allclose(coordinates(road,[[20,4,0]])[0,:2],[20,3.2])
        np.testing.assert_allclose(world(road,20,3.2),[20,4])

    def test_curve_lane_keeper_does_not_change_lanes(self):
        a=np.linspace(0,1.2,100);p=np.c_[60*np.sin(a),60*(1-np.cos(a))]
        road={'boundaries_y':[1.75,-1.75,-5.25],'reference_line':{'points_W_xy':p.tolist()}}
        f=RoadFrame(p);s=np.linspace(2,68,100);xy=f.world(s,0);yaw=f.project(xy)[:,2]
        positions=np.c_[xy,np.zeros(len(xy))];original=positions.copy()
        result=timeline(np.arange(100)*.2,positions,yaw,road,4.7,2.)
        self.assertEqual(result['events'],[])
        self.assertTrue(all(r['lane_id']==0 for r in result['records']))
        np.testing.assert_array_equal(original,positions)
