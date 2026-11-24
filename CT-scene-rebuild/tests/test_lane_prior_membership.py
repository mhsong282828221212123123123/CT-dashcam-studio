import unittest
import numpy as np
from ct_scene.lane_prior_geometry import refine,bounded_smooth,confirmed_handoff_targets
from ct_scene.road_geometry import lane_at


class LaneMembershipGeometryTests(unittest.TestCase):
    road=dict(boundaries_y=[1.75,-1.75,-5.25],heading_evidence=dict(slope=0.))

    def rows(self,ys):
        return [dict(time_s=i*.2,position_W=[float(i),y,0.],quality=.3) for i,y in enumerate(ys)]

    def test_temporal_fit_preserves_constant_speed_and_respects_outside_evidence(self):
        t=np.arange(12)*.2;line=2.*t-8.
        result=bounded_smooth(t,line,np.ones(12),np.full(12,-np.inf),np.full(12,-5.3))
        self.assertTrue(np.all(result<=-5.3))
        unchanged=bounded_smooth(t,line,np.ones(12),np.full(12,-np.inf),np.full(12,np.inf))
        np.testing.assert_allclose(unchanged,line,atol=1e-10)

    def test_roadside_parking_and_opposing_contacts_cannot_become_lane_members(self):
        rows=self.rows([-.2,-.3,-19.9683,19.9662,-.1,-.3])
        positions,applied,lanes=refine(rows,self.road,[0]*6,[0.]*6,[.3]*6)
        self.assertIsNone(lane_at(self.road,positions[2,0],positions[2,1]))
        self.assertIsNone(lane_at(self.road,positions[3,0],positions[3,1]))
        self.assertLess(positions[2,1],-5.25)
        self.assertGreater(positions[3,1],1.75)
        self.assertEqual(lanes[2:4],[None,None])
        self.assertFalse(applied[2]);self.assertFalse(applied[3])
        self.assertTrue(applied[0])

    def test_unknown_junction_and_conflicting_lane_are_not_snapped(self):
        road=dict(self.road,junctions=[dict(road_station_range_m=[.5,2.5])])
        rows=self.rows([-3.]*4)
        result,applied,_=refine(rows,road,[0]*4,[0.]*4,[.3]*4)
        np.testing.assert_allclose(result,[r['position_W'] for r in rows],atol=1e-10)
        self.assertEqual(applied.tolist(),[False]*4)

    def test_outside_position_is_preserved_with_curved_reference_and_smoothing(self):
        road=dict(self.road,reference_line=dict(points_W_xy=[[0,0],[20,0],[35,15]]))
        rows=self.rows([-.3,-.1,-.4,-18.,-.2,-.3,-.1])
        result,applied,_=refine(rows,road,[0]*7,[0.]*7,[.3]*7)
        self.assertIsNone(lane_at(road,result[3,0],result[3,1]))
        self.assertFalse(applied[3])
        self.assertEqual(lane_at(road,result[1,0],result[1,1]),0)

    def test_only_confirmed_front_handoff_allows_modest_side_boundary_error(self):
        rows=[dict(r,camera_id='right_repeater') for r in self.rows([-5.65]*5)]
        unknown,used,_=refine(rows,self.road,[1]*5,[-3.5]*5,[.3]*5)
        self.assertFalse(np.any(used));self.assertTrue(np.all(unknown[:,1]<-5.25))
        linked,used,_=refine(rows,self.road,[1]*5,[-3.5]*5,[.3]*5,confirmed_front_handoff=True)
        self.assertTrue(np.all(used));self.assertTrue(np.all(linked[:,1]>=-5.25))
        far=[dict(r,position_W=[r['position_W'][0],-18.,0.]) for r in rows]
        linked,used,_=refine(far,self.road,[1]*5,[-3.5]*5,[.3]*5,confirmed_front_handoff=True)
        self.assertFalse(np.any(used));self.assertTrue(np.all(linked[:,1]<-5.25))

    def test_lane_disappearing_after_section_boundary_is_not_reused(self):
        road=dict(self.road,lane_sections=[dict(id='two',station_range_m=[0.,2.5],boundaries_y=[1.75,-1.75,-5.25]),
                                          dict(id='one',station_range_m=[2.5,10.],boundaries_y=[1.75,-1.75])])
        rows=[dict(r,camera_id='right_repeater') for r in self.rows([-3.]*5)]
        lanes,targets=confirmed_handoff_targets(rows,road,1)
        self.assertEqual(lanes,[1,1,1,-2,-2])
        self.assertEqual(targets[-2:],[-3.,-3.])
        result,applied,_=refine(rows,road,lanes,targets,[.3]*5,confirmed_front_handoff=True)
        self.assertFalse(any(applied[-2:]));self.assertTrue(np.isfinite(result).all())
