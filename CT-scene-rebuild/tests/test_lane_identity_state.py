import copy
import unittest
import numpy as np
from ct_scene.lane_state import timeline
from ct_scene.identity_graph import select_handoffs,identity_evidence_sufficient
from ct_scene.surrounding import identity_observation_usable
from ct_scene.scene_audit import visible_ids_at,observation_coverage


ROAD={'boundaries_y':[5.25,1.75,-1.75,-5.25]}


class LaneStateTests(unittest.TestCase):
    def test_left_change_completes_only_with_entire_footprint_inside(self):
        t=np.arange(0,8,.1);y=np.interp(t,[0,2,4,7.9],[0,0,3.5,3.5]);p=np.c_[t*5,y,t*0];before=p.copy();road=copy.deepcopy(ROAD)
        r=timeline(t,p,t*0,road,4.7,2.0)
        self.assertEqual(len(r['events']),1);e=r['events'][0];self.assertEqual(e['direction'],'left')
        self.assertGreater(e['completion_s'],e['crossing_s'])
        completed=next(x for x in r['records'] if x['time_s']>=e['completion_s']);self.assertGreaterEqual(completed['footprint_clearance_m'],.12)
        self.assertEqual(r['records'][-1]['phase'],'stable');np.testing.assert_array_equal(p,before);self.assertEqual(road,ROAD)

    def test_crossed_centre_with_body_on_line_is_not_completion(self):
        t=np.arange(0,7,.1);y=np.interp(t,[0,2,4,6.9],[0,0,2.0,2.0]);p=np.c_[t*5,y,t*0]
        r=timeline(t,p,t*0,ROAD,4.7,2.0)
        self.assertEqual(len(r['events']),1);self.assertIsNone(r['events'][0]['completion_s'])
        self.assertEqual(r['records'][-1]['phase'],'crossed_not_fully_inside')

    def test_boundary_jitter_does_not_create_repeated_changes(self):
        t=np.arange(0,6,.1);y=np.where(t<1,0,np.where(np.arange(len(t))%2,1.72,1.78));p=np.c_[t*5,y,t*0]
        r=timeline(t,p,t*0,ROAD,4.7,2.)
        self.assertEqual(r['events'],[]);self.assertTrue(all(x['lane_id']==1 for x in r['records']))

    def test_stationary_heading_noise_does_not_move_lane(self):
        t=np.arange(0,6,.1);p=np.zeros((len(t),3));yaw=np.sin(t)*.5
        self.assertEqual(timeline(t,p,yaw,ROAD,4.7,2.)['events'],[])

    def test_unknown_road_gap_does_not_prove_a_lane_change(self):
        t=np.arange(0,6,.1);y=np.where(t<2,0,np.where(t<3,7,3.5));p=np.c_[t*5,y,t*0]
        r=timeline(t,p,t*0,ROAD,4.7,2.)
        self.assertEqual(r['events'],[]);self.assertEqual(r['records'][-1]['lane_id'],0)

    def test_removed_left_bay_does_not_renumber_ego_lane(self):
        t=np.arange(0,6,.1);p=np.c_[t*10,t*0,t*0];road=dict(ROAD,marking_station_x=5,post_marking_boundaries_y=[1.75,-1.75,-5.25])
        r=timeline(t,p,t*0,road,4.7,2.)
        self.assertTrue(all(x['lane_id']==1 for x in r['records']));self.assertEqual(r['events'],[])


class ObservationStateTests(unittest.TestCase):
    def test_presence_does_not_depend_on_success_of_3d_contacts(self):
        presence={'A':np.arange(0,2,.2)}
        self.assertEqual(visible_ids_at(presence,.6),{'A'})
        self.assertEqual(visible_ids_at(presence,.7),{'A'})
        self.assertEqual(visible_ids_at({'A':np.array([0.,2.])},1),set())

    def test_audit_flags_ghost_when_source_is_observed_and_keeps_rear_reference(self):
        rows=[dict(local_track_id='front-A',camera_id='front',camera_time_s=0.,frame_id='F',support='detector_observation'),
            dict(local_track_id='back-A',camera_id='back',camera_time_s=0.,frame_id='R',support='detector_observation')]
        actors=[dict(global_id='G',local_ids=['front-A'])];states=[dict(time_s=0.,actors=[dict(id='G',ghost=True)])]
        audit=observation_coverage(rows,actors,states,[],{})
        self.assertEqual(len(audit['observed_ghost_conflicts']),1)
        self.assertEqual(audit['camera_summary']['back']['reference_only_samples'],1)


class IdentityGraphTests(unittest.TestCase):
    def test_future_full_view_can_resolve_long_past_edge_track_but_not_brief_glimpse(self):
        past={'identity_evidence':{'usable_observations':0,'observed_samples':60,'observed_span_s':12}}
        future={'identity_evidence':{'usable_observations':5}}
        self.assertTrue(identity_evidence_sufficient(past,future))
        past['identity_evidence']['observed_span_s']=2
        self.assertFalse(identity_evidence_sufficient(past,future))
        self.assertFalse(identity_evidence_sufficient(future,past))

    def test_partial_or_flow_boxes_cannot_supply_identity_proof(self):
        row={'support':'detector_observation','bbox_xyxy':[1,100,700,800],'quality_flags':[]}
        self.assertFalse(identity_observation_usable(row,1280,960))
        row['bbox_xyxy']=[100,100,700,800];self.assertTrue(identity_observation_usable(row,1280,960))
        row['support']='flow_observation';self.assertFalse(identity_observation_usable(row,1280,960))

    def test_visibility_failure_keeps_candidate_unresolved(self):
        edge=self.edge('A','B',.1);edge['identity_eligible']=False
        result=select_handoffs([edge],self.nodes())
        self.assertFalse(result[0]['selected']);self.assertEqual(result[0]['reason'],'needs_clear_identity_observations')

    def nodes(self):return [dict(id=k,start_s=a,end_s=b) for k,a,b in [('A',0,1),('B',2,3),('C',4,5)]]
    def edge(self,a,b,cost):return dict(a=a,b=b,**{'from':a,'to':b},cost=cost)

    def test_cost_separation_does_not_allow_two_successors(self):
        edges=select_handoffs([self.edge('A','B',.1),self.edge('A','C',.5)],self.nodes())
        self.assertEqual([e['to'] for e in edges if e['selected']],['B'])

    def test_chain_is_not_ambiguity_between_predecessor_and_successor(self):
        edges=select_handoffs([self.edge('A','B',.1),self.edge('B','C',.1)],self.nodes())
        self.assertTrue(all(e['selected'] for e in edges))

    def test_similar_predecessors_leave_identity_unresolved(self):
        edges=select_handoffs([self.edge('A','C',.1),self.edge('B','C',.15)],self.nodes())
        self.assertFalse(any(e['selected'] for e in edges))

    def test_non_chronological_edge_is_rejected(self):
        edges=select_handoffs([self.edge('C','A',.1)],self.nodes())
        self.assertFalse(edges[0]['selected']);self.assertEqual(edges[0]['reason'],'non_chronological_handoff')
