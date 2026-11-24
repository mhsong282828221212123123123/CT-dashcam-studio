import unittest
import numpy as np
from ct_scene.surrounding import lane_from_boundaries,association_edges,consolidate_camera_fragments,persistent_yellow_boundary,export_frame_count,merge_anchor_samples,road_heading_from_stations,corridor_heading,actor_corridor_lane,project_vehicle_height,fit_right_edge
from ct_scene.tracking import suppress_family_duplicates
from ct_scene.surrounding import actor_lane_priors
from ct_scene.surrounding import clear_future_motion,reconstruct_handoff,fill_handoff_gaps
from ct_scene.surrounding import support_intermediate_bridges
from ct_scene.identity_graph import identity_evidence_sufficient
from ct_scene.full_timeline import inferred_anchor_asset


def actor(key,camera,times,x,box,lane=1,hist=None):
    return dict(id=key,camera_id=camera,frozen_lead=False,lane_index=lane,
        start_s=times[0],end_s=times[-1],appearance=np.ones(128)/128 if hist is None else hist,
        samples=[dict(time_s=t,position_W=[x+t,0,0],bbox_xyxy=box,image_width=1280,
            support='detector_observation') for t in times])


class SurroundingTests(unittest.TestCase):
    def test_partial_intermediate_needs_independent_clear_successor(self):
        a=actor('A','front',[0,1],0,[100,100,300,300]);b=actor('B','right_repeater',[2,3.5],0,[0,100,900,700]);c=actor('C','front',[4,5],0,[100,100,300,300])
        a['identity_evidence']={'usable_observations':4};b['identity_evidence']={'wide_side_body_observations':8,'wide_side_body_span_s':1.5};c['identity_evidence']={'usable_observations':5}
        edge=dict(a='A',b='B',**{'from':'A','to':'B'},cost=.5,kind='future_reappearance_with_fov_boundary',identity_eligible=False,selected=False)
        self.assertFalse(support_intermediate_bridges([dict(edge)],[a,b,c])[0]['selected'])
        nxt=dict(a='B',b='C',**{'from':'B','to':'C'},cost=.5,kind='future_reappearance_with_fov_boundary',identity_eligible=True,selected=True,future_clear_motion={'distance_m':1})
        result=support_intermediate_bridges([dict(edge),dict(nxt)],[a,b,c])
        self.assertTrue(all(e['selected'] for e in result))
        c['appearance']=np.eye(1,128,0)[0]
        self.assertFalse(support_intermediate_bridges([dict(edge),dict(nxt)],[a,b,c])[0]['selected'])

    def test_clear_future_view_replaces_clipped_entry_without_moving_clear_rows(self):
        a=actor('side','right_repeater',[0,1,2],0,[0,200,700,700])
        for r in a['samples']:r['position_W']=[10*r['time_s'],0,0]
        b=actor('front','front',[3,3.2,4,4.2,4.4,4.6,4.8,5],0,[600,300,900,600])
        for r in b['samples']:
            r['position_W']=[10*r['time_s'],0,0]
            if r['time_s']<4:r.update(position_W=[10*r['time_s']+9,0,0],bbox_xyxy=[1240,300,1280,600])
        fit=clear_future_motion(a,b)
        self.assertIsNotNone(fit);self.assertLess(fit['distance_m'],1e-7)
        clear=[list(r['position_W']) for r in b['samples'] if r['time_s']>=4]
        reconstruct_handoff(a,b,fit)
        self.assertAlmostEqual(b['samples'][0]['position_W'][0],30)
        self.assertEqual(clear,[r['position_W'] for r in b['samples'] if r['time_s']>=4])

    def test_wide_partial_past_does_not_make_a_cropped_future_eligible(self):
        wide={'identity_evidence':{'usable_observations':0,'wide_side_body_observations':12,'wide_side_body_span_s':2,'observed_span_s':4}}
        clear={'identity_evidence':{'usable_observations':6}}
        self.assertTrue(identity_evidence_sufficient(wide,clear))
        self.assertFalse(identity_evidence_sufficient(clear,wide))

    def test_handoff_completion_is_bounded_and_keeps_observed_endpoints(self):
        rows=[dict(time_s=t,position_W=[10*t,0,0],yaw_rad=0,camera_id=cam,frame_id=str(t),lane_index=1,support='detector_observation')
              for t,cam in [(0,'left_repeater'),(.2,'left_repeater'),(.4,'left_repeater'),(2,'front'),(2.2,'front'),(2.4,'front')]]
        a=dict(global_id='car',samples=rows.copy());fill_handoff_gaps(a,4)
        added=[r for r in a['samples'] if r['support']=='offline_two_endpoint_gap']
        self.assertTrue(added)
        self.assertTrue(all(r['frame_id'] is None and r['endpoint_frame_ids']==['0.4','2'] for r in added))
        self.assertTrue(all(4<=r['position_W'][0]<=20 for r in added))
        self.assertTrue(all(r in a['samples'] for r in rows))
        self.assertEqual(len(a['handoff_gaps']),1)
        b=dict(global_id='car',samples=rows.copy());fill_handoff_gaps(b,1)
        self.assertEqual(b['samples'],rows)

    def test_clear_future_side_view_resolves_far_approach_without_false_lane_change(self):
        rows=[dict(time_s=i*.2,position_W=[i,-5 if i<20 else -1,0],camera_id='right_repeater',
            image_width=1280,image_height=960,support='detector_observation',
            bbox_xyxy=[400,400,500,490] if i<20 else [400,300,800,600]) for i in range(35)]
        lanes,_=actor_lane_priors(rows,[4,0,-4,-8],0,2)
        self.assertTrue(np.all(lanes==1))
        # Clear, sustained evidence in both lanes must still permit a change.
        for r in rows:r['bbox_xyxy']=[400,300,800,600]
        lanes,_=actor_lane_priors(rows,[4,0,-4,-8],0,2)
        self.assertEqual(lanes[0],2);self.assertEqual(lanes[-1],1)

    def test_actor_lane_prior_follows_sustained_geometry_instead_of_track_median(self):
        times=np.arange(0,8,.2)
        rows=[dict(time_s=float(t),position_W=[10*t,-5 if t<5 else -1,0]) for t in times]
        lanes,centres=actor_lane_priors(rows,[4,0,-4,-8],0,2)
        self.assertTrue(np.all(lanes[:20]==2))
        self.assertTrue(np.all(lanes[-10:]==1))
        self.assertTrue(np.all(centres[-10:]==-2))

    def test_actor_lane_prior_rejects_brief_boundary_jitter_and_outside_contact(self):
        rows=[dict(time_s=i*.2,position_W=[i,-2,0]) for i in range(30)]
        for i in [8,9]:rows[i]['position_W'][1]=-4.8
        for i in range(20,30):rows[i]['position_W'][1]=-10
        lanes,_=actor_lane_priors(rows,[4,0,-4,-8],0,1)
        self.assertTrue(np.all(lanes==1))

    def test_auto_anchor_assets_distinguish_bus_box_truck_and_car(self):
        for cls,name in [(5,'bus'),(7,'box_truck'),(2,'sedan')]:
            row={'class_id':cls,'support':'detector_observation','bbox_xyxy':[100,100,300,200]}
            self.assertEqual(inferred_anchor_asset([row],'assets').name,'tesla_fsd_'+name+'.glb')

    def test_right_road_edge_fit_is_robust_and_requires_longitudinal_coverage(self):
        points=[(x,-5+.01*x+(5 if x==160 else 0),str(fid)) for x in [40,80,120,160,200,240,280] for fid in range(2) for _ in range(20)]
        fit=fit_right_edge(points)
        self.assertAlmostEqual(fit['slope'],.01,delta=.001)
        self.assertAlmostEqual(fit['right_boundary_y'],-5,delta=.2)
        self.assertIsNone(fit_right_edge([p for p in points if p[0]<120]))

    def test_side_height_geometry_mirrors_between_repeaters(self):
        profile={'model':'SIMPLE_RADIAL','params':[1000,640,480,0]};box=[400,200,880,700]
        a=project_vehicle_height(box,profile,{'yaw_deg':135,'offset_W_ego_m':[0,0,0]},1.5)
        b=project_vehicle_height(box,profile,{'yaw_deg':-135,'offset_W_ego_m':[0,0,0]},1.5)
        self.assertAlmostEqual(a[0],b[0]);self.assertAlmostEqual(a[1],-b[1])

    def test_vehicle_family_nms_removes_car_truck_double_box(self):
        a=dict(class_id=2,score=.9,bbox_xyxy=[100,100,400,500])
        b=dict(class_id=7,score=.8,bbox_xyxy=[101,101,401,501])
        c=dict(class_id=2,score=.8,bbox_xyxy=[500,100,700,400])
        kept,removed=suppress_family_duplicates([a,b,c])
        self.assertEqual(kept,[a,c]);self.assertEqual(removed,[b])

    def test_large_side_vehicle_near_boundary_is_kept_but_background_is_not(self):
        a=actor('a','right_repeater',[0,1,2],0,[50,100,450,400])
        for r in a['samples']:r['position_W'][1]=-5.72
        bounds=[5.17,1.6,-1.8,-5.3]
        lane,support,_=actor_corridor_lane(a,bounds,0)
        self.assertEqual(lane,2);self.assertEqual(support,'foreground_side_boundary_uncertainty')
        for r in a['samples']:r['bbox_xyxy']=[50,100,110,140]
        self.assertIsNone(actor_corridor_lane(a,bounds,0)[0])

    def test_large_far_outside_side_vehicle_does_not_create_a_new_lane(self):
        a=actor('a','right_repeater',[0,1,2],0,[50,100,450,400])
        for r in a['samples']:r['position_W'][1]=-10
        self.assertIsNone(actor_corridor_lane(a,[5.17,1.6,-1.8,-5.3],0)[0])

    def test_side_height_projection_does_not_treat_bbox_bottom_as_contact(self):
        profile={'model':'SIMPLE_RADIAL','params':[1000,640,480,0]}
        mount={'yaw_deg':0,'offset_W_ego_m':[0,0,0]}
        a=project_vehicle_height([400,200,880,700],profile,mount,1.5)
        b=project_vehicle_height([400,100,880,600],profile,mount,1.5)
        np.testing.assert_allclose(a,[3,0,0]);np.testing.assert_allclose(a,b)
        with self.assertRaises(ValueError):project_vehicle_height([400,200,880,200],profile,mount,1.5)

    def test_co_visible_similar_front_side_cars_are_not_identity_evidence(self):
        a=actor('a','front',[0,1,2],0,[1200,100,1280,500])
        b=actor('b','right_repeater',[0,1,2],0,[200,100,700,500])
        self.assertEqual(association_edges([a,b]),[])

    def test_shared_boxes_followed_by_separate_simultaneous_cars_are_not_unioned(self):
        a=actor('a','right_repeater',[0,1,2,3],0,[0,100,600,900])
        b=actor('b','right_repeater',[0,1,2,3],0,[0,100,600,900])
        for r in b['samples'][1:]:r['bbox_xyxy']=[800,400,1000,600]
        merged,edges=consolidate_camera_fragments([a,b])
        self.assertEqual(len(merged),2);self.assertEqual(edges,[])

    def test_contact_jitter_does_not_rotate_corridor_car_sideways(self):
        self.assertAlmostEqual(corridor_heading([.1,3,0],.016,0),np.arctan(.016))
        self.assertAlmostEqual(corridor_heading([10,1,0],.016,0),np.arctan(.1))
        self.assertEqual(corridor_heading([0,0,0],.016,.1),.1)

    def test_future_motion_recovers_accelerating_border_handoff(self):
        a=actor('a','left_repeater',[0,1,2],0,[1200,200,1280,800])
        for r in a['samples']:r['position_W']=[0,0,0]
        b=actor('b','front',[4,5,6,7],0,[0,200,100,800])
        for r in b['samples']:r['position_W']=[10*(r['time_s']-2),0,0]
        edges=association_edges([a,b])
        self.assertEqual(len(edges),1);self.assertTrue(edges[0]['selected'])

    def test_camera_paint_heading_uses_road_stations_not_stationary_count(self):
        points=[(0,5,'stationary','left_repeater')]*1000
        points += [(x,5+.015*x,str(i),'front') for x in [40,80,120,160,200] for i in range(30)]
        self.assertAlmostEqual(road_heading_from_stations(points,5)['slope'],.015)
        self.assertEqual(road_heading_from_stations([p for p in points if p[0]<100],5)['slope'],0)

    def test_source_end_controls_export_despite_early_actor_loss(self):
        self.assertEqual(export_frame_count({'source_duration_s':59.8835},[0,33],20),1198)
        with self.assertRaises(ValueError):export_frame_count({'source_duration_s':float('nan')},[0,33],20)

    def test_future_camera_observations_extend_anchor_without_replacing_past(self):
        a=actor('a','front',[0,1,2],0,[100,100,200,200])
        b=actor('b','right_repeater',[1,2,3,4],100,[100,100,200,200])
        samples=merge_anchor_samples(a,[a,b])
        self.assertEqual([s['time_s'] for s in samples],[0,1,2,3,4])
        self.assertEqual(samples[:3],a['samples'])
        self.assertEqual(samples[-1]['position_W'],[104,0,0])

    def test_dense_ground_paint_wins_over_sparse_persistent_background(self):
        paint=[(5.15+j*.001,str(i)) for i in range(8) for j in range(30)]
        background=[(7.75,str(i)) for i in range(20)]
        result=persistent_yellow_boundary(paint+background)
        self.assertAlmostEqual(result['boundary_y'],5.1645)
        self.assertEqual(result['support_frames'],8)

    def test_one_frame_yellow_patch_does_not_prove_road_boundary(self):
        self.assertIsNone(persistent_yellow_boundary([(5.2,'one_frame')]*500))

    def test_nonuniform_lanes_and_centreline_exclusion(self):
        bounds=[4.2,1.8,-1.4,-4.9]
        self.assertEqual([lane_from_boundaries(y,bounds) for y in [3,0,-3]],[0,1,2])
        self.assertIsNone(lane_from_boundaries(5,bounds))
        with self.assertRaises(ValueError):lane_from_boundaries(0,[1,2,-2])

    def test_front_side_handoff_keeps_different_coobserved_car_separate(self):
        front=actor('front','front',[0,1,2],0,[1200,400,1280,900])
        side=actor('side','right_repeater',[3,4,5],0,[0,100,600,900])
        other=actor('other','right_repeater',[0,1,2,3,4,5],-15,[800,400,1000,600])
        edges=association_edges([front,side,other])
        self.assertEqual([(e['a'],e['b']) for e in edges if e['selected']],[('front','side')])

    def test_ambiguous_future_reappearances_are_not_merged(self):
        front=actor('a','front',[0,1,2],0,[1200,400,1280,900])
        sides=[actor(k,'right_repeater',[3,4,5],0,[0,100,600,900]) for k in ['b','c']]
        self.assertFalse(any(e['selected'] for e in association_edges([front]+sides)))

    def test_large_same_camera_duplicate_box_can_bridge_appearance_change(self):
        a=actor('a','right_repeater',[0,1,2],0,[0,100,600,900],hist=np.eye(1,128,0)[0])
        b=actor('b','right_repeater',[2,3,4],0,[0,100,600,900],hist=np.eye(1,128,127)[0])
        merged,edges=consolidate_camera_fragments([a,b])
        self.assertEqual(len(merged),1);self.assertEqual(len(merged[0]['samples']),5)
        self.assertEqual(len(edges),1)

    def test_neighbour_same_colour_is_not_a_duplicate(self):
        a=actor('a','right_repeater',[0,1,2],0,[0,100,600,900])
        b=actor('b','right_repeater',[0,1,2],10,[800,400,1000,600])
        merged,edges=consolidate_camera_fragments([a,b])
        self.assertEqual(len(merged),2);self.assertEqual(edges,[])
