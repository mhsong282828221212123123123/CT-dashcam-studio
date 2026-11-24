import unittest
import numpy as np
from ct_scene.curved_road import fit_reference,route_index,supported_boundaries,phase_protection_windows,vehicle_supported_paint_gaps,observed_paint_phase,continuous_edge_bounds
from ct_scene.road_frame import RoadFrame
from ct_scene.road_geometry import lane_at,supported
from ct_scene.road_perception import restore_mask
from ct_scene.junctions import infer,straight_openings,crossing_motion


class CurveEvidenceTests(unittest.TestCase):
    def test_sparse_long_dash_cannot_erase_stronger_outer_stripe_on_empty_road(self):
        frame=RoadFrame([[0,0],[100,0]]);ego=np.c_[np.arange(20),np.zeros(20),np.zeros(20)]
        solids={0:[(1.75,str(i)) for i in range(10)],-1:[(-1.75,str(i)) for i in range(40)]}
        peak=dict(kind='white',index=1,y=5.25,frames=80)
        bounds,review=continuous_edge_bounds(solids,frame,3.5,[],ego,4,paint_peaks=[peak])
        self.assertIsNone(bounds);self.assertEqual(review['right_edge_candidate']['y'],-1.75)
        self.assertEqual(review['status'],'inner_continuous_candidate_has_stronger_adjacent_separator')
        peak['frames']=15
        bounds,_=continuous_edge_bounds(solids,frame,3.5,[],ego,4,paint_peaks=[peak])
        self.assertEqual(len(bounds),2)

    def test_stripes_need_not_have_nominal_half_width_phase(self):
        reference=dict(points_W_xy=[[0,0],[120,0]])
        def paint(shift):
            return [dict(kind=kind,frame_id=str(i),points_W_xy=[[i*8,d+shift],[i*8+3,d+shift]])
                    for i in range(12) for kind,d in [('yellow',5.25),('white',1.75),('white',-1.75)]]
        before,_=supported_boundaries(paint(0),reference,3.5,[])
        after,review=supported_boundaries(paint(1),reference,3.5,[])
        self.assertTrue(np.allclose(np.asarray(after)-before,1.))
        self.assertEqual(len(after),3);self.assertAlmostEqual(review['paint_phase']['phase_m'],2.75)
        self.assertFalse(review['paint_phase']['actors_used']);self.assertFalse(review['paint_phase']['ego_used'])

    def test_long_stop_cannot_outvote_spatially_repeated_stripe_phase(self):
        projected=[(0.,1.75,dict(kind='white',frame_id=f'stopped-{i}')) for i in range(1000)]
        projected += [(s,2.75,dict(kind='white',frame_id=f'{s}-{i}')) for s in [30,60,90,120,150] for i in range(3)]
        phase,review=observed_paint_phase(projected,3.5)
        self.assertAlmostEqual(phase,2.75);self.assertEqual(review['spatial_bins_with_repeated_frames'],5)

    def test_left_separator_traffic_does_not_invalidate_right_solid_edge(self):
        reference=dict(points_W_xy=[[0,0],[100,0]]);segments=[]
        for i in range(12):
            for d in [5.25,1.75,-1.75,-5.25]:
                segments.append(dict(kind='white',frame_id=str(i),points_W_xy=[[i*4,d],[i*4+10,d]],ground_length_m=10.,color_continuity=.95))
        rows=[dict(time_s=j*.2,position_W=[j,3.5,0],support='detector_observation',bbox_xyxy=[100,300,300,500],image_width=1280) for j in range(40)]
        bounds,review=supported_boundaries(segments,reference,3.5,[dict(id='adjacent',camera_id='front',samples=rows)],ego_positions=[[0,0,0],[40,0,0]])
        self.assertEqual(bounds,[5.25,1.75,-1.75])
        self.assertEqual(review['peaks'][-1]['kind'],'continuous_right_edge_hypothesis')

    def test_left_vehicle_extent_is_symmetric_but_never_crosses_yellow_divider(self):
        reference=dict(points_W_xy=[[0,0],[100,0]])
        rows=[dict(time_s=j*.2,position_W=[j,5.8,0],unrefined_position_W=[j,5.8,0],support='detector_observation',camera_id='front',bbox_xyxy=[100,300,300,500],image_width=1280) for j in range(40)]
        actors=[dict(id='outer',camera_id='front',samples=rows)]
        for kind,expected in [('white',3),('yellow',2)]:
            segments=[dict(kind=kind if d==5.25 else 'white',frame_id=str(i),points_W_xy=[[i*4,d],[i*4+3,d]]) for i in range(12) for d in [5.25,1.75,-1.75]]
            bounds,review=supported_boundaries(segments,reference,3.5,actors)
            self.assertEqual(len(bounds)-1,expected)
            if kind=='white':self.assertEqual(review['vehicle_extent'][0]['side'],'left')

    def test_right_yellow_curb_cannot_be_expanded_by_vehicle_depth_noise(self):
        reference=dict(points_W_xy=[[0,0],[100,0]])
        segments=[dict(kind='yellow',frame_id=str(i),points_W_xy=[[i*4,d],[i*4+3,d]]) for i in range(12) for d in [1.75,-1.75]]
        rows=[dict(time_s=j*.2,position_W=[j,-2.5,0],unrefined_position_W=[j,-2.5,0],support='detector_observation',camera_id='front',bbox_xyxy=[100,300,300,500],image_width=1280) for j in range(40)]
        bounds,_=supported_boundaries(segments,reference,3.5,[dict(id='noisy',camera_id='front',samples=rows)])
        self.assertEqual(bounds,[1.75,-1.75])

    def test_local_separator_preserves_known_divider_until_actual_yellow_diverges(self):
        reference=dict(points_W_xy=[[0,0],[100,0]])
        for outer_kind,count in [('white',2),('yellow',3)]:
            segments=[dict(kind=outer_kind if d==8.75 else 'white',frame_id=str(i),points_W_xy=[[i*4,d],[i*4+3,d]]) for i in range(12) for d in [8.75,5.25,1.75,-1.75]]
            bounds,review=supported_boundaries(segments,reference,3.5,[],inherited_divider=5.25)
            self.assertEqual(len(bounds)-1,count)
            if outer_kind=='white':self.assertEqual(review['peaks'][0]['kind'],'inherited_yellow_divider_with_local_separator')

    def test_constant_heading_error_is_not_independent_lane_change(self):
        w=[dict(start_s=1.,end_s=8.)];t=np.arange(10.)
        road=dict(heading_evidence=dict(slope=np.tan(np.radians(3))))
        self.assertEqual(phase_protection_windows(w,t,np.full(10,np.radians(7)),road),[])
        yaw=np.radians([3,3,4,6,5,2,1,2,3,3])
        self.assertEqual(phase_protection_windows(w,t,yaw,road),w)

    def test_yellow_and_white_bound_single_lane_and_exclude_opposite_paint(self):
        reference=dict(points_W_xy=[[0,0],[100,0]])
        segments=[]
        for i in range(12):
            for kind,d in [('yellow',1.7),('white',-1.8),('white',5.3)]:
                segments.append(dict(kind=kind,frame_id=str(i),points_W_xy=[[i*4,d],[i*4+3,d]]))
        bounds,review=supported_boundaries(segments,reference,3.5,[])
        self.assertEqual(bounds,[1.7,-1.8]);self.assertEqual(review['peaks'][0]['kind'],'yellow_divider')

    def test_visible_far_left_stripes_cannot_exclude_own_route(self):
        reference=dict(points_W_xy=[[0,0],[100,0]]);segments=[]
        for i in range(12):
            for d in [5.25,8.75]:segments.append(dict(kind='white',frame_id=str(i),points_W_xy=[[i*4,d],[i*4+3,d]]))
        bounds,review=supported_boundaries(segments,reference,3.5,[],ego_positions=[[0,0,0],[40,0,0]])
        self.assertIsNone(bounds);self.assertEqual(review['status'],'rejected_bounds_excluding_own_observed_route')

    def test_occluded_internal_stripes_are_labelled_priors_between_outer_paint(self):
        reference=dict(points_W_xy=[[0,0],[100,0]])
        peaks=[dict(index=k,y=y,frames=10) for k,y in [(2,8.75),(1,5.25),(-2,-5.25)]]
        actors=[]
        for i,d in enumerate([4.8,1.8,-1.6]):
            rows=[dict(time_s=j*.2,position_W=[j,d,0],unrefined_position_W=[j,d,0],support='detector_observation',bbox_xyxy=[100,300,300,500],image_width=1280) for j in range(40)]
            actors.append(dict(id=str(i),camera_id='front',samples=rows))
        bounds,review=vehicle_supported_paint_gaps(dict(peaks=peaks),reference,3.5,actors,[[0,1.2,0],[40,1.2,0]])
        self.assertEqual(bounds,[5.25,1.75,-1.75,-5.25]);self.assertFalse(review['independent_lane_count_measurement'])
        for r in actors[0]['samples']:r['position_W'][0]*=-1;r['unrefined_position_W'][0]*=-1
        _,review=vehicle_supported_paint_gaps(dict(peaks=peaks),reference,3.5,actors,[[0,1.2,0],[40,1.2,0]])
        self.assertNotIn('0',[p['local_id'] for p in review['vehicles']])

    def test_forward_vehicle_beyond_observed_stripe_supports_only_one_outer_prior(self):
        reference=dict(points_W_xy=[[0,0],[100,0]])
        peaks=[dict(index=k,y=y,frames=10) for k,y in [(2,8.75),(1,5.25),(-2,-5.25)]]
        actors=[]
        for i,d in enumerate([4.7,1.7,-5.8]):
            rows=[dict(time_s=j*.2,position_W=[j,d,0],unrefined_position_W=[j,d,0],support='detector_observation',bbox_xyxy=[100,300,300,500],image_width=1280) for j in range(40)]
            actors.append(dict(id=str(i),camera_id='front',samples=rows))
        bounds,review=vehicle_supported_paint_gaps(dict(peaks=peaks),reference,3.5,actors,[[0,1.2,0],[40,1.2,0]])
        self.assertEqual(len(bounds)-1,4);self.assertEqual(review['outer_paint'][-1]['kind'],'one_outer_vehicle_corridor_width_prior')
        self.assertIsNone(vehicle_supported_paint_gaps(dict(peaks=peaks),reference,3.5,actors,[[0,1.2,0],[40,1.2,0]],max_lanes=3)[0])

    def test_continuous_inner_edges_exclude_hatched_shoulders(self):
        reference=dict(points_W_xy=[[0,0],[100,0]]);segments=[]
        for i in range(12):
            for d in [5.25,1.75,-1.75,-5.25]:
                segments.append(dict(kind='white',frame_id=str(i),points_W_xy=[[i*4,d],[i*4+10,d]],ground_length_m=10.,color_continuity=.95))
        bounds,review=supported_boundaries(segments,reference,3.5,[],ego_positions=[[0,0,0],[40,0,0]])
        self.assertEqual(bounds,[1.75,-1.75]);self.assertEqual(review['status'],'repeated_continuous_carriageway_edges')

    def test_solid_bus_separator_cannot_erase_supported_traffic_lane(self):
        reference=dict(points_W_xy=[[0,0],[100,0]]);segments=[]
        for i in range(12):
            for d in [1.75,-1.75,-5.25]:segments.append(dict(kind='white',frame_id=str(i),points_W_xy=[[i*4,d],[i*4+10,d]],ground_length_m=10.,color_continuity=.95))
        rows=[dict(time_s=j*.2,position_W=[j,-3.5,0],support='detector_observation',bbox_xyxy=[100,300,300,500],image_width=1280) for j in range(40)]
        bounds,review=supported_boundaries(segments,reference,3.5,[dict(id='bus',camera_id='front',samples=rows)],ego_positions=[[0,0,0],[40,0,0]])
        self.assertEqual(len(bounds)-1,2)

    def test_repeated_camera_paint_keeps_bends_between_stations(self):
        angle=np.linspace(0,1.3,150);ego=np.c_[80*np.sin(angle),80*(1-np.cos(angle)),angle*0]
        helper=route_index(ego,yaws=angle);segments=[]
        for i,s in enumerate(np.arange(5,100,10)):
            for j in range(4):
                segments.append(dict(kind='white',frame_id=f'{i}-{j}',points_W_xy=helper.world([s-2,s+2],1.75).tolist()))
        line,review=fit_reference(segments,ego,[1.75,-1.75,-5.25],yaws=angle)
        self.assertIsNotNone(line);self.assertGreater(len(line['points_W_xy']),30)
        frame=RoadFrame(line['points_W_xy'],line['station_origin_m'])
        local=frame.project(ego[15:120]);self.assertLess(np.max(abs(local[:,1])),.3)
        self.assertFalse(review['actors_used'])

    def test_paint_missing_does_not_create_camera_supported_road(self):
        ego=np.c_[np.arange(100),np.zeros((100,2))]
        line,review=fit_reference([],ego,[1.75,-1.75]);self.assertIsNone(line)

    def test_long_unobserved_turn_does_not_discard_supported_road_before_and_after(self):
        angle=np.linspace(0,1.5,240);ego=np.c_[140*np.sin(angle),140*(1-np.cos(angle)),angle*0];helper=route_index(ego,yaws=angle);paint=[]
        for i,s in enumerate([0,10,20,160,170,180]):
            for j in range(4):paint.append(dict(kind='white',frame_id=f'{i}-{j}',points_W_xy=helper.world([s-2,s+2],1.75).tolist()))
        line,review=fit_reference(paint,ego,[1.75,-1.75],yaws=angle)
        self.assertIsNotNone(line);self.assertLess(review['supported_fraction'],.65)
        self.assertEqual(line['camera_coverage'],'partial');self.assertTrue(line['unsupported_intervals_m'])
        road=dict(boundaries_y=[1.75,-1.75],reference_line=line)
        gap=line['unsupported_intervals_m'][0];self.assertIsNone(lane_at(road,sum(gap)/2,0))
        self.assertIsNotNone(lane_at(road,gap[0]-2,0));self.assertFalse(review['ego_modified'])

    def test_unknown_gap_has_no_lane_membership(self):
        road={'boundaries_y':[1.75,-1.75],'reference_line':{'unsupported_intervals_m':[[10,20]],'supported_extent_m':[-10,40]}}
        self.assertIsNone(lane_at(road,15,0));self.assertEqual(lane_at(road,25,0),0)
        self.assertFalse(supported(road,60))

    def test_raw_mask_inverse_letterbox_has_no_padding(self):
        mask=np.zeros((640,640),np.uint8);mask[80:560]=1
        restored=restore_mask(mask,(960,1280));self.assertEqual(restored.shape,(960,1280));self.assertTrue(restored.all())

    def test_map_only_is_not_a_camera_junction(self):
        t=np.arange(0,10,.2);ego=np.c_[t*5,t*0,t*0]
        road={'boundaries_y':[1.75,-1.75],'reference_line':{'points_W_xy':[[0,0],[60,0]],'unsupported_intervals_m':[[5,30]]}}
        maps=[dict(time_s=float(x),junction_candidates=[dict(distance_m=10)]) for x in t]
        self.assertEqual(infer(t,ego,t*0,road,maps),[])

    def test_straight_intersection_uses_camera_station_not_gps_lane(self):
        t=np.arange(0,10,.2);ego=np.c_[t*5,t*0,t*0]
        road=dict(boundaries_y=[1.75,-1.75],reference_line=dict(points_W_xy=[[0,0],[60,0]]),junctions=[])
        paint=[dict(kind='crossbar',frame_id=str(i),time_s=float(x),points_W_xy=[[30,-1.5],[30,1.6]]) for i,x in enumerate([2,2.4,2.8,3.2])]
        maps=[dict(time_s=float(x),junction_candidates=[dict(node_id=10,distance_m=25,forward_m=35-x*5)]) for x in [1,2,3,4]]
        self.assertEqual(straight_openings(t,ego,road,[],maps),[])
        self.assertEqual(straight_openings(t,ego,road,paint,[]),[])
        result=straight_openings(t,ego,road,paint,maps)
        self.assertEqual(len(result),1);self.assertEqual(result[0]['road_station_range_m'],[28,42])
        road['junctions']=result
        self.assertIsNone(lane_at(road,35,0))

    def test_crossing_inside_nominal_lane_is_not_parallel_lane_follower(self):
        road=dict(boundaries_y=[1.75,-1.75],reference_line=dict(points_W_xy=[[0,0],[60,0]]),
                  junctions=[dict(polygon_W_xy=[[20,-12],[40,-12],[40,12],[20,12]],road_station_range_m=[20,40])])
        rows=[dict(time_s=i*.2,position_W=[30,-6+i*1.5,0],bbox_xyxy=[0,0,100,100],support='detector_observation') for i in range(9)]
        self.assertTrue(crossing_motion(dict(samples=rows),road))
        for r in rows:r['position_W']=[30,0,0]
        self.assertFalse(crossing_motion(dict(samples=rows),road))

    def test_adjacent_shared_crosswalk_can_confirm_junction_behind_queue(self):
        t=np.arange(0,10,.2);ego=np.c_[t*5,t*0,t*0]
        road=dict(boundaries_y=[1.75,-1.75],reference_line=dict(points_W_xy=[[0,0],[60,0]]),junctions=[])
        paint=[dict(kind='crossbar',frame_id=str(i),time_s=float(x),points_W_xy=[[30,4],[30,8]]) for i,x in enumerate([2,2.4,2.8,3.2])]
        maps=[dict(time_s=float(x),junction_candidates=[dict(node_id=10,distance_m=25,forward_m=35-x*5)]) for x in [1,2,3,4]]
        result=straight_openings(t,ego,road,paint,maps)
        self.assertEqual(len(result),1);self.assertEqual(result[0]['road_station_range_m'],[28,42])
        self.assertIn('adjacent_surface_junction_evidence_only',result[0]['camera_station_peaks'][0]['camera_paint_roles'])
        self.assertEqual(straight_openings(t,ego,road,paint,[]),[])

    def test_nearby_distinct_mapped_junctions_are_not_chain_merged(self):
        t=np.arange(0,30,.2);ego=np.c_[t*5,t*0,t*0];road=dict(boundaries_y=[1.75,-1.75],reference_line=dict(points_W_xy=[[0,0],[150,0]]),junctions=[])
        paint=[];maps=[]
        for s,node in [(30,10),(50,10),(70,20),(90,20)]:
            for i,now in enumerate([s/5-2,s/5-1.4,s/5-.8]):
                paint.append(dict(kind='crossbar',frame_id=f'{s}-{i}',time_s=now,points_W_xy=[[s,-1.5],[s,1.6]]))
                maps.append(dict(time_s=now,junction_candidates=[dict(node_id=node,distance_m=10,forward_m=s-now*5)]))
        result=straight_openings(t,ego,road,paint,maps)
        self.assertEqual(len(result),2)

    def test_turn_plus_camera_gap_and_repeated_crossbar_works_without_map(self):
        t=np.arange(0,10,.2);yaw=np.clip((t-2)/5,0,1)*np.pi/2
        ego=np.zeros((len(t),3))
        for i in range(1,len(t)):ego[i]=ego[i-1]+np.array([np.cos(yaw[i]),np.sin(yaw[i]),0])
        road={'boundaries_y':[1.75,-1.75],'reference_line':{'points_W_xy':ego[::4,:2].tolist(),'unsupported_intervals_m':[[0,50]]}}
        result=infer(t,ego,yaw,road,crossing_times=[.8,1.,1.2,1.4])
        self.assertEqual(len(result),1);self.assertIn('camera_crossing',result[0]['support'])
