import copy,unittest
from unittest.mock import patch
import numpy as np
from ct_scene.local_solid_corridor import fit,unresolved_side_height_position
from ct_scene.road_geometry import boundaries_at,profile_boundaries
from ct_scene.lane_state import timeline


class LocalSolidCorridorTests(unittest.TestCase):
    def fixture(self):
        road=dict(boundaries_y=[8.75,5.25,1.75,-1.75],left_pockets=[],junctions=[],reference_line=dict(points_W_xy=[[0,0],[500,0]],supported_extent_m=[0,500]))
        paint=[]
        for s in range(40,501,5):
            lines=[('yellow',1.75),('white',-1.75)] if s<300 else [('yellow',5.25),('white',1.75),('white',-1.75)]
            for kind,d in lines:paint.append(dict(kind=kind,ground_length_m=3. if s>=300 and kind=='white' and d==1.75 else 12.,color_continuity=.99,frame_id=str(s),points_W_xy=[[s,d],[s+(3. if s>=300 and kind=='white' and d==1.75 else 12.),d]]))
        ego=np.c_[np.arange(0,501,2),np.zeros(251),np.zeros(251)]
        return road,paint,[],ego
    def test_camera_narrow_ramp_then_wider_road_not_global_max_width(self):
        args=self.fixture();before=copy.deepcopy(args);sections,proof=fit(*args);self.assertTrue(proof['changes'])
        road=dict(args[0],lane_sections=sections);self.assertEqual(len(boundaries_at(road,200))-1,1);self.assertEqual(len(boundaries_at(road,450))-1,2)
        self.assertEqual(args[0],before[0]);self.assertEqual(args[1],before[1]);np.testing.assert_array_equal(args[3],before[3])
        t=np.arange(0,50.01,.1);poses=np.c_[t*10,t*0,t*0];self.assertEqual(timeline(t,poses,t*0,road,4.7,2.)['events'],[])
    def test_missing_or_brief_paint_cannot_narrow_road(self):
        road,paint,actors,ego=self.fixture()
        for selected in [[],paint[:8]]:self.assertFalse(fit(road,selected,actors,ego)[1]['changes'])
    def test_vehicle_alone_cannot_create_narrow_road(self):
        road,paint,actors,ego=self.fixture();actors=[dict(camera_id='front',samples=[dict(time_s=i,support='detector_observation',bbox_xyxy=[100,100,300,300],image_width=1280,position_W=[i*10,0,0]) for i in range(30)])]
        self.assertFalse(fit(road,[],actors,ego)[1]['changes'])
    def test_existing_junction_or_local_layout_not_overridden(self):
        for key in ['junctions','lane_sections']:
            road,paint,actors,ego=self.fixture();road[key]=[dict(existing=True)]
            self.assertFalse(fit(road,paint,actors,ego)[1]['changes'])

    def test_actual_parallel_front_traffic_vetoes_solid_stripe_alias(self):
        road,paint,actors,ego=self.fixture()
        actors=[dict(id='actual-forward',camera_id='front',samples=[dict(time_s=s/15,support='detector_observation',bbox_xyxy=[100,100,240,300],image_width=1280,position_W=[s,3.5,0]) for s in range(80,281,5)])]
        sections,proof=fit(road,paint,actors,ego)
        self.assertEqual(sections,[]);self.assertEqual(proof['status'],'rejected_clear_parallel_traffic_beyond_narrow_edges')

    def test_losing_right_merge_lane_does_not_shift_left_separators(self):
        road=dict(boundaries_y=[5.25,1.75,-1.75,-5.25],junctions=[],reference_line=dict(points_W_xy=[[0,0],[600,0]],supported_extent_m=[0,600]))
        paint=[]
        for x in range(20,601,4):
            lines=[('yellow',-1.75),('white',-5.25)] if x<300 else [('yellow',5.25),('white',1.75),('white',-1.75),('white',-5.25)] if x<420 else [('yellow',5.25),('white',1.75),('white',-1.75)]
            for kind,d in lines:
                length=12. if kind=='yellow' or d==lines[-1][1] else 3.
                paint.append(dict(kind=kind,ground_length_m=length,color_continuity=.99,frame_id=str(x),points_W_xy=[[x,d],[x+length,d]]))
        x=np.arange(0,601,2);y=-3.5+3.5*np.clip((x-340)/100,0,1);ego=np.c_[x,y,x*0]
        sections,proof=fit(road,paint,[],ego);self.assertTrue(proof['changes'])
        values=profile_boundaries(sections[0],450.)
        self.assertAlmostEqual(values[1],1.75,delta=.2)
        self.assertEqual(len(boundaries_at(dict(road,lane_sections=sections),560))-1,2)
    def test_narrow_shape_is_not_extrapolated_across_long_unknown_tail(self):
        road,paint,actors,ego=self.fixture();paint=[r for r in paint if r['points_W_xy'][0][0]<230]
        sections,proof=fit(road,paint,actors,ego);self.assertTrue(proof['changes'])
        self.assertEqual(len(boundaries_at(dict(road,lane_sections=sections),480))-1,3)

    def test_mixed_semantic_and_continuous_width_votes_are_time_ordered(self):
        road,paint,actors,ego=self.fixture()
        def observed_bounds(segments,reference,width,actors,max_lanes,ego_positions):
            s=float(np.mean(ego_positions[:,0]))
            if s<300:return [1.75,-1.75],dict(status='semantic_paint_with_optional_outer_vehicle_extent',peaks=[dict(kind='yellow_divider',y=1.75,frames=10)])
            if s<380:return [8.75,5.25,1.75,-1.75],dict(status='semantic_paint_with_optional_outer_vehicle_extent' if s<360 else 'repeated_continuous_carriageway_edges',peaks=[])
            return [5.25,1.75,-1.75],dict(status='repeated_continuous_carriageway_edges',peaks=[])
        with patch('ct_scene.local_solid_corridor.supported_boundaries',observed_bounds):sections,proof=fit(road,paint,actors,ego)
        self.assertTrue(proof['changes']);self.assertEqual(len(boundaries_at(dict(road,lane_sections=sections),325))-1,3)

    def test_weak_side_height_outside_confirmed_narrow_band_is_unlocalized(self):
        road,paint,actors,ego=self.fixture();sections,_=fit(road,paint,actors,ego);road=dict(road,lane_sections=sections)
        actor=dict(frozen_lead=False,lane_index=1);sample=dict(camera_id='left_repeater',projection_support='apparent_vehicle_height_prior')
        self.assertTrue(unresolved_side_height_position(road,actor,sample,[200,3.5,0]));self.assertFalse(unresolved_side_height_position(road,actor,sample,[200,0,0]))
        for changed_actor,changed_sample in [(dict(actor,frozen_lead=True),sample),(dict(actor,lane_index=-2),sample),(actor,dict(sample,camera_id='front')),(actor,dict(sample,projection_support='repeated_clear_ground_contact_after_conflicting_height_prior'))]:
            self.assertFalse(unresolved_side_height_position(road,changed_actor,changed_sample,[200,3.5,0]))

    def test_side_height_outside_camera_local_section_is_not_placed_on_other_road(self):
        road=dict(boundaries_y=[5.25,1.75,-1.75],reference_line=dict(points_W_xy=[[0,0],[300,0]]),lane_sections=[dict(id='local',station_range_m=[0,300],boundaries_y=[5.25,1.75,-1.75],support='local_camera_paint_with_explicit_optional_vehicle_width_priors')])
        a=dict(frozen_lead=False,lane_index=0);r=dict(camera_id='left_repeater',projection_support='apparent_vehicle_height_prior')
        self.assertTrue(unresolved_side_height_position(road,a,r,[200,7.5,0]));self.assertFalse(unresolved_side_height_position(road,a,r,[200,5.7,0]));self.assertFalse(unresolved_side_height_position(road,dict(a,lane_index=-2),r,[200,7.5,0]))
