import copy,math,unittest
import numpy as np
from ct_scene.junction_branches import hints,opening_display_polygons


def fixture():
    road=dict(boundaries_y=[5.25,1.75,-1.75],reference_line=dict(points_W_xy=[[0,0],[100,0]]),
              junctions=[dict(id='camera',road_station_range_m=[40,60],polygon_W_xy=[[40,-10],[60,-10],[60,10],[40,10]],
                              camera_station_peaks=[dict(nodes=[10],times=[3,5])])])
    branches=[dict(neighbor_node_id=i,relative_bearing_deg=b) for i,b in enumerate([0,180,-90])]
    maps=[dict(time_s=t,junction_candidates=[dict(node_id=10,level=['0',False,False],distance_m=10,branches=branches)]) for t in [3.,4.,5.]]
    return road,maps,np.arange(0,9.),np.zeros(9)


class BranchHintTests(unittest.TestCase):
    def test_wide_search_interval_is_not_a_large_asphalt_rectangle(self):
        road,maps,t,yaw=fixture();j=road['junctions'][0];j['road_station_range_m']=[20,90];j['camera_station_peaks']=[dict(s=s,nodes=[10],times=[3,5]) for s in [35,40,75]]
        before=copy.deepcopy(j);polygons=opening_display_polygons(road,j);points=np.array(polygons[0])
        self.assertLess(np.ptp(points[:,0]),20);self.assertAlmostEqual(np.mean(points[:,0]),40);self.assertEqual(j,before)
        out,_=hints(road,maps,t,yaw);self.assertEqual(out[0]['branch_display_hints'][0]['origin_W_xy'],[40.,1.75])

    def test_t_junction_has_only_actual_side_and_keeps_camera_anchor(self):
        road,maps,t,yaw=fixture();original=copy.deepcopy(road);out,review=hints(road,maps,t,yaw)
        arms=out[0]['branch_display_hints'];self.assertEqual(len(arms),1)
        self.assertAlmostEqual(arms[0]['heading_W_rad'],math.pi/2);self.assertEqual(arms[0]['origin_W_xy'],[50.,1.75])
        self.assertEqual(out[0]['polygon_W_xy'],road['junctions'][0]['polygon_W_xy']);self.assertEqual(road,original)

    def test_missing_map_and_map_without_camera_opening_create_no_branch(self):
        road,maps,t,yaw=fixture();out,_=hints(road,[],t,yaw);self.assertFalse(out[0]['branch_display_hints'])
        road['junctions']=[];self.assertEqual(hints(road,maps,t,yaw)[0],[])

    def test_grade_ambiguity_abstains(self):
        road,maps,t,yaw=fixture()
        for row in maps:
            other=copy.deepcopy(row['junction_candidates'][0]);other['level']=['1',True,False];row['junction_candidates'].append(other)
        out,_=hints(road,maps,t,yaw);self.assertFalse(out[0]['branch_display_hints'])

    def test_gps_and_lane_tags_never_position_or_resize_branch(self):
        road,maps,t,yaw=fixture();before=hints(road,maps,t,yaw)
        for row in maps:
            for candidate in row['junction_candidates']:candidate.update(latlon=[0,0],lanes=99,width=400)
        self.assertEqual(before,hints(road,maps,t,yaw))

    def test_turning_path_does_not_draw_its_driven_arms_twice(self):
        road,maps,t,yaw=fixture();road['reference_line']['points_W_xy']=[[0,0],[50,0],[50,50]]
        for row in maps:row['junction_candidates'][0]['branches'].append(dict(neighbor_node_id=9,relative_bearing_deg=90))
        out,_=hints(road,maps,t,yaw);angles=sorted(round(a['heading_W_rad'],5) for a in out[0]['branch_display_hints'])
        self.assertEqual(angles,[round(-math.pi/2,5),0.])

    def test_terminal_hint_only_ahead_of_observed_motion(self):
        road,maps,t,yaw=fixture()
        for row in maps:
            c=row['junction_candidates'][0];c['degree']=3;c['branches']=[dict(neighbor_node_id=i,relative_bearing_deg=b) for i,b in enumerate([180,-90,90])]
        future,_=hints(road,maps,t,yaw,observed_end_station=35)
        self.assertEqual(future[0]['terminal_display_station_m'],50)
        traversed,_=hints(road,maps,t,yaw,observed_end_station=70)
        self.assertNotIn('terminal_display_station_m',traversed[0])

    def test_turn_scope_hull_does_not_fill_inside_corner_as_road(self):
        road,maps,t,yaw=fixture();road['reference_line']['points_W_xy']=[[0,0],[50,0],[50,50]]
        j=dict(road['junctions'][0],road_station_range_m=[30,70],turn_deg=90)
        polygons=opening_display_polygons(road,j)
        self.assertGreater(len(polygons),10)
        self.assertTrue(all(len(p)==4 for p in polygons))
        self.assertFalse(any(min(v[0] for v in p)<40 and max(v[1] for v in p)>15 for p in polygons))

    def test_staggered_two_node_crossing_keeps_both_perpendicular_arms(self):
        road,_,t,yaw=fixture()
        road['junctions'][0]['camera_station_peaks']=[dict(s=50.,nodes=[10,11],times=[3.,5.])]
        b10=[dict(neighbor_node_id=1,relative_bearing_deg=0),dict(neighbor_node_id=11,relative_bearing_deg=180),dict(neighbor_node_id=2,relative_bearing_deg=-90)]
        b11=[dict(neighbor_node_id=10,relative_bearing_deg=0),dict(neighbor_node_id=3,relative_bearing_deg=180),dict(neighbor_node_id=4,relative_bearing_deg=90)]
        maps=[dict(time_s=tm,junction_candidates=[
            dict(node_id=10,level=['0',False,False],distance_m=10,degree=3,branches=b10),
            dict(node_id=11,level=['0',False,False],distance_m=14,degree=3,branches=b11),
        ]) for tm in [3.,4.,5.]]
        out,_=hints(road,maps,t,yaw)
        angles=sorted(round(a['heading_W_rad'],5) for a in out[0]['branch_display_hints'])
        self.assertEqual(angles,[round(-math.pi/2,5),round(math.pi/2,5)])

    def test_shallow_main_road_split_is_not_drawn_as_cross_branch(self):
        road,maps,t,yaw=fixture()
        for row in maps:
            row['junction_candidates'][0]['branches'].append(dict(neighbor_node_id=99,relative_bearing_deg=22))
        out,_=hints(road,maps,t,yaw)
        angles=[round(a['heading_W_rad'],5) for a in out[0]['branch_display_hints']]
        self.assertEqual(angles,[round(math.pi/2,5)])

    def test_turning_junction_uses_straight_entry_exit_apex_as_branch_origin(self):
        angles=[-math.pi/2+math.pi/2*k/16 for k in range(17)]
        arc=[[40.+20.*math.cos(a),20.+20.*math.sin(a)] for a in angles]
        pts=[[0.,0.]]+arc+[[60.,80.]]
        road=dict(boundaries_y=[3.5,0.,-3.5],reference_line=dict(points_W_xy=pts),
                  junctions=[dict(id='turn-j',road_station_range_m=[38.,74.],turn_deg=90.,
                                  polygon_W_xy=[[38,-5],[74,-5],[74,5],[38,5]],
                                  camera_station_peaks=[dict(nodes=[10],times=[3,5])])])
        branches=[dict(neighbor_node_id=i,relative_bearing_deg=b) for i,b in enumerate([0,180,-90,90])]
        maps=[dict(time_s=tm,junction_candidates=[dict(node_id=10,level=['0',False,False],distance_m=10,branches=branches)]) for tm in [3.,4.,5.]]
        out,_=hints(road,maps,np.arange(0,9.),np.zeros(9))
        self.assertIn('turn_apex_W_xy',out[0])
        self.assertAlmostEqual(out[0]['turn_apex_W_xy'][0],60.,delta=1.0)
        self.assertAlmostEqual(out[0]['turn_apex_W_xy'][1],0.,delta=1.0)
        self.assertEqual(len(out[0]['branch_display_hints']),2)
        for arm in out[0]['branch_display_hints']:
            self.assertAlmostEqual(arm['origin_W_xy'][0],60.,delta=1.0)
            self.assertAlmostEqual(arm['origin_W_xy'][1],0.,delta=1.0)
