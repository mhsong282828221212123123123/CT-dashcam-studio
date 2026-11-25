import copy,math,unittest
from ct_scene.junction_display import display_bounds,corridor_gaps,mouth_edges,branch_edges,junction_markings,turn_apex
from ct_scene.junction_branches import hints,opening_display_polygons
from ct_scene.road_frame import coordinates


class JunctionDisplayTests(unittest.TestCase):
    def fixture(self):
        return dict(reference_line=dict(points_W_xy=[[0,0],[200,0]]),boundaries_y=[-2,-9],lane_sections=[dict(id='a',station_range_m=[0,40],boundaries_y=[-1.5,-5,-8.5]),dict(id='b',station_range_m=[60,200],boundaries_y=[-2,-5.5,-9])],junctions=[dict(id='j',road_station_range_m=[40,60],camera_station_peaks=[dict(s=50,nodes=[1],times=[0,2])],polygon_W_xy=[[40,-9],[60,-9],[60,0],[40,0]])])

    def test_missing_corridor_interpolates_outer_camera_edges(self):
        road=self.fixture();before=copy.deepcopy(road)
        self.assertEqual(corridor_gaps(road),[(40.,60.)]);self.assertEqual(display_bounds(road,50),[-1.75,-8.75]);self.assertEqual(road,before)
        road['junctions']=[];self.assertEqual(corridor_gaps(road),[])

    def test_branch_origin_is_in_actual_corridor_and_does_not_move_road(self):
        road=self.fixture();before=copy.deepcopy(road)
        records=[dict(time_s=t,junction_candidates=[dict(node_id=1,distance_m=10,degree=3,level=[0],branches=[dict(neighbor_node_id=i,relative_bearing_deg=a) for i,a in enumerate([0,90,-90])])]) for t in [0,1,2]]
        junctions,review=hints(road,records,[0,1,2],[0,0,0]);self.assertEqual(road,before)
        arms=junctions[0]['branch_display_hints'];self.assertEqual(len(arms),2)
        for arm in arms:self.assertAlmostEqual(coordinates(road,arm['origin_W_xy'])[1],-5.25)

    def test_t_junction_keeps_closed_side_and_branch_edges_start_outside_main(self):
        road=self.fixture();road['junctions'][0]['branch_display_hints']=[dict(origin_W_xy=[50,-5.25],heading_W_rad=-math.pi/2,width_m=3.5,length_m=25)]
        self.assertEqual(mouth_edges(road,50),(True,False));self.assertEqual(mouth_edges(road,70),(True,True))
        edges=branch_edges(road);self.assertEqual(len(edges),2)
        self.assertTrue(all(start[1]<-8.75 for start,end in edges))

    def test_uncertain_scope_is_not_a_giant_asphalt_square(self):
        road=self.fixture();polygon=opening_display_polygons(road,road['junctions'][0])[0]
        lateral=[coordinates(road,p)[1] for p in polygon]
        self.assertLessEqual(max(lateral)-min(lateral),7.01)

    def test_procedural_junction_markings_builds_hub_crosswalks_stops_and_branch_centerlines(self):
        road=self.fixture()
        road['junctions'][0]['branch_display_hints']=[
            dict(origin_W_xy=[50,-5.25],heading_W_rad=-math.pi/2,width_m=6.0,length_m=25),
            dict(origin_W_xy=[50,-5.25],heading_W_rad=math.pi/2,width_m=6.0,length_m=25),
        ]
        before=copy.deepcopy(road)
        marks=junction_markings(road)
        self.assertEqual(road,before)
        self.assertEqual(len(marks['hubs']),1)
        self.assertEqual(len(marks['stop_bars']),2)
        self.assertGreater(len(marks['crosswalks']),6)
        self.assertGreater(len(marks['branch_centers']),1)
        # Opposing carriageway expands crosswalk count across the two-way width
        road_2way=dict(road,opposing_display=dict(enabled=True,lanes=2,lane_width_m=3.5))
        marks_2way=junction_markings(road_2way)
        self.assertGreater(len(marks_2way['crosswalks']),len(marks['crosswalks']))

    def test_turning_junction_places_crosswalks_at_entry_and_exit_not_curve_midpoint(self):
        # Smooth 90-degree right turn: straight entry along +X to (40,0), arc of R=20 to (60,-20), straight exit along -Y
        angles=[math.pi/2-math.pi/2*k/16 for k in range(17)]
        arc=[[40.+20.*math.cos(a),-20.+20.*math.sin(a)] for a in angles]
        pts=[[0.,0.]]+arc+[[60.,-80.]]
        road=dict(reference_line=dict(points_W_xy=pts,supported_extent_m=[0.,140.]),
                  boundaries_y=[3.5,0.,-3.5],
                  opposing_display=dict(enabled=True,lanes=2,lane_width_m=3.5),
                  junctions=[dict(id='turn-0',road_station_range_m=[38.,74.],turn_deg=-90.,
                                  polygon_W_xy=[[38,-5],[74,-5],[74,5],[38,5]],
                                  branch_display_hints=[dict(origin_W_xy=[60.,0.],heading_W_rad=0.,width_m=6.,length_m=25.),
                                                        dict(origin_W_xy=[60.,0.],heading_W_rad=math.pi/2,width_m=6.,length_m=25.)])])
        apex=turn_apex(road,road['junctions'][0])
        self.assertAlmostEqual(float(apex[0]),60.,delta=1.0)
        self.assertAlmostEqual(float(apex[1]),0.,delta=1.0)
        self.assertEqual(mouth_edges(road,56.),(False,False))
        marks=junction_markings(road)
        self.assertEqual(len(marks['hubs']),1)
        self.assertEqual(len(marks['stop_bars']),2)
        # Main-road stop bars must sit near entry (38-3.4=34.6) and exit (74+3.4=77.4), never inside [44, 68]
        for sb in marks['stop_bars']:
            cx=sum(p[0] for p in sb)/4;cy=sum(p[1] for p in sb)/4
            s=float(coordinates(road,[cx,cy])[0])
            self.assertFalse(42.<s<70.,f'stop bar placed inside turn curve at s={s:.1f}')
        # Verify entry/exit crosswalks are outside the curve midpoint (branch crosswalks sit beyond apex)
        for cw in marks['crosswalks']:
            cx=sum(p[0] for p in cw)/4;cy=sum(p[1] for p in cw)/4
            s,d,_=coordinates(road,[cx,cy])
            if abs(d)<8.:
                self.assertFalse(42.<float(s)<70.,f'crosswalk placed inside turn curve at s={s:.1f}')
