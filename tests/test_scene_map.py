import math,unittest
import numpy as np
from core.scene_map import RoadMap,apply_map_roads,basis
from core.driving_scene import Scene,Actor,road_display_scene

def way(points,nodes=None,ident=1,**tags):
    return {'type':'way','id':ident,'tags':{'highway':'primary','lanes':'2','oneway':'yes',**tags},
        'nodes':nodes or list(range(len(points))),
        'geometry':[{'lat':37+z/111320,'lon':127+x/(111320*math.cos(math.radians(37)))} for x,z in points]}

def telemetry(x,z,heading=0):
    return {'lat':37+z/111320,'lon':127+x/(111320*math.cos(math.radians(37))),
            'heading':heading,'speed_kmh':30}

class MapTests(unittest.TestCase):
    def test_map_cannot_override_camera_lane_change_frame(self):
        scenes=[]
        for i,x in enumerate((0.,.7,1.4,2.1,2.8,3.5)):
            s=Scene(i*.1,telemetry(-x,0),actors=[Actor('other','car',-2-x,20)])
            s.road_view={'rotation':np.eye(2).tolist(),'offset':[x,0.],'source':'camera_test_reference'}
            s.observed_lane_segments=[];scenes.append(s)
        apply_map_roads(scenes,{'payload':{'elements':[way([(0,-50),(0,100)])]}},[{} for s in scenes])
        rendered=[road_display_scene(s) for s in scenes]
        self.assertAlmostEqual(rendered[-1].ego_display[0],3.5)
        self.assertTrue(all(abs(s.actors[0].x+2)<.001 for s in rendered))

    def test_curved_shape_is_preserved_without_generating_lanes(self):
        s=Scene(0,telemetry(0,0));s.observed_lane_segments=[]
        apply_map_roads([s],{'payload':{'elements':[way([(0,-20),(0,20),(10,40),(30,60)])]}},[{}])
        self.assertTrue(any(max(abs(p[0]) for p in path['points'])>20 for path in s.road_prior['paths']))
        self.assertFalse(s.lanes)

    def test_intersection_topology_has_no_synthetic_lane_lines(self):
        payload={'elements':[way([(0,-30),(0,0),(0,30)],nodes=[1,2,3]),
                             way([(-30,0),(0,0),(30,0)],nodes=[4,2,5],ident=2)]}
        s=Scene(0,telemetry(0,-10));s.observed_lane_segments=[]
        apply_map_roads([s],{'payload':payload},[{}])
        self.assertEqual(len(s.road_prior['junctions']),1)
        self.assertFalse(s.lanes)

    def test_missing_gps_does_not_create_road_or_origin_fix(self):
        s=Scene(0,None);s.observed_lane_segments=[]
        self.assertFalse(apply_map_roads([s],{'payload':{'elements':[]}},[{}]))

    def test_oneway_opposite_carriageway_rejected(self):
        roads=RoadMap({'elements':[way([(0,-30),(0,30)])]},[37,127])
        self.assertFalse(roads.candidates(np.zeros(2),180))

    def test_map_lane_and_width_tags_cannot_change_camera_evidence(self):
        segment={'points':[(1.5,4),(1.6,8)],'observed':True}
        outputs=[]
        for lanes,width in [('2','7'),('9','40'),('unknown','invalid')]:
            s=Scene(0,telemetry(12,0),actors=[Actor('a','car',3,10)],lanes=[segment])
            s.observed_lane_segments=[segment];audit=[{}]
            apply_map_roads([s],{'payload':{'elements':[way([(0,-30),(0,30)],lanes=lanes,width=width)]}},audit)
            self.assertEqual(s.lanes,[segment])
            self.assertEqual((s.actors[0].x,s.actors[0].z),(3,10))
            self.assertEqual(audit[0]['road_map']['generated_lanes'],0)
            self.assertFalse(getattr(s,'road_view',None))
            outputs.append(s.road_prior)
        self.assertEqual(outputs[0],outputs[1]);self.assertEqual(outputs[1],outputs[2])

    def test_large_gps_offset_does_not_disable_camera_lane_display(self):
        segment={'points':[(1.5,4),(1.6,8)],'observed':True}
        s=Scene(0,telemetry(12,0),lanes=[segment]);s.observed_lane_segments=[segment];audit=[{}]
        apply_map_roads([s],{'payload':{'elements':[way([(0,-30),(0,30)])]}},audit)
        self.assertEqual(s.lanes,[segment]);self.assertEqual(audit[0]['road_map']['status'],'shape_prior')
        self.assertGreater(audit[0]['road_map']['gps_distance_m'],10)
