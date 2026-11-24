import copy,unittest
import numpy as np
from core.scene_map import apply_map_roads,confirm_map_junctions
from core.driving_scene import Scene,Actor,road_display_scene
from test_scene_map import way,telemetry


class MapContextTests(unittest.TestCase):
    def test_bridge_or_unrelated_intersection_is_not_host_junction(self):
        for tags in ({'bridge':'yes','layer':'1'},{}):
            s=self.scene()
            # The second junction is nearby, but on a different road or level.
            payload={'elements':[way([(0,-30),(0,30)],nodes=[1,2]),
                way([(12,-30),(12,0),(12,30)],nodes=[3,4,5],ident=2,**tags),
                way([(-20,0),(12,0),(30,0)],nodes=[6,4,7],ident=3,**tags)]}
            apply_map_roads([s],{'payload':payload},[{}])
            self.assertFalse(s.road_context['nearby_junction_candidate'])

    def test_map_marker_does_not_claim_camera_confirmation(self):
        s=self.scene();audit={}
        payload={'elements':[way([(0,-30),(0,0),(0,30)],nodes=[1,2,3]),
            way([(-30,0),(0,0),(30,0)],nodes=[4,2,5],ident=2)]}
        apply_map_roads([s],{'payload':payload},[audit]);confirm_map_junctions([s],[audit])
        self.assertEqual(s.road_context['junction_indication']['state'],'candidate')
        self.assertFalse(s.road_context['junction_indication']['camera_confirmed'])
        audit['road_junction']=dict(osm_supported=True,state='inside')
        confirm_map_junctions([s],[audit])
        self.assertTrue(s.road_context['junction_indication']['camera_confirmed'])

    def scene(self,x=0):
        s=Scene(0,telemetry(x,-10),actors=[Actor('lead','car',0,20,yaw=.2)],
                lanes=[dict(points=[[-1.75,0],[-1.75,20]],dashed=False)])
        s.road_view=dict(rotation=np.eye(2).tolist(),offset=[1.,0.])
        s.ego_display_override=(1.,0.,.1)
        s.road_surface=[[-2,-10],[2,-10],[2,20],[-2,20]]
        s.road_surfaces=[[[-8,10],[8,10],[8,15],[-8,15]]]
        return s

    def test_intersection_hint_preserves_complete_camera_scene(self):
        s=self.scene();before=copy.deepcopy(s)
        payload={'elements':[way([(0,-30),(0,0),(0,30)],nodes=[1,2,3]),
                             way([(-30,0),(0,0),(30,0)],nodes=[4,2,5],ident=2)]}
        audit={};apply_map_roads([s],{'payload':payload},[audit])
        self.assertTrue(s.road_context['nearby_junction_candidate'])
        self.assertTrue(s.road_context['connected_branch_candidate'])
        for field in ['actors','lanes','road_surface','road_surfaces','road_view','ego_display_override']:
            self.assertEqual(getattr(s,field),getattr(before,field))
        self.assertEqual(road_display_scene(s).ego_display,road_display_scene(before).ego_display)
        self.assertFalse(audit['road_map']['scene_geometry_modified'])

    def test_curve_hint_does_not_bend_camera_road(self):
        s=self.scene();before=copy.deepcopy(s.road_surface)
        apply_map_roads([s],{'payload':{'elements':[way([(0,-30),(0,10),(10,30),(30,40)])]}},[{}])
        self.assertTrue(s.road_context['curved_way_candidate'])
        self.assertEqual(s.road_surface,before)

    def test_gps_error_and_osm_width_cannot_change_lane_or_host(self):
        for x in (0,12,1000):
            s=self.scene(x);before=copy.deepcopy(s)
            apply_map_roads([s],{'payload':{'elements':[way([(0,-30),(0,30)],lanes='9',width='40')]}},[{}])
            for field in ['actors','lanes','road_surface','road_surfaces','road_view','ego_display_override']:
                self.assertEqual(getattr(s,field),getattr(before,field))
