import unittest
import cv2
import numpy as np
from core.driving_scene import Scene
from core.scene_world_tracks import WorldTracks,rotation
from core.scene_junction import controlled_turn_intervals,apply_controlled_turn_surfaces
from core.scene_vehicle_road import add_vehicle_corridors
from core.scene_ego_turn import apply_ego_turns


class ControlledJunctionTests(unittest.TestCase):
    def fixture(self,signals=True):
        headings=np.r_[np.zeros(15),np.linspace(0,-90,31),np.full(20,-90)]
        world=WorldTracks();scenes=[];poses=[];audits=[];rows=[]
        for i,heading in enumerate(headings):
            telemetry=dict(speed_kmh=36,heading=float(heading))
            world.update([],i*.1,telemetry)
            scene=Scene(i*.1,telemetry)
            if i<15 or i>45:scene.lanes=[dict(points=[[x,5],[x,20]]) for x in (-1.75,1.75)]
            scenes.append(scene);poses.append((world.pose.copy(),rotation(world.angle)));audits.append({})
            heads=[dict(head_type='horizontal_candidate',confidence=.9,state='green',support=4)] if signals and i<25 else []
            rows.append(dict(cameras=dict(front=dict(signals=heads))))
        return scenes,poses,rows,audits

    def test_turn_alone_does_not_invent_intersection(self):
        scenes,poses,rows,audits=self.fixture(False)
        self.assertEqual(controlled_turn_intervals(scenes,poses,rows,audits),[])

    def test_osm_replaces_missing_signal_but_requires_turn_and_paint_gap(self):
        scenes,poses,rows,audits=self.fixture(False)
        for s in scenes:
            s.road_context={'junction_candidates':[dict(node_id=12,distance_m=25,point=[0,25])]}
        self.assertEqual(len(controlled_turn_intervals(scenes,poses,rows,audits)),1)
        self.assertTrue(any(a.get('road_junction',{}).get('osm_supported') for a in audits))
        for s in scenes:s.lanes=[dict(points=[[0,0],[0,20]])]
        self.assertEqual(controlled_turn_intervals(scenes,poses,rows,[{} for s in scenes]),[])

    def test_a_single_map_candidate_frame_is_insufficient(self):
        scenes,poses,rows,audits=self.fixture(False)
        scenes[14].road_context={'junction_candidates':[dict(node_id=12,distance_m=25,point=[0,25])]}
        self.assertEqual(controlled_turn_intervals(scenes,poses,rows,audits),[])

    def test_controlled_turn_breaks_markings_and_restarts_exit_road(self):
        scenes,poses,rows,audits=self.fixture()
        events=controlled_turn_intervals(scenes,poses,rows,audits)
        self.assertEqual(len(events),1)
        entry,exit=events[0]
        add_vehicle_corridors(scenes,poses,audits)
        self.assertEqual(audits[exit]['travel_corridors']['segment_start_s'],scenes[exit].time)
        apply_ego_turns(scenes,poses,audits)
        apply_controlled_turn_surfaces(scenes,poses,audits,events)
        middle=(entry+exit)//2;scene=scenes[middle]
        self.assertTrue(scene.junction_surface_estimated)
        for lane in scene.lanes:
            for point in lane['points']:
                self.assertTrue(all(cv2.pointPolygonTest(np.asarray(p,np.float32),tuple(point),False)<0
                                    for p in scene.road_surfaces))
        self.assertTrue(audits[middle]['road_junction']['parallel_markings_interrupted'])
