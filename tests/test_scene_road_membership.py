import unittest
import numpy as np
from core.driving_scene import Actor,Scene
from core.scene_road_membership import exclude_unrelated_vehicles


class RoadMembershipTests(unittest.TestCase):
    def test_map_junction_context_prevents_guessed_corridor_exclusion(self):
        scenes,audits=self.fixture()
        for scene in scenes:
            scene.road_context={'nearby_junction_candidate':True,'geometry_authority':False}
        exclude_unrelated_vehicles(scenes,audits)
        self.assertTrue(all(len(s.actors)==2 for s in scenes))

    def fixture(self):
        scenes=[Scene(i*.1,None,actors=[Actor('lead','car',3.5,20),
                                      Actor('opposite','car',15,30)]) for i in range(10)]
        audits=[]
        for scene in scenes:
            scene.road_view=dict(rotation=np.eye(2).tolist(),offset=[0.,0.])
            audits.append(dict(travel_corridors=dict(occupied_lane_indices=[0,1],
                lane_width_assumed_m=3.5,segment_start_s=0.,phase_source='camera_lane_phase')))
        return scenes,audits

    def test_opposite_road_observations_and_their_ghosts_are_excluded(self):
        scenes,audits=self.fixture();scenes[-1].actors[1].tracking_state='predicted'
        exclude_unrelated_vehicles(scenes,audits)
        for scene,audit in zip(scenes,audits):
            self.assertEqual([a.key for a in scene.actors],['lead'])
            self.assertEqual(audit['road_scope_excluded'][0]['id'],'opposite')

    def test_established_lead_survives_bad_curve_projection(self):
        scenes,audits=self.fixture()
        for s in scenes[5:]:s.actors[0].x=9.
        exclude_unrelated_vehicles(scenes,audits)
        self.assertTrue(all(any(a.key=='lead' for a in s.actors) for s in scenes))

    def test_underestimated_road_width_does_not_erase_real_adjacent_lanes(self):
        scenes,audits=self.fixture()
        for scene,audit in zip(scenes,audits):
            audit['travel_corridors']['occupied_lane_indices']=[0]
            scene.actors=[Actor('near','car',3.76,3.45),Actor('far_lane','car',7.,20)]
        exclude_unrelated_vehicles(scenes,audits)
        self.assertTrue(all(len(scene.actors)==2 for scene in scenes))

    def test_unknown_road_cannot_exclude_other_vehicles(self):
        scenes,audits=self.fixture()
        for audit in audits:audit['travel_corridors']['phase_source']='unobserved_road_reference'
        exclude_unrelated_vehicles(scenes,audits)
        self.assertTrue(all(len(s.actors)==2 for s in scenes))

    def test_junction_crossing_vehicle_is_preserved(self):
        scenes,audits=self.fixture()
        for audit in audits:audit['camera_junction']={'active':True}
        exclude_unrelated_vehicles(scenes,audits)
        self.assertTrue(all(len(s.actors)==2 for s in scenes))

    def test_longitudinal_range_does_not_discard_same_road_rear_vehicle(self):
        scenes,audits=self.fixture()
        for s in scenes:s.actors=[Actor('rear','car',3.5,-50)]
        exclude_unrelated_vehicles(scenes,audits)
        self.assertTrue(all(len(s.actors)==1 for s in scenes))
