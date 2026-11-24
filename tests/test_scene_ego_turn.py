import math
import unittest
from dataclasses import replace
import numpy as np
from core.driving_scene import Actor,Scene,road_display_scene
from core.scene_world_tracks import WorldTracks,rotation
from core.scene_vehicle_road import add_vehicle_corridors
from core.scene_ego_turn import apply_ego_turns


class EgoTurnTests(unittest.TestCase):
    def sequence(self,headings,speed=36):
        tracker=WorldTracks();scenes=[];poses=[];audits=[]
        for i,heading in enumerate(headings):
            telemetry=dict(speed_kmh=speed,heading=float(heading),steering_deg=-135)
            tracker.update([],i*.1,telemetry)
            scenes.append(Scene(i*.1,telemetry))
            poses.append((tracker.pose.copy(),rotation(tracker.angle)))
            audits.append({})
        add_vehicle_corridors(scenes,poses,audits)
        return scenes,poses,audits

    def test_heading_turn_survives_missing_paint_without_lateral_drift(self):
        headings=np.r_[np.zeros(15),np.linspace(0,-100,51),np.full(20,-100.)]
        scenes,poses,audits=self.sequence(headings)
        apply_ego_turns(scenes,poses,audits)
        self.assertAlmostEqual(audits[-1]['ego_heading']['world_heading_change_deg'],-100.)
        self.assertGreater(max(s.ego_display_override[2] for s in scenes),math.radians(10))
        self.assertTrue(any(s.ego_turn_path_estimated for s in scenes))
        self.assertTrue(all(abs(s.ego_display_override[0])<1e-9 for s in scenes))
        self.assertTrue(all(s.ego_position_state=='held_uncertain' for s in scenes))
        self.assertGreater(np.ptp(np.array(scenes[30].lanes[0]['points'])[:,0]),5.)

    def test_heading_wrap_does_not_create_a_full_rotation(self):
        scenes,poses,audits=self.sequence(np.r_[np.full(10,359),np.linspace(359,363,31)%360])
        apply_ego_turns(scenes,poses,audits)
        self.assertAlmostEqual(audits[-1]['ego_heading']['world_heading_change_deg'],4.)
        self.assertLess(max(abs(s.ego_display_override[2]) for s in scenes),math.radians(4))
        self.assertFalse(any(s.ego_turn_path_estimated for s in scenes))

    def test_stopped_heading_noise_and_steering_do_not_turn_vehicle(self):
        scenes,poses,audits=self.sequence(np.linspace(0,100,40),speed=0)
        apply_ego_turns(scenes,poses,audits)
        self.assertTrue(all(abs(s.ego_display_override[2])<1e-9 for s in scenes))

    def test_missing_heading_is_not_replaced_with_steering_wheel_angle(self):
        scenes,poses,audits=self.sequence(np.linspace(0,90,40))
        for scene in scenes:scene.telemetry.pop('heading')
        apply_ego_turns(scenes,poses,audits)
        self.assertTrue(all(a['ego_heading']['source']=='unavailable' for a in audits))
        self.assertTrue(all(abs(s.ego_display_override[2])<1e-9 for s in scenes))

    def test_common_transform_rotates_actors_and_preserves_scope_reference(self):
        scenes,poses,audits=self.sequence(np.linspace(0,90,50))
        for scene in scenes:scene.actors=[Actor('lead','car',3.5,20,yaw=.2)]
        apply_ego_turns(scenes,poses,audits)
        scene=scenes[30];display=road_display_scene(scene)
        Q=np.asarray(scene.road_view['rotation'])
        np.testing.assert_allclose([display.actors[0].x,display.actors[0].z],Q@[3.5,20])
        self.assertAlmostEqual(display.actors[0].yaw,.2+scene.ego_display_override[2])
        np.testing.assert_allclose(scene.road_scope_view['rotation'],np.eye(2))

    def test_bad_paint_angle_cannot_reverse_a_right_turn(self):
        scenes,poses,audits=self.sequence(np.linspace(0,40,50))
        for scene in scenes:
            scene.road_view['rotation']=rotation(-.25).tolist()
            scene.ego_display_override=(0.,0.,.25)
        apply_ego_turns(scenes,poses,audits)
        self.assertLess(scenes[30].ego_display_override[2],0.)

    def test_lane_change_phase_is_preserved_and_not_used_as_turn_geometry(self):
        scenes,poses,audits=self.sequence(np.zeros(61))
        # Explicit road observations determine a completed lane change.
        for i,scene in enumerate(scenes):
            x=3.5*np.clip((i*.1-2)/2,0,1)
            origin,R=poses[i];poses[i]=(origin+[x,0],R)
            scene.lanes=[dict(points=[[edge-x,5],[edge-x,20]]) for edge in (-1.75,1.75,5.25)]
        add_vehicle_corridors(scenes,poses,audits)
        before=[s.ego_display_override[0] for s in scenes]
        apply_ego_turns(scenes,poses,audits)
        np.testing.assert_allclose([s.ego_display_override[0] for s in scenes],before)
        self.assertAlmostEqual(before[-1],3.5)
        self.assertFalse(any(s.ego_turn_path_estimated for s in scenes))

    def test_render_sampling_interpolates_heading_and_curved_geometry(self):
        from core.scene_fusion import FusedSceneTimeline
        scenes,poses,audits=self.sequence(np.linspace(0,90,50))
        apply_ego_turns(scenes,poses,audits)
        for scene in scenes:
            scene.coverage={};scene.fusion_count=0
        timeline=FusedSceneTimeline.__new__(FusedSceneTimeline)
        timeline.scenes=scenes;timeline.max_gap=.2
        sample=timeline.sample(2.05)
        self.assertAlmostEqual(sample.ego_display_override[2],
            (scenes[20].ego_display_override[2]+scenes[21].ego_display_override[2])/2)
        np.testing.assert_allclose(sample.lanes[0]['points'],
            (np.array(scenes[20].lanes[0]['points'])+np.array(scenes[21].lanes[0]['points']))/2)
