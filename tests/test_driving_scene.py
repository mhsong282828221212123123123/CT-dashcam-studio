import unittest
import numpy as np
from core.driving_scene import GroundAssumption, ObservedSceneTimeline, DrivingRenderer, demo_scene


def row(t, objects=(), available=True):
    return {'time_s':t,'ego':None,'cameras':{'front':{'available':available,'objects':list(objects)}}}


def obj(key='front:1', box=(560,400,100,250)):
    return {'track_id':key,'track_status':'confirmed','kind':'car','confidence':.8,'bbox':list(box)}


class DrivingSceneTests(unittest.TestCase):
    def test_yaw_interpolation_uses_short_arc_at_every_render_frame(self):
        from core.driving_scene import Scene,Actor
        timeline=ObservedSceneTimeline([row(0),row(.1)],sample_hz=10)
        timeline.scenes=[Scene(0,{},actors=[Actor('a','car',0,10,yaw=np.deg2rad(179))]),
                         Scene(.1,{},actors=[Actor('a','car',0,10,yaw=np.deg2rad(-179))])]
        for t,expected in [(.025,179.5),(.05,180),(.075,180.5)]:
            self.assertAlmostEqual(np.rad2deg(timeline.sample(t).actors[0].yaw),expected)
    def test_ground_projection_rejects_horizon_and_clipped_contact(self):
        projection=GroundAssumption()
        self.assertIsNone(projection.project(640,480,1280,960))
        self.assertIsNone(projection.project(640,955,1280,960))
        self.assertIsNone(projection.project(float('nan'),600,1280,960))
        left=projection.project(400,700,1280,960)
        right=projection.project(880,700,1280,960)
        self.assertGreater(left[0],0)
        self.assertLess(right[0],0)
        self.assertAlmostEqual(left[1],right[1])

    def test_observed_mode_never_generates_lanes_path_or_missing_actors(self):
        timeline=ObservedSceneTimeline([row(0,[obj()]),row(.2,[],False)])
        first=timeline.at(0)
        self.assertEqual(first.mode,'observed')
        self.assertEqual(first.lanes,[])
        self.assertEqual(first.path,[])
        self.assertTrue(first.actors[0].estimated)
        self.assertEqual(timeline.at(1).actors,[])
        self.assertEqual(timeline.sample(.1).actors,[])

    def test_only_matching_tracks_interpolate_and_seeking_is_pure(self):
        timeline=ObservedSceneTimeline([row(0,[obj()]),row(.2,[obj(box=(600,400,100,250))])])
        a,b=timeline.at(0).actors[0],timeline.at(1).actors[0]
        mid=timeline.sample(.1).actors[0]
        self.assertAlmostEqual(mid.x,(a.x+b.x)/2)
        timeline.sample(.19)
        self.assertEqual(timeline.sample(.1),timeline.sample(.1))

    def test_tentative_or_invalid_boxes_are_not_placed(self):
        tentative=obj(); tentative['track_status']='tentative'
        invalid=obj('bad',(1,1,-1,2))
        scene=ObservedSceneTimeline([row(0,[tentative,invalid])]).at(0)
        self.assertEqual(scene.actors,[])
        self.assertEqual(scene.rejected,1)

    def test_long_observation_gap_does_not_hold_an_old_vehicle(self):
        timeline=ObservedSceneTimeline([row(0,[obj()]),row(3,[obj()])])
        middle=timeline.sample(1.)
        self.assertEqual(middle.actors,[])
        self.assertIsNone(middle.telemetry)

    def test_demo_is_explicit_and_time_driven(self):
        a,b=demo_scene(2),demo_scene(8)
        self.assertEqual(a.mode,'demo')
        self.assertIn('SYNTHETIC',a.note)
        self.assertTrue(a.lanes)
        self.assertTrue(b.telemetry['brake_applied'])
        self.assertNotEqual(a.actors[0].x,b.actors[0].x)
        renderer=DrivingRenderer(480,320)
        image=renderer.render(a)
        self.assertGreater(image.mean(),180)
        renderer.render(b)
        np.testing.assert_array_equal(image,renderer.render(a))
