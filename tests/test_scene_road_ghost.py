from dataclasses import replace
import unittest
import numpy as np
from core.driving_scene import Actor,Scene,DrivingRenderer,actor_is_ghost
from core.scene_road import add_camera_corridors as add_travel_corridors
from core.scene_world_tracks import WorldTracks


class RoadGhostTests(unittest.TestCase):
    def test_occlusion_only_spatial_hypothesis_remains_a_ghost(self):
        actor=Actor('partial','car',3,20,image_observed=True,position_hypothesis=True)
        self.assertTrue(actor_is_ghost(actor))

    def test_camera_visible_uncertain_position_uses_opaque_body(self):
        renderer=DrivingRenderer(480,320)
        actor=Actor('lead','car',0,10)
        observed=renderer.render(Scene(0,None,actors=[actor]))
        visible=renderer.render(Scene(0,None,actors=[replace(actor,tracking_state='image_supported',image_observed=True)]))
        blind=renderer.render(Scene(0,None,actors=[replace(actor,tracking_state='predicted',image_observed=False)]))
        x,y=map(round,renderer.project([[0,1,10]])[0][:2]);roi=np.s_[y-20:y+21,x-20:x+21]
        np.testing.assert_array_equal(observed[roi],visible[roi])
        self.assertGreater(np.abs(observed[roi].astype(float)-blind[roi]).sum(),2000)

    def test_outer_boundaries_solid_and_internal_boundaries_dashed(self):
        scene=Scene(0,{'speed_kmh':0},actors=[Actor('one','car',.4,10)],
            lanes=[{'points':[(x,4),(x,15)],'observed':True} for x in (-5.25,-1.75,1.75,5.25)])
        add_travel_corridors([scene],[(np.zeros(2),np.eye(2))],[{}])
        self.assertEqual([l['dashed'] for l in scene.lanes],[False,True,True,False])
        self.assertTrue(scene.road_surface)
        self.assertEqual((scene.actors[0].x,scene.actors[0].z),(.4,10))

    def test_camera_road_moves_relative_to_host_without_recentering(self):
        scenes=[Scene(i*.1,{'speed_kmh':0},lanes=[{'points':[(x-i*.5,4),(x-i*.5,15)]} for x in (-1.75,1.75)]) for i in range(3)]
        add_travel_corridors(scenes,[(np.zeros(2),np.eye(2))]*3,[{} for s in scenes])
        self.assertAlmostEqual(scenes[-1].lanes[0]['points'][0][0],-2.75)

    def test_missing_boundaries_have_bounded_memory(self):
        scenes=[Scene(i*.1,{'speed_kmh':0},lanes=[{'points':[(x,4),(x,15)]} for x in (-1.75,1.75)] if i==0 else []) for i in range(15)]
        add_travel_corridors(scenes,[(np.zeros(2),np.eye(2))]*15,[{} for s in scenes])
        self.assertTrue(scenes[5].road_surface);self.assertFalse(scenes[-1].road_surface)

    def test_vehicle_positions_never_create_lane_geometry(self):
        actor=Actor('one','car',3.7,12)
        segment={'points':[(1.5,4),(1.6,10)],'observed':True}
        scenes=[Scene(i*.1,{'speed_kmh':20},actors=[replace(actor)],lanes=[segment]) for i in range(100)]
        poses=[(np.array([0.,i*.5]),np.eye(2)) for i in range(100)];audit=[{} for s in scenes]
        add_travel_corridors(scenes,poses,audit)
        self.assertEqual((scenes[-1].actors[0].x,scenes[-1].actors[0].z),(3.7,12))
        self.assertEqual(scenes[-1].lanes,[segment])
        self.assertEqual(scenes[-1].road_surface,[])

    def test_host_route_is_not_misrepresented_as_curved_road(self):
        scenes=[Scene(i*.1,{'speed_kmh':20}) for i in range(100)]
        poses=[(np.array([.002*(i*.5)**2,i*.5]),np.eye(2)) for i in range(100)]
        audit=[{} for s in scenes];add_travel_corridors(scenes,poses,audit)
        self.assertTrue(all(not s.lanes for s in scenes))

    def test_ghost_blends_with_background_and_fades_with_age(self):
        renderer=DrivingRenderer(480,320)
        actor=Actor('one','car',4,6)
        base=renderer.render(Scene(0,{'speed_kmh':0}))
        opaque=renderer.render(Scene(0,{'speed_kmh':0},actors=[actor]))
        ghost=replace(actor,tracking_state='predicted',prediction_horizon=2,prediction_age=.1)
        fresh=renderer.render(Scene(0,{'speed_kmh':0},actors=[ghost]))
        faded=renderer.render(Scene(0,{'speed_kmh':0},actors=[replace(ghost,prediction_age=1.8)]))
        p=renderer.project([[4,1,6]])[0];x,y=map(round,p[:2]);roi=np.s_[max(0,y-24):y+25,max(0,x-22):x+23]
        difference=lambda a:float(np.abs(a[roi].astype(float)-base[roi]).sum())
        self.assertGreater(difference(fresh),0)
        self.assertLess(difference(fresh),difference(opaque))
        self.assertLess(difference(faded),difference(fresh)*.65)

    def test_established_side_track_survives_short_blind_spot_then_expires(self):
        tracker=WorldTracks();actor=Actor('one','car',3.5,-3,sources=('left_repeater:1',))
        for i in range(20):tracker.update([actor],i*.1,{'speed_kmh':20,'heading':0})
        for i in range(20,32):out,_=tracker.update([],i*.1,{'speed_kmh':20,'heading':0})
        self.assertEqual(out[0].tracking_state,'predicted');self.assertEqual(out[0].prediction_horizon,2.)
        for i in range(32,43):out,_=tracker.update([],i*.1,{'speed_kmh':20,'heading':0})
        self.assertEqual(out,[])
