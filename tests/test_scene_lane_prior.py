import copy
import math
import unittest
import numpy as np

from core.driving_scene import Actor,Scene
from core.scene_lane_prior import apply_lane_containment_prior
from core.scene_world_tracks import rotation


class LanePriorTests(unittest.TestCase):
    def fixture(self):
        scenes=[];audits=[]
        for i in range(41):
            s=Scene(i*.1,None,actors=[Actor('near_edge','car',1.05,12),Actor('off_centre','car',-3.1,15)])
            s.road_view=dict(rotation=np.eye(2).tolist(),offset=[0.,0.])
            s.ego_display_override=(.25,0.,0.)
            scenes.append(s)
            audits.append(dict(travel_corridors=dict(occupied_lane_indices=[-1,0],lane_width_assumed_m=3.5,
                                                     phase_source='camera_lane_phase',segment_start_s=0.)))
        return scenes,audits

    def test_soft_footprint_containment_does_not_snap_to_centre_or_move_host(self):
        scenes,audits=self.fixture();ego=copy.deepcopy([s.ego_display_override for s in scenes])
        changed=apply_lane_containment_prior(scenes,audits)
        self.assertEqual(changed,len(scenes))
        self.assertTrue(all(.7<s.actors[0].x<1.05 for s in scenes))
        self.assertTrue(all(s.actors[1].x==-3.1 for s in scenes))
        self.assertEqual(ego,[s.ego_display_override for s in scenes])
        self.assertEqual(audits[0]['lane_containment_prior'][0]['raw_position'],[1.05,12.])

    def test_real_lane_change_is_released_instead_of_pulled_back(self):
        scenes,audits=self.fixture()
        for i,s in enumerate(scenes):s.actors=[Actor('changing','car',-3.5*i/40,12)]
        before=[s.actors[0].x for s in scenes]
        apply_lane_containment_prior(scenes,audits)
        np.testing.assert_allclose([s.actors[0].x for s in scenes],before)

    def test_turns_junctions_and_unknown_road_are_not_forced_into_straight_lanes(self):
        for case in ('turn','junction','unknown','ghost'):
            scenes,audits=self.fixture()
            for s,a in zip(scenes,audits):
                if case=='turn':s.actors[0].yaw=math.pi/3
                if case=='junction':a['road_junction']={'state':'inside'}
                if case=='unknown':a['travel_corridors']['phase_source']='unobserved_road_reference'
                if case=='ghost':s.actors[0].position_hypothesis=True
            apply_lane_containment_prior(scenes,audits)
            self.assertTrue(all(s.actors[0].x==1.05 for s in scenes),case)

    def test_curved_local_road_coordinates_are_used_for_containment(self):
        scenes,audits=self.fixture();Q=rotation(.35)
        for s in scenes:
            s.road_view['rotation']=Q.tolist();p=Q.T@np.array([1.05,12.])
            s.actors=[Actor('curve','car',*p,yaw=.35)]
        apply_lane_containment_prior(scenes,audits)
        self.assertTrue(all(.7<(Q@np.array([s.actors[0].x,s.actors[0].z]))[0]<1.05 for s in scenes))
        self.assertTrue(all(abs((Q@np.array([s.actors[0].x,s.actors[0].z]))[1]-12)<1e-8 for s in scenes))

    def test_image_disagreement_prevents_lane_correction(self):
        from unittest.mock import patch
        scenes,audits=self.fixture()
        with patch('core.scene_lane_prior._image_consistent',return_value=False):
            apply_lane_containment_prior(scenes,audits,[{}]*len(scenes),[{}]*len(scenes))
        self.assertTrue(all(s.actors[0].x==1.05 for s in scenes))

    def test_missing_road_releases_without_introducing_a_lateral_jump(self):
        scenes,audits=self.fixture()
        for a in audits[20:]:a['travel_corridors']['phase_source']='held_without_two_sided_road_support'
        apply_lane_containment_prior(scenes,audits)
        correction=np.array([s.actors[0].x-1.05 for s in scenes])
        self.assertTrue(np.any(correction[:20]<-.02))
        np.testing.assert_array_equal(correction[20:],0.)
        self.assertLessEqual(np.max(abs(np.diff(correction)))/.1,.350001)

    def test_display_camera_rotation_does_not_change_the_road_constraint(self):
        a,aa=self.fixture();b,bb=self.fixture()
        for scene in b:
            scene.road_scope_view=copy.deepcopy(scene.road_view)
            scene.road_view['rotation']=rotation(.2).tolist()
        apply_lane_containment_prior(a,aa);apply_lane_containment_prior(b,bb)
        np.testing.assert_allclose([[v.x,v.z] for s in a for v in s.actors],
                                   [[v.x,v.z] for s in b for v in s.actors])
