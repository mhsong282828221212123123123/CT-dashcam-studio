import unittest
from dataclasses import replace
import numpy as np
from core.driving_scene import Actor, Scene
from core.scene_handoff import stitch_handoffs


class HandoffTests(unittest.TestCase):
    def fixture(self):
        scenes=[]; rows=[]
        for i in range(20):
            actors=[]; cameras={'front':{'objects':[]},'left_repeater':{'objects':[]}}
            if i<5:
                actors.append(Actor('old','truck',3.,i*.3,sources=('left_repeater:1',)))
                cameras['left_repeater']['objects']=[{'track_id':'left_repeater:1','bbox':[1200,100,80,200]}]
            else:
                actors.append(Actor('old','truck',3.5,i*.3,tracking_state='predicted'))
            if i>=10:
                actors.append(Actor('new','truck',3.,i*.3,sources=('front:2',)))
                cameras['front']['objects']=[{'track_id':'front:2','bbox':[0,100,80,200]}]
            scenes.append(Scene(i*.1,{},actors=actors));rows.append({'cameras':cameras})
        return scenes,rows,[(np.zeros(2),np.eye(2)) for _ in scenes],[{} for _ in scenes],{'front':1280,'left_repeater':1280}

    def test_unique_blind_handoff_has_one_identity_and_inferred_gap(self):
        args=self.fixture()
        self.assertEqual(stitch_handoffs(*args),1)
        self.assertEqual([a.key for a in args[0][12].actors],['old'])
        self.assertEqual(args[0][7].actors[0].tracking_state,'interpolated')
        self.assertEqual(args[0][7].actors[0].sources,())
        self.assertAlmostEqual(args[0][7].actors[0].x,3.)
        self.assertEqual(args[3][12]['visible_ids'],['old'])
        self.assertEqual(args[3][7]['world_tracks'][0]['state'],'interpolated')

    def test_two_possible_entrants_are_not_merged(self):
        args=self.fixture()
        for s in args[0][10:]:
            s.actors.append(replace(s.actors[-1],key='other',x=4.))
        self.assertEqual(stitch_handoffs(*args),0)

    def test_odometry_reset_forbids_bridge(self):
        args=self.fixture();args[3][7]['world_track_events']=[{'reason':'tracks reset'}]
        self.assertEqual(stitch_handoffs(*args),0)

    def test_front_entry_away_from_edge_is_not_a_handoff(self):
        args=self.fixture()
        args[1][10]['cameras']['front']['objects'][0]['bbox'][0]=100
        self.assertEqual(stitch_handoffs(*args),0)

    def long_reverse_fixture(self):
        scenes=[]; rows=[]
        for i in range(50):
            actors=[]; cameras={'front':{'objects':[]},'right_repeater':{'objects':[]}}
            if i<6:
                actors=[Actor('old','car',-3.,i*.3,sources=('front:1',))]
                cameras['front']['objects']=[{'track_id':'front:1','bbox':[1200,100,80,200]}]
            if i>=35:
                actors=[Actor('new','car',-3.,i*.3,sources=('right_repeater:2',))]
                cameras['right_repeater']['objects']=[{'track_id':'right_repeater:2','bbox':[0,100,80,200]}]
            scenes.append(Scene(i*.1,{},actors=actors));rows.append({'cameras':cameras})
        return scenes,rows,[(np.zeros(2),np.eye(2)) for _ in scenes],[{} for _ in scenes],{'front':1280,'right_repeater':1280}

    def test_long_reverse_handoff_uses_future_observations_after_ghost_expiry(self):
        args=self.long_reverse_fixture()
        self.assertEqual(stitch_handoffs(*args),1)
        self.assertEqual(args[0][20].actors[0].key,'old')
        self.assertEqual(args[0][20].actors[0].tracking_state,'interpolated')
        self.assertEqual(args[0][40].actors[0].key,'old')

    def test_incompatible_future_motion_is_not_joined(self):
        args=self.long_reverse_fixture()
        for s in args[0][35:]:
            s.actors[0].z=10.5-(s.time-3.5)*10
        self.assertEqual(stitch_handoffs(*args),0)

    def test_asynchronous_boundary_evidence_is_not_joined(self):
        args=self.long_reverse_fixture()
        args[1][35]['cameras']['right_repeater']['source_time_s']=3.
        self.assertEqual(stitch_handoffs(*args),0)
