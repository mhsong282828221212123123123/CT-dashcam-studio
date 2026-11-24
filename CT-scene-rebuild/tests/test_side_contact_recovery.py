import copy
import unittest
from ct_scene.side_contact_recovery import recover


class ContactRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.road=dict(boundaries_y=[9,5.5,2,-1.5],left_boundary_role='inferred_extent_not_confirmed_centreline')
        self.actor=dict(camera_id='left_repeater',samples=[dict(time_s=i*.2,position_W=[i*5,12,0],side_ground_contact_W=[i*5,7,0]) for i in range(30)])

    def test_observed_contacts_replace_height_without_translation(self):
        before=copy.deepcopy(self.actor);r=recover(self.actor,self.road)
        self.assertEqual(r['lane'],0)
        self.assertEqual([s['position_W'] for s in r['samples']],[s['side_ground_contact_W'] for s in before['samples']])
        self.assertEqual(self.actor,before)

    def test_opposing_and_stationary_contacts_rejected(self):
        for speed in (-20,0,70):
            a=copy.deepcopy(self.actor)
            for s in a['samples']:s['side_ground_contact_W'][0]=s['time_s']*speed
            self.assertIsNone(recover(a,self.road))

    def test_outside_contacts_never_snapped_into_road(self):
        for s in self.actor['samples']:s['side_ground_contact_W'][1]=12
        self.assertIsNone(recover(self.actor,self.road))

    def test_confirmed_divider_and_junction_abstain(self):
        for r in [dict(self.road,left_boundary_role='camera_observed_yellow_divider_estimated_geometry'),dict(self.road,junctions=[{}]),dict(self.road,lane_geometry_status='unknown_route_display_only')]:
            self.assertIsNone(recover(self.actor,r))

    def test_missing_short_or_inconsistent_ground_abstains(self):
        a=copy.deepcopy(self.actor)
        for s in a['samples']:s.pop('side_ground_contact_W')
        self.assertIsNone(recover(a,self.road))
        self.assertIsNone(recover(dict(self.actor,samples=self.actor['samples'][:10]),self.road))
        for i,s in enumerate(self.actor['samples']):s['side_ground_contact_W'][1]=7 if i%2 else 3
        self.assertIsNone(recover(self.actor,self.road))
