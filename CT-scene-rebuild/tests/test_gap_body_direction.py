import copy
import unittest
import numpy as np
from ct_scene.surrounding import fill_handoff_gaps


class GapBodyDirectionTests(unittest.TestCase):
    def actor(self,end_angle):
        return dict(global_id='car',samples=[
            dict(time_s=0.,position_W=[0.,0.,0.],yaw_rad=0.,frame_id='F',camera_id='front',lane_index=0),
            dict(time_s=1.,position_W=[1.,3.,0.],yaw_rad=end_angle,frame_id='S',camera_id='left_repeater',lane_index=0)])

    def test_lateral_contact_difference_does_not_turn_parallel_body(self):
        actor=self.actor(0.);before=copy.deepcopy(actor['samples']);fill_handoff_gaps(actor,4)
        self.assertEqual([r for r in actor['samples'] if r['frame_id']],before)
        self.assertTrue(all(abs(r['yaw_rad'])<1e-10 for r in actor['samples']))

    def test_real_endpoint_turn_is_preserved(self):
        actor=self.actor(np.pi/2);fill_handoff_gaps(actor,4)
        yaw=[r['yaw_rad'] for r in actor['samples']]
        self.assertTrue(np.all(np.diff(yaw)>0));self.assertAlmostEqual(yaw[-1],np.pi/2)
