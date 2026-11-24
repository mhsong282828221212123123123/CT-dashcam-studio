import copy,unittest
import numpy as np
from ct_scene.actor_rear_axis import align_actor


class RearAxisTests(unittest.TestCase):
    def fixture(self):
        t=np.arange(5)*.2;ego=np.zeros((5,3));rows=[dict(time_s=float(s),position_W=[10.,0,0],yaw_rad=float(np.pi),yaw_before_ego_maneuver=float(np.pi)) for s in t]
        actor=dict(samples=rows,frozen_lead=False);cues=[dict(frame_id=str(i),time_s=float(s)) for i,s in enumerate(t)]
        return actor,cues,t,ego

    def test_stopped_rear_cue_corrects_front_back_without_moving_or_rotating_axis(self):
        a,c,t,e=self.fixture();p=[r['position_W'][:] for r in a['samples']];r=align_actor(a,c,t,e)
        self.assertEqual(r['status'],'source_rear_antipodal_flip')
        self.assertEqual(p,[r['position_W'] for r in a['samples']])
        self.assertTrue(all(np.cos(r['yaw_rad'])>.99 and np.cos(r['yaw_before_ego_maneuver'])>.99 for r in a['samples']))

    def test_frozen_correct_or_ambiguous_side_views_are_not_changed(self):
        for angle,frozen in [(0.,False),(np.pi,True),(np.pi/2,False)]:
            a,c,t,e=self.fixture();a['frozen_lead']=frozen
            for r in a['samples']:r['yaw_rad']=angle
            before=copy.deepcopy(a);align_actor(a,c,t,e);self.assertEqual(a,before)

    def test_one_or_duplicated_cue_cannot_flip_body(self):
        a,c,t,e=self.fixture();before=copy.deepcopy(a);align_actor(a,[c[0]]*5,t,e);self.assertEqual(a,before)

    def test_false_motion_axis_rotation_between_repeated_rear_anchors_is_removed_offline(self):
        a,c,t,e=self.fixture()
        for row,angle in zip(a['samples'],[0,np.pi/2,np.pi,np.pi,np.pi]):row['yaw_rad']=float(angle)
        align_actor(a,c,t,e)
        self.assertTrue(all(np.cos(r['yaw_rad'])>.99 for r in a['samples']))
