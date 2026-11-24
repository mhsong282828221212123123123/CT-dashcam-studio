import copy,unittest
import numpy as np
from ct_scene.actor_rear_axis import orient_unpublished

class InitialBodyPhaseTests(unittest.TestCase):
    def test_source_rear_resolves_backward_velocity_phase_before_freeze(self):
        t=np.arange(0,2,.2);rows=[dict(time_s=float(s),position_W=[12+s,0,0],yaw_rad=np.pi) for s in t]
        cues=[dict(time_s=float(s),frame_id=str(i)) for i,s in enumerate(t)];before=copy.deepcopy(rows)
        out,r=orient_unpublished(rows,cues,t,np.zeros((len(t),3)))
        self.assertTrue(r['accepted']);self.assertEqual(rows,before)
        self.assertTrue(all(np.cos(s['yaw_rad'])>.99 for s in out))
        self.assertEqual([s['position_W'] for s in out],[s['position_W'] for s in rows])

    def test_sparse_or_correct_phase_is_not_overridden(self):
        t=np.arange(0,2,.2);cues=[dict(time_s=float(s),frame_id=str(i)) for i,s in enumerate(t)]
        for yaw,selected in ((np.pi,cues[:2]),(0.,cues)):
            rows=[dict(time_s=float(s),position_W=[12+s,0,0],yaw_rad=yaw) for s in t]
            out,r=orient_unpublished(rows,selected,t,np.zeros((len(t),3)))
            self.assertFalse(r['accepted']);self.assertEqual(out,rows)
