import copy,json,unittest
import numpy as np
from ct_scene.clear_handoff import fit,repair


def usable(row,w,h):
    b=row['bbox_xyxy'];return min(b[0],b[1],w-b[2],h-b[3])>5


class ClearHandoffTests(unittest.TestCase):
    def fixture(self):
        def actor(camera,times,split,past):
            rows=[]
            for i,t in enumerate(times):
                clipped=t>split if past else t<split
                rows.append(dict(time_s=float(t),frame_id=camera+str(i),position_W=[10.,5. if past and clipped else -5. if clipped else 0.,0.],
                                 bbox_xyxy=[1260,300,1280,600] if past and clipped else [0,300,30,600] if clipped else [100,200,400,500],support='detector_observation',yaw_rad=0.))
            return dict(camera_id=camera,start_s=rows[0]['time_s'],end_s=rows[-1]['time_s'],samples=rows,lane_index=-2,frozen_lead=False)
        return actor('front',np.arange(0,1.41,.2),1.,True),actor('right_repeater',np.arange(2.,3.81,.2),2.6,False)

    def test_clear_views_agree_despite_bad_clipped_endpoints(self):
        a,b=self.fixture();before=copy.deepcopy((a,b));r=fit(a,b,usable)
        self.assertLess(r['distance_m'],.01);self.assertTrue(r['stationary']);json.dumps(r)
        self.assertEqual((a,b),before)
        repair(a,b,r)
        self.assertTrue(all(np.linalg.norm(np.array(s['position_W'])-[10,0,0])<1e-8 for x in [a,b] for s in x['samples']))
        for x,old in zip([a,b],before):
            for row,previous in zip(x['samples'],old['samples']):
                if usable(previous,1280,960):self.assertEqual(row,previous)

    def test_incompatible_motion_or_rear_camera_abstains(self):
        a,b=self.fixture()
        for row in b['samples']:row['position_W'][0]+=10*row['time_s']
        self.assertIsNone(fit(a,b,usable))
        a,b=self.fixture();b['camera_id']='back';self.assertIsNone(fit(a,b,usable))

    def test_unrelated_distant_clear_car_does_not_fit_and_frozen_anchor_is_preserved(self):
        a,b=self.fixture()
        for row in b['samples']:row['position_W'][0]+=30
        self.assertGreater(fit(a,b,usable)['distance_m'],6)
        a,b=self.fixture();r=fit(a,b,usable);a['frozen_lead']=True;before=copy.deepcopy(a);repair(a,b,r);self.assertEqual(a,before)

    def test_camera_yaw_does_not_require_world_x_forward(self):
        a,b=self.fixture()
        for actor in [a,b]:
            for row in actor['samples']:row['position_W']=[0.,5*row['time_s'],0.]
        r=fit(a,b,usable);self.assertLess(r['distance_m'],1e-8);self.assertFalse(r['stationary'])
