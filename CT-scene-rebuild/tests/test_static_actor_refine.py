import copy,itertools,unittest
import numpy as np
from ct_scene.static_actor_refine import fit_pose,projected_box,refine_actor


class StaticMeshReprojectionTests(unittest.TestCase):
    def fixture(self,occluded=False,moving=False):
        vertices=np.array(list(itertools.product([-2.35,2.35],[-.9,.9],[0,1.45])));K=np.array([[600,0,640],[0,600,480],[0,0,1.]])
        rows=[];P=np.array([12.,2.,0.])
        for i in range(24):
            c='front' if i<10 else 'right_repeater';cp=np.array([i*.12,0,1.3]) if i<10 else np.array([8+(i-10)*.15,-7,.85]);yaw=0 if i<10 else np.pi/2
            actual=P+np.array([max(0,i-16)*.8,0,0]) if moving else P
            box=projected_box(vertices,actual,0,cp,yaw,K,np.zeros(5))
            if occluded and i>=18:box[3]-=(box[3]-box[1])*.4
            rows.append(dict(time_s=float(i),frame_id=str(i),camera_id=c,camera_position=cp,camera_yaw=yaw,K=K,D=np.zeros(5),box=box))
        return vertices,rows,P

    def test_later_two_camera_reprojection_recovers_fixed_body_pose(self):
        v,rows,p=self.fixture();before=copy.deepcopy(rows);r=fit_pose(v,rows,p+[.7,.6,0],.08)
        self.assertTrue(r['accepted']);self.assertLess(np.linalg.norm(np.array(r['position_W'])-p),.15);self.assertLess(abs(r['yaw_rad']),.1)
        self.assertTrue(set(r['train_frame_ids']).isdisjoint(r['held_out_frame_ids']))
        for a,b in zip(rows,before):np.testing.assert_array_equal(a['box'],b['box']);np.testing.assert_array_equal(a['camera_position'],b['camera_position'])

    def test_foreground_hiding_wheels_is_not_a_farther_ground_contact(self):
        v,rows,p=self.fixture(occluded=True);r=fit_pose(v,rows,p,0)
        self.assertTrue(r['accepted']);self.assertGreater(r['metrics']['uncensored_full_box_held_out_p90'],.25)
        self.assertLess(r['metrics']['held_out_p90'],.01)

    def test_car_moving_in_withheld_source_frames_and_one_camera_reject(self):
        v,rows,p=self.fixture(moving=True);r=fit_pose(v,rows,p,0);self.assertFalse(r['accepted'])
        v,rows,p=self.fixture()
        for row in rows:row['camera_id']='front'
        self.assertFalse(fit_pose(v,rows,p,0)['accepted'])

    def test_frozen_actor_never_modified(self):
        actor=dict(frozen_lead=True,samples=[dict(position_W=[1,2,0])]);before=copy.deepcopy(actor)
        r=refine_actor(actor,{},None,None,None,None,None,None,None)
        self.assertFalse(r['accepted']);self.assertEqual(actor,before)
