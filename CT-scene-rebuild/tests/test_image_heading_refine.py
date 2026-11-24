import copy,unittest
import numpy as np
from ct_scene.image_heading_refine import refine_actor


class ImageHeadingRefineTests(unittest.TestCase):
    def fixture(self):
        t=np.arange(0,2.01,.2);a=dict(local_ids=['front'],model='sedan',lane_index=-2,frozen_lead=False,samples=[dict(time_s=float(s),position_W=[10,3,0],yaw_rad=np.pi/2) for s in t])
        p=[dict(local_track_id='front',frame_id=str(i),camera_time_s=float(s),accepted=True,camera_heading_rad=0.,mirror_error_deg=2.) for i,s in enumerate(t)]
        return a,p,t,np.zeros(len(t))

    def test_image_heading_changes_body_without_moving_car(self):
        a,p,t,y=self.fixture();positions=[r['position_W'][:] for r in a['samples']];r=refine_actor(a,p,t,y)
        self.assertEqual(r['status'],'image_heading_refined');self.assertEqual(positions,[r['position_W'] for r in a['samples']])
        self.assertTrue(all(abs(np.sin(r['yaw_rad']))<1e-6 for r in a['samples']))

    def test_observer_rotation_does_not_rotate_a_world_stationary_car(self):
        a,p,t,y=self.fixture();y=t*.2
        for r in p:r['camera_heading_rad']=-r['camera_time_s']*.2
        refine_actor(a,p,t,y);self.assertTrue(all(abs(r['yaw_rad'])<1e-6 for r in a['samples']))

    def test_frozen_lane_constrained_and_heavy_assets_are_preserved(self):
        for key,value in [('frozen_lead',True),('lane_index',0),('model','heavy_truck')]:
            a,p,t,y=self.fixture();a[key]=value;before=copy.deepcopy(a);refine_actor(a,p,t,y);self.assertEqual(a,before)

    def test_sparse_or_rejected_source_and_long_internal_gaps_abstain(self):
        a,p,t,y=self.fixture();before=copy.deepcopy(a);refine_actor(a,p[:2],t,y);self.assertEqual(a,before)
        for q in p:q['accepted']=False
        refine_actor(a,p,t,y);self.assertEqual(a,before)
        a,p,t,y=self.fixture();p+= [dict(q,camera_time_s=q['camera_time_s']+8,frame_id='later-'+q['frame_id']) for q in p]
        before=copy.deepcopy(a);r=refine_actor(a,p,t,y)
        self.assertEqual(r['status'],'separated_image_windows_abstained');self.assertEqual(a,before)

    def test_real_turn_in_image_anchors_is_retained(self):
        a,p,t,y=self.fixture()
        for q in p:q['camera_heading_rad']=-q['camera_time_s']*np.pi/4
        refine_actor(a,p,t,y)
        self.assertLess(a['samples'][-1]['yaw_rad'],-np.pi/3)
        self.assertAlmostEqual(a['samples'][5]['yaw_rad'],-np.pi/4,places=5)

    def test_observed_stationary_body_can_keep_heading_beyond_blind_gap_budget(self):
        a,p,t,y=self.fixture();a['samples']=[dict(time_s=float(s),position_W=[10,3,0],yaw_rad=np.pi/2,support='detector_observation',bbox_xyxy=[100,200,400,500],frame_id=str(i)) for i,s in enumerate(np.arange(0,10.01,.2))]
        refine_actor(a,p,t,y);self.assertAlmostEqual(a['samples'][-1]['yaw_rad'],0.)
        self.assertIn('stationarity',a['samples'][-1]['heading_support'])
        a,p,t,y=self.fixture();a['samples']=[dict(time_s=float(s),position_W=[10+3*s,3,0],yaw_rad=np.pi/2,support='detector_observation',bbox_xyxy=[100,200,400,500],frame_id=str(i)) for i,s in enumerate(np.arange(0,10.01,.2))]
        refine_actor(a,p,t,y);self.assertAlmostEqual(a['samples'][-1]['yaw_rad'],np.pi/2)
