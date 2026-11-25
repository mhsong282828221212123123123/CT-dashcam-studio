import copy,unittest
import numpy as np
from ct_scene.far_lead_stability import refine_far_leads,smooth_curve

PROFILE=dict(model='SIMPLE_RADIAL',params=[1000,640,480,0])
MOUNT=dict(yaw_deg=0,offset_W_ego_m=[0,0,0])
DIMS=dict(sedan=[4.6,1.9,1.5],ego=[4.7,1.9,1.5])


def fixture(depth_fn=lambda t:70+t,old_fn=lambda t:50+12*np.sin(2*t)):
    ts=np.arange(0,5.001,.05);states=[];samples=[]
    for t in ts:
        states.append(dict(time_s=float(t),ego_position_W=[10*t,0,0],ego_yaw_rad=0,actors=[
            dict(id='lead',model='sedan',position_W=[10*t+old_fn(t)+2.3,0,0],yaw_rad=0,ghost=False)]))
    for t in np.arange(0,5.001,.2):
        height=1500/depth_fn(t)
        samples.append(dict(time_s=float(t),support='detector_observation',bbox_xyxy=[620,500-height,660,500]))
    actor=dict(id='lead',global_id='lead',camera_id='front',model='sedan',frozen_lead=True,samples=samples)
    return states,[actor]


class FarLeadTests(unittest.TestCase):
    def apply(self,states,actors):return refine_far_leads(states,actors,PROFILE,DIMS,MOUNT)

    def test_far_contact_oscillation_replaced_using_bidirectional_size(self):
        states,actors=fixture();raw=copy.deepcopy(actors);before=copy.deepcopy(states)
        proof=self.apply(states,actors)
        self.assertEqual(proof['applied_actor_ids'],['lead']);self.assertEqual(raw,actors)
        for old,new in zip(before,states):
            self.assertEqual(old['ego_position_W'],new['ego_position_W'])
            self.assertEqual(old['actors'][0]['yaw_rad'],new['actors'][0]['yaw_rad'])
            expected=10*new['time_s']+70+new['time_s']+2.3
            self.assertLess(abs(new['actors'][0]['position_W'][0]-expected),.2)
        v=np.diff([s['actors'][0]['position_W'][0] for s in states])/.05
        self.assertLess(np.max(abs(v-11)),.3)

    def test_near_track_and_genuine_size_supported_approach_unchanged(self):
        for depth,old in [(lambda t:20+t,lambda t:20+t),(lambda t:90-5*t,lambda t:90-5*t)]:
            states,actors=fixture(depth,old);before=copy.deepcopy(states)
            self.assertFalse(self.apply(states,actors)['applied_actor_ids']);self.assertEqual(states,before)

    def test_size_supported_fast_approach_is_not_speed_capped(self):
        states,actors=fixture(lambda t:130-14*t,lambda t:130-14*t)
        before=copy.deepcopy(states)
        self.assertFalse(self.apply(states,actors)['applied_actor_ids']);self.assertEqual(states,before)

    def test_gap_border_and_non_anchor_abstain(self):
        for mode in ['gap','border','ordinary','side']:
            states,actors=fixture();a=actors[0]
            if mode=='gap':a['samples']=a['samples'][:5]
            if mode=='border':
                for r in a['samples']:r['bbox_xyxy'][0]=0
            if mode=='ordinary':a['frozen_lead']=False
            if mode=='side':a['camera_id']='right_repeater'
            before=copy.deepcopy(states)
            self.assertFalse(self.apply(states,actors)['applied_actor_ids']);self.assertEqual(states,before)

    def test_other_actor_and_visibility_are_unchanged(self):
        states,actors=fixture()
        for s in states:s['actors'].append(dict(id='other',model='sedan',position_W=[0,8,0],yaw_rad=.3,ghost=True))
        before=copy.deepcopy(states);self.apply(states,actors)
        for old,new in zip(before,states):
            self.assertEqual(old['actors'][1],new['actors'][1]);self.assertEqual(old['actors'][0]['ghost'],new['actors'][0]['ghost'])

    def test_new_body_overlap_rejects_whole_correction(self):
        states,actors=fixture()
        for s in states:s['actors'].append(dict(id='other',model='sedan',position_W=[10*s['time_s']+70+s['time_s']+2.3,0,0],yaw_rad=0,ghost=False))
        before=copy.deepcopy(states);proof=self.apply(states,actors)
        self.assertEqual(proof['actors'][0]['reason'],'candidate_introduces_body_overlap');self.assertEqual(states,before)

    def test_offline_smoother_resists_one_bad_size_observation(self):
        ts=np.arange(0,10,.2);y=np.log(.02+ts*.0001);bad=y.copy();bad[25]+=.5
        fitted=smooth_curve(ts,bad,ts)
        self.assertLess(np.max(abs(fitted-y)),.02)
