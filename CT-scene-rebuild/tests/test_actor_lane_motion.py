import unittest
import numpy as np
from ct_scene.actor_lane_motion import estimate


def rows():
    return [dict(time_s=float(t),position_W=[10*t,0,0],camera_id='front',support='detector_observation',
                 image_width=1280,image_height=960,bbox_xyxy=[500,300,720,500]) for t in np.arange(0,8,.2)]


class ActorLaneMotionTests(unittest.TestCase):
    def test_sustained_clear_change_has_continuous_lateral_target(self):
        rs=rows()
        for r in rs:r['position_W'][1]=float(np.clip((r['time_s']-2)/3,0,1)*3.5)
        lanes,target,review=estimate(rs,[5.25,1.75,-1.75,-5.25],0,1)
        self.assertEqual(len(review['events']),1)
        self.assertEqual((lanes[0],lanes[-1]),(1,0))
        self.assertLess(float(max(abs(np.diff(target)))),.6)
        self.assertTrue(np.all(np.diff(target)>=-1e-9))

    def test_far_approach_drift_does_not_establish_a_new_lane(self):
        rs=rows()
        for r in rs:
            if r['time_s']>=3:r.update(position_W=[r['position_W'][0],3.5,0],bbox_xyxy=[500,400,560,460])
        lanes,target,review=estimate(rs,[5.25,1.75,-1.75,-5.25],0,1)
        self.assertTrue(np.all(lanes==1));self.assertFalse(review['events'])

    def test_camera_bias_cannot_be_interpreted_as_a_lane_change(self):
        rs=rows()
        for r in rs:
            if r['time_s']>=4:r.update(camera_id='right_repeater',position_W=[r['position_W'][0],3.5,0])
        lanes,target,review=estimate(rs,[5.25,1.75,-1.75,-5.25],0,1)
        self.assertTrue(np.all(lanes==1));self.assertFalse(review['events'])
        self.assertTrue(review['rejected_candidates'])

    def test_world_lane_is_independent_of_ego_pose_or_observer_distance(self):
        rs=rows()
        for r in rs:
            r['position_W'][1]=.02*r['position_W'][0]
            r['ego_position_W']=[r['time_s']*20,r['time_s'],0]
            r['ego_lane']=0 if r['time_s']>3 else 1
        lanes,_,review=estimate(rs,[5.25,1.75,-1.75,-5.25],.02,1)
        self.assertTrue(np.all(lanes==1));self.assertFalse(review['events'])


if __name__=='__main__':unittest.main()
