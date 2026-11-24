import unittest
import numpy as np
from core.driving_scene import Actor,Scene
from core.scene_motion_heading import motion_headings,apply_motion_headings
from core.scene_world_tracks import rotation


class MotionHeadingTests(unittest.TestCase):
    def test_image_heading_wrap_does_not_snap_body_angle(self):
        scenes=[Scene(i*.1,{},actors=[Actor('a','car',0,20,sources=('front:1',))]) for i in range(2)]
        poses=[(np.zeros(2),np.eye(2))]*2
        priors={(0,'front:1'):{'yaw':np.radians(160)},(1,'front:1'):{'yaw':np.radians(-160)}}
        apply_motion_headings(scenes,poses,[{},{}],priors)
        change=(scenes[1].actors[0].yaw-scenes[0].actors[0].yaw+np.pi)%(2*np.pi)-np.pi
        self.assertAlmostEqual(np.degrees(change),9)

    def test_curved_motion_updates_tangent_at_each_analysis_frame(self):
        scenes=[];poses=[]
        for t in np.arange(0,4,.1):
            position=[10*np.sin(.5*t),20+10*(1-np.cos(.5*t))]
            scenes.append(Scene(float(t),{},actors=[Actor('turn','car',*position)]))
            poses.append((np.zeros(2),np.eye(2)))
        evidence=motion_headings(scenes,poses)
        for i in range(10,30):
            self.assertIn((i,'turn'),evidence)
            expected=np.pi/2-.5*scenes[i].time
            self.assertAlmostEqual(evidence[i,'turn']['yaw'],expected,delta=.03)
    def fixture(self,velocity):
        poses=[];scenes=[]
        for i in range(21):
            t=i*.1;position=np.array([0.,10*t]);R=rotation(t*.2)
            local=R.T@(np.array([4.,30.])+np.array(velocity)*t-position)
            poses.append((position,R));scenes.append(Scene(t,{},actors=[Actor('a','car',*local)]))
        return scenes,poses

    def test_stationary_world_object_is_not_moving_when_ego_passes(self):
        self.assertEqual(motion_headings(*self.fixture([0,0])),{})

    def test_oncoming_motion_compensates_for_turning_ego(self):
        scenes,poses=self.fixture([0,-10]);result=motion_headings(scenes,poses)
        e=result[10,'a'];axis=poses[10][1]@np.array([np.sin(e['yaw']),np.cos(e['yaw'])])
        np.testing.assert_allclose(axis,[0,-1],atol=1e-8)
        self.assertFalse(e['body_orientation_verified'])

    def test_camera_position_jump_is_not_heading_evidence(self):
        scenes,poses=self.fixture([0,0])
        for s in scenes[10:]:s.actors[0].x+=5
        self.assertNotIn((10,'a'),motion_headings(scenes,poses))

    def test_model_front_follows_transverse_travel_and_positions_do_not_change(self):
        scenes,poses=self.fixture([5,0]);before=[(s.actors[0].x,s.actors[0].z) for s in scenes]
        audits=[{} for s in scenes];apply_motion_headings(scenes,poses,audits)
        for i in range(5,16):
            actor=scenes[i].actors[0];axis=poses[i][1]@np.array([np.sin(actor.yaw),np.cos(actor.yaw)])
            np.testing.assert_allclose(axis,[1,0],atol=1e-8)
        self.assertEqual(before,[(s.actors[0].x,s.actors[0].z) for s in scenes])

    def test_stopped_model_keeps_heading_when_host_turns(self):
        scenes,poses=self.fixture([5,0])
        for i in range(21,51):
            t=i*.1;R=rotation(t*.2);origin=np.array([0.,10*t]);local=R.T@(np.array([14.,30.])-origin)
            poses.append((origin,R));scenes.append(Scene(t,{},actors=[Actor('a','car',*local)]))
        apply_motion_headings(scenes,poses,[{} for s in scenes])
        a=scenes[-1].actors[0];axis=poses[-1][1]@np.array([np.sin(a.yaw),np.cos(a.yaw)])
        np.testing.assert_allclose(axis,[1,0],atol=1e-8)
