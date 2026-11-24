import math
import unittest
from types import SimpleNamespace
import numpy as np
from core.driving_scene import Actor,Scene
from core.scene_world_tracks import rotation
from tools.validate_single_clip_reference import motion_outliers


class MotionAuditTests(unittest.TestCase):
    def test_legal_highway_translation_is_not_reported_as_a_jump(self):
        scenes=[Scene(i*.1,{'speed_mps':30.,'speed_kmh':108},actors=[Actor('lead','car',0,25)]) for i in range(8)]
        poses=[(np.array([0.,i*3.]),np.eye(2)) for i in range(8)]
        t=SimpleNamespace(scenes=scenes,ego_poses=poses)
        self.assertEqual(motion_outliers(t),[])
        scenes[4].actors[0].x=15
        self.assertTrue(motion_outliers(t))

    def test_host_rotation_does_not_rotate_a_stationary_world_vehicle(self):
        scenes=[];poses=[]
        for i in range(8):
            theta=i*.12;R=rotation(theta)
            origin=np.array([2.5*(1-math.cos(theta)),2.5*math.sin(theta)])
            local=R.T@(np.array([0.,20.])-origin)
            scenes.append(Scene(i*.1,{'speed_mps':3.},actors=[Actor('parked','car',*local,yaw=theta)]))
            poses.append((origin,R))
        self.assertEqual(motion_outliers(SimpleNamespace(scenes=scenes,ego_poses=poses)),[])

    def test_stationary_body_rotation_is_still_flagged(self):
        scenes=[Scene(0.,{'speed_kmh':0},actors=[Actor('a','car',0,20)]),
                Scene(.1,{'speed_kmh':0},actors=[Actor('a','car',0,20,yaw=.5)])]
        self.assertTrue(motion_outliers(SimpleNamespace(scenes=scenes,ego_poses=[(np.zeros(2),np.eye(2))]*2)))
