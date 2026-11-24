import math,unittest
import numpy as np
from core.scene_fusion import CameraGeometry,corners,dimensions
from core.scene_trajectory_fit import fit_trajectory
from core.scene_world_tracks import rotation


class TrajectoryFitTests(unittest.TestCase):
    def test_turning_host_does_not_reverse_vehicle_heading_compensation(self):
        camera=CameraGeometry();observations=[];seed=[]
        for t in np.linspace(0,1.5,16):
            point=np.array([2.,20+5*t]);angle=.12*t;R=rotation(angle);origin=np.array([-.3*t,2*t])
            local=R.T@(point-origin)
            b=camera.visible_bounds(corners(*local,dimensions('car'),yaw=angle,kind='car'))
            observations.append(dict(time=float(t),camera=camera,bbox=np.r_[b[:2],b[2:]-b[:2]],
                                     ego_origin=origin,ego_rotation=R))
            seed.append(point+[1,2])
        result=fit_trajectory(observations,seed)
        self.assertTrue(result['accepted'])
        self.assertGreater(result['validation_iou_min'],.99)
        np.testing.assert_allclose(result['world_velocity'],[0,5],atol=.05)

    def test_transverse_vehicle_is_recovered_on_withheld_times(self):
        camera=CameraGeometry();observations=[];seed=[]
        for t in np.linspace(0,1.5,16):
            point=np.array([-4+4*t,20.])
            b=camera.visible_bounds(corners(*point,dimensions('car'),yaw=math.pi/2,kind='car'))
            observations.append(dict(time=float(t),camera=camera,bbox=np.r_[b[:2],b[2:]-b[:2]],
                                     ego_origin=np.zeros(2),ego_rotation=np.eye(2)))
            seed.append(point+[1,2])
        result=fit_trajectory(observations,seed)
        self.assertTrue(result['accepted'])
        self.assertGreater(result['validation_iou_min'],.99)
        self.assertEqual(result['withheld_samples'],8)
        np.testing.assert_allclose(result['world_velocity'],[4,0],atol=.05)

    def test_insufficient_temporal_evidence_is_not_accepted(self):
        self.assertIsNone(fit_trajectory([],[]))

    def test_turn_rate_recovers_curved_vehicle_motion_on_withheld_frames(self):
        camera=CameraGeometry();observations=[];seed=[]
        omega=.35;velocity=np.array([4.,2.]);origin=np.array([0.,22.])
        yaw=math.atan2(velocity[0],velocity[1])
        for t in np.linspace(-1.,1.,21):
            angle=omega*t
            s=math.sin(angle)/omega;c=(1-math.cos(angle))/omega
            point=origin+np.array([velocity[0]*s+velocity[1]*c,
                                   -velocity[0]*c+velocity[1]*s])
            b=camera.visible_bounds(corners(*point,dimensions('car'),yaw=yaw+angle,kind='car'))
            observations.append(dict(time=float(t),camera=camera,bbox=np.r_[b[:2],b[2:]-b[:2]],
                                     ego_origin=np.zeros(2),ego_rotation=np.eye(2)))
            seed.append(point+[.5,1.])
        result=fit_trajectory(observations,seed,turn_rate=True)
        self.assertTrue(result['accepted'])
        self.assertGreater(result['validation_iou_min'],.99)
        self.assertAlmostEqual(result['yaw_rate_rad_s'],omega,places=2)
        np.testing.assert_allclose(result['world_velocity'],velocity,atol=.05)
