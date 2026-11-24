import math,unittest
import numpy as np
from core.scene_fusion import CameraGeometry,corners,dimensions,fit_box
from core.scene_stopped_motion import stopped_motion_priors
from core.scene_geometry import footprints_overlap
from core.driving_scene import Actor


class StoppedMotionTests(unittest.TestCase):
    def rows(self,speed=0,tracks=8):
        camera=CameraGeometry(horizon=500)
        rows=[]
        for i,t in enumerate(np.arange(0,4,.1)):
            objects=[]
            for k in range(tracks):
                x=-10+6*t;z=25+k*3
                p=camera.visible_bounds(corners(x,z,dimensions('car'),yaw=math.pi/2,kind='car'))
                objects.append(dict(track_id=f'front:{k}',track_status='confirmed',kind='car',bbox=np.r_[p[:2],p[2:]-p[:2]].tolist()))
            rows.append(dict(time_s=float(t),ego=dict(data=dict(speed_kmh=speed)),cameras=dict(front=dict(available=True,source_time_s=float(t),objects=objects))))
        return rows

    def test_stopped_crossing_silhouettes_recover_pitch_and_direction(self):
        h,d,reports=stopped_motion_priors(self.rows(),CameraGeometry())
        self.assertTrue(h);self.assertTrue(d)
        self.assertAlmostEqual(np.median(list(h.values())),500,delta=7)
        angles=[v['yaw'] for v in d.values()]
        self.assertLess(np.median(abs(np.array(angles)-math.pi/2)),.15)
        self.assertGreaterEqual(next(iter(reports.values()))['track_votes'],6)

    def test_moving_host_or_insufficient_tracks_do_not_self_calibrate(self):
        for rows in (self.rows(speed=30),self.rows(tracks=3)):
            self.assertEqual(stopped_motion_priors(rows,CameraGeometry()),({},{},{}))

    def test_crossing_body_reprojects_with_its_travel_yaw(self):
        camera=CameraGeometry(horizon=500);yaw=1.3;position=(4,25)
        p=camera.visible_bounds(corners(*position,dimensions('car'),yaw=yaw,kind='car'))
        fit=fit_box(camera,np.r_[p[:2],p[2:]-p[:2]],'car',yaw=yaw)
        self.assertIsNotNone(fit);np.testing.assert_allclose(fit[0],position,atol=.15)

    def test_rotated_footprints_use_vehicle_orientation(self):
        a=Actor('a','car',0,0,yaw=math.pi/2);b=Actor('b','car',0,3,yaw=math.pi/2)
        self.assertFalse(footprints_overlap(a,b))
        b.x,b.z=3,0
        self.assertTrue(footprints_overlap(a,b))
