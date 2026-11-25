import unittest,math
import numpy as np
from ct_scene.side_yaw import track_axis_candidates,fit_camera_yaw

P=dict(model='SIMPLE_RADIAL',params=[1000,640,480,0])

class SideYawTests(unittest.TestCase):
    def rows(self,yaw_deg=147.):
        yaw=math.radians(yaw_deg);c,s=math.cos(yaw),math.sin(yaw);R=np.array([[c,s],[-s,c]])
        rows=[]
        for t in np.arange(0,5.001,.2):
            x,y=R@np.array([-40+6*t,3.5]);cx=640-1000*y/x;top=480+1000*(.85-1.5)/x;bottom=480+1000*.85/x;w=1000*2./x
            rows.append(dict(time_s=float(t),support='detector_observation',score=.9,bbox_xyxy=[cx-w/2,top,cx+w/2,bottom]))
        return rows

    def test_recovers_camera_axis_without_using_road_or_absolute_size(self):
        ts=np.arange(0,5.001,.2);ys=ts*0
        for height in [1.5,1.8]:
            windows=track_axis_candidates(self.rows(),P,ts,ys,135.,height)
            self.assertGreater(len(windows),0)
            for w in windows:self.assertAlmostEqual(w['yaw_deg'],147.,places=5)

    def test_turn_and_constant_relative_position_do_not_calibrate(self):
        ts=np.arange(0,5.001,.2)
        self.assertEqual(track_axis_candidates(self.rows(),P,ts,ts*math.radians(5),135.,1.5),[])
        rows=self.rows();b=rows[0]['bbox_xyxy']
        for r in rows:r['bbox_xyxy']=b
        self.assertEqual(track_axis_candidates(rows,P,ts,ts*0,135.,1.5),[])

    def test_requires_cross_track_consistency(self):
        def rows(values):return [dict(track_id=str(i),median_yaw_deg=v) for i,v in enumerate(values)]
        self.assertTrue(fit_camera_yaw(rows([146,148,147,149]),147)['accepted'])
        self.assertFalse(fit_camera_yaw(rows([146,148]),147)['accepted'])
        self.assertFalse(fit_camera_yaw(rows([120,135,158,170]),147)['accepted'])
