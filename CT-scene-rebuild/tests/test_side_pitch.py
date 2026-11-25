import math,unittest
import numpy as np
from ct_scene.side_pitch import pitch_candidate,fit_camera_pitch
from ct_scene.surrounding import project_contact

P=dict(model='SIMPLE_RADIAL',params=[1000,640,480,0])

class SidePitchTests(unittest.TestCase):
    def box(self,height=2.,depth=20.,pitch=-.035):
        top=math.tan(math.atan((.85-height)/depth)-pitch)
        bottom=math.tan(math.atan(.85/depth)-pitch)
        return [590,480+1000*top,690,480+1000*bottom]

    def test_recovers_pitch_from_synthetic_known_geometry(self):
        b=self.box();r=pitch_candidate(b,P,.85,2.)
        self.assertAlmostEqual(r['delta_pitch_rad'],-.035,places=7)
        mount=dict(height_m=.85,yaw_deg=0,offset_W_ego_m=[0,0,0],pitch_correction_rad=r['delta_pitch_rad'])
        p=project_contact(b,P,mount,max_depth=70,minimum_ray_y=.003)
        self.assertAlmostEqual(p[0],20.,places=5)

    def records(self,values):
        return [dict(track_id=str(j),model='car',time_s=i*.2,delta_pitch_rad=v+(.0002 if i%2 else -.0002)) for j,v in enumerate(values) for i in range(12)]

    def test_uses_multiple_tracks_and_abstains_on_inconsistent_or_insufficient_evidence(self):
        good=fit_camera_pitch(self.records([-.035,-.033,-.037,-.034]))
        self.assertTrue(good['accepted']);self.assertLess(abs(good['delta_pitch_rad']+.035),.002)
        self.assertGreaterEqual(good['sigma_pitch_rad'],.006)
        self.assertLess(good['leave_one_track_out']['after_median_rad'],good['leave_one_track_out']['before_median_rad'])
        self.assertFalse(fit_camera_pitch(self.records([-.035,-.034]))['accepted'])
        self.assertFalse(fit_camera_pitch(self.records([-.07,-.03,.02,.06]))['accepted'])

    def test_partial_tiny_or_border_objects_supply_no_calibration(self):
        for b in [[0,300,180,600],[400,500,420,520],[500,480,1000,520]]:
            self.assertIsNone(pitch_candidate(b,P,.85,1.5))
