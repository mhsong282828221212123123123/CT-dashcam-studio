import copy,unittest
import numpy as np
from ct_scene.road_boundary_confidence import boundary_support,lane_confidence,membership_supported
from ct_scene.kalman_tracker import smooth_actor_rts
from ct_scene.local_solid_corridor import unresolved_side_height_position
from evaluate_position_source_controls import evaluate


class BoundaryConfidenceTests(unittest.TestCase):
    def road(self):
        return dict(boundaries_y=[3.5,0.,-3.5],left_pockets=[],junctions=[],reference_line=dict(points_W_xy=[[0,0],[300,0]],supported_extent_m=[0,300]),lane_sections=[dict(id='r',station_range_m=[0,300],boundaries_y=[3.5,0.,-3.5],support='local_camera_paint',evidence=dict(peaks=[dict(y=3.5,kind='yellow_divider',frames=20),dict(y=0.,kind='white',frames=30),dict(y=-3.5,kind='vehicle_extent',frames=999)]))])
    def test_repeated_paint_is_stronger_than_vehicle_extent(self):
        r=self.road();before=copy.deepcopy(r)
        self.assertEqual(lane_confidence(r,100,0),1.)
        self.assertLessEqual(lane_confidence(r,100,1),.2)
        self.assertEqual(r,before)
    def test_unknown_lane_never_becomes_stronger_when_observation_is_weak(self):
        a=dict(lane_motion_evidence=dict(clear_observations=0))
        self.assertFalse(membership_supported(a,dict(lane_prior='observed_membership_gated_corridor_prior',lane_measurement_usable=False)))
        self.assertTrue(membership_supported(a,dict(lane_measurement_usable=True)))
        self.assertFalse(membership_supported({},dict(lane_prior='disabled_outside_unknown_or_conflicting_membership')))
    def test_rts_keeps_unverified_raw_bearing_off_lane_center(self):
        r=self.road();times=np.arange(12)*.2
        rows=[dict(time_s=float(t),camera_id='front',support='detector_observation',position_W=[20+10*t,3.1,0],unrefined_position_W=[20+10*t,3.1,0],position_sigma_m=2.,bbox_xyxy=[600,450,625,474],image_width=1280,lane_index=0,lane_prior='disabled_unknown',lane_measurement_usable=False) for t in times]
        a=dict(id='a',global_id='a',lane_index=0,samples=rows,lane_motion_evidence=dict(clear_observations=0))
        smooth_actor_rts(a,r,times,np.c_[times*10,times*0,times*0])
        self.assertLess(max(abs(x['position_W'][1]-3.1) for x in rows),.05)
    def test_calibrated_side_margin_does_not_admit_remote_or_uncalibrated_cars(self):
        r=self.road();a=dict(lane_index=0);row=dict(camera_id='left_repeater',projection_support='apparent_vehicle_height_prior',side_projection_diagnostics=dict(pitch_calibration_accepted=True,clear_ground_candidate=True,yaw_sigma_rad=.052,yaw_sigma_m=1.2))
        self.assertFalse(unresolved_side_height_position(r,a,row,[100,5.,0]))
        self.assertTrue(unresolved_side_height_position(r,a,row,[100,9.,0]))
        row['side_projection_diagnostics']['pitch_calibration_accepted']=False
        self.assertTrue(unresolved_side_height_position(r,a,row,[100,5.,0]))
    def test_missing_and_merged_source_cars_cannot_pass_separation(self):
        c=dict(time_s=0.,kind='distinct_nonoverlapping_bodies',ids=['a','b']);dims=dict(sedan=[4.5,1.8,1.5])
        actor=dict(id='merged',global_id='merged',local_ids=['a','b']);body=dict(id='merged',model='sedan',position_W=[0,0,0],yaw_rad=0.)
        self.assertFalse(evaluate(c,[dict(time_s=0.,actors=[])],[actor],dims)['passed'])
        self.assertFalse(evaluate(c,[dict(time_s=0.,actors=[body])],[actor],dims)['passed'])
