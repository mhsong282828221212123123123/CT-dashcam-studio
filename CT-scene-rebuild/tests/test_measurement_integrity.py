import copy,unittest
import numpy as np
from ct_scene.kalman_tracker import smooth_actor_rts
from ct_scene.collision_camera_policy import unresolved_front_depth_axis
from compare_camera_reprojection import compare


class MeasurementIntegrityTests(unittest.TestCase):
    def road(self):
        return dict(boundaries_y=[3.5,0.,-3.5],left_pockets=[],junctions=[],reference_line=dict(points_W_xy=[[0,0],[100,0]],supported_extent_m=[0,100]),lane_sections=[dict(id='a',station_range_m=[0,40],boundaries_y=[3.5,0.,-3.5]),dict(id='b',station_range_m=[60,100],boundaries_y=[3.5,0.,-3.5])])
    def test_inferred_boxes_are_not_independent_measurements(self):
        ts=np.arange(11)*.2;ego=np.c_[ts*5,ts*0,ts*0]
        rows=[dict(time_s=float(t),camera_id='front',support='detector_observation',position_W=[30+8*t,1.5,0.],unrefined_position_W=[30+8*t,1.5,0.],position_sigma_m=1.,bbox_xyxy=[600,450,650,500],lane_index=0,lane_prior='disabled_unknown') for t in ts]
        rows[5]['support']='offline_two_endpoint_gap'
        a=dict(id='a',global_id='a',lane_index=0,samples=rows,lane_motion_evidence=dict(clear_observations=0));b=copy.deepcopy(a)
        b['samples'][5]['unrefined_position_W']=[500.,90.,0.];b['samples'][5]['position_W']=[500.,90.,0.]
        x=smooth_actor_rts(a,self.road(),ts,ego);y=smooth_actor_rts(b,self.road(),ts,ego)
        np.testing.assert_allclose([r['position_W'] for r in a['samples']],[r['position_W'] for r in b['samples']],atol=1e-8)
        self.assertLess(np.linalg.norm(np.array(b['samples'][5]['position_W'])-[38,1.5,0]),.01)
        self.assertEqual(y['observations'],10)
        self.assertEqual(b['samples'][5]['unrefined_position_W'],[500.,90.,0.])
    def test_depth_axis_only_unlocks_unresolved_road_without_changing_valid_lane_policy(self):
        r=self.road();row=dict(camera_id='front',support='detector_observation',position_W=[50,1.5,0],unrefined_position_W=[50,1.5,0],bbox_xyxy=[600,450,650,500])
        axis=unresolved_front_depth_axis(row,[50,1.5,0],r,[0,0,0],[0,0,0])
        np.testing.assert_allclose(axis,np.array([50,1.5])/np.linalg.norm([50,1.5]))
        self.assertIsNone(unresolved_front_depth_axis(row,[30,1.5,0],r,[0,0,0],[0,0,0]))
        self.assertIsNone(unresolved_front_depth_axis(row,[110,1.5,0],r,[0,0,0],[0,0,0]))
    def test_prediction_or_truncated_box_never_establishes_camera_axis(self):
        row=dict(camera_id='front',support='offline_two_endpoint_gap',position_W=[50,1.5,0],bbox_xyxy=[600,450,650,500])
        self.assertIsNone(unresolved_front_depth_axis(row,[50,1.5,0],self.road(),[0,0,0],[0,0,0]))
        row.update(support='detector_observation',quality_flags=['truncated'])
        self.assertIsNone(unresolved_front_depth_axis(row,[50,1.5,0],self.road(),[0,0,0],[0,0,0]))
    def test_reprojection_uses_original_frame_box_not_changed_global_identity(self):
        row=dict(camera_id='front',id='old',frame_id='f',time_s=0.,source_bbox=[10,10,30,30],height_ratio=1.,bbox_iou=.8,centre_x_error_px=1.)
        before=dict(records=[row],summary={});after=dict(records=[dict(row,id='new',bbox_iou=.9)],summary={})
        r=compare(before,after)['comparison']['front']
        self.assertEqual(r['common_projected_observations'],1);self.assertEqual(r['lost_projections'],0)
