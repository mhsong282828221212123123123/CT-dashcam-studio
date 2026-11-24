import unittest,copy
import numpy as np
from ct_scene.surrounding import project_front,asset_class
from ct_scene.offline_paths import augment_frozen_geometry


PROFILE={'model':'SIMPLE_RADIAL','resolution':[1280,960],'params':[1653.,640,480,0.]}
MOUNT={'height_m':1.3,'yaw_deg':0,'offset_W_ego_m':[0,0,0]}


class FrontFallbackTests(unittest.TestCase):
    def test_clear_later_truck_views_outvote_tiny_distant_car_boxes(self):
        distant=[dict(bbox_xyxy=[100,470,130,500],support='detector_observation',class_id=2,score=.8) for _ in range(80)]
        clear=[dict(bbox_xyxy=[110,410,210,530],support='detector_observation',class_id=7,score=.8) for _ in range(20)]
        self.assertEqual(asset_class(distant+clear,'left_repeater',{'sedan','truck'})[0],'truck')
        self.assertEqual(asset_class(distant+clear[:1],'left_repeater',{'sedan','truck'})[0],'sedan')
    def test_two_sided_body_heading_wins_over_depth_prior_loop(self):
        trusted=[dict(frame_id='a',time_s=0.,position_W=[0.,0.,0.],yaw_rad=.1,support='detector_observation'),dict(frame_id='b',time_s=4.,position_W=[0.,0.,0.],yaw_rad=.3,support='detector_observation')]
        before=copy.deepcopy(trusted)
        candidates=[dict(frame_id=str(i),time_s=float(i),position_W=[float(i),float(i%2),0.],support='detector_observation') for i in (1,2,3)]
        result=augment_frozen_geometry(trusted,candidates)
        np.testing.assert_allclose([r['yaw_rad'] for r in result],np.linspace(.1,.3,5))
        self.assertEqual([result[0],result[-1]],before)

    def test_filled_geometry_cannot_reverse_frozen_body_front(self):
        # A brief reverse contact drift is not a physical 180-degree body turn.
        trusted=[dict(frame_id='anchor',time_s=1.,position_W=[5.,0.,0.],yaw_rad=np.pi,support='detector_observation')]
        before=copy.deepcopy(trusted)
        candidates=[dict(frame_id=str(i),time_s=i/5,position_W=[float(i),0.,0.],support='detector_observation') for i in range(5)]
        result=augment_frozen_geometry(trusted,candidates)
        self.assertEqual(result[-1],before[0]);self.assertEqual(trusted,before)
        self.assertTrue(all(np.cos(r['yaw_rad']-np.pi)>.99 for r in result))

    def test_cropped_sedan_is_not_made_a_van_from_truck_false_class(self):
        partial=[dict(bbox_xyxy=[0,200,530,690],support='detector_observation',quality_flags=['truncated'],class_id=7) for _ in range(20)]
        self.assertEqual(asset_class(partial,'right_repeater',{'sedan','truck','van'})[0],'sedan')
        full=[dict(bbox_xyxy=[650,310,1080,605],support='detector_observation',quality_flags=[],class_id=7) for _ in range(20)]
        self.assertEqual(asset_class(full,'right_repeater',{'sedan','truck','van'})[0],'truck')

    def test_positive_far_contact_keeps_depth_model_and_order(self):
        row=dict(bbox_xyxy=[515,458,575,509],support='detector_observation',quality_flags=[])
        p,support=project_front(row,PROFILE,MOUNT,1.65)
        self.assertTrue(70<p[0]<80);self.assertEqual(support,'far_ground_contact_low_confidence')
        nearer,_=project_front(dict(row,bbox_xyxy=[515,458,575,516]),PROFILE,MOUNT,1.65)
        self.assertLess(nearer[0],p[0])

    def test_height_fallback_only_for_unresolved_horizon(self):
        row=dict(bbox_xyxy=[515,420,575,471],support='detector_observation',quality_flags=[])
        p,support=project_front(row,PROFILE,MOUNT,1.65)
        self.assertTrue(40<p[0]<60);self.assertEqual(support,'observed_height_prior_after_unresolved_contact')

    def test_truncated_flow_or_tiny_box_cannot_supply_height_fallback(self):
        row=dict(bbox_xyxy=[515,458,575,509],support='detector_observation',quality_flags=[])
        for change in [dict(quality_flags=['truncated']),dict(support='flow_observation'),dict(bbox_xyxy=[515,480,535,499])]:
            with self.assertRaises(ValueError):project_front(dict(row,**change),PROFILE,MOUNT,1.65)

    def test_existing_frozen_samples_stay_exact(self):
        trusted=[dict(frame_id=str(i),time_s=float(i),position_W=[i*5.,0.,0.],yaw_rad=0.,support='detector_observation') for i in range(3)]
        before=copy.deepcopy(trusted)
        candidates=[dict(frame_id=str(i),time_s=float(i),position_W=[i*5.+.2,0.,0.],support='detector_observation') for i in range(3,7)]
        result=augment_frozen_geometry(trusted,candidates)
        self.assertEqual(result[:3],before);self.assertEqual(trusted,before);self.assertEqual(len(result),7)
        self.assertTrue(all(r['uses_future_observations'] for r in result[3:]))
