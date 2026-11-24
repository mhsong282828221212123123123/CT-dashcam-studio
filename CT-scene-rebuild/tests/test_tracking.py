import unittest
import numpy as np
import cv2
from ct_scene.tracking import assignment,match_cost,stitch,fill_camera_gaps,flow_box
from ct_scene.offline_paths import WorldAnchor,complete_gap,smooth_positions


class TrackingTests(unittest.TestCase):
    def test_assignment_does_not_use_greedy_nearest(self):
        self.assertEqual(set(assignment([[.1,.2],[.11,.9]],.8)),{(0,1),(1,0)})
        self.assertEqual(assignment([[9]],.8),[])

    def test_partial_small_box_cannot_steal_a_large_vehicle(self):
        hist=np.ones(128)/128
        tr={'class_id':2,'last_box':np.array([100,100,400,600]),'last_time':0.,'velocity':np.zeros(2),'appearance':hist}
        false={'class_id':2,'bbox_xyxy':[100,60,400,210],'appearance':hist}
        correct={'class_id':2,'bbox_xyxy':[103,100,403,600],'appearance':hist}
        self.assertGreater(match_cost(tr,false,.2),100)
        self.assertLess(match_cost(tr,correct,.2),.2)

    def test_entering_border_vehicle_can_grow_without_starting_new_id(self):
        hist=np.ones(128)/128
        tr={'class_id':2,'last_box':np.array([.2,365.8,42.1,699.7]),'last_time':0.,'velocity':np.zeros(2),'appearance':hist,'last_quality_flags':['truncated']}
        incoming={'class_id':2,'bbox_xyxy':[.2,245.5,102.7,723.8],'appearance':hist,'quality_flags':['truncated']}
        self.assertLess(match_cost(tr,incoming,.2),.8)
        tr['last_quality_flags']=[]
        self.assertGreater(match_cost(tr,incoming,.2),100)

    def test_border_exception_does_not_allow_arbitrary_scale_or_other_colour(self):
        hist=np.eye(1,128,0)[0]
        tr={'class_id':2,'last_box':np.array([0,300,40,700]),'last_time':0.,'velocity':np.zeros(2),'appearance':hist,'last_quality_flags':['truncated']}
        other={'class_id':2,'bbox_xyxy':[0,200,500,750],'appearance':hist,'quality_flags':['truncated']}
        self.assertGreater(match_cost(tr,other,.2),100)
        other['bbox_xyxy']=[0,200,100,750];other['appearance']=np.eye(1,128,127)[0]
        self.assertGreater(match_cost(tr,other,.2),100)

    def test_gap_requires_two_observed_ends_and_cannot_cross_raw_cameras(self):
        a={'frame_id':'a','camera_id':'front','camera_time_s':0.,'bbox_xyxy':[10,10,30,30],'class_id':2}
        b=dict(a,frame_id='b',camera_time_s=2.,bbox_xyxy=[30,10,50,30])
        frames=[{'frame_id':str(t),'camera_id':'front','camera_time_s':t} for t in (-1.,1.,3.)]
        result=fill_camera_gaps([a,b],frames)
        gap=[r for r in result if r.get('uses_future_observation')]
        self.assertEqual(len(gap),1);self.assertEqual(gap[0]['bbox_xyxy'],[20,10,40,30])
        self.assertIsNone(gap[0]['observed_bbox_xyxy'])
        with self.assertRaises(ValueError):fill_camera_gaps([a,dict(b,camera_id='back')],frames)

    def test_similar_car_alternatives_remain_unmerged(self):
        hist=np.ones(128)/128
        def tr(name,t):return {'id':name,'camera_id':'front','class_id':2,'last_time':t,'last_box':np.array([10,10,30,30]),
            'velocity':np.zeros(2),'appearance':hist,'initial_appearance':hist,'observations':[{'camera_time_s':t,'bbox_xyxy':[10,10,30,30]}]}
        roots,edges=stitch([tr('A',0),tr('B',2),tr('C',2)])
        self.assertEqual(roots,{'A':'A','B':'B','C':'C'});self.assertFalse(any(e['selected'] for e in edges))
        roots,_=stitch([tr('A',0),tr('B',2)]);self.assertEqual(roots['A'],roots['B'])

    def test_flow_uses_real_features_and_rejects_textureless_prediction(self):
        a=np.random.default_rng(2).integers(0,256,(180,240),dtype=np.uint8)
        b=cv2.warpAffine(a,np.array([[1.,0,3],[0,1,2]]),(240,180))
        result=flow_box(a,b,[40,30,190,150]);self.assertIsNotNone(result)
        np.testing.assert_allclose(result[0],[43,32,193,152],atol=.4)
        self.assertIsNone(flow_box(np.zeros_like(a),np.zeros_like(a),[40,30,190,150]))


class OfflinePathTests(unittest.TestCase):
    def test_full_interval_fit_preserves_linear_motion_and_uses_future(self):
        t=np.linspace(0,4,21);p=np.column_stack([t*3,np.zeros_like(t),np.zeros_like(t)])
        np.testing.assert_allclose(smooth_positions(t,p),p,atol=1e-9)
        changed=p.copy();changed[-3:,0]+=2
        fitted=smooth_positions(t,changed)
        self.assertGreater(abs(fitted[10,0]-p[10,0]),.001)
        np.testing.assert_array_equal(p[:,0],t*3)

    def test_smoother_rejects_duplicate_time_or_negative_confidence(self):
        p=np.zeros((3,3))
        with self.assertRaises(ValueError):smooth_positions([0,1,1],p)
        with self.assertRaises(ValueError):smooth_positions([0,1,2],p,[1,-1,1])

    def test_rear_to_front_same_lane_is_filled_from_both_ends(self):
        before=WorldAnchor('car',0,(-20,3.5,0),(10,0,0),('road-A',2),('rear_frame',))
        after=WorldAnchor('car',4,(20,3.5,0),(10,0,0),('road-A',2),('front_frame',))
        result=complete_gap(before,after,[1,2,3])
        self.assertEqual([r['lane_key'] for r in result],[('road-A',2)]*3)
        np.testing.assert_allclose([r['position_W'][0] for r in result],[-10,0,10])
        self.assertTrue(all(r['uses_future_observation'] and r['render_state']=='ghost' for r in result))

    def test_endpoint_changes_revise_past_gap_without_overshoot(self):
        a=WorldAnchor('A',0,(0,0,0),(1000,0,0));b=WorldAnchor('A',2,(10,0,0),(1000,0,0))
        p=complete_gap(a,b,np.linspace(.01,1.99,30));self.assertTrue(all(0<=r['position_W'][0]<=10 for r in p))
        newer=WorldAnchor('A',2,(20,0,0))
        self.assertNotEqual(complete_gap(a,b,[1])[0]['position_W'],complete_gap(a,newer,[1])[0]['position_W'])
        with self.assertRaises(ValueError):complete_gap(a,WorldAnchor('B',2,(10,0,0)),[1])

    def test_lane_number_alone_does_not_connect_different_road_segments(self):
        a=WorldAnchor('A',0,None,lane_key=('road-A',2));b=WorldAnchor('A',2,None,lane_key=('road-B',2))
        result=complete_gap(a,b,[1])[0];self.assertIsNone(result['lane_key']);self.assertIsNone(result['position_W'])
        with self.assertRaises(ValueError):WorldAnchor('A',0,(0,0,0),clock_status='unverified')
