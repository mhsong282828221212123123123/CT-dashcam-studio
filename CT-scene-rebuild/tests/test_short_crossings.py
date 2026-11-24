import unittest
from ct_scene.surrounding import short_crossing_candidate,asset_class
from ct_scene.junctions import crossing_motion


class ShortCrossingTests(unittest.TestCase):
    def test_seven_real_detections_may_cross_in_less_than_two_seconds(self):
        rows=[dict(camera_time_s=i*.2,time_s=i*.2,bbox_xyxy=[100,300,800,600],class_id=7,score=.9,
                   support='detector_observation',position_W=[30,-6+2*i,0]) for i in range(7)]
        self.assertTrue(short_crossing_candidate(rows))
        road=dict(reference_line=dict(points_W_xy=[[0,0],[60,0]]),junctions=[dict(polygon_W_xy=[[20,-15],[40,-15],[40,15],[20,15]])])
        self.assertTrue(crossing_motion(dict(samples=rows),road))
        road['junctions']=[];self.assertFalse(crossing_motion(dict(samples=rows),road))
        for r in rows:r['support']='offline_two_endpoint_gap'
        self.assertFalse(short_crossing_candidate(rows))

    def test_two_clear_side_views_plus_five_strong_truck_frames_support_truck(self):
        rows=[dict(camera_time_s=i*.2,bbox_xyxy=[100,300,800,600],class_id=7,score=.9,
                   quality_flags=[] if i in (3,4) else ['truncated'],support='detector_observation') for i in range(7)]
        model,evidence=asset_class(rows,'front',{'heavy_truck':None,'box_truck':None})
        self.assertEqual(model,'heavy_truck')
        for r in rows:r['quality_flags']=['truncated']
        self.assertEqual(asset_class(rows,'front',{'heavy_truck':None,'box_truck':None})[0],'sedan')
