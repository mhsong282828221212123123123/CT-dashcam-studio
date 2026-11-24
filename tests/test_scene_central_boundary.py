import unittest
import cv2
import numpy as np
from core.scene_central_boundary import CentralBoundaryTracker,detect_central_boundary
from core.scene_fusion import CameraGeometry,corners,dimensions,FusedSceneTimeline


class CentralBoundaryTests(unittest.TestCase):
    def fixture(self,colour=(0,220,220)):
        camera=CameraGeometry();image=np.full((960,1280,3),70,np.uint8)
        points=camera.pixels(np.array([[1.75,0.,4.],[1.75,0.,18.]]))
        cv2.line(image,tuple(points[0].astype(int)),tuple(points[1].astype(int)),colour,12)
        return camera,image

    def objects(self):
        return [dict(track_id='front:own',bbox=[650,500,100,100]),
                dict(track_id='front:opposite',bbox=[250,500,100,100]),
                dict(track_id='front:edge',bbox=[450,500,100,100])]

    def test_visible_yellow_boundary_excludes_only_wholly_foreign_contact(self):
        camera,image=self.fixture();tracker=CentralBoundaryTracker();objects=self.objects()
        for i in range(8):evidence=tracker.update(image,objects,camera,i*.1)
        self.assertTrue(evidence['active'])
        self.assertEqual(objects[1]['road_relation'],'beyond_central_boundary')
        self.assertNotEqual(objects[0]['road_relation'],'beyond_central_boundary')
        self.assertNotEqual(objects[2]['road_relation'],'beyond_central_boundary')

    def test_no_invisible_boundary_through_a_gap(self):
        camera,image=self.fixture();tracker=CentralBoundaryTracker();objects=self.objects()
        for i in range(8):tracker.update(image,objects,camera,i*.1)
        result=tracker.update(np.full_like(image,70),objects,camera,.8)
        self.assertFalse(result['active'])
        self.assertTrue(all(o['road_relation']=='unknown' for o in objects))

    def test_established_host_side_track_survives_later_crossing(self):
        camera,image=self.fixture();tracker=CentralBoundaryTracker();objects=self.objects()[:1]
        for i in range(8):tracker.update(image,objects,camera,i*.1)
        objects[0]['bbox']=[250,500,100,100]
        for i in range(8,16):tracker.update(image,objects,camera,i*.1)
        self.assertNotEqual(objects[0]['road_relation'],'beyond_central_boundary')

    def test_white_line_is_not_claimed_as_central_separator(self):
        camera,image=self.fixture((230,230,230))
        self.assertIsNone(detect_central_boundary(image,[],camera))

    def test_outside_detection_cannot_enter_fitting_or_lane_votes(self):
        camera=CameraGeometry();rows=[]
        for i in range(15):
            bounds=camera.visible_bounds(corners(7.,15.,dimensions('car'),kind='car'))
            obj=dict(track_id='front:opposite',bbox=np.r_[bounds[:2],bounds[2:]-bounds[:2]].tolist(),
                kind='car',confidence=.95,track_status='confirmed',road_relation='beyond_central_boundary')
            rows.append(dict(time_s=i*.1,ego=dict(data=dict(speed_kmh=0,heading=0)),
                cameras=dict(front=dict(available=True,source_time_s=i*.1,objects=[obj]))))
        timeline=FusedSceneTimeline(rows,dict(sample_hz=10,cameras=dict(front=dict(width=1280,height=960))))
        self.assertTrue(all(not scene.actors for scene in timeline.scenes))
        self.assertTrue(all(a['travel_corridors']['display_lane_count']==1 for a in timeline.audit))
