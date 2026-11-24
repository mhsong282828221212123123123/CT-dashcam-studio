import unittest
from types import SimpleNamespace
import cv2
import numpy as np
from core.scene_road_evidence import supported_curves,road_cross_sections,segmented_curves
from core.scene_road import add_camera_corridors as add_travel_corridors


class RoadEvidenceTests(unittest.TestCase):
    def test_camera_road_extent_is_not_collapsed_to_ego_lane(self):
        scene=SimpleNamespace(time=0,lanes=[{'points':[[-1,5],[-1,10]]}],
                              camera_road_sections=[[[-6,5],[6,5]],[[-5,10],[7,10]]],
                              camera_road_edges=[[[-6,5],[-5,10]]])
        audit={}
        add_travel_corridors([scene],[(np.zeros(2),np.eye(2))],[audit])
        self.assertGreater(len(scene.road_surfaces),0)
        # Surface tessellation can change; measured extent must survive.
        self.assertAlmostEqual(np.ptp(np.asarray(scene.road_surfaces).reshape(-1,2)[:,0]),13.)
        self.assertTrue(scene.lanes[0]['dashed'])
        self.assertFalse(scene.lanes[-1]['dashed'])
        self.assertFalse(audit['travel_corridors']['lane_count_inferred'])

    def test_road_memory_follows_ego_motion_and_resets_with_odometry(self):
        def frame(t,sections=()):
            return SimpleNamespace(time=t,lanes=[],camera_road_sections=sections,camera_road_edges=[])
        first=frame(0,[[[-4,5],[4,5]],[[-4,10],[4,10]]])
        second=frame(.5);third=frame(.6)
        audits=[{},{},{'world_track_events':[{'reason':'odometry unavailable or discontinuous; tracks reset'}]}]
        add_travel_corridors([first,second,third],[(np.zeros(2),np.eye(2)),
            (np.array([0,5]),np.eye(2)),(np.zeros(2),np.eye(2))],audits)
        np.testing.assert_allclose(np.asarray(second.road_surfaces[0])[:,1],[0,0,5,5])
        self.assertEqual(third.road_surfaces,[])

    def test_past_paint_moves_behind_host_without_recentering(self):
        points=[[2,float(z)] for z in np.linspace(4,20,40)]
        frames=[SimpleNamespace(time=t,lanes=[{'points':points}],camera_road_sections=[],camera_road_edges=[])
                for t in (0,.5)]
        add_travel_corridors(frames,[(np.zeros(2),np.eye(2)),(np.array([1,10]),np.eye(2))],[{},{}])
        past=[lane for lane in frames[-1].lanes if lane.get('source')=='past_camera_paint_world_coordinates']
        self.assertTrue(past)
        self.assertLess(min(p[1] for lane in past for p in lane['points']),0)
        self.assertTrue(all(abs(p[0]-1)<1e-8 for lane in past for p in lane['points']))

    def test_future_camera_paint_extends_road_without_crossing_odom_reset(self):
        def scenes():
            return [SimpleNamespace(time=t,lanes=[{'points':[[2,float(z)] for z in range(5,21)]}],
                    camera_road_sections=[[[-4,5],[4,5]],[[-4,20],[4,20]]],camera_road_edges=[])
                    for t in (0,.2)]
        poses=[(np.zeros(2),np.eye(2)),(np.array([0,5]),np.eye(2))]
        frames=scenes();add_travel_corridors(frames,poses,[{},{}])
        future=[lane for lane in frames[0].lanes if lane.get('source')=='future_camera_paint_world_coordinates']
        self.assertTrue(future)
        self.assertTrue(all(p[1]>20 for lane in future for p in lane['points']))
        frames=scenes();add_travel_corridors(frames,poses,[{},
            {'world_track_events':[{'reason':'odometry unavailable or discontinuous; tracks reset'}]}])
        self.assertFalse(any(l.get('source')=='future_camera_paint_world_coordinates' for l in frames[0].lanes))

    def test_segmentation_supports_five_boundaries_and_rejects_stop_line(self):
        paint=np.zeros((960,1280),np.uint8)
        for x in (100,350,600,850,1100):
            cv2.line(paint,(x,600),(x+50,850),1,7)
        cv2.line(paint,(100,540),(1100,540),1,9)
        curves=segmented_curves(paint,np.ones_like(paint))
        self.assertEqual(len(curves),5)

    def test_multiple_curved_paint_boundaries_survive_without_two_line_cap(self):
        h,w=960,1280;paint=np.zeros((h,w),np.uint8);road=np.ones_like(paint);hypotheses=[]
        for i,x in enumerate((150,420,690,960)):
            ys=np.linspace(500,880,40);xs=x+100*((ys-600)/300)**2
            points=np.column_stack([xs,ys])
            cv2.polylines(paint,[points.astype(np.int32)],False,1,9)
            hypotheses.append({'id':str(i),'points':points.tolist()})
        curves=supported_curves(hypotheses,paint,road)
        self.assertEqual(len(curves),4)
        self.assertTrue(all(c['boundary_type']=='unknown' for c in curves))

    def test_curve_prediction_without_paint_is_not_a_lane(self):
        blank=np.zeros((960,1280),np.uint8)
        curve={'id':'invented','points':[[500,y] for y in range(500,880,10)]}
        self.assertEqual(supported_curves([curve],blank,np.ones_like(blank)),[])

    def test_vehicle_edges_do_not_count_as_paint_support(self):
        paint=np.zeros((960,1280),np.uint8);paint[500:880,495:505]=1
        curve={'id':'body','points':[[500,y] for y in range(500,880,10)]}
        self.assertEqual(supported_curves([curve],paint,np.ones_like(paint),
                                         [{'bbox':[450,480,100,420]}]),[])

    def test_road_width_is_independent_of_lane_count(self):
        road=np.zeros((960,1280),np.uint8);road[500:900,100:1180]=1
        sections=road_cross_sections(road)
        self.assertGreater(len(sections),10)
        self.assertEqual(sections[-1][1][0]-sections[-1][0][0],1079)
        self.assertEqual(road_cross_sections(np.zeros_like(road)),[])
