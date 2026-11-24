import unittest
import numpy as np
from ct_scene.road_sections import fit
from ct_scene.road_geometry import lane_at,boundaries_at
from ct_scene.lane_state import timeline


def example():
    road=dict(boundaries_y=[8.75,5.25,1.75,-1.75,-5.25],marking_station_x=None,heading_evidence=dict(slope=0),
              reference_line=dict(points_W_xy=[[0,0],[180,0]],supported_extent_m=[0,180]),
              junctions=[dict(road_station_range_m=[70,85],polygon_W_xy=[[70,-10],[85,-10],[85,12],[70,12]])])
    paint=[]
    for lo,hi,left,whites in [(0,70,8.75,[5.25,1.75,-1.75,-5.25]),(85,180,5.25,[1.75,-1.75])]:
        for i,x in enumerate(np.linspace(lo+3,hi-5,9)):
            for kind,d in [('yellow',left)]+[('white',d) for d in whites]:
                paint.append(dict(kind=kind,frame_id=f'{lo}-{i}',points_W_xy=[[x,d],[x+3,d]]))
    ego=np.c_[np.arange(0,175,2),np.zeros((88,2))]
    sections,review=fit(road,paint,[],ego,3.5);road['lane_sections']=sections
    return road,review


class RoadSectionTests(unittest.TestCase):
    def test_camera_paint_changes_lane_count_by_section(self):
        road,review=example()
        self.assertEqual(review['status'],'camera_supported_piecewise_lane_sections')
        self.assertEqual([s['lane_count'] for s in road['lane_sections']],[4,2])
        self.assertEqual(lane_at(road,20,0),2);self.assertEqual(lane_at(road,100,0),1)
        self.assertIsNone(lane_at(road,78,0))

    def test_road_narrows_without_vehicle_lane_change_then_real_left_change(self):
        road,_=example();t=np.arange(0,34,.1);x=t*5;d=np.clip((x-110)/20,0,1)*3.5;p=np.c_[x,d,t*0]
        report=timeline(t,p,t*0,road,4.7,2.)
        self.assertEqual(len(report['events']),1)
        e=report['events'][0];self.assertEqual((e['from_lane'],e['to_lane'],e['direction']),(1,0,'left'))
        self.assertEqual(e['road_section_id'],'road-section-1')
        self.assertGreater(e['crossing_s'],22)

    def test_map_opening_without_camera_lane_paint_cannot_create_sections(self):
        road,_=example();sections,review=fit(road,[],[],np.c_[np.arange(0,175,2),np.zeros((88,2))],3.5)
        self.assertEqual(sections,[]);self.assertEqual(review['status'],'insufficient_two_sided_local_support')

    def test_turning_into_another_road_does_not_reuse_approach_divider(self):
        for turn,expected in [(0,1),(90,2)]:
            road=dict(boundaries_y=[1.75,-1.75],left_boundary_y=1.75,left_boundary_role='camera_observed_yellow_divider_estimated_geometry',reference_line=dict(points_W_xy=[[0,0],[240,0]],supported_extent_m=[0,240]),junctions=[dict(turn_deg=turn,road_station_range_m=[80,100])])
            paint=[]
            for lo,hi,lines in [(0,80,[('yellow',1.75),('white',-1.75)]),(100,240,[('white',5.25),('white',1.75),('white',-1.75)])]:
                for i,x in enumerate(np.linspace(lo+5,hi-5,12)):
                    for kind,d in lines:paint.append(dict(kind=kind,frame_id=f'{lo}-{i}',points_W_xy=[[x,d],[x+3,d]]))
            ego=np.c_[np.arange(0,240,2),np.zeros((120,2))];sections,review=fit(road,paint,[],ego,3.5)
            if turn==0:
                self.assertEqual(sections,[]);self.assertEqual(review['status'],'local_geometry_consistent_with_global')
            else:
                self.assertEqual([x['lane_count'] for x in sections],[1,expected]);self.assertEqual(review['regions'][1]['inherited_approach_divider'],None)
