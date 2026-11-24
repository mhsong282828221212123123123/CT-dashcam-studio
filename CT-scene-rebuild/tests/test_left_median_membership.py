import unittest
import numpy as np
from ct_scene.left_median_membership import inside_pair
from ct_scene.curved_road import supported_boundaries
from ct_scene.inside_median_pockets import extend


class LeftMedianMembershipTests(unittest.TestCase):
    def test_persistent_right_car_supports_one_outer_band_but_not_across_yellow_curb(self):
        ref=dict(points_W_xy=[[0,0],[100,0]])
        paint=[dict(kind=kind,frame_id=str(i),points_W_xy=[[i*3,d],[i*3+2,d]],ground_length_m=2) for i in range(15) for kind,d in [('white',1.75),('white',-1.75)]]
        actor=dict(id='right',camera_id='right_repeater',samples=[dict(frame_id='car-'+str(i),camera_id='right_repeater',time_s=i*.2,position_W=[10+i, -3.5,0],support='detector_observation',bbox_xyxy=[200,350,500,550],image_width=1280) for i in range(12)])
        bounds,review=supported_boundaries(paint,ref,3.5,[actor],8);self.assertEqual(bounds,[1.75,-1.75])
        second=dict(actor,id='right2',samples=[dict(r,position_W=[r['position_W'][0]+8,-3.8,0],bbox_xyxy=[600,350,900,550]) for r in actor['samples']])
        bounds,review=supported_boundaries(paint,ref,3.5,[actor,second],8);self.assertEqual(bounds,[1.75,-1.75,-5.25])
        yellow=[dict(r,kind='yellow') if np.mean(np.array(r['points_W_xy'])[:,1])==-1.75 else r for r in paint]
        bounds,review=supported_boundaries(yellow,ref,3.5,[actor],8);self.assertEqual(bounds,[1.75,-1.75])
        for i,r in enumerate(actor['samples']):r['position_W'][0]=30-i
        bounds,_=supported_boundaries(paint,ref,3.5,[actor],8);self.assertEqual(bounds,[1.75,-1.75])

    def test_two_far_side_ground_lines_support_inside_but_single_near_side_and_facade_do_not(self):
        box=[280,320,960,720];lines=[[990,665,1279,692],[1000,680,1279,705]]
        self.assertIsNotNone(inside_pair(box,lines,1280,960));self.assertIsNone(inside_pair(box,lines[:1],1280,960))
        self.assertIsNone(inside_pair(box,[[x-800,y,X-800,Y] for x,y,X,Y in lines],1280,960))
        self.assertIsNone(inside_pair(box,[[x,y-170,X,Y-170] for x,y,X,Y in lines],1280,960))

    def test_inside_median_band_does_not_move_main_lanes_or_replace_existing_pocket(self):
        ref=dict(points_W_xy=[[0,0],[100,0]])
        paint=[dict(kind=kind,frame_id=str(i),points_W_xy=[[i*3,d],[i*3+2,d]],ground_length_m=2) for i in range(15) for kind,d in [('white',1.75),('white',-1.75),('white',-5.25)]]
        actor=dict(id='left',camera_id='left_repeater',inside_median_frame_ids=['car-'+str(i) for i in range(12)],samples=[dict(frame_id='car-'+str(i),time_s=i*.2,position_W=[10+i,3.5,0]) for i in range(12)])
        bounds,review=supported_boundaries(paint,ref,3.5,[actor],8,inherited_divider=1.75)
        self.assertEqual(bounds,[1.75,-1.75,-5.25])
        road=dict(reference_line=dict(ref,supported_extent_m=[0,100]),boundaries_y=bounds,junctions=[dict(road_station_range_m=[50,65])]);fit=dict(pockets=[],occupancy_used=False)
        result,review=extend(road,[actor],fit);self.assertEqual(len(result['pockets']),1);self.assertEqual(result['pockets'][0]['lane_id'],-1);self.assertEqual(result['pockets'][0]['end_x'],50)
        self.assertEqual(road['boundaries_y'],bounds);self.assertEqual(fit['pockets'],[])
        again,review=extend(road,[actor],result);self.assertEqual(again,result);self.assertEqual(review[0]['status'],'existing_supported_pocket_preserved')
        later=dict(actor,samples=[dict(r,frame_id='later-'+r['frame_id']) for r in actor['samples']]);yellow=[dict(r,kind='yellow') if np.mean(np.array(r['points_W_xy'])[:,1])==1.75 else r for r in paint]
        bounds,review=supported_boundaries(yellow,ref,3.5,[later],8)
        self.assertEqual(bounds,[1.75,-1.75,-5.25])
