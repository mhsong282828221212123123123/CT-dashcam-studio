import copy
import unittest
import numpy as np
from ct_scene.pocket_lane import fit_curved_pockets,vehicle_supported_curved_pockets
from ct_scene.road_frame import frame_for
from ct_scene.road_geometry import lane_at
from ct_scene.surrounding import actor_corridor_lane


def fixture():
    road=dict(boundaries_y=[5.2,1.7,-1.8],left_boundary_y=5.2,lane_width_m_prior=3.5,
              reference_line=dict(points_W_xy=[[0,0],[80,80],[100,140]],supported_extent_m=[0,100]),
              junctions=[dict(road_station_range_m=[100,120],polygon_W_xy=[[60,60],[95,60],[95,110],[60,110]])])
    points=[]
    for i,x in enumerate(range(20,100,10)):
        for j in range(3):
            for d in [5.2,1.7,8.4]:points.append(dict(x=x,y=d,kind='white',camera_id='front',frame_id=f'{i}-{j}'))
    for i,x in enumerate([20,25,30]):points.append(dict(x=x,y=8.4,kind='yellow',camera_id='front',frame_id=f'y-{i}'))
    return road,dict(coordinate_system='road_station_lateral',ground_support='road_mask_ground_region',points=points)


class CurvedPocketTests(unittest.TestCase):
    def test_observed_edges_can_enter_junction_without_inventing_return(self):
        road,evidence=fixture();original=copy.deepcopy(road);result=fit_curved_pockets(evidence,road)
        self.assertEqual(len(result['pockets']),1);self.assertEqual(road,original)
        p=result['pockets'][0];self.assertEqual(p['end_x'],100)
        self.assertIn('no_closed_return_claim',p['exit_support']);self.assertFalse(result['side_metric_geometry_used'])
        road['left_pockets']=result['pockets'];self.assertEqual(lane_at(road,50,6.5),-1)
        self.assertEqual(lane_at(road,50,3.5),0);self.assertIsNone(lane_at(road,110,6.5))

    def test_no_pocket_from_single_colour_or_unsupported_ending(self):
        for failure in ['yellow','inner','junction','single_frame']:
            road,evidence=fixture()
            if failure=='yellow':evidence['points']=[p for p in evidence['points'] if p['kind']!='yellow']
            if failure=='inner':evidence['points']=[p for p in evidence['points'] if p['y']!=5.2]
            if failure=='junction':road['junctions']=[]
            if failure=='single_frame':
                for p in evidence['points']:
                    if p['kind']=='yellow':p['frame_id']='one'
            self.assertFalse(fit_curved_pockets(evidence,road)['pockets'],failure)

    def test_station_admission_is_independent_of_world_orientation(self):
        road,evidence=fixture();road['left_pockets']=fit_curved_pockets(evidence,road)['pockets'];frame=frame_for(road)
        rows=[dict(time_s=i*.2,position_W=[*frame.world(40+i,6.5),0],bbox_xyxy=[100,100,300,300]) for i in range(20)]
        actor=dict(camera_id='front',samples=rows)
        self.assertEqual(actor_corridor_lane(actor,road['boundaries_y'],0,road)[0],-1)
        for i,r in enumerate(rows):r['position_W']=[*frame.world(70-i,6.5),0]
        self.assertIsNone(actor_corridor_lane(actor,road['boundaries_y'],0,road)[0])

    def test_unaccepted_side_projection_is_not_silently_fused(self):
        road,evidence=fixture();evidence['points'][0]['camera_id']='left_repeater'
        with self.assertRaises(ValueError):fit_curved_pockets(evidence,road)

    def test_inferred_queue_corridor_requires_rear_cue_and_ground_separators(self):
        road,evidence=fixture();evidence['points']=[p for p in evidence['points'] if p['kind']!='yellow'];frame=frame_for(road)
        rows=[dict(time_s=i*.2,position_W=[*frame.world(80-i*.1,6.5),0],quality=1.,support='detector_observation',frame_id=f'car-{i}') for i in range(16)]
        actor=dict(id='car',camera_id='front',samples=rows)
        cues=dict(car=dict(samples=[dict(frame_id=f'car-{i}',time_s=i*.2) for i in [5,8,11]]))
        original=copy.deepcopy(rows);geometry=fit_curved_pockets(evidence,road)
        fit=vehicle_supported_curved_pockets(evidence,road,[actor],cues,geometry)
        self.assertEqual(len(fit['pockets']),1);self.assertTrue(fit['occupancy_used']);self.assertEqual(rows,original)
        self.assertIn('inferred',fit['pockets'][0]['geometry_status'])
        self.assertFalse(vehicle_supported_curved_pockets(evidence,road,[actor],{},geometry)['pockets'])
        evidence['points']=[p for p in evidence['points'] if p['y']!=5.2]
        self.assertFalse(vehicle_supported_curved_pockets(evidence,road,[actor],cues,geometry)['pockets'])

    def test_ungated_paint_cannot_establish_curved_ground_geometry(self):
        road,evidence=fixture();evidence['ground_support']='legacy_unverified_ground_region'
        with self.assertRaises(ValueError):fit_curved_pockets(evidence,road)

    def test_a_few_jittered_contacts_cannot_admit_car_beyond_observed_median(self):
        road,evidence=fixture();road['left_boundary_role']='camera_observed_yellow_divider_estimated_geometry';frame=frame_for(road)
        rows=[dict(time_s=i*.2,position_W=[*frame.world(40+i,4.8 if i<3 else 6.5),0],bbox_xyxy=[100,100,300,300]) for i in range(12)]
        actor=dict(camera_id='front',samples=rows)
        self.assertIsNone(actor_corridor_lane(actor,road['boundaries_y'],0,road)[0])
        road['left_pockets']=fit_curved_pockets(evidence,road)['pockets']
        self.assertEqual(actor_corridor_lane(actor,road['boundaries_y'],0,road)[0],-1)
