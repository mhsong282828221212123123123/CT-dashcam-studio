import copy,unittest
from ct_scene.side_paint_geometry import fit,apply,extend_calibrated_tail


class SidePaintTests(unittest.TestCase):
    def range_records(self):
        return [dict(frame_id=str(i),time_s=i*.2,kind='far',relative_station_m=-3-i*.2,
                     measured_d=2.28-(-.16)*(-3-i*.2)-.6+(i%3-1)*.005,target_d=2.28) for i in range(60)]

    def test_range_separated_single_stripe_can_correct_yaw_like_bias(self):
        r=fit(self.range_records());self.assertTrue(r['accepted'])
        self.assertEqual(r['model_type'],'relative_station_shear')
        self.assertAlmostEqual(r['range_shear'],-.16,places=2)
        self.assertFalse(r['vehicle_positions_used'])

    def test_single_stripe_without_range_or_later_agreement_abstains(self):
        rs=self.range_records()
        for r in rs:r['relative_station_m']=-5
        self.assertFalse(fit(rs)['accepted'])
        rs=self.range_records()
        for r in rs:
            if r['time_s']>=8:r['measured_d']+=1.5
        self.assertFalse(fit(rs)['accepted'])

    def tail_actor(self):
        return dict(global_id='linked',end_s=1.,calibrated_tail_camera_ids=['right_repeater'],samples=[dict(time_s=i*.2,position_W=[i*5.,.5,0],yaw_rad=0,frame_id=str(i),camera_id='right_repeater') for i in range(6)])

    def test_short_calibrated_tail_coasts_without_changing_lane(self):
        a=self.tail_actor();r=extend_calibrated_tail(a,{'boundaries_y':[9.4,5.85,2.28,-1.29]},1.5)
        self.assertTrue(r);self.assertEqual(a['end_s'],1.5)
        self.assertEqual(a['samples'][-1]['position_W'][1],.5)
        self.assertEqual(a['samples'][-1]['support'],'offline_camera_tail_prediction')

    def test_long_or_unconfirmed_camera_tail_is_not_filled(self):
        road={'boundaries_y':[9.4,5.85,2.28,-1.29]};a=self.tail_actor()
        self.assertFalse(extend_calibrated_tail(a,road,2.))
        a.pop('calibrated_tail_camera_ids');self.assertFalse(extend_calibrated_tail(a,road,1.5))

    def occluded_pair(self,overlap=True):
        rear=dict(id='truck',model='truck',camera_id='left_repeater',samples=[])
        front=dict(id='car',model='sedan',camera_id='left_repeater',samples=[])
        for i in range(30):
            t=i*.2;late=i>=18
            rear['samples'].append(dict(frame_id=str(i),time_s=t,position_W=[i*4.,12,0],side_ground_contact_W=[i*4.,7.5 if late else 6.9,0],bbox_xyxy=[100,430,180,520]))
            front['samples'].append(dict(frame_id=str(i),time_s=t,position_W=[i*4.+8,5.5,0],side_ground_contact_W=[i*4.+8,5.5,0],bbox_xyxy=[(115 if late else 135) if overlap else 250,470,220 if overlap else 340,540]))
        return rear,front

    def test_hidden_truck_contact_uses_short_visible_occlusion_motion(self):
        rear,front=self.occluded_pair();road=dict(boundaries_y=[9.4,5.85,2.28,-1.29])
        apply([rear,front],road,{'left_repeater':fit(self.records())})
        self.assertEqual(rear['side_paint_geometry']['lane'],1)
        self.assertTrue(rear['side_paint_geometry']['occluded_contact_lane_supported'])
        self.assertTrue(any(r.get('support')=='offline_occluded_contact_reconstruction' for r in rear['samples']))
        self.assertEqual(len(rear['samples']),30)

    def test_foreground_lane_does_not_change_unoccluded_neighbor(self):
        rear,front=self.occluded_pair(overlap=False);road=dict(boundaries_y=[9.4,5.85,2.28,-1.29])
        apply([rear,front],road,{'left_repeater':fit(self.records())})
        self.assertEqual(rear['side_paint_geometry']['lane'],0)
        self.assertFalse(rear['side_paint_geometry'].get('occluded_contact_lane_supported',False))

    def records(self):
        return [dict(frame_id=str(i),time_s=i*.2,kind=kind,measured_d=x+(i%3-1)*.02,target_d=y)
                for i in range(40) for kind,x,y in [('near',3.35,2.28),('far',6.63,5.85)]]

    def test_two_independent_paint_stripes_calibrate_without_cars(self):
        rows=self.records();before=copy.deepcopy(rows);r=fit(rows)
        self.assertTrue(r['accepted']);self.assertFalse(r['vehicle_positions_used']);self.assertFalse(r['metric_calibration'])
        self.assertLess(r['metrics']['held_out_p90_m'],.1);self.assertEqual(rows,before)

    def test_one_line_or_later_paint_disagreement_abstains(self):
        self.assertFalse(fit([r for r in self.records() if r['kind']=='near'])['accepted'])
        rows=self.records()
        for r in rows:
            if r['time_s']>5:r['target_d']+=1.5
        self.assertFalse(fit(rows)['accepted'])

    def test_lane_comes_from_corrected_ground_and_paint_not_height(self):
        actor=dict(id='side',camera_id='left_repeater',samples=[dict(time_s=i*.2,position_W=[i*5.,12,0],side_ground_contact_W=[i*5.,6.2,0]) for i in range(20)])
        road=dict(boundaries_y=[9.4,5.85,2.28,-1.29]);r=apply([actor],road,{'left_repeater':fit(self.records())})
        self.assertEqual(r[0]['lane'],1)
        self.assertTrue(all(s['paint_lane_index']==1 and s['position_W'][1]<5.85 for s in actor['samples']))

    def test_missing_paint_does_not_pull_outside_car_inside(self):
        actor=dict(id='outside',camera_id='left_repeater',samples=[dict(time_s=i*.2,position_W=[i*5.,30,0],side_ground_contact_W=[i*5.,30,0]) for i in range(20)])
        before=copy.deepcopy(actor)
        self.assertEqual(apply([actor],dict(boundaries_y=[9.4,5.85,2.28,-1.29]),{'left_repeater':fit(self.records())}),[])
        self.assertEqual(actor,before)
