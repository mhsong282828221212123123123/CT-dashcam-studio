import copy,unittest
import numpy as np
from ct_scene.blindspot_continuity import build,at


class BlindspotTests(unittest.TestCase):
    def fixture(self):
        t=np.arange(0,6.01,.2);ego=np.c_[t*22,t*0,t*0]
        road=dict(reference_line=dict(points_W_xy=[[-20,0],[300,0]],station_origin_m=-20,supported_extent_m=[-20,300]),boundaries_y=[5.25,1.75,-1.75,-5.25],junctions=[])
        rows=[dict(time_s=float(x),frame_id=str(i),camera_id='left_repeater',support='detector_observation',position_W=[24.5*x-7,3.5,0],yaw_rad=0.,bbox_xyxy=[950,200,1250,500]) for i,x in enumerate(t[:11])]
        actor=dict(global_id='car',model='sedan',frozen_lead=False,samples=rows,start_s=0,end_s=2.)
        return actor,road,t,ego,np.zeros(len(t))

    def test_coast_is_bounded_ghost_and_preserves_observations(self):
        actor,road,t,ego,yaw=self.fixture();before=copy.deepcopy(actor)
        coasts,review=build([actor],road,t,ego,yaw)
        self.assertIn('car',coasts);self.assertEqual(actor,before)
        self.assertLessEqual(review[0]['end_s']-2.,2.5)
        state=at(coasts,2.3)[0];self.assertTrue(state['ghost']);self.assertFalse(state['source_visible']);self.assertFalse(review[0]['identity_merged'])
        self.assertEqual(at(coasts,5.),[])
        self.assertTrue(np.isfinite(state['position_W']).all())

    def test_new_real_observation_stops_prediction_without_merging_ids(self):
        actor,road,t,ego,yaw=self.fixture();other=copy.deepcopy(actor);other['global_id']='other';other['samples']=[dict(actor['samples'][-1],time_s=3.,position_W=[66.5,3.5,0])]
        coasts,review=build([actor,other],road,t,ego,yaw)
        self.assertEqual(review[0]['stop_reason'],'nearby_real_observation');self.assertLess(review[0]['end_s'],3.)
        self.assertEqual(other['global_id'],'other')

    def test_turning_unclipped_distant_or_departing_car_does_not_coast(self):
        for kind in ['turn','unclipped','distant','departing']:
            actor,road,t,ego,yaw=self.fixture()
            if kind=='turn':yaw=t*.2
            if kind=='unclipped':
                for r in actor['samples']:r['bbox_xyxy']=[100,200,500,500]
            if kind=='distant':
                for r in actor['samples']:r['position_W'][1]=15
            if kind=='departing':
                for r in actor['samples']:r['position_W'][0]-=5*r['time_s']
            self.assertEqual(build([actor],road,t,ego,yaw)[0],{},kind)

    def test_supported_junction_blocks_one_sided_coast(self):
        actor,road,t,ego,yaw=self.fixture();road['junctions']=[dict(road_station_range_m=[-20,300])]
        self.assertEqual(build([actor],road,t,ego,yaw)[0],{})

    def test_first_prediction_joins_the_actual_observation_coverage_end(self):
        actor,road,t,ego,yaw=self.fixture();coasts,_=build([actor],road,t,ego,yaw)
        start=coasts['car']['samples'][0]
        self.assertAlmostEqual(start['time_s'],2.1)
        np.testing.assert_allclose(start['position_W'],actor['samples'][-1]['position_W'])
        step=at(coasts,2.15)[0]
        self.assertLess(np.linalg.norm(step['position_W']-start['position_W'])/.05,30)
