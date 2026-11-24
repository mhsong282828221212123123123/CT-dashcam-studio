import copy,unittest
import numpy as np
from ct_scene.pillar_bridges import build
from ct_scene.road_frame import coordinates


class PillarBridgeTests(unittest.TestCase):
    def fixture(self):
        t=np.arange(0,10.01,.2);ego=np.c_[t*22,t*0,t*0]
        road=dict(reference_line=dict(points_W_xy=[[-20,0],[400,0]],station_origin_m=-20,supported_extent_m=[-20,400]),boundaries_y=[5.25,1.75,-1.75,-5.25],junctions=[])
        def actor(key,camera,stamps):
            rows=[dict(time_s=float(x),frame_id=key+str(i),camera_id=camera,support='detector_observation',position_W=[24.5*x-7,3.5,0],yaw_rad=0.,bbox_xyxy=[950,200,1250,500] if camera!='front' else [0,200,200,500] if i<3 else [100,200,400,500]) for i,x in enumerate(stamps)]
            return dict(id=key,global_id=key,model='sedan',appearance=[1.,0.],camera_id=camera,frozen_lead=False,samples=rows,start_s=float(stamps[0]),end_s=float(stamps[-1]))
        a=actor('side','left_repeater',np.arange(0,2.01,.2));b=actor('front','front',np.arange(6,8.41,.2))
        return [a,b],road,t,ego,np.zeros(len(t))

    def test_future_reappearance_bounds_long_pillar_path_without_merging_ids(self):
        actors,road,t,ego,yaw=self.fixture();old=copy.deepcopy(actors);coasts,review=build(actors,road,t,ego,yaw)
        self.assertEqual(actors,old);self.assertEqual(len(review),1);self.assertFalse(review[0]['identity_merged'])
        path=coasts['side']['samples'];self.assertAlmostEqual(path[0]['time_s'],2.1);self.assertAlmostEqual(path[-1]['time_s'],5.9)
        stations=coordinates(road,[r['position_W'] for r in path])[:,0];self.assertTrue(np.all(np.diff(stations)>0));self.assertTrue(np.all(stations>=42.-1e-6));self.assertTrue(np.all(stations<=140.+1e-6))

    def test_two_similar_future_cars_are_ambiguous_and_not_joined(self):
        actors,road,t,ego,yaw=self.fixture();other=copy.deepcopy(actors[1]);other['id']=other['global_id']='other';actors.append(other)
        self.assertEqual(build(actors,road,t,ego,yaw)[0],{})

    def test_wrong_lane_impossible_motion_or_different_appearance_is_rejected(self):
        for kind in ['lane','motion','appearance']:
            actors,road,t,ego,yaw=self.fixture()
            if kind=='lane':
                for r in actors[1]['samples']:r['position_W'][1]=0
            if kind=='motion':
                for r in actors[1]['samples']:r['position_W'][0]+=100
            if kind=='appearance':actors[1]['appearance']=[0.,1.]
            self.assertEqual(build(actors,road,t,ego,yaw)[0],{},kind)

    def test_no_future_observation_cannot_create_a_long_bridge(self):
        actors,road,t,ego,yaw=self.fixture();self.assertEqual(build(actors[:1],road,t,ego,yaw)[0],{})

    def test_front_to_side_is_supported_when_ego_passes_the_car(self):
        actors,road,t,ego,yaw=self.fixture()
        for index,actor in enumerate(actors):
            actor['camera_id']='front' if index==0 else 'left_repeater'
            for i,row in enumerate(actor['samples']):
                row['camera_id']=actor['camera_id'];row['position_W'][0]=20*row['time_s']+8
                row['bbox_xyxy']=[30,200,300,500] if index==0 else [1200,200,1280,500] if i<3 else [950,200,1250,500]
        coasts,review=build(actors,road,t,ego,yaw)
        self.assertIn('side',coasts);self.assertEqual(review[0]['from_camera'],'front');self.assertEqual(review[0]['to_camera'],'left_repeater')
