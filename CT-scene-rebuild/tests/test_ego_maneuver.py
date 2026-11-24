import copy
import unittest
import numpy as np
from ct_scene.ego_maneuver import windows,compensate_actors,camera_view
from ct_scene.surrounding import trajectory_at


class EgoManeuverTests(unittest.TestCase):
    def setUp(self):
        self.road=dict(boundaries_y=[5.25,1.75,-1.75,-5.25],heading_evidence={'slope':0})
        self.interval=dict(start_s=2.,end_s=6.,maneuver_start_s=3.,completion_s=5.,reference_lateral_m=0.,reference_yaw_rad=0.)

    def actor(self):
        rows=[dict(time_s=i*.2,position_W=[i,0,0],yaw_rad=0.,support='detector_observation') for i in range(41)]
        for r in rows:
            if 2<=r['time_s']<=6:r['position_W'][1]=np.sin((r['time_s']-2)*np.pi/2)
        return dict(global_id='car',samples=rows,lane_motion_evidence_by_camera=[])

    def test_only_maneuver_lateral_geometry_is_changed(self):
        actor=self.actor();before=copy.deepcopy(actor);review=compensate_actors([actor],[self.interval],self.road)
        self.assertGreater(review[0]['modified_samples'],0)
        for a,b in zip(actor['samples'],before['samples']):
            self.assertEqual(a['position_W'][0],b['position_W'][0])
            if not 2<=a['time_s']<=6:self.assertEqual(a,b)
            else:self.assertAlmostEqual(a['position_W'][1],0)
        np.testing.assert_allclose(trajectory_at(actor,6.05,restore_maneuver=True),trajectory_at(before,6.05))

    def test_independent_other_lane_change_is_not_flattened(self):
        actor=self.actor();actor['lane_motion_evidence_by_camera']=[{'events':[{'start_s':3,'end_s':4}]}];before=copy.deepcopy(actor)
        review=compensate_actors([actor],[self.interval],self.road)
        self.assertEqual(actor,before);self.assertEqual(review[0]['reason'],'independent_actor_lane_change_evidence')

    def test_disagreeing_endpoint_lanes_are_not_forced_together(self):
        actor=self.actor()
        for r in actor['samples']:
            if r['time_s']>6:r['position_W'][1]=3.5
        before=copy.deepcopy(actor);review=compensate_actors([actor],[self.interval],self.road)
        self.assertEqual(actor,before);self.assertEqual(review[0]['modified_samples'],0)

    def test_camera_is_road_anchored_only_during_maneuver(self):
        p=np.array([30.,2.,0.]);origin,yaw,weight=camera_view(4,p,.15,[self.interval],self.road)
        self.assertEqual(weight,1);self.assertEqual(origin[1],0);self.assertEqual(yaw,0)
        np.testing.assert_array_equal(p,[30,2,0])
        for t in [1,2,6,7]:
            origin,yaw,weight=camera_view(t,p,.15,[self.interval],self.road)
            np.testing.assert_array_equal(origin,p);self.assertEqual(yaw,.15);self.assertEqual(weight,0)

    def test_long_curve_event_is_not_an_isolated_lane_change(self):
        t=np.arange(31.);p=np.c_[t,np.zeros(len(t)),np.zeros(len(t))]
        event=dict(start_s=2,completion_s=24,from_lane=1,to_lane=0)
        self.assertEqual(windows([event],t,p,np.zeros(len(t)),self.road),[])
        event['completion_s']=5
        self.assertEqual(len(windows([event],t,p,np.zeros(len(t)),self.road)),1)


if __name__=='__main__':unittest.main()
