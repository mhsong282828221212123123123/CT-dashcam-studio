import copy,unittest
from ct_scene.side_paint_geometry import complete_near_camera_edges


class NearCameraEdgeTests(unittest.TestCase):
    road={'boundaries_y':[9.4,5.85,2.28,-1.29]}

    def fixture(self):
        rows=[dict(frame_id=str(i),time_s=i*.2,position_W=[i*4.,.5,0],support='detector_observation',paint_lane_supported=True,paint_lane_index=2) for i in range(6)]
        actor=dict(id='near',samples=rows,side_paint_geometry=dict(model_type='two_stripe_affine',lane=2))
        extra=dict(frame_id='edge',time_s=1.4,position_W=[999,999,0],bbox_xyxy=[800,300,1000,550],image_width=1000,image_height=960,support='detector_observation')
        return actor,extra

    def test_large_clipped_body_keeps_same_identity_and_lane(self):
        actor,extra=self.fixture();original=copy.deepcopy(extra)
        n=complete_near_camera_edges(actor,self.road,[extra])
        self.assertEqual(n,1);self.assertEqual(extra,original)
        point=actor['samples'][-1]['position_W'];self.assertAlmostEqual(point[0],28.);self.assertEqual(point[1],.5)
        self.assertEqual(actor['samples'][-1]['support'],'offline_camera_edge_prediction')
        self.assertFalse(actor['near_camera_edge_completion']['new_identity_created'])

    def test_small_distant_or_long_gap_is_not_extended(self):
        for change in [dict(bbox_xyxy=[900,400,1000,440]),dict(time_s=4.),dict(bbox_xyxy=[500,300,800,550])]:
            actor,extra=self.fixture();extra.update(change)
            self.assertEqual(complete_near_camera_edges(actor,self.road,[extra]),0)

    def test_lateral_manoeuvre_cannot_be_frozen_to_old_lane(self):
        actor,extra=self.fixture()
        for row in actor['samples']:row['position_W'][1]+=.8*row['time_s']
        self.assertEqual(complete_near_camera_edges(actor,self.road,[extra]),0)

    def test_no_paint_calibration_means_no_new_geometry(self):
        actor,extra=self.fixture();actor.pop('side_paint_geometry')
        self.assertEqual(complete_near_camera_edges(actor,self.road,[extra]),0)
