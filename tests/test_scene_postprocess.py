import unittest
from dataclasses import replace
import numpy as np
from core.driving_scene import Scene,Actor
from core.scene_fusion import CameraGeometry,corners,dimensions
from core.scene_postprocess import recover_positions


class PostprocessTests(unittest.TestCase):
    def test_box_overlap_alone_cannot_hide_wrong_ground_contact(self):
        args=self.fixture()
        for row in args[1][2:19]:
            box=row['cameras']['front']['objects'][0]['bbox'];box[1]-=box[3]*.25
        self.assertEqual(recover_positions(*args),0)

    def test_already_correct_ghost_is_not_counted_as_improved(self):
        args=self.fixture();args[0][10].actors=[Actor('one','car',2,18,tracking_state='predicted',geometry_kind='car')]
        self.assertEqual(recover_positions(*args),16)
        self.assertEqual(args[0][10].actors[0].tracking_state,'predicted')

    def test_later_stable_track_can_backfill_earlier_visible_detections(self):
        args=self.fixture()
        for i,scene in enumerate(args[0]):
            scene.actors=[Actor('one','car',2,20-i*.2,sources=('front:1',),geometry_kind='car')] if i>=10 else []
        self.assertEqual(recover_positions(*args),10)
        for i in range(10):
            self.assertAlmostEqual(args[0][i].actors[0].z,20-i*.2)
            self.assertEqual(args[-1][i]['postprocess'][0]['method'],'future_supported_backfill')

    def test_existing_ghost_is_refined_not_duplicated(self):
        args=self.fixture();args[0][10].actors=[Actor('one','car',2,18.3,tracking_state='predicted')]
        self.assertEqual(recover_positions(*args),17)
        self.assertEqual(len(args[0][10].actors),1)
        self.assertAlmostEqual(args[0][10].actors[0].z,18.)
        self.assertEqual(args[-1][10]['postprocess'][0]['prior_state'],'predicted')

    def fixture(self,camera='front'):
        geom=CameraGeometry();scenes=[];rows=[];poses=[];cameras=[];audits=[]
        for i in range(21):
            actor=Actor('one','car',2.,20-i*.2,sources=(camera+':1',),geometry_kind='car')
            box=geom.visible_bounds(corners(actor.x,actor.z,dimensions('car'),kind='car'))
            obj={'track_id':camera+':1','kind':'car','track_status':'confirmed','bbox':np.r_[box[:2],box[2:]-box[:2]].tolist()}
            scenes.append(Scene(i*.1,{'speed_kmh':7.2,'heading':0},actors=[actor] if i in (0,1,19,20) else []))
            rows.append({'cameras':{camera:{'available':True,'source_time_s':i*.1,'objects':[obj]}}})
            poses.append((np.array([0.,i*.2]),np.eye(2)));cameras.append({camera:geom});audits.append({})
        return scenes,rows,poses,cameras,audits

    def test_future_observations_recover_missing_positions_in_world_frame(self):
        args=self.fixture();count=recover_positions(*args)
        self.assertEqual(count,17)
        for i in range(2,19):
            a=args[0][i].actors[0]
            self.assertAlmostEqual(a.x,2.);self.assertAlmostEqual(a.z,20-i*.2)
            self.assertEqual(a.tracking_state,'reconstructed')
            self.assertGreater(args[-1][i]['postprocess'][0]['reprojection_iou'],.99)
        self.assertEqual(recover_positions(*args),0) # repeat pass cannot duplicate

    def test_image_disagreement_rejects_a_plausible_motion_bridge(self):
        args=self.fixture()
        for row in args[1][2:19]:row['cameras']['front']['objects'][0]['bbox'][0]+=400
        self.assertEqual(recover_positions(*args),0)

    def test_rear_alone_cannot_create_recovered_positions(self):
        self.assertEqual(recover_positions(*self.fixture('back')),0)

    def test_without_future_support_no_recovery(self):
        args=self.fixture();args[0][-1].actors=[]
        self.assertEqual(recover_positions(*args),0)

    def test_odometry_reset_disables_bridge(self):
        args=self.fixture();args[-1][10]['world_track_events']=[{'reason':'odometry unavailable; tracks reset'}]
        self.assertEqual(recover_positions(*args),0)

    def test_conflicting_vehicle_is_not_overwritten_or_duplicated(self):
        args=self.fixture()
        for i in range(2,19):args[0][i].actors=[Actor('other','car',2,20-i*.2)]
        self.assertEqual(recover_positions(*args),0)

    def test_abrupt_motion_change_is_not_smoothed_away(self):
        args=self.fixture();args[0][-1].actors[0].z+=10
        self.assertEqual(recover_positions(*args),0)
