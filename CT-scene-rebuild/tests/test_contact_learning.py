import unittest
import numpy as np
from ct_scene.contact_learning import guard_splits,contact_target,ridge
from ct_scene.learned_contact import preprocess

class ContactLearningTests(unittest.TestCase):
    def test_same_recorded_date_cannot_be_training_and_test(self):
        records=[dict(split=s,session=d,source_image_sha256=str(i),annotation_status='visual_review_by_codex') for i,(s,d) in enumerate([('train','a'),('validation','b'),('test','c')])]
        guard_splits(records);records[-1]['session']='a'
        with self.assertRaises(ValueError):guard_splits(records)

    def test_identical_source_image_cannot_cross_split(self):
        records=[dict(split=s,session=str(i),source_image_sha256='same',annotation_status='visual_review_by_codex') for i,s in enumerate(['train','validation','test'])]
        with self.assertRaises(ValueError):guard_splits(records)

    def test_occluded_contact_does_not_become_a_training_target(self):
        self.assertIsNone(contact_target(dict(contact_tile_xy=None)))
        r=dict(contact_tile_xy=[30,50],tile_transform=dict(dx=10,dy=10,scale=2),crop_rect_xyxy=[100,200,140,260],bbox_xyxy=[100,200,120,240])
        np.testing.assert_allclose(contact_target(r),[.5,.5])

    def test_ridge_constant_features_remain_finite(self):
        w,b=ridge(np.ones((3,5)),np.array([[.1,.2],[.2,.3],[.3,.4]]),.1)
        np.testing.assert_allclose(w,0);np.testing.assert_allclose(b,[.2,.3])

    def test_cpu_encoder_preprocess_is_rgb_nchw(self):
        x=preprocess(np.ones((20,40,3),np.uint8)*128)
        self.assertEqual(x.shape,(1,3,224,224));self.assertEqual(x.dtype,np.float32);self.assertTrue(x.flags.c_contiguous)

    def test_marked_tires_must_match_contact_and_cannot_be_occluded(self):
        records=[dict(split=s,session=str(i),source_image_sha256=str(i),annotation_status='visual_review_by_codex') for i,s in enumerate(['train','validation','test'])]
        records[0].update(wheel_contacts_tile_xy=[[10,20],[30,40]],contact_tile_xy=[20,30],contact_visibility='visible_tire_pair')
        guard_splits(records)
        records[0]['contact_tile_xy']=[50,50]
        with self.assertRaises(ValueError):guard_splits(records)
        records[0]['contact_tile_xy']=[20,30];records[0]['contact_visibility']='occluded_by_rail'
        with self.assertRaises(ValueError):guard_splits(records)

    def test_outside_or_nonfinite_tire_markers_are_rejected(self):
        for pair in [[[10,20],[400,40]],[[10,20],[float('nan'),40]]]:
            records=[dict(split=s,session=str(i),source_image_sha256=str(i),annotation_status='visual_review_by_codex') for i,s in enumerate(['train','validation','test'])]
            records[0].update(wheel_contacts_tile_xy=pair,contact_tile_xy=[20,30])
            with self.assertRaises(ValueError):guard_splits(records)

    def test_neutral_review_status_accepts_gemini_without_claiming_codex_review(self):
        rows=[dict(split=s,session=str(i),source_image_sha256=str(i),annotation_status='visual_reviewed',reviewer='Gemini') for i,s in enumerate(['train','validation','test'])]
        guard_splits(rows)
        rows[0]['annotation_status']='unreviewed'
        with self.assertRaises(ValueError):guard_splits(rows)
