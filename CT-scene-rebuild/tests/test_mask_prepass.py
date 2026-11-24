import unittest
import numpy as np
from ct_scene.mask_prepass import prepare_image,decode_output,static_mask


class MaskPrepassTests(unittest.TestCase):
    def test_letterbox_raw_restore_nms_and_static_class_filter(self):
        image=np.zeros((960,1280,3),np.uint8)
        tensor,mapping=prepare_image(image)
        self.assertEqual(tensor.shape,(1,3,640,640))
        raw=np.zeros((1,84,3),np.float32)
        raw[0,:4,:]=np.array([[320,320,50],[320,320,50],[200,200,20],[100,100,20]])
        raw[0,6,0]=.9;raw[0,6,1]=.7;raw[0,13,2]=.95
        rows=decode_output(raw,mapping,1280,960)
        self.assertEqual(len(rows),1)
        self.assertEqual(rows[0]['class_id'],2)
        np.testing.assert_allclose(rows[0]['bbox_xyxy'],[440,380,840,580])
        mask=static_mask(1280,960,rows)
        self.assertEqual(mask[400,500],0)
        self.assertEqual(mask[0,0],255)

    def test_no_dynamic_detections_is_unmasked_not_fake_road(self):
        mask=static_mask(100,80,[])
        self.assertTrue(np.all(mask==255))
        _,mapping=prepare_image(np.zeros((80,100,3),np.uint8))
        with self.assertRaises(ValueError):decode_output(np.zeros((1,85,3)),mapping,100,80)
