import unittest
import numpy as np
from core.scene_road_model import RoadEvidenceDetector


class FakeSession:
    def run(self,names,inputs):
        self.input=next(iter(inputs.values()))
        road=np.zeros((1,2,640,640),np.float32);road[0,0]=1
        road[0,1,320:]=2
        lane=np.full((1,1,640,640),.49,np.float32);lane[:,:,:,320:]=.51
        return [road,lane]


class RoadModelTests(unittest.TestCase):
    def test_v2_uses_rgb_zero_one_and_single_channel_lane_probability(self):
        model=RoadEvidenceDetector.__new__(RoadEvidenceDetector)
        model.v2=True;model.input='images';model.session=FakeSession()
        image=np.full((960,1280,3),[0,128,255],np.uint8)
        model.detect(image)
        blob=model.session.input
        np.testing.assert_allclose(blob[0,:,320,320],[1,128/255,0],atol=1e-6)
        road,paint=model.last_masks
        self.assertEqual(road.shape,(960,1280));self.assertEqual(paint.shape,(960,1280))
        self.assertEqual(road[100,100],0);self.assertEqual(road[800,100],1)
        self.assertEqual(paint[800,100],0);self.assertEqual(paint[800,1100],1)
