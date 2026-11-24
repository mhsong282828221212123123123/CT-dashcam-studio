import unittest
import numpy as np
from ct_scene.ego_turn_rotation import corrected_yaw
from ct_scene.background_rotation import dynamic_exclusion


class OwnTurnRotationTests(unittest.TestCase):
    def fixture(self):
        t=np.arange(0,15.01,.2);sei=-np.pi/2*np.clip((t-5)/5,0,1);visual=-np.pi/2*np.clip((t-4)/5,0,1)
        pairs={camera:[dict(start_s=float(a),end_s=float(b),accepted=True,visual_delta_deg=float(np.degrees(y-x))) for a,b,x,y in zip(t[:-1],t[1:],visual[:-1],visual[1:])] for camera in ['front','left_repeater','right_repeater']}
        return t,sei,visual,pairs

    def test_two_static_camera_rotations_correct_lag_but_keep_own_endpoints(self):
        t,sei,visual,pairs=self.fixture();result,review=corrected_yaw(t,sei,pairs)
        self.assertTrue(any(r['accepted'] for r in review));self.assertAlmostEqual(result[0],sei[0]);self.assertAlmostEqual(result[-1],sei[-1]);self.assertGreater(np.max(abs(result-sei)),.2)
        np.testing.assert_allclose(result,visual,atol=1e-10)

    def test_one_camera_cannot_change_ego_and_disagreement_abstains(self):
        t,sei,visual,pairs=self.fixture();result,_=corrected_yaw(t,sei,{'front':pairs['front']});np.testing.assert_array_equal(result,sei)
        for i,camera in enumerate(pairs):
            for r in pairs[camera]:r['visual_delta_deg']+=i*3
        result,review=corrected_yaw(t,sei,pairs);np.testing.assert_array_equal(result,sei)

    def test_lane_change_size_rotation_is_not_a_large_turn(self):
        t,sei,visual,pairs=self.fixture();sei*=.1;result,review=corrected_yaw(t,sei,pairs);np.testing.assert_array_equal(result,sei);self.assertFalse(any(r['accepted'] for r in review))

    def test_endpoint_drift_rejected_and_missing_pairs_never_extrapolated(self):
        t,sei,visual,pairs=self.fixture()
        for rows in pairs.values():
            for r in rows:r['visual_delta_deg']*=1.5
        result,review=corrected_yaw(t,sei,pairs);np.testing.assert_array_equal(result,sei)
        t,sei,visual,pairs=self.fixture()
        for camera in pairs:pairs[camera]=[r for r in pairs[camera] if r['end_s']<5]
        result,review=corrected_yaw(t,sei,pairs);np.testing.assert_array_equal(result,sei)

    def test_tracked_actors_and_repeater_mirror_are_masked(self):
        image=np.ones((960,1280),np.uint8)*200;rows=[dict(bbox_xyxy=[300,400,600,700])]
        mask=dynamic_exclusion(image,rows,'right_repeater');self.assertEqual(int(mask[500,450]),0);self.assertEqual(int(mask[200,1000]),0);self.assertEqual(int(mask[600,800]),255)
