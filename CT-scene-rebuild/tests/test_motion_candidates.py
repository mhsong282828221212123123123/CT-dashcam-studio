import unittest
import numpy as np
import cv2
import tempfile
import json
from pathlib import Path
from ct_scene.motion_candidates import estimate_pair


class MotionCandidateTests(unittest.TestCase):
    def scene(self,translation):
        rng=np.random.default_rng(41)
        points=np.column_stack([rng.uniform(-4,4,300),rng.uniform(-3,3,300),rng.uniform(8,35,300)])
        R,_=cv2.Rodrigues(np.array([.005,.02,-.003]))
        moved=points@R.T+translation
        a=points[:,:2]/points[:,2:]*1200+[640,480]
        b=moved[:,:2]/moved[:,2:]*1200+[640,480]
        lens={'fx':1200,'fy':1200,'cx':640,'cy':480,'coefficients_k1_k2_p1_p2_k3':[0]*5,'status':'synthetic_known_only'}
        return np.column_stack([a,b]),lens,R

    def test_known_rotation_and_translation_direction(self):
        matches,lens,R=self.scene(np.array([.3,.01,.1]))
        result=estimate_pair(matches,lens)
        self.assertEqual(result['status'],'conditional_candidate',result)
        np.testing.assert_allclose(result['R_Cnext_Cprevious'],R,atol=1e-3)
        t=np.array(result['translation_direction_Cnext']);expected=np.array([.3,.01,.1]);expected/=np.linalg.norm(expected)
        self.assertGreater(t@expected,.99)
        self.assertEqual(result['translation_units'],'unit_vector_only_no_metric_scale')

    def test_pure_rotation_does_not_claim_translation(self):
        matches,lens,_=self.scene(np.zeros(3))
        result=estimate_pair(matches,lens)
        self.assertEqual(result['status'],'rejected')

    def test_unknown_resolution_cannot_use_a_lens_profile(self):
        from ct_scene.artifacts import publish
        from ct_scene.motion_candidates import run
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            publish(root/'features',stage='static_feature_candidates',payloads={'report.json':b'{}'},
                    inputs=[],config={},code_revision='test',runtime={})
            (root/'lens.json').write_text(json.dumps({'model':'pinhole_radial_tangential','resolution':[1280,960]}))
            with self.assertRaises(ValueError):run(root/'features',root/'lens.json',root/'result')
