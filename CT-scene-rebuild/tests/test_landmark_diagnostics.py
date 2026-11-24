import unittest
import numpy as np
from ct_scene.landmark_diagnostics import planar_check


class LandmarkDiagnosticsTests(unittest.TestCase):
    def test_check_points_do_not_participate_in_fit(self):
        a=np.array([[10,10],[100,10],[10,100],[100,100],[50,50],[70,60]],float)
        b=a+[20,30];fit=[True]*4+[False]*2
        first=planar_check(a,b,fit)
        b[-1]+=[80,0]
        second=planar_check(a,b,fit)
        np.testing.assert_allclose(first['H'],second['H'])
        self.assertGreater(second['check_p95_px'],70)

    def test_collinear_controls_and_no_check_rejected(self):
        a=np.array([[10,10],[20,20],[30,30],[40,40],[50,50]],float)
        with self.assertRaises(ValueError):planar_check(a,a,[True]*4+[False])
        with self.assertRaises(ValueError):planar_check(a,a,[True]*5)
