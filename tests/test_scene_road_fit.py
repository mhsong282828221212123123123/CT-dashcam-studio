import unittest
import numpy as np
from core.scene_road_fit import coherent_paths


class RoadFitTests(unittest.TestCase):
    def test_curvature_and_multiple_boundaries_survive_noisy_observations(self):
        rng=np.random.default_rng(8);segments=[]
        for offset in (-5.4,-1.8,1.8,5.4):
            for k in range(6):
                z=np.linspace(-8+k,35+k,40)
                x=offset+.003*z*z+rng.normal(0,.07,len(z))
                segments.append({'points':np.column_stack([x,z]).tolist()})
        paths=coherent_paths(segments)
        self.assertEqual(len(paths),4)
        for path,offset in zip(paths,(-5.4,-1.8,1.8,5.4)):
            p=np.asarray(path['points'])
            self.assertLess(np.max(abs(p[:,0]-offset-.003*p[:,1]**2)),.15)
            self.assertFalse(path['extrapolated'])

    def test_two_observed_boundaries_do_not_create_extra_lanes(self):
        segments=[{'points':[[x,z] for z in range(5,25)]} for x in (-1.8,1.8)]
        self.assertEqual(len(coherent_paths(segments)),2)
        self.assertEqual(coherent_paths([]),[])

    def test_no_paint_is_inserted_across_large_unobserved_gap(self):
        segments=[{'points':[[1,float(z)] for z in np.linspace(a,b,20)]}
                  for a,b in ((0,5),(15,20))]
        paths=coherent_paths(segments)
        self.assertEqual(len(paths),2)
        self.assertTrue(all(not 6.5<p[1]<13.5 for lane in paths for p in lane['points']))
