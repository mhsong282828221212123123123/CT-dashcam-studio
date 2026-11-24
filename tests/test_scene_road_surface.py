import unittest
import numpy as np
from core.scene_road_surface import consensus_surface


def quad(left,right,near,far):
    return [[left,near],[right,near],[right,far],[left,far]]


class RoadSurfaceTests(unittest.TestCase):
    def test_outlier_edge_does_not_expand_entire_road(self):
        result=consensus_surface([[quad(-4,4,0,10)],[quad(-4.1,4.1,0,10)],
                                  [quad(-12,15,0,10)]])
        points=np.array(result).reshape(-1,2)
        self.assertGreater(len(result),0)
        self.assertGreaterEqual(points[:,0].min(),-4.1)
        self.assertLessEqual(points[:,0].max(),4.1)

    def test_no_longitudinal_gap_fill(self):
        result=consensus_surface([[quad(-4,4,0,4),quad(-4,4,8,12)]]*3)
        self.assertTrue(result)
        self.assertFalse(any(min(p[1] for p in q)<6<max(p[1] for p in q) for q in result))

    def test_disconnected_roads_are_not_joined(self):
        result=consensus_surface([[quad(-8,-2,0,10),quad(2,8,0,10)]]*3)
        self.assertTrue(result)
        self.assertFalse(any(min(p[0] for p in q)<0<max(p[0] for p in q) for q in result))

    def test_curve_is_retained_without_fixed_width_grid(self):
        frames=[[[z*.2-3,z],[z*.2+3,z],[(z+1)*.2+3,z+1],[(z+1)*.2-3,z+1]] for z in range(10)]
        result=consensus_surface([frames]*3)
        self.assertTrue(result)
        for q in result:
            for x,z in q:self.assertAlmostEqual(abs(x-z*.2),3)
