import unittest
from core.scene_road_fit import retain_unmatched_observations


class UnmatchedRoadTests(unittest.TestCase):
    def test_stable_ego_boundary_does_not_erase_new_adjacent_lane(self):
        fitted=[{'points':[[0,0],[0,20]]}]
        observed=[{'points':[[0,5],[0,10],[0,15]]},
                  {'points':[[3,5],[3,10],[3,15]]}]
        result=retain_unmatched_observations(observed,fitted)
        self.assertEqual(len(result),1)
        self.assertEqual(result[0]['points'],[[3.,5.],[3.,10.],[3.,15.]])

    def test_branch_beyond_observed_fit_is_preserved_without_extrapolation(self):
        result=retain_unmatched_observations([{'points':[[0,5],[0,10],[1,15],[2,20]]}],
                                            [{'points':[[0,0],[0,10]]}])
        self.assertEqual(result[0]['points'],[[1.,15.],[2.,20.]])
