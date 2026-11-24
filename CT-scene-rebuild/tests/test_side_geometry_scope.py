import unittest
from ct_scene.surrounding import side_fragment_has_geometry


class SideGeometryScopeTests(unittest.TestCase):
    def test_tall_border_sliver_cannot_be_a_full_height_vehicle(self):
        rows=[dict(bbox_xyxy=[1130,135,1279,948]) for _ in range(65)]
        self.assertFalse(side_fragment_has_geometry(rows,1280,960))

    def test_later_full_body_preserves_earlier_fragment_track(self):
        rows=[dict(bbox_xyxy=[1130,135,1279,948]) for _ in range(65)]
        rows.extend(dict(bbox_xyxy=[300,350,900,800]) for _ in range(3))
        self.assertTrue(side_fragment_has_geometry(rows,1280,960))
