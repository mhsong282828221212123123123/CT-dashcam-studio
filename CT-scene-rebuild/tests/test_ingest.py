import tempfile
from pathlib import Path
import unittest
from ct_scene.ingest import nearest_index, discover


class IngestTests(unittest.TestCase):
    def test_variable_pts_not_average_fps(self):
        times = [0, .031, .067, .140, .171]
        self.assertEqual(nearest_index(times, .115), 3)
        self.assertEqual(nearest_index(times, -.2), 0)
        self.assertEqual(nearest_index(times, 1), 4)
        with self.assertRaises(ValueError):
            nearest_index([], .5)

    def test_missing_channels_remain_missing(self):
        with tempfile.TemporaryDirectory() as tmp:
            front = Path(tmp)/'clip-front.mp4'
            front.touch()
            (Path(tmp)/'clip-left_repeater.mp4').touch()
            (Path(tmp)/'other-back.mp4').touch()
            self.assertEqual(set(discover(front)), {'front', 'left_repeater'})
