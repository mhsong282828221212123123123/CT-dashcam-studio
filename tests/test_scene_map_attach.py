import json,tempfile,unittest
from pathlib import Path
from tools.attach_scene_map import attach


class MapAttachTests(unittest.TestCase):
    def test_attaching_map_preserves_source_and_observation_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source=root/'source';source.mkdir()
            (source/'manifest.json').write_text('{"cameras": {}}')
            content=b'{"time_s": 1}\n'
            (source/'observations.jsonl').write_bytes(content)
            path=root/'map.json';path.write_text('{"elements": []}')
            output=root/'new';attach(source,path,output)
            self.assertEqual((output/'observations.jsonl').read_bytes(),content)
            self.assertNotIn('road_map',json.loads((source/'manifest.json').read_text()))
            self.assertIn('road_map',json.loads((output/'manifest.json').read_text()))
            with self.assertRaises(FileExistsError):attach(source,path,output)
