import tempfile
import json
from pathlib import Path
import unittest
from ct_scene.catalog import inventory


class CatalogTests(unittest.TestCase):
    def test_manifest_exposure_groups_whole_date_without_claiming_unseen(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp);source=p/'source';legacy=p/'legacy'
            source.mkdir();legacy.mkdir()
            for prefix in ['2026-10-15_12-00-00','2026-10-15_13-00-00','2026-10-16_12-00-00']:
                (source/(prefix+'-front.mp4')).touch()
            (legacy/'manifest.json').write_text(json.dumps({'source':'2026-10-15_12-00-00-front.mp4'}))
            (legacy/'catalog.json').write_text(json.dumps({'inventory':['2026-10-16_12-00-00-front.mp4']}))
            inventory([source],legacy,p/'result')
            rows=[json.loads(s) for s in (p/'result/sources.jsonl').read_text().splitlines()]
            self.assertEqual([r['split'] for r in rows],['legacy_regression','legacy_regression','unassigned'])
            self.assertTrue(all(r['unseen_status']=='NOT_ESTABLISHED' for r in rows))
            self.assertTrue(all(r['candidate_copies'][0]['front']['sha256'] is None for r in rows))
