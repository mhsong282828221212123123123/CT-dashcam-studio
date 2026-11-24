import json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from ct_scene.pipeline import write_state,output_lock,analysis_revision


class PipelineStateTests(unittest.TestCase):
    def test_ui_only_change_keeps_cache_but_analysis_change_invalidates(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'geometry.py').write_text('geometry v1');(root/'review_app.py').write_text('UI v1')
            initial=analysis_revision(root)
            (root/'review_app.py').write_text('UI v2');(root/'scene_replay.py').write_text('display settings')
            self.assertEqual(analysis_revision(root),initial)
            (root/'geometry.py').write_text('geometry v2')
            self.assertNotEqual(analysis_revision(root),initial)

    def test_transient_windows_replace_lock_keeps_old_state_until_commit(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'pipeline.json';write_state(path,{'status':'old'})
            original=Path.replace;calls=[]
            def replace(source,target):
                calls.append(1)
                if len(calls)==1:
                    self.assertEqual(json.loads(path.read_text()),{'status':'old'})
                    raise PermissionError('temporary reader')
                return original(source,target)
            with patch.object(Path,'replace',replace),patch('ct_scene.pipeline.time.sleep'):
                write_state(path,{'status':'new'})
            self.assertEqual(json.loads(path.read_text()),{'status':'new'})

    def test_persistent_lock_never_publishes_partial_state(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'pipeline.json';write_state(path,{'status':'old'})
            with patch.object(Path,'replace',side_effect=PermissionError('locked')),patch('ct_scene.pipeline.time.sleep'):
                with self.assertRaises(PermissionError):write_state(path,{'status':'new'})
            self.assertEqual(json.loads(path.read_text()),{'status':'old'})

    def test_worker_lock_excludes_second_worker_and_releases_on_error(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'worker.lock'
            with output_lock(path):
                with self.assertRaises(ValueError):
                    with output_lock(path):pass
            with self.assertRaises(RuntimeError):
                with output_lock(path):raise RuntimeError('cancelled stage')
            with output_lock(path):pass
