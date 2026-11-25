import tempfile,unittest
from pathlib import Path
from ct_scene.runtime_paths import runtime_python,scene_models_root

class RuntimePathTests(unittest.TestCase):
    def test_nested_source_reuses_runtime_without_selecting_older_source(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'editor/CT-scene-rebuild';source.mkdir(parents=True)
            runtime=root/'CT-scene-rebuild/.venv/Scripts/python.exe';runtime.parent.mkdir(parents=True);runtime.touch()
            self.assertEqual(runtime_python(source),runtime)
            models=root/'scene-models';(models/'model-review').mkdir(parents=True);(models/'yolo11x.onnx').touch();(models/'model-review/yolopv2-roads.onnx').touch()
            self.assertEqual(scene_models_root(source),models)
            local=source/'.venv/Scripts/python.exe';local.parent.mkdir(parents=True);local.touch()
            self.assertEqual(runtime_python(source),local)

    def test_missing_runtime_has_a_clear_error(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(FileNotFoundError):runtime_python(Path(d)/'editor/engine')
