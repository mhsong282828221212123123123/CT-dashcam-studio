import ast
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from ct_scene.artifacts import publish, verify, canonical
from ct_scene.geometry import Clock, transform, inverse, front_camera_to_body, project_pinhole


class ArtifactTests(unittest.TestCase):
    def write(self, dest, payloads=None, cancelled=lambda: False):
        return publish(dest, stage='test', payloads=payloads or {'states.json': b'[]'},
                       inputs=[], config={'rate': 10}, code_revision='test',
                       runtime={'python': 'test'}, cancelled=cancelled)

    def test_replay_and_tamper(self):
        with tempfile.TemporaryDirectory() as temp:
            dest = Path(temp)/'result'
            self.write(dest)
            self.assertEqual(verify(dest)['quality_gate'], 'NOT_EVALUATED')
            (dest/'states.json').write_bytes(b'[1]')
            with self.assertRaises(ValueError):
                verify(dest)

    def test_content_identity_excludes_run_name(self):
        with tempfile.TemporaryDirectory() as temp:
            a = self.write(Path(temp)/'a')
            b = self.write(Path(temp)/'b')
            self.assertEqual(a['artifact_id'], b['artifact_id'])

    def test_no_overwrite(self):
        with tempfile.TemporaryDirectory() as temp:
            dest = Path(temp)/'result'
            a = self.write(dest)
            with self.assertRaises(FileExistsError):
                self.write(dest, {'other': b'new'})
            self.assertEqual(verify(dest)['artifact_id'], a['artifact_id'])

    def test_cancel_does_not_publish(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(InterruptedError):
                self.write(Path(temp)/'result', cancelled=lambda: True)
            self.assertEqual(list(Path(temp).iterdir()), [])

    def test_escape_and_nonfinite_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            for name in ('../outside', '/outside', 'C:/outside', 'x\\y', 'artifact.json'):
                with self.assertRaises(ValueError):
                    self.write(Path(temp)/'result', {name: b'x'})
            self.assertEqual(list(Path(temp).iterdir()), [])
        with self.assertRaises(ValueError):
            canonical({'unknown': float('nan')})

    def test_manifest_tampering(self):
        with tempfile.TemporaryDirectory() as temp:
            dest = Path(temp)/'result'
            self.write(dest)
            path = dest/'artifact.json'
            m = json.loads(path.read_bytes())
            m['quality_gate'] = 'PASS'
            path.write_bytes(canonical(m))
            with self.assertRaises(ValueError):
                verify(dest)


class GeometryTests(unittest.TestCase):
    def test_axes_and_world_camera_roundtrip(self):
        T_B_C = front_camera_to_body()
        np.testing.assert_allclose(transform(T_B_C, [[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
                                   [[0, -1, 0], [0, 0, -1], [1, 0, 0]])
        angle = .63
        c, s = np.cos(angle), np.sin(angle)
        T_W_B = np.eye(4)
        T_W_B[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
        T_W_B[:3, 3] = [15, -3, 2]
        p = np.array([[1, 2, 10], [-3, 1, 20]])
        T_W_C = T_W_B @ T_B_C
        np.testing.assert_allclose(transform(inverse(T_W_C), transform(T_W_C, p)), p, atol=1e-12)

    def test_near_plane_and_projection(self):
        K = [[1000, 0, 640], [0, 1000, 480], [0, 0, 1]]
        pixels, valid = project_pinhole([[0, 0, 10], [1, -1, 10], [0, 0, -1]], K)
        np.testing.assert_allclose(pixels[:2], [[640, 480], [740, 380]])
        self.assertEqual(valid.tolist(), [True, True, False])
        self.assertTrue(np.isnan(pixels[2]).all())

    def test_reflected_transform_rejected(self):
        T = np.eye(4)
        T[0, 0] = -1
        with self.assertRaises(ValueError):
            inverse(T)

    def test_clock_sign_epoch_and_prior(self):
        clock = Clock(offset_s=.2, scale=1.001, camera_epoch_s=100, session_epoch_s=5)
        self.assertAlmostEqual(clock.to_session(110), 15.21)
        self.assertEqual(clock.status, 'unverified_prior')
        with self.assertRaises(ValueError):
            Clock(scale=0)

    def test_no_legacy_imports(self):
        root = Path(__file__).parents[1]/'src'
        for path in root.rglob('*.py'):
            for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
                if isinstance(node, ast.ImportFrom):
                    self.assertFalse((node.module or '').startswith(('core', 'ui', 'pickle')))
                elif isinstance(node, ast.Import):
                    for name in node.names:
                        self.assertFalse(name.name.startswith(('core', 'ui', 'pickle')))


if __name__ == '__main__':
    unittest.main()
