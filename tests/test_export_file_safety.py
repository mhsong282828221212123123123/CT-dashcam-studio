import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from PyQt6.QtWidgets import QApplication
from core.exporter import ExportWorker


class ExportFileSafetyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_existing_output_survives_all_unsuccessful_paths(self):
        for outcome in ('cancel', 'late_cancel', 'encoder_error', 'start_error', 'replace_error', 'success'):
            with self.subTest(outcome=outcome), tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / '기존 영상.mp4'
                output.write_bytes(b'previous export')
                worker = ExportWorker([], {}, 36, 36, 1, (960, 540), str(output))
                errors, finished, cancelled = [], [], []
                worker.error.connect(errors.append)
                worker.finished.connect(finished.append)
                worker.cancelled.connect(lambda: cancelled.append(True))
                proc = Mock()
                proc.wait.return_value = 1 if outcome == 'encoder_error' else 0
                if outcome == 'cancel':
                    worker.stop()
                if outcome == 'late_cancel':
                    def finish_encoding():
                        worker.stop()
                        return 0
                    proc.wait.side_effect = finish_encoding

                def start_encoder(command, **kwargs):
                    target = Path(command[-1])
                    self.assertEqual(target.parent, output.parent)
                    self.assertNotEqual(target, output)
                    self.assertEqual(output.read_bytes(), b'previous export')
                    if outcome == 'start_error':
                        raise OSError('could not start encoder')
                    target.write_bytes(b'new export')
                    return proc

                real_replace = os.replace
                def commit(source, destination):
                    if outcome == 'replace_error':
                        raise PermissionError('destination locked')
                    real_replace(source, destination)

                with patch('core.exporter.imageio_ffmpeg.get_ffmpeg_exe', return_value=__file__), patch('core.exporter.subprocess.Popen', side_effect=start_encoder), patch('core.exporter.os.replace', side_effect=commit):
                    worker.run()
                self.assertEqual(output.read_bytes(), b'new export' if outcome == 'success' else b'previous export')
                self.assertEqual(list(Path(directory).iterdir()), [output])
                self.assertEqual(finished, [str(output)] if outcome == 'success' else [])
                self.assertEqual(bool(cancelled), outcome in ('cancel', 'late_cancel'))
                self.assertEqual(bool(errors), outcome in ('encoder_error', 'start_error', 'replace_error'))
