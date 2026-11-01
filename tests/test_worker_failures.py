import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import unittest
from unittest.mock import Mock, patch
from PyQt6.QtWidgets import QApplication
from ui.loader_worker import ClipLoaderWorker
from core.scanner import LightMotionScanner
from core.exporter import ExportWorker


class WorkerFailureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_loader_reports_unexpected_error(self):
        worker = ClipLoaderWorker([None])
        errors, finished = [], []
        worker.error.connect(errors.append)
        worker.finished.connect(lambda *args: finished.append(args))
        worker.run()
        self.assertEqual(len(errors), 1)
        self.assertFalse(finished)

    def test_loader_releases_capture_after_metadata_error(self):
        cap = Mock()
        cap.get.side_effect = OSError('device removed')
        worker = ClipLoaderWorker([{'cams': {'front': 'front.mp4'}}])
        finished = []
        worker.finished.connect(lambda *args: finished.append(args))
        with patch('ui.loader_worker.os.path.exists', return_value=True), patch('ui.loader_worker.RealTeslaSEIDecoder'), patch('ui.loader_worker.cv2.VideoCapture', return_value=cap):
            worker.run()
        cap.release.assert_called_once()
        self.assertEqual(finished[0][2], 2160)

    def test_scanner_reports_error_and_releases_capture(self):
        cap = Mock()
        cap.get.side_effect = OSError('device removed')
        worker = LightMotionScanner([{'cams': {'front': 'front.mp4'}}])
        errors, finished = [], []
        worker.error.connect(errors.append)
        worker.events_found.connect(finished.append)
        with patch('core.scanner.os.path.exists', return_value=True), patch('core.scanner.cv2.VideoCapture', return_value=cap):
            worker.run()
        self.assertEqual(len(errors), 1)
        self.assertFalse(finished)
        cap.release.assert_called_once()

    def exporter(self, clips=None):
        worker = ExportWorker(clips or [], {}, 36, 36, 1, (960, 540), 'test-output.mp4')
        worker.error.connect(self.errors.append)
        worker.finished.connect(self.finished.append)
        return worker

    def setUp(self):
        self.errors, self.finished = [], []

    def test_ffmpeg_lookup_failure_does_not_raise_during_cleanup(self):
        worker = self.exporter()
        with patch('core.exporter.imageio_ffmpeg.get_ffmpeg_exe', side_effect=RuntimeError('missing ffmpeg')):
            worker.run()
        self.assertEqual(len(self.errors), 1)
        self.assertFalse(self.finished)

    def test_ffmpeg_nonzero_exit_is_failure(self):
        worker = self.exporter()
        proc = Mock()
        proc.wait.return_value = 1
        with patch('core.exporter.imageio_ffmpeg.get_ffmpeg_exe', return_value='ffmpeg.exe'), patch('core.exporter.os.path.exists', return_value=True), patch('core.exporter.subprocess.Popen', return_value=proc):
            worker.run()
        self.assertEqual(len(self.errors), 1)
        self.assertIn('code 1', self.errors[0])
        self.assertFalse(self.finished)

    def test_cancel_survives_broken_ffmpeg_pipe(self):
        worker = self.exporter()
        worker.stop()
        cancelled = []
        worker.cancelled.connect(lambda: cancelled.append(True))
        proc = Mock()
        proc.stdin.close.side_effect = BrokenPipeError('encoder closed')
        with patch('core.exporter.imageio_ffmpeg.get_ffmpeg_exe', return_value='ffmpeg.exe'), patch('core.exporter.os.path.exists', return_value=True), patch('core.exporter.os.remove'), patch('core.exporter.subprocess.Popen', return_value=proc):
            worker.run()
        self.assertEqual(cancelled, [True])
        self.assertFalse(self.errors)
        self.assertFalse(self.finished)
        proc.terminate.assert_called_once()
        proc.wait.assert_called_once()

    def test_decoder_failure_releases_video_and_stops_ffmpeg(self):
        worker = self.exporter([{'cams': {'front': 'front.mp4'}, 'start_f': 0, 'end_f': 0}])
        cap, proc = Mock(), Mock()
        with patch('core.exporter.imageio_ffmpeg.get_ffmpeg_exe', return_value='ffmpeg.exe'), patch('core.exporter.os.path.exists', return_value=True), patch('core.exporter.subprocess.Popen', return_value=proc), patch('core.exporter.cv2.VideoCapture', return_value=cap), patch('core.exporter.RealTeslaSEIDecoder', side_effect=OSError('read failed')):
            worker.run()
        self.assertEqual(len(self.errors), 1)
        self.assertFalse(self.finished)
        cap.release.assert_called_once()
        proc.kill.assert_called_once()
        proc.wait.assert_called_once()


if __name__ == '__main__':
    unittest.main()
