import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock,patch
import numpy as np
from PyQt6.QtWidgets import QApplication
from core.scene_preview import render_ego
from ui.scene_dialog import SceneAnalysisDialog, SceneWorker


class SceneDialogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.app=QApplication.instance() or QApplication([])

    def test_lights_use_observed_flags_and_missing_data_has_distinct_display(self):
        off=render_ego({'speed_kmh':0})
        brake=render_ego({'speed_kmh':0,'brake_applied':True})
        left=render_ego({'speed_kmh':0,'left_blinker':True})
        right=render_ego({'speed_kmh':0,'right_blinker':True})
        self.assertGreater(np.count_nonzero(off!=brake),10)
        self.assertGreater(np.count_nonzero(off!=left),10)
        self.assertGreater(np.count_nonzero(left!=right),10)
        self.assertFalse(np.array_equal(render_ego(None),off))

    def test_close_requests_cancellation_without_destroying_running_thread(self):
        dialog=SceneAnalysisDialog(); dialog.show(); self.app.processEvents()
        worker=Mock(); worker.isRunning.return_value=True; dialog.worker=worker
        dialog.reject()
        self.assertTrue(dialog.isVisible()); worker.requestInterruption.assert_called_once()
        dialog.worker_finished()
        self.assertFalse(dialog.isVisible()); worker.deleteLater.assert_called_once()
        dialog.deleteLater()

    def test_worker_reports_model_failure(self):
        worker=SceneWorker('front.mp4','out','missing.onnx',640,0,1)
        errors=[]; results=[]; worker.failed.connect(errors.append); worker.result.connect(results.append)
        with patch('ui.scene_dialog.SceneDetector',side_effect=ValueError('bad model')): worker.run()
        self.assertEqual(errors,['bad model']); self.assertEqual(results,[])

    def test_export_button_starts_background_job_for_loaded_scene(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'front.mp4').write_bytes(b'original fixture')
            (root/'manifest.json').write_text(json.dumps({'sample_hz':10,'source_front_path':str(root/'front.mp4'),'cameras':{'front':{'file':'front.mp4'}}}))
            (root/'observations.jsonl').write_text('\n'.join(json.dumps({'time_s':i/10,'ego':None,'cameras':{}}) for i in range(2)))
            dialog=SceneAnalysisDialog()
            self.assertFalse(dialog.export_button.isEnabled())
            dialog.load_result(root);self.assertTrue(dialog.export_button.isEnabled())
            worker=Mock()
            with (patch('ui.scene_dialog.QFileDialog.getSaveFileName',return_value=(str(root/'export.mp4'),'MP4')),
                  patch('ui.scene_dialog.SceneExportWorker',return_value=worker) as factory):
                dialog.export_video()
                factory.assert_called_once_with(dialog.scene_timeline,str(root/'export.mp4'),dialog,manifest=dialog.scene_manifest,source_directory=str(root))
                worker.start.assert_called_once()
                self.assertFalse(dialog.export_button.isEnabled())
                self.assertTrue(dialog.cancel_button.isEnabled())
                dialog.cancel_analysis();worker.requestInterruption.assert_called_once()
                dialog.worker_finished()
            self.assertTrue(dialog.export_button.isEnabled())
            dialog.view_mode.setCurrentIndex(1);self.assertFalse(dialog.export_button.isEnabled())
            dialog.close()

    def test_result_playback_and_unavailable_telemetry(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'manifest.json').write_text(json.dumps({'sample_hz':5}))
            (root/'observations.jsonl').write_text('\n'.join(json.dumps({'time_s':i/5,'ego':None,'cameras':{}}) for i in range(2)))
            dialog=SceneAnalysisDialog(); dialog.load_result(root)
            self.assertEqual(dialog.slider.maximum(),1)
            self.assertFalse(dialog.ego.pixmap().isNull())
            dialog.toggle_play(); self.assertTrue(dialog.timer.isActive())
            dialog.clock=Mock(); dialog.clock.isValid.return_value=True; dialog.clock.elapsed.return_value=200
            dialog.next_frame(); self.assertEqual(dialog.slider.value(),1)
            dialog.next_frame(); self.assertFalse(dialog.timer.isActive())
            dialog.close()
