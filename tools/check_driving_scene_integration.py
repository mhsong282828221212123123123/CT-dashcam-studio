"""Exercise the editor's actual embedded scene tab and isolated analysis command."""
import os
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
import argparse
import hashlib
import json
import sys
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt6.QtCore import QProcess, QTimer
from PyQt6.QtGui import QCloseEvent
from PyQt6.QtWidgets import QApplication, QFileDialog, QMessageBox
from ui.main_window import CTDashcamStudio, UpdateCheckWorker

p = argparse.ArgumentParser()
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=False)
root = Path(__file__).resolve().parents[2] / 'CT-scene-rebuild'
scene = root / 'runs/local-extents-aug20-full-001'
front = Path('E:/tesla/a3/TeslaCam/SavedClips/2026-08-20_07-39-45/2026-08-20_07-34-02-front.mp4')
app = QApplication([])
with patch.object(UpdateCheckWorker, 'start'):
    window = CTDashcamStudio()
window.active_clip_list = [{'cams': {'front': str(front)}}]
window.current_active_clip_idx = 0
window.scene_v2_action.trigger()
panel = window.driving_scene_panel
review = panel.review
assert review.source.text() == str(front)
assert window.centralWidget() is window.scene_tabs
assert not review.isWindow()
window.scene_v2_action.trigger()
assert panel is window.driving_scene_panel and window.scene_tabs.count() == 2
assert review.load(scene)
window.show()
review.player.play()


def finish():
    try:
        checks = dict(embedded_in_editor=review.window() is window,
                      selected_clip_transferred=review.source.text() == str(front),
                      single_tab_reused=window.scene_tabs.count() == 2,
                      video_decoded=not review.video.image.isNull(),
                      full_video_duration=review.player.duration() > 59000)
        target = a.output / 'export.mp4'
        with patch.object(QFileDialog, 'getSaveFileName', return_value=(str(target), 'MP4')):
            review.save_video()
        checks['export_exact'] = hashlib.sha256(target.read_bytes()).digest() == hashlib.sha256((scene / 'surrounding-world.mp4').read_bytes()).digest()
        calls = []
        with patch.object(QProcess, 'start', lambda self, program, args: calls.append((program, args))):
            review.analyze()
        assert len(calls) == 1
        program, args = calls[0]
        checks['isolated_analysis_python'] = Path(program) == root / '.venv/Scripts/python.exe'
        checks['analysis_front_passed'] = args[args.index('--front') + 1] == str(front)
        env = review.process.processEnvironment()
        checks['isolated_analysis_source'] = env.value('PYTHONPATH') == str(root / 'src')
        event = QCloseEvent()
        with patch.object(QMessageBox, 'information', return_value=QMessageBox.StandardButton.Ok):
            window.closeEvent(event)
        checks['editor_close_protects_analysis'] = not event.isAccepted()
        review.finished(-1, None)
        assert review.load(scene)
        checks['analysis_controls_restored'] = review.start.isEnabled() and not review.cancel.isEnabled()
        checks['gpu_runtime_not_loaded_in_editor'] = 'onnxruntime' not in sys.modules
        window.scene_tabs.setCurrentIndex(0)
        window.scene_v2_action.trigger()
        checks['editing_tab_retained'] = window.scene_tabs.tabText(0) == '영상 편집'
        window.grab().save(str(a.output / 'editor.png'))
        result = dict(checks=checks, passed=all(checks.values()),
                      scope='Actual offscreen editor tab, video and export; analysis command inspected without repeating inference')
        (a.output / 'review.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps(result), flush=True)
        window.close()
        app.exit(0 if result['passed'] else 1)
    except Exception:
        import traceback
        traceback.print_exc()
        review.process = None
        window.close()
        app.exit(1)


QTimer.singleShot(4000, finish)
sys.exit(app.exec())
