"""Real UI worker rejects an incomplete source before costly model inference."""
import os
os.environ['QT_QPA_PLATFORM']='offscreen'
import json,sys
from pathlib import Path
from ct_scene.review_app import ReviewWindow,ROOT
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

app=QApplication([]);window=ReviewWindow();window.show()
assert window.load('runs/practical-display-oct22-full-001');original=window.result
target=ROOT/'runs/preflight-ui-negative-001';assert not target.exists()
window.source.setText('E:/tesla/a3/TeslaCam/SavedClips/2026-08-20_07-39-45/2026-08-20_07-30-00-front.mp4')
window.output.setText(str(target));window.backend.setCurrentIndex(window.backend.findData('cpu'))
def start():
    window.analyze();assert window.process is not None
    def done(code,status):
        try:
            state=json.loads((target/'pipeline.json').read_text())
            assert code!=0 and state['status']=='failed' and window.result==original
            assert state['config']['backend']=='cpu' and state['config']['osm_sha256']
            assert not (target/'source').exists() and not (target/'detections').exists()
            assert window.process is None and window.start.isEnabled() and not window.cancel.isEnabled()
            assert '원본/주행 데이터' in window.log.toPlainText()
            result=dict(incomplete_source_rejected=True,no_image_extraction_or_inference=True,
                        cpu_selection_forwarded=True,local_map_auto_selected=True,previous_result_preserved=True,controls_restored=True)
            (target/'ui-review.json').write_text(json.dumps(result,indent=2));window.grab().save(str(target/'ui.png'))
            print(json.dumps(result),flush=True);window.player.stop();app.exit(0)
        except Exception as error:print(repr(error),flush=True);app.exit(1)
    window.process.finished.connect(done)
QTimer.singleShot(200,start)
QTimer.singleShot(30000,lambda:(window.stop(),app.exit(2)))
sys.exit(app.exec())
