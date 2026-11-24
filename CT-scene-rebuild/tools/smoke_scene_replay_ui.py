"""Run the screen-settings dialog and real replay worker through Qt."""
import os
os.environ['QT_QPA_PLATFORM']='offscreen'
import json,sys
from pathlib import Path
from ct_scene.review_app import ReviewWindow
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication,QDialog,QDoubleSpinBox

app=QApplication([]);window=ReviewWindow();window.show()
assert window.load('runs/practical-display-oct22-full-001')
original=window.result;output=Path('runs/replay-ui-smoke-001');output.mkdir(exist_ok=False)
def accept_settings():
    dialog=app.activeModalWidget()
    assert isinstance(dialog,QDialog)
    spins=dialog.findChildren(QDoubleSpinBox);spins[0].setValue(1.4);spins[1].setValue(5.)
    dialog.accept()

def start():
    QTimer.singleShot(100,accept_settings)
    window.replay_copy()
    assert window.process is not None and not window.replay.isEnabled()
    target=window.pending_result
    def done(code,status):
        try:
            assert code==0 and window.result==target and window.replay.isEnabled()
            assert window.manual.isEnabled() is False
            assert (original/'render-states.jsonl').read_bytes()==(target/'render-states.jsonl').read_bytes()
            report=json.loads((target/'report.json').read_text())
            assert report['replay_style']['camera_zoom']==1.4
            assert abs(sum(report['replay_style']['road_rgb'])/3-.05)<1e-6
            result=dict(dialog_worker_completed=True,result=str(target),geometry_bytes_unchanged=True,
                        settings_applied=True,replay_button_restored=True,manual_score_separation=True)
            (output/'review.json').write_text(json.dumps(result,indent=2));window.grab().save(str(output/'window.png'))
            print(json.dumps(result),flush=True);window.player.stop();app.exit(0)
        except Exception as error:print(repr(error),flush=True);app.exit(1)
    window.process.finished.connect(done)

QTimer.singleShot(200,start)
QTimer.singleShot(90000,lambda:(window.stop(),app.exit(2)))
sys.exit(app.exec())
