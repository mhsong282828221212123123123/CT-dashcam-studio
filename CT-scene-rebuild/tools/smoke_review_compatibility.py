"""Play current/replay/older results and check only compatible actions enable."""
import os
os.environ['QT_QPA_PLATFORM']='offscreen'
import json,sys
from pathlib import Path
from ct_scene.review_app import ReviewWindow
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

app=QApplication([]);window=ReviewWindow();window.show();out=Path('runs/review-compatibility-smoke-001');out.mkdir(exist_ok=False)
cases=[('runs/practical-fresh-aug25-pipeline-001',True,True),
       ('runs/frozen-replay-oct22-full-001',True,False),
       ('runs/scope-handoff-fourteen-005/branch_aug28',False,True)]
results=[];frames=[]
window.video.videoSink().videoFrameChanged.connect(lambda f:frames.append(f.isValid()))
def next_case():
    if len(results)==len(cases):
        previous=window.result
        assert window.load('runs/preflight-ui-negative-001') is False
        assert window.result==previous
        (out/'review.json').write_text(json.dumps(dict(cases=results,invalid_input_preserves_result=True),indent=2))
        window.grab().save(str(out/'window.png'));print(json.dumps(results),flush=True);window.player.stop();app.exit(0);return
    path,replay,manual=cases[len(results)];frames.clear()
    assert window.load(path)
    assert window.replay.isEnabled()==replay and window.manual.isEnabled()==manual
    window.player.play()
    def check():
        try:
            assert any(frames) and window.player.duration()>30000
            results.append(dict(path=path,replay_enabled=replay,manual_enabled=manual,valid_frames=sum(frames)))
            next_case()
        except Exception as error:print(repr(error),flush=True);app.exit(1)
    QTimer.singleShot(900,check)
QTimer.singleShot(200,next_case)
QTimer.singleShot(15000,lambda:app.exit(2))
sys.exit(app.exec())
