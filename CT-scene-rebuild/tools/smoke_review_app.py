"""Qt event-loop media readback and export parity; no source modification."""
import os
os.environ['QT_QPA_PLATFORM']='offscreen'
import hashlib,json,sys
from pathlib import Path
from ct_scene.review_app import ReviewWindow
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication,QFileDialog

app=QApplication([]);window=ReviewWindow();window.show();frames=[]
assert window.load('runs/oct02-curved-road-004')
out=Path('runs/review-app-smoke-001');out.mkdir(exist_ok=True)
window.video.videoSink().videoFrameChanged.connect(lambda f:frames.append(f.isValid()))
window.player.play()
def finish():
    try:
        duration=window.player.duration();position=window.player.position()
        window.grab().save(str(out/'window.png'))
        target=out/'export-parity.mp4'
        QFileDialog.getSaveFileName=lambda *a,**kw:(str(target),'MP4 (*.mp4)')
        window.save_video();original=window.result/'surrounding-world.mp4'
        equal=hashlib.sha256(target.read_bytes()).digest()==hashlib.sha256(original.read_bytes()).digest()
        result=dict(opened_result=True,duration_ms=duration,position_ms=position,valid_video_frames=sum(frames),export_bytes_exact=equal,ui='offscreen_media_smoke_not_manual_usability_review')
        (out/'review.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result),flush=True)
        assert duration>59000 and position>0 and any(frames) and equal
        app.exit(0)
    except Exception as error:
        print(str(error),flush=True);app.exit(1)
QTimer.singleShot(2500,finish)
sys.exit(app.exec())
