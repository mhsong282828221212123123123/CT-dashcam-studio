"""Exercise recent-result opening, real Qt decoding, export and source protection."""
import os
os.environ['QT_QPA_PLATFORM']='offscreen'
import hashlib,json,sys
from pathlib import Path
from ct_scene.review_app import ReviewWindow
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication,QFileDialog

app=QApplication([]);window=ReviewWindow();window.show();frames=[]
import argparse
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
out=args.output;out.mkdir(exist_ok=False)
target_scene=Path('runs/practical-display-oct22-full-001').resolve()
index=next(i for i in range(window.recent.count()) if Path(window.recent.itemData(i))==target_scene)
window.recent.setCurrentIndex(index);window.load_recent()
assert window.result==target_scene and window.export.isEnabled()
window.video.videoSink().videoFrameChanged.connect(lambda f:frames.append(f.isValid()))
def start_playback():
    window.seek(31000);window.player.play()
QTimer.singleShot(500,start_playback)

def finish():
    try:
        target=out/'export.mp4'
        QFileDialog.getSaveFileName=lambda *a,**kw:(str(target),'MP4 (*.mp4)')
        window.save_video()
        parity=hashlib.sha256(target.read_bytes()).digest()==hashlib.sha256((target_scene/'surrounding-world.mp4').read_bytes()).digest()
        protected=target_scene/'report.json';original=protected.read_bytes()
        QFileDialog.getSaveFileName=lambda *a,**kw:(str(protected),'MP4 (*.mp4)')
        window.save_video();assert protected.read_bytes()==original
        result=dict(recent_result_opened=True,valid_video_frames=sum(frames),duration_ms=window.player.duration(),position_ms=window.player.position(),export_bytes_exact=parity,result_directory_protected=True,scope='offscreen Qt media execution; not manual desktop review')
        window.grab().save(str(out/'window.png'))
        (out/'review.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
        assert result['valid_video_frames']>0 and result['duration_ms']>59000 and result['position_ms']>31000 and parity
        window.player.stop();app.exit(0)
    except Exception as error:
        print(repr(error),flush=True);app.exit(1)

QTimer.singleShot(2500,finish)
sys.exit(app.exec())
