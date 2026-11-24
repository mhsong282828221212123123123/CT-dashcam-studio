"""Check Qt cancellation returns controls after the worker actually stops."""
import os,sys,json
os.environ['QT_QPA_PLATFORM']='offscreen'
from pathlib import Path
from ct_scene.review_app import ReviewWindow
from PyQt6.QtCore import QProcess,QTimer
from PyQt6.QtWidgets import QApplication
app=QApplication([]);window=ReviewWindow();process=QProcess(window);window.process=process
process.finished.connect(window.finished);window.start.setEnabled(False);window.cancel.setEnabled(True)
process.start(sys.executable,['-c','import time; time.sleep(20)'])
QTimer.singleShot(200,window.stop)
def check():
    ok=window.process is None and window.start.isEnabled() and not window.cancel.isEnabled()
    out=Path('runs/review-app-cancel-smoke-001');out.mkdir(exist_ok=True)
    (out/'review.json').write_text(json.dumps(dict(worker_stopped_and_controls_restored=ok)),encoding='utf-8')
    print('Cancellation restored controls:',ok,flush=True);app.exit(0 if ok else 1)
QTimer.singleShot(3800,check);sys.exit(app.exec())
