"""Real Qt editor integration; inspect analysis dispatch without launching inference."""
import argparse,os,sys,json,hashlib
from pathlib import Path
from unittest.mock import patch
p=argparse.ArgumentParser();p.add_argument('--editor',type=Path,required=True);p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--front',default='E:/tesla/a6/RecentClips/2026-11-04_17-30-35-front.mp4');a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False);os.environ['QT_QPA_PLATFORM']='offscreen';sys.path.insert(0,str(a.editor.resolve()))
from PyQt6.QtWidgets import QApplication,QFileDialog,QMessageBox
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QCloseEvent
import ui.main_window as mw
mw.UpdateCheckWorker.start=lambda self:None
app=QApplication([]);window=mw.CTDashcamStudio();window.show();window.active_clip_list=[dict(cams=dict(front=a.front))];window.current_active_clip_idx=0;window.open_scene_v2();review=window.driving_scene_panel.review
import ct_scene.review_app as ra
expected=(a.editor/'CT-scene-rebuild').resolve();checks={}
checks['checkout_engine_source_loaded']=Path(ra.__file__).resolve()==expected/'src/ct_scene/review_app.py'
checks['embedded_in_editor']=review.parent() is window.driving_scene_panel
checks['selected_clip_transferred']=review.source.text()==a.front
window.open_scene_v2();checks['single_tab_reused']=window.scene_tabs.count()==2
checks['editing_tab_retained']=window.scene_tabs.tabText(0)=='영상 편집'
assert review.load(str(a.scene.resolve()))
report=json.loads((a.scene/'report.json').read_text());expected_fail=report.get('collision_constraints',{}).get('after',{}).get('gate')=='FAIL'
checks['stored_failure_surfaced']=('검증 실패' in review.status.text())==expected_fail
completed=False

def done(frame):
    global completed
    if completed or not frame.isValid():return
    completed=True
    try:
        review.player.pause();checks['video_decoded']=True
        checks['full_video_duration']=abs(review.player.duration()/1000-report['duration_s'])<.15
        export=a.output/'export.mp4'
        with patch.object(QFileDialog,'getSaveFileName',return_value=(str(export),'MP4 (*.mp4)')):review.save_video()
        sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
        checks['export_exact']=sha(export)==sha(a.scene/'surrounding-world.mp4')
        captured=[]
        with patch.object(ra.QProcess,'start',lambda self,program,args:captured.append((program,args))):review.analyze()
        assert captured and review.process is not None
        process=review.process;program,args=captured[0]
        checks['isolated_analysis_python']=Path(program).resolve()==Path('G:/3d/CT-scene-rebuild/.venv/Scripts/python.exe').resolve()
        checks['analysis_front_passed']=args[args.index('--front')+1]==a.front
        checks['isolated_analysis_source']=Path(process.processEnvironment().value('PYTHONPATH')).resolve()==expected/'src'
        checks['analysis_config_from_checkout']=Path(args[args.index('--prototype-config')+1]).resolve()==expected/'configs/surrounding-prototype.json'
        checks['cuda_excluded']='cuda' not in args
        event=QCloseEvent()
        with patch.object(QMessageBox,'information',return_value=None):window.closeEvent(event)
        checks['editor_close_protects_analysis']=not event.isAccepted()
        review.finished(1,None);checks['analysis_controls_restored']=review.start.isEnabled() and not review.cancel.isEnabled()
        checks['gpu_runtime_not_loaded_in_editor']='torch' not in sys.modules and 'onnxruntime' not in sys.modules
        window.grab().save(str(a.output/'editor.png'))
        result=dict(checks=checks,passed=all(checks.values()),scope='Actual embedded editor, Qt decode and exact video export; command dispatch inspected without duplicate inference')
        (a.output/'review.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
        review.player.stop();window.close();app.exit(0 if result['passed'] else 1)
    except Exception as e:
        print(repr(e),flush=True);review.process=None;review.player.stop();window.close();app.exit(1)
review.video.videoSink().videoFrameChanged.connect(done)
QTimer.singleShot(100,review.player.play)
QTimer.singleShot(30000,lambda:app.exit(2))
sys.exit(app.exec())
