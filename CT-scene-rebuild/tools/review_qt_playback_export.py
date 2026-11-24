"""Offscreen review UI opens/decodes the actual MP4 and exports identical bytes."""
import os
os.environ['QT_QPA_PLATFORM']='offscreen'
import argparse,json,sys
from pathlib import Path
from ct_scene import review_app
from ct_scene.ingest import file_hash
from ct_scene.artifacts import canonical
ap=argparse.ArgumentParser();ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--expect-source-collapse',action='store_true');a=ap.parse_args();a.output.mkdir(exist_ok=False)
app=review_app.QApplication([]);window=review_app.ReviewWindow();window.show();loaded=window.load(a.scene);status={}
def finish():
    target=a.output/'ui-export.mp4';review_app.QFileDialog.getSaveFileName=staticmethod(lambda *args,**kwargs:(str(target),'MP4 (*.mp4)'));window.save_video();window.grab().save(str(a.output/'ui.png'))
    if not window.video.image.isNull():window.video.image.save(str(a.output/'decoded-frame.png'))
    meta=json.loads(target.with_suffix('.export.json').read_text()) if target.exists() else {}
    checks=dict(result_loaded=bool(loaded),decoded_image=not window.video.image.isNull(),player_duration_positive=window.player.duration()>0,export_bytes_identical=target.exists() and file_hash(target)==file_hash(a.scene/'surrounding-world.mp4'),source_manifest_preserved=meta.get('source_artifact_id')==window.manifest['artifact_id'])
    if a.expect_source_collapse:checks['world_collapse_warning_shown']=window.presence_screen['gate']=='REJECTED' and '차량 누락' in window.status.text() and meta.get('quality_gate')=='WORLD_COVERAGE_REVIEW_REQUIRED'
    status.update(checks=checks,passed=all(checks.values()),player_error=window.player.errorString());(a.output/'review.json').write_bytes(canonical(status));window.close();app.quit()
window.player.play();review_app.QTimer.singleShot(4000,finish);app.exec();print(json.dumps(status));sys.exit(0 if status.get('passed') else 2)
