"""Synthetic correction E2E; never a source-reconstruction accuracy claim."""
import os
os.environ['QT_QPA_PLATFORM']='offscreen'
import json
import argparse
from pathlib import Path
import cv2
import numpy as np
from ct_scene.artifacts import verify
from ct_scene.review_app import ReviewWindow
from PyQt6.QtWidgets import QApplication,QFileDialog

ap=argparse.ArgumentParser();ap.add_argument('--manual',type=Path,default=Path('runs/manual-review-integration-001'));ap.add_argument('--output',type=Path,default=Path('runs/manual-review-integration-check-001'));args=ap.parse_args()
original=Path('runs/oct18-camera-edge-continuation-001');manual=args.manual
out=args.output;out.mkdir(exist_ok=False)
sm=verify(original);mm=verify(manual)
load=lambda p:json.loads(p.read_text(encoding='utf-8'))
a=[json.loads(l) for l in (original/'render-states.jsonl').read_text().splitlines()]
b=[json.loads(l) for l in (manual/'render-states.jsonl').read_text().splitlines()]
assert len(a)==len(b)==850
changed=[]
for old,new in zip(a,b):
    assert old['camera_view']==new['camera_view'] and old['ego_lamps']==new['ego_lamps']
    assert [q['id'] for q in old['actors']]==[q['id'] for q in new['actors']]
    if not 0<old['time_s']<2:assert old==new
    if new.get('manual_corrections'):changed.append(new['time_s'])
target=b[20]['actors'][0];auto=a[20]['actors'][0]
assert np.allclose(np.array(target['position_W'])-auto['position_W'],[.5,.25,0])
assert np.isclose(target['yaw_rad']-auto['yaw_rad'],np.deg2rad(5))
assert verify(original)['artifact_id']==sm['artifact_id']
caps=[cv2.VideoCapture(str(p/'surrounding-world.mp4')) for p in [original,manual]]
counts=[int(c.get(cv2.CAP_PROP_FRAME_COUNT)) for c in caps];assert counts==[850,850]
images=[]
for c in caps:
    c.set(cv2.CAP_PROP_POS_MSEC,1000);ok,im=c.read();assert ok;images.append(im);c.release()
# Source strip survives re-encoding; exact byte parity is not claimed for H264.
mse=np.mean((images[0][720:960].astype(float)-images[1][720:960])**2)
psnr=float(10*np.log10(255**2/max(mse,1e-12)));assert psnr>30
cv2.imwrite(str(out/'manual-1s.png'),images[1])
app=QApplication([]);window=ReviewWindow();assert window.load(manual)
assert not window.manual.isEnabled() and '수동 교정본' in window.status.text()
export=out/'manual-export.mp4';QFileDialog.getSaveFileName=lambda *a,**k:(str(export),'MP4')
window.save_video();assert export.read_bytes()==(manual/'surrounding-world.mp4').read_bytes()
assert load(export.with_suffix('.export.json'))['manual_corrections'] is True
window.close()
result=dict(scope='synthetic correction integration only; not source accuracy',automatic_artifact_unchanged=True,
            original_actors_and_ego_preserved=True,frames=counts,modified_frames=len(changed),modified_time_range_s=[min(changed),max(changed)],
            manual_peak_offset_m=[.5,.25],manual_peak_yaw_deg=5,raw_four_camera_strip_psnr_db=psnr,
            derivative_ui_label_and_export_metadata=True,export_bytes_exact=True,
            automatic_quality_score_eligible=load(manual/'report.json')['automatic_quality_score_eligible'])
(out/'review.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result))
