"""Internal multi-clip reconstruction boards; never marks quality accepted."""
import json,os,sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
os.environ['QT_QPA_PLATFORM']='offscreen'
from core.typography import configure_font_backend
configure_font_backend()
from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QFontDatabase
app=QApplication([]);QFontDatabase.addApplicationFont('C:/Windows/Fonts/malgun.ttf')
from core.themes import apply_theme
apply_theme('graphite',persist=False)
from core.driving_scene import DrivingRenderer
from core.scene_fusion import FusedSceneTimeline
from core.scene_analysis import CameraReader

root=Path(sys.argv[1]);variant=sys.argv[2] if len(sys.argv)>2 else 'road-sequences'
tag=sys.argv[3] if len(sys.argv)>3 else variant
out=root/('reconstruction-review-'+tag);out.mkdir(exist_ok=True)
cases=json.loads((root/'baseline-cases.json').read_text(encoding='utf-8'))
renderer=DrivingRenderer(960,640)
for case in cases:
    target=root/variant/case['id']
    if not (target/'manifest.json').exists():continue
    rows=[json.loads(s) for s in (target/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    manifest=json.loads((target/'manifest.json').read_text(encoding='utf-8'))
    timeline=FusedSceneTimeline(rows,manifest)
    i=len(rows)//2;t=rows[i]['time_s'];frame=renderer.render(timeline.scenes[i])
    tiles=[]
    for camera in ('left_repeater','front','right_repeater'):
        reader=CameraReader(Path(case['source']).parent/manifest['cameras'][camera]['file'])
        try:_,original=reader.read_at(t)
        finally:reader.close()
        tile=cv2.resize(original,(320,240));cv2.putText(tile,camera,(5,20),0,.5,(0,255,255),1);tiles.append(tile)
    board=np.vstack([frame,np.hstack(tiles)])
    cv2.putText(board,case['id'],(10,620),0,.65,(0,0,0),1)
    cv2.imencode('.jpg',board)[1].tofile(str(out/(case['id']+'.jpg')))
    print('Reviewed frame',case['id'],t,flush=True)
