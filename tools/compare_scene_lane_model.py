"""Save camera-space lane hypotheses for visual review, not acceptance scores."""
import json,sys,time
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_lane_model import LaneModel
from core.scene_features import detect_lane_segments

root=Path(sys.argv[1]);viewport=sys.argv[2] if len(sys.argv)>2 else 'full'
out=root/('lane-model-review-'+viewport);out.mkdir(exist_ok=True)
model=LaneModel('../scene-models/ufldv2/ufldv2.onnx')
cases=json.loads((root/'baseline-cases.json').read_text(encoding='utf-8'))
reports=[];tiles=[]
for case in cases:
    # Only development scenes inform implementation choices.
    if case['split']!='development':continue
    cap=cv2.VideoCapture(case['source']);cap.set(cv2.CAP_PROP_POS_MSEC,case['review_time_s']*1000)
    ok,frame=cap.read();cap.release()
    if not ok:continue
    start=time.perf_counter();lanes=model.detect(frame,viewport);elapsed=time.perf_counter()-start
    old=detect_lane_segments(frame)
    panels=[]
    for label,segments in [('BASELINE',old),('UFLDv2 HYPOTHESES',lanes)]:
        panel=frame.copy()
        for segment in segments:
            cv2.polylines(panel,[np.round(segment['points']).astype(np.int32)],False,(0,255,255),4)
        panel=cv2.resize(panel,(512,384));cv2.putText(panel,label,(10,25),0,.55,(0,255,255),2)
        panels.append(panel)
    tile=np.concatenate(panels,axis=1)
    cv2.putText(tile,case['id'],(10,370),0,.6,(0,255,255),2)
    cv2.imencode('.jpg',tile)[1].tofile(str(out/(case['id']+'.jpg')))
    tiles.append(tile)
    reports.append(dict(id=case['id'],lanes=lanes,baseline_count=len(old),seconds=elapsed))
for i in range(0,len(tiles),3):
    cv2.imencode('.jpg',np.concatenate(tiles[i:i+3],axis=0))[1].tofile(str(out/f'sheet-{i//3}.jpg'))
(out/'predictions.json').write_text(json.dumps(dict(providers=model.providers,cases=reports),indent=2),encoding='utf-8')
print('Development frames',len(reports),'providers',model.providers,flush=True)
