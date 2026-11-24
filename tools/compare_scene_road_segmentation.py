"""YOLOP road/paint evidence in original camera coordinates; development only."""
import json,sys,time
from pathlib import Path
import cv2,numpy as np
runtime=Path(__file__).resolve().parents[2]/'scene-runtime-directml'
sys.path.insert(0,str(runtime))
import onnxruntime as ort
root=Path(sys.argv[1]);out=root/'road-segmentation-review';out.mkdir(exist_ok=True)
options=ort.SessionOptions();options.enable_mem_pattern=False
options.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
session=ort.InferenceSession('../scene-models/yolop/yolop-640-640.onnx',sess_options=options,
                            providers=['DmlExecutionProvider','CPUExecutionProvider'])
session.disable_fallback()
cases=json.loads((root/'baseline-cases.json').read_text(encoding='utf-8'))
tiles=[];reports=[]
for case in cases:
    if case['split']!='development':continue
    cap=cv2.VideoCapture(case['source']);cap.set(cv2.CAP_PROP_POS_MSEC,case['review_time_s']*1000)
    ok,frame=cap.read();cap.release()
    if not ok:continue
    h,w=frame.shape[:2];scale=min(640/w,640/h);rw,rh=round(w*scale),round(h*scale)
    px,py=(640-rw)//2,(640-rh)//2
    canvas=np.full((640,640,3),114,np.uint8)
    canvas[py:py+rh,px:px+rw]=cv2.resize(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB),(rw,rh))
    blob=(canvas.astype(np.float32)/255.-np.array([.485,.456,.406],np.float32))/np.array([.229,.224,.225],np.float32)
    start=time.perf_counter()
    road,lane=session.run(['drive_area_seg','lane_line_seg'],{'images':np.ascontiguousarray(blob.transpose(2,0,1)[None])})
    masks=[]
    for output in (road,lane):
        scores=np.stack([cv2.resize(channel[py:py+rh,px:px+rw],(w,h)) for channel in output[0]])
        masks.append(scores.argmax(0).astype(np.uint8))
    road,lane=masks
    overlay=frame.copy()
    overlay[road==1]=(overlay[road==1]*.65+np.array([0,180,0])*.35).astype(np.uint8)
    overlay[lane==1]=(0,0,255)
    tile=np.concatenate([cv2.resize(frame,(512,384)),cv2.resize(overlay,(512,384))],axis=1)
    cv2.putText(tile,case['id'],(10,370),0,.6,(0,255,255),2)
    cv2.imencode('.jpg',tile)[1].tofile(str(out/(case['id']+'.jpg')))
    np.savez_compressed(out/(case['id']+'.npz'),road=road,lane=lane)
    reports.append(dict(id=case['id'],seconds=time.perf_counter()-start,road_pixels=int(road.sum()),lane_pixels=int(lane.sum())))
    tiles.append(tile)
for i in range(0,len(tiles),3):
    cv2.imencode('.jpg',np.concatenate(tiles[i:i+3]))[1].tofile(str(out/f'sheet-{i//3}.jpg'))
(out/'report.json').write_text(json.dumps(dict(providers=session.get_providers(),cases=reports),indent=2),encoding='utf-8')
print('Compared',len(reports),'development frames',flush=True)
