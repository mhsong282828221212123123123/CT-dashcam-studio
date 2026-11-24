"""Same original frames, serial DirectML model comparison. Not 3D acceptance."""
import gc,json,subprocess,time,sys,hashlib
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import SceneDetector,CameraReader,iou
from core.scene_instance_model import InstanceSceneDetector
from core.scene_road_model import RoadEvidenceDetector

root=Path('../scene-results');out=root/'perception-model-review';out.mkdir(exist_ok=True)
models=Path('../scene-models');cases=json.loads((root/'quality-rebuild/fresh-validation/run-configuration.json').read_text())['cases']
frames=[]
for case in cases:
    reader=CameraReader(case['source'])
    try:
        for t in (5.,15.,25.):
            _,frame=reader.read_at(t)
            if frame is None:raise RuntimeError('Missing original frame')
            frames.append((case['id'],t,frame))
    finally:reader.close()

def memory():
    try:return int(subprocess.check_output(['nvidia-smi','--query-gpu=memory.used','--format=csv,noheader,nounits'],text=True).splitlines()[0])
    except (OSError,ValueError,subprocess.CalledProcessError):return None

report=dict(cuda_used=False,frames=len(frames),device='RTX 4070 Ti 12GB; 4GB device not validated',models={})
only=set(sys.argv[1:])
if only and (out/'comparison.json').exists():report=json.loads((out/'comparison.json').read_text())
for name,filename,kind in [('yolo11x','yolo11x.onnx','boxes'),
    ('yolo26m-seg','model-review/yolo26m-seg.onnx','masks'),
    ('yolo26m-seg-960','model-review/yolo26m-seg-960.onnx','masks'),
    ('yolop','yolop/yolop-640-640.onnx','roads'),
    ('yolopv2','model-review/yolopv2-roads.onnx','roads')]:
    if only and name not in only:continue
    path=models/filename;print('Loading',name,flush=True);before=memory()
    if kind=='roads':model=RoadEvidenceDetector(path,'directml');info=model.info
    else:
        cls=InstanceSceneDetector if kind=='masks' else SceneDetector
        model=cls(path,.25,960 if name.endswith('-960') else 640,backend='directml',include_traffic_lights=True);info=model.inference_info
        if info['effective']!='directml':raise RuntimeError(info)
    model.detect(frames[0][2]);durations=[];observations=[];samples=[]
    for clip,t,frame in frames:
        start=time.perf_counter();found=model.detect(frame);durations.append((time.perf_counter()-start)*1000)
        samples.append(memory());draw=frame.copy()
        if kind=='roads':
            road,paint=model.last_masks
            color=np.zeros_like(frame);color[road!=0]=[30,150,30];color[paint!=0]=[20,20,255]
            valid=(road!=0)|(paint!=0);draw[valid]=(.6*frame[valid]+.4*color[valid]).astype(np.uint8)
            observation=dict(road_fraction=float(road.mean()),paint_fraction=float(paint.mean()),curves=len(found['lanes']))
        else:
            observation=[vars(d) for d in found]
            for d in found:
                polygon=getattr(d,'mask_polygon',None)
                if polygon:cv2.polylines(draw,[np.array(polygon,np.int32)],True,(255,100,0),2)
                x,y,w,h=map(round,d.bbox);cv2.rectangle(draw,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.putText(draw,f'{d.kind} {d.confidence:.2f}',(x,max(20,y-4)),0,.5,(0,255,255),1)
        observations.append(dict(clip=clip,time_s=t,result=observation))
        cv2.putText(draw,name,(10,30),0,.9,(0,255,255),2)
        cv2.imencode('.jpg',draw)[1].tofile(str(out/f'{name}-{clip}-{t:g}.jpg'))
    report['models'][name]=dict(runtime=info,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        mean_ms=float(np.mean(durations)),p95_ms=float(np.percentile(durations,95)),
        gpu_memory_before_mib=before,gpu_memory_samples_mib=samples,
        memory_semantics='whole GPU sampled AFTER inference; not transient peak or model-only allocation',
        observations=observations)
    (out/'comparison.json').write_text(json.dumps(report,indent=2))
    print(name,'mean_ms',round(np.mean(durations),2),'sampled_gpu_memory_max',max((v for v in samples if v is not None),default=None),flush=True)
    del model;gc.collect()
print('Done',flush=True)
