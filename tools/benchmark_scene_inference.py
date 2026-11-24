"""Same-frame CPU/DirectML comparison; agreement is not ground-truth accuracy."""
import argparse,json,time,sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import SceneDetector,CameraReader,iou

def run(selection,output):
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    clips=json.loads(Path(selection).read_text())['selected'];frames=[]
    for name,clip in clips.items():
        source=Path(clip['file'])
        for camera in ('front','left_repeater','right_repeater'):
            reader=CameraReader(source.with_name(source.name.replace('-front','-'+camera)))
            try:
                for t in (3.,8.,13.,18.,23.,33.):
                    _,im=reader.read_at(t);frames.append((f'{name}-{camera}-{t:g}',im))
            finally:reader.close()
    models=Path(__file__).resolve().parents[2]/'scene-models';report={};baseline=None
    for name,model,backend,detail in [('cpu-v8s','yolov8s.onnx','cpu','standard'),
            ('dml-v8s','yolov8s.onnx','directml','standard'),
            ('dml-v11x','yolo11x.onnx','directml','standard'),
            ('dml-v11x-960','yolo11x-960.onnx','directml','standard'),
            ('dml-v8s-tiled','yolov8s.onnx','directml','tiled')]:
        detector=SceneDetector(models/model,.25,960 if '960' in name else 640,backend=backend,detail=detail)
        detector.detect(frames[0][1]);results=[];times=[]
        for key,im in frames:
            start=time.perf_counter();found=detector.detect(im);times.append(time.perf_counter()-start)
            results.append([vars(d) for d in found])
            if name!='cpu-v8s':
                draw=im.copy()
                for d in found:
                    x,y,w,h=map(round,d.bbox);cv2.rectangle(draw,(x,y),(x+w,y+h),(0,230,0),2)
                    cv2.putText(draw,f'{d.kind} {d.confidence:.2f}',(x,max(18,y-4)),0,.5,(0,230,0),1)
                cv2.imencode('.jpg',draw)[1].tofile(str(out/f'{name}-{key}.jpg'))
        if baseline is None:baseline=results
        matched=sum(any(a['kind']==b['kind'] and iou(a['bbox'],b['bbox'])>.95 for b in found)
                    for reference,found in zip(baseline,results) for a in reference)
        report[name]={'runtime':detector.inference_info,'frames':len(frames),'mean_ms':1000*float(np.mean(times)),
            'p95_ms':1000*float(np.percentile(times,95)),'detections':sum(map(len,results)),
            'baseline_agreement_iou95':matched/max(1,sum(map(len,baseline))),
            'observations':dict(zip([k for k,_ in frames],results))}
        print(name,{k:v for k,v in report[name].items() if k!='observations'},flush=True)
        (out/'benchmark.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--selection',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();run(a.selection,a.output)
