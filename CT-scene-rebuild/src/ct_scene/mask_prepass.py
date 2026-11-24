"""Independent raw-image vehicle masking for static calibration evidence."""
from pathlib import Path
import ast
import io
import json
import time
from collections import Counter
import cv2
import numpy as np
from PIL import Image
from .artifacts import canonical,digest,publish,verify
from .ingest import file_hash
from .pixels import CropResize

DYNAMIC={0,1,2,3,5,7,15,16}


def prepare_image(rgb,size=640):
    h,w=rgb.shape[:2]
    scale=min(size/w,size/h)
    nw,nh=round(w*scale),round(h*scale)
    left,top=(size-nw)//2,(size-nh)//2
    canvas=np.full((size,size,3),114,np.uint8)
    canvas[top:top+nh,left:left+nw]=cv2.resize(rgb,(nw,nh))
    transform=CropResize(0,0,w,h,nw,nh,left,top)
    return np.ascontiguousarray(canvas.transpose(2,0,1)[None],dtype=np.float32)/255,transform


def decode_output(raw,mapping,width,height,threshold=.15):
    raw=np.asarray(raw)
    if raw.ndim!=3 or raw.shape[0]!=1 or raw.shape[1]!=84 or not np.isfinite(raw).all():
        raise ValueError('Only finite COCO Detect [1,84,N] supported')
    rows=raw[0].T
    classes=np.argmax(rows[:,4:],axis=1);scores=rows[np.arange(len(rows)),classes+4]
    rows=rows[(scores>=threshold)&np.isin(classes,list(DYNAMIC))]
    classes=classes[(scores>=threshold)&np.isin(classes,list(DYNAMIC))]
    results=[]
    for row,cls in zip(rows,classes):
        cx,cy,bw,bh=row[:4]
        if bw<=0 or bh<=0:continue
        corners=mapping.backward([[cx-bw/2,cy-bh/2],[cx+bw/2,cy+bh/2]])
        lo,hi=np.maximum(corners[0],[0,0]),np.minimum(corners[1],[width,height])
        if np.any(hi<=lo):continue
        results.append({'class_id':int(cls),'score':float(row[4+cls]),'bbox_xyxy':np.r_[lo,hi].tolist()})
    kept=[]
    for cls in sorted({r['class_id'] for r in results}):
        group=[r for r in results if r['class_id']==cls]
        boxes=[[r['bbox_xyxy'][0],r['bbox_xyxy'][1],r['bbox_xyxy'][2]-r['bbox_xyxy'][0],r['bbox_xyxy'][3]-r['bbox_xyxy'][1]] for r in group]
        ids=cv2.dnn.NMSBoxes(boxes,[r['score'] for r in group],threshold,.55)
        kept.extend(group[int(i)] for i in np.asarray(ids).reshape(-1))
    return kept


def static_mask(width,height,detections):
    mask=np.full((height,width),255,np.uint8)
    for row in detections:
        x1,y1,x2,y2=row['bbox_xyxy'];px=max(6,(x2-x1)*.08);py=max(6,(y2-y1)*.08)
        mask[max(0,int(y1-py)):min(height,int(np.ceil(y2+py))),max(0,int(x1-px)):min(width,int(np.ceil(x2+px)))]=0
    return mask


class RawMaskDetector:
    def __init__(self,model,profile_dir,backend='directml'):
        import onnxruntime as ort
        self.ort=ort;self.model=str(model);self.profile_dir=Path(profile_dir)
        self.profile_dir.mkdir(parents=True,exist_ok=True)
        self.fallback=[];self.proofs=[];self.profile_open=True
        try:self.session=self._session(backend)
        except Exception as exc:
            self.fallback.append(str(exc));self.session=self._session('cpu')
        if self.session.get_inputs()[0].shape!=[1,3,640,640]:raise ValueError('Mask prepass requires fixed 640 COCO model')
        metadata=self.session.get_modelmeta().custom_metadata_map
        names=ast.literal_eval(metadata.get('names','{}'))
        if names.get(2)!='car' or names.get(7)!='truck':raise ValueError('Unexpected model class vocabulary')

    def _session(self,backend):
        ort=self.ort;o=ort.SessionOptions();o.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
        o.enable_mem_pattern=False;o.intra_op_num_threads=2;o.inter_op_num_threads=1;o.enable_profiling=True
        o.profile_file_prefix=str(self.profile_dir/f'{backend}-{len(self.fallback)}')
        if backend=='directml' and 'DmlExecutionProvider' not in ort.get_available_providers():raise ValueError('DirectML unavailable')
        providers=[('DmlExecutionProvider',{'device_id':0}),'CPUExecutionProvider'] if backend=='directml' else ['CPUExecutionProvider']
        self.profile_open=True
        return ort.InferenceSession(self.model,o,providers=providers)

    def _proof(self):
        if not self.profile_open:return
        path=self.session.end_profiling();events=json.loads(Path(path).read_text())
        providers=Counter(e.get('args',{}).get('provider') for e in events if e.get('args',{}).get('provider'))
        self.proofs.append({'profile_file':str(path),'profile_sha256':file_hash(Path(path)),'kernel_provider_counts':dict(providers)})
        self.profile_open=False

    def detect(self,rgb):
        tensor,mapping=prepare_image(rgb)
        try:output=self.session.run(None,{'images':tensor})[0]
        except Exception as exc:
            self._proof()
            if self.session.get_providers()==['CPUExecutionProvider']:raise
            self.fallback.append(str(exc));self.session=self._session('cpu')
            output=self.session.run(None,{'images':tensor})[0]
        self._proof()
        return decode_output(output,mapping,rgb.shape[1],rgb.shape[0]),self.session.get_providers()


def run(package,model,destination,backend='directml'):
    package=Path(package);manifest=verify(package)
    rows=json.loads((package/'frames.json').read_text(encoding='utf-8'))
    profiles=Path(destination).parent/(Path(destination).name+'-profiles')
    detector=RawMaskDetector(model,profiles,backend)
    payloads={};records=[];elapsed=[]
    start=time.perf_counter()
    for i,f in enumerate(rows):
        rgb=np.asarray(Image.open(package/f['image']).convert('RGB'))
        before=time.perf_counter();detections,providers=detector.detect(rgb);elapsed.append(time.perf_counter()-before)
        mask=static_mask(f['width'],f['height'],detections)
        buf=io.BytesIO();Image.fromarray(mask).save(buf,format='PNG');name=f'masks/{i:05d}.png';payloads[name]=buf.getvalue()
        records.append({'frame_id':f['frame_id'],'camera_id':f['camera_id'],'camera_time_s':f['camera_time_s'],
                        'space_id':'raw','detections':detections,'static_mask':name,'providers':providers,
                        'masked_fraction':float(np.mean(mask==0)),
                        'purpose':'calibration_exclusion_only_not_ground_truth'})
        if i%100==0:print(f'raw masks {i}/{len(rows)}',flush=True)
    for i,proof in enumerate(detector.proofs):payloads[f'profiles/{i}.json']=Path(proof['profile_file']).read_bytes()
    report={'frames':len(rows),'model_sha256':file_hash(Path(model)),'requested_backend':backend,
            'profile_proofs':detector.proofs,'fallback_reasons':detector.fallback,
            'inference_mean_s':float(np.mean(elapsed)),'wall_time_s':time.perf_counter()-start,
            'mask_quality':'NOT_EVALUATED','geometry_quality':'NOT_EVALUATED','cuda':False}
    payloads['observations.jsonl']=b''.join(canonical(r)+b'\n' for r in records)
    payloads['report.json']=canonical(report)
    return publish(destination,stage='raw_mask_prepass',payloads=payloads,
                   inputs=[{'role':'raw_source_package','artifact_id':manifest['artifact_id']}],
                   config={'threshold':.15,'nms_iou':.55,'mask_padding_fraction':.08,'input_size':640,'backend':backend},
                   model_hashes=[report['model_sha256']],code_revision=digest(Path(__file__).read_bytes()),
                   runtime={'onnxruntime':detector.ort.__version__})
