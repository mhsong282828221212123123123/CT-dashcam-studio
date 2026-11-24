"""Independent YOLOPv2 road/lane ONNX adapter, raw-pixel masks and DML proof."""
from pathlib import Path
from collections import Counter
import io,json,time
import cv2,numpy as np
from PIL import Image
from .artifacts import canonical,publish,verify,digest
from .ingest import file_hash
from .mask_prepass import prepare_image


def restore_mask(mask,shape):
    h,w=shape;scale=min(640/w,640/h);nw,nh=round(w*scale),round(h*scale)
    left,top=(640-nw)//2,(640-nh)//2
    return cv2.resize(np.asarray(mask,np.uint8)[top:top+nh,left:left+nw],(w,h),interpolation=cv2.INTER_NEAREST)


def run(package,model,destination,backend='directml'):
    import onnxruntime as ort
    package,destination=Path(package),Path(destination);m=verify(package)
    if destination.exists():raise FileExistsError(destination)
    work=destination.parent/(destination.name+'-work');work.mkdir(parents=True,exist_ok=False)
    o=ort.SessionOptions();o.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL;o.enable_mem_pattern=False
    o.intra_op_num_threads=2;o.inter_op_num_threads=1;o.enable_profiling=True;o.profile_file_prefix=str(work/'ort')
    if backend=='directml' and 'DmlExecutionProvider' not in ort.get_available_providers():raise ValueError('DirectML unavailable; select explicit CPU run')
    session=ort.InferenceSession(str(model),o,providers=['DmlExecutionProvider','CPUExecutionProvider'] if backend=='directml' else ['CPUExecutionProvider'])
    if session.get_inputs()[0].shape!=[1,3,640,640] or [x.name for x in session.get_outputs()]!=['road','lane']:
        raise ValueError('Expected fixed 640 YOLOPv2 road/lane export')
    frames=sorted((f for f in json.loads((package/'frames.json').read_text()) if f['camera_id']=='front'),key=lambda f:f['camera_time_s'])
    payloads={};records=[];samples=[];start=time.perf_counter()
    for i,f in enumerate(frames):
        rgb=np.asarray(Image.open(package/f['image']).convert('RGB'));tensor,_=prepare_image(rgb)
        road,lane=session.run(None,{'images':tensor})
        if road.shape!=(1,2,640,640) or lane.shape!=(1,1,640,640) or not np.isfinite(road).all() or not np.isfinite(lane).all():raise ValueError('Invalid road output')
        rm=restore_mask(np.argmax(road[0],axis=0),rgb.shape[:2])*255
        lm=restore_mask(lane[0,0]>.5,rgb.shape[:2])*255
        paths={}
        for name,mask in [('road',rm),('lane',lm)]:
            path=f'masks/{i:05d}-{name}.png';buf=io.BytesIO();Image.fromarray(mask).save(buf,format='PNG');payloads[path]=buf.getvalue();paths[name]=path
        records.append(dict(frame_id=f['frame_id'],time_s=f['camera_time_s'],space_id='raw',**paths))
        if i%50==0:
            overlay=rgb.copy();overlay[rm>0]=(overlay[rm>0]*.65+np.array([0,180,80])*.35).astype('uint8');overlay[lm>0]=[255,220,0]
            samples.append(Image.fromarray(overlay).resize((480,360)))
            print('Road masks',i,'/',len(frames),flush=True)
    profile=Path(session.end_profiling());events=json.loads(profile.read_text());providers=Counter(e.get('args',{}).get('provider') for e in events if e.get('args',{}).get('provider'))
    sheet=Image.new('RGB',(1440,360*max(1,(len(samples)+2)//3)))
    for i,s in enumerate(samples):sheet.paste(s,((i%3)*480,(i//3)*360))
    buf=io.BytesIO();sheet.save(buf,format='JPEG',quality=93);payloads['proof/contact.jpg']=buf.getvalue()
    report=dict(frames=len(records),elapsed_s=time.perf_counter()-start,kernel_provider_counts=dict(providers),requested_backend=backend,cuda=False,
                role='camera road evidence, not metric truth',preprocessing='RGB /255 letterbox 640; raw-space inverse crop',profile_sha256=file_hash(profile))
    payloads.update({'frames.json':canonical(records),'report.json':canonical(report)})
    return publish(destination,stage='road_perception',payloads=payloads,inputs=[dict(role='source',artifact_id=m['artifact_id'])],
                   config={'model':str(model),'backend':backend},model_hashes=[file_hash(Path(model))],code_revision=digest(Path(__file__).read_bytes()),runtime={'ort':ort.__version__,'cuda':False})
