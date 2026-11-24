"""Experimental image-only body heading; no XYZ or detector replacement.

KITTI bin centres are pi/2,3pi/2 for alpha+pi, so alpha_prime=alpha+pi/2
equals atan2(regression)+bin*pi. Rear facing away at image centre is zero.
W uses forward X and left Y: heading_camera=-(alpha_prime+camera_ray).
This adapter is not enabled in the default scene pipeline.
"""
from pathlib import Path
from collections import Counter
import json,math
import cv2,numpy as np
from .artifacts import verify,publish,canonical,digest
from .ingest import file_hash
from .affine_probe import camera_matrix
from .rear_view_cue import paired_red_lamps


def wrap(a):return float(np.arctan2(np.sin(a),np.cos(a)))


def decode(orientation,confidence):
    o=np.asarray(orientation,float).reshape(2,2);c=np.asarray(confidence,float).reshape(2)
    if not np.isfinite(o).all() or not np.isfinite(c).all():raise ValueError('Nonfinite orientation prediction')
    k=int(np.argmax(c))
    if np.linalg.norm(o[k])<.5:raise ValueError('Degenerate direction vector')
    p=np.exp(c-c.max());p/=p.sum()
    return wrap(math.atan2(o[k,1],o[k,0])+k*math.pi),float(p[k])


def camera_heading(alpha_prime,ray):return wrap(-alpha_prime-ray)


def foreground_occlusion(box,others):
    box=np.asarray(box,float);area=max(1.,float(np.prod(box[2:]-box[:2])));fractions=[]
    for other in others:
        o=np.asarray(other,float)
        if o[3]<=box[3]+3:continue
        overlap=np.maximum(0,np.minimum(o[2:],box[2:])-np.maximum(o[:2],box[:2]));fractions.append(float(np.prod(overlap))/area)
    return max(fractions,default=0.)


def preprocess(rgb,box):
    h,w=rgb.shape[:2];x,y,X,Y=np.asarray(box,float)
    if not np.isfinite(box).all() or min(x,y,w-X,h-Y)<3 or X-x<80 or Y-y<60:raise ValueError('partial_or_small_body')
    crop=rgb[int(y):min(h,int(Y)+1),int(x):min(w,int(X)+1)]
    crop=cv2.resize(crop,(224,224),interpolation=cv2.INTER_CUBIC).astype('float32')/255.
    normalized=(crop-np.array([.485,.456,.406],dtype='float32'))/np.array([.229,.224,.225],dtype='float32')
    return np.ascontiguousarray(normalized.transpose(2,0,1)[None])


class ImageHeadingModel:
    def __init__(self,model,profile_dir,backend='directml'):
        import onnxruntime as ort
        if backend not in ('directml','cpu'):raise ValueError('Explicit DirectML or CPU backend required')
        if backend=='directml' and 'DmlExecutionProvider' not in ort.get_available_providers():raise ValueError('DirectML unavailable')
        options=ort.SessionOptions();options.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL;options.enable_mem_pattern=False
        options.intra_op_num_threads=2;options.inter_op_num_threads=1;options.enable_profiling=True
        Path(profile_dir).mkdir(parents=True,exist_ok=True);options.profile_file_prefix=str(Path(profile_dir)/'orientation-ort')
        self.session=ort.InferenceSession(str(model),options,providers=['DmlExecutionProvider','CPUExecutionProvider'] if backend=='directml' else ['CPUExecutionProvider'])
        if self.session.get_inputs()[0].shape!=[1,3,224,224] or [x.name for x in self.session.get_outputs()]!=['orientation','confidence','dimension']:raise ValueError('Unexpected orientation model interface')
        self.backend=backend

    def predict(self,rgb,box,occluders=()):
        occluded=foreground_occlusion(box,occluders)
        if occluded>.15:return dict(accepted=False,reason='foreground_body_occlusion',occlusion_fraction=occluded)
        try:tensor=preprocess(rgb,box)
        except ValueError as e:return dict(accepted=False,reason=str(e))
        a,p=decode(*self.session.run(None,{'image':tensor})[:2])
        m,mp=decode(*self.session.run(None,{'image':np.ascontiguousarray(tensor[:,:,:,::-1])})[:2])
        error=abs(wrap(a+m));accepted=error<=math.radians(25)
        mean=wrap(np.angle(np.exp(1j*a)+np.exp(-1j*m))) if accepted else a
        return dict(accepted=accepted,reason='mirror_consistent_candidate' if accepted else 'mirror_disagreement',alpha_prime_rad=mean,
                    raw_alpha_prime_rad=a,mirrored_alpha_prime_rad=m,mirror_error_deg=math.degrees(error),
                    bin_probability=p,mirrored_bin_probability=mp,dimensions_used=False)

    def finish_profile(self):
        path=Path(self.session.end_profiling());events=json.loads(path.read_text());counts=Counter(e.get('args',{}).get('provider') for e in events if e.get('args',{}).get('provider'))
        return dict(requested_backend=self.backend,kernel_provider_counts=dict(counts),profile=str(path),cuda=False)


def run(package,tracks,model,optics,destination,backend='directml'):
    """Independent experimental evidence artifact; no actor/road edits."""
    import time
    from collections import defaultdict
    package,tracks,destination=map(Path,[package,tracks,destination])
    if destination.exists():raise FileExistsError(destination)
    pm,tm=verify(package),verify(tracks)
    if not any(r['artifact_id']==pm['artifact_id'] for r in tm['inputs']):raise ValueError('Track/source mismatch')
    revision=lambda:digest(b''.join(Path(__file__).with_name(n).read_bytes() for n in ['image_vehicle_heading.py','affine_probe.py','rear_view_cue.py']))
    started_revision=revision();work=destination.parent/(destination.name+'-work');work.mkdir(parents=True,exist_ok=False)
    predictor=ImageHeadingModel(model,work/'ort',backend);profile=json.loads(Path(optics).read_text())['cameras']['front'];K,D=camera_matrix(profile)
    frames=json.loads((package/'frames.json').read_text());by=defaultdict(list)
    for row in map(json.loads,(tracks/'observations.jsonl').read_text().splitlines()):by[row['frame_id']].append(row)
    results=[];start=time.perf_counter()
    for frame in sorted((f for f in frames if f['camera_id']=='front'),key=lambda f:f['camera_time_s']):
        rows=by[frame['frame_id']];targets=[r for r in rows if r['support']=='detector_observation' and r['class_id']==2]
        if not targets:continue
        if profile['resolution']!=[frame['width'],frame['height']]:raise ValueError('Front optics/source dimensions differ')
        rgb=cv2.cvtColor(cv2.imread(str(package/frame['image'])),cv2.COLOR_BGR2RGB)
        for row in targets:
            box=row['bbox_xyxy'];others=[r['bbox_xyxy'] for r in rows if r['local_track_id']!=row['local_track_id'] and r['support'] in ('detector_observation','flow_observation')]
            prediction=predictor.predict(rgb,box,others)
            if 'alpha_prime_rad' in prediction:
                point=np.array([[[(box[0]+box[2])/2,(box[1]+box[3])/2]]],float);ray=float(np.arctan(cv2.undistortPoints(point,K,D)[0,0,0]));prediction.update(camera_heading_rad=camera_heading(prediction['alpha_prime_rad'],ray),ray_rad=ray)
                rear=paired_red_lamps(cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),box)['rear_cue']
                prediction['rear_appearance_cue']=rear
                if rear and math.cos(prediction['alpha_prime_rad'])<-.3:prediction.update(accepted=False,reason='model_direction_conflicts_with_observed_rear')
            results.append(dict(frame_id=frame['frame_id'],camera_time_s=frame['camera_time_s'],camera_id='front',local_track_id=row['local_track_id'],bbox_xyxy=box,**prediction))
    report=dict(scope='experimental front image body-angle candidates; no XYZ, road or identity changes',observations=len(results),accepted=sum(r['accepted'] for r in results),reasons=dict(Counter(r['reason'] for r in results)),elapsed_s=time.perf_counter()-start,profile=predictor.finish_profile(),runtime_adopted=False,optics_sha256=digest(canonical(profile)))
    if revision()!=started_revision:raise ValueError('Orientation evidence source changed during inference')
    return publish(destination,stage='vehicle_image_heading',payloads={'predictions.jsonl':b''.join(canonical(r)+b'\n' for r in results),'report.json':canonical(report)},
                   inputs=[dict(role='source',artifact_id=pm['artifact_id']),dict(role='tracks',artifact_id=tm['artifact_id'])],config=dict(optics=profile,backend=backend,mirror_limit_deg=25,minimum_body_pixels=[80,60],foreground_occlusion_limit=.15,classes=[2],crop_space='raw_front_rgb'),
                   model_hashes=[file_hash(Path(model))],code_revision=started_revision,runtime=dict(cuda=False))
