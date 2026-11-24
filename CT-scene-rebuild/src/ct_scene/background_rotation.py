"""Static-background relative rotations, with tracked dynamic objects masked.

This evidence estimates rotation only. It never triangulates buildings, fits
metric camera mounts, or uses another vehicle's world position as own motion.
"""
from pathlib import Path
import json
import cv2
import numpy as np
from PIL import Image
from .affine_probe import camera_matrix
from .artifacts import canonical,digest,publish,verify
from .ego_turn_rotation import corrected_yaw


def dynamic_exclusion(image,rows,camera):
    h,w=image.shape;mask=np.zeros_like(image);mask[round(h*.0833):round(h*.8854),round(w*.0234):round(w*.9766)]=255
    if camera=='right_repeater':mask[:round(h*.3125),round(w*.5469):]=0
    if camera=='left_repeater':mask[:round(h*.3125),:round(w*.4297)]=0
    for r in rows:
        x0,y0,x1,y1=np.round(r['bbox_xyxy']).astype(int)
        mask[max(0,y0-15):min(h,y1+15),max(0,x0-15):min(w,x1+15)]=0
    return mask


def pair_rotation(a,b,mask_a,mask_b,K,D,prior):
    row=dict(accepted=False,reason='insufficient_static_correspondences')
    p=cv2.goodFeaturesToTrack(a,1500,.01,8,mask=mask_a,blockSize=7)
    if p is None:return row
    q,s,_=cv2.calcOpticalFlowPyrLK(a,b,p,None,winSize=(25,25),maxLevel=4)
    if q is None:return row
    back,bs,_=cv2.calcOpticalFlowPyrLK(b,a,q,None,winSize=(25,25),maxLevel=4)
    if back is None:return row
    p=p[:,0];q=q[:,0];back=back[:,0];h,w=b.shape
    good=(s[:,0]>0)&(bs[:,0]>0)&np.isfinite(q).all(1)&(np.linalg.norm(back-p,axis=1)<1)&(q[:,0]>=0)&(q[:,0]<w)&(q[:,1]>=0)&(q[:,1]<h)
    ids=np.flatnonzero(good);xy=np.clip(np.rint(q[ids]).astype(int),[0,0],[w-1,h-1]);ids=ids[mask_b[xy[:,1],xy[:,0]]>0];p=p[ids];q=q[ids];row['matches']=len(p)
    if len(p)<40:return row
    pn=cv2.undistortPoints(p[:,None].astype(float),K,D);qn=cv2.undistortPoints(q[:,None].astype(float),K,D)
    E,inliers=cv2.findEssentialMat(pn,qn,np.eye(3),method=cv2.RANSAC,prob=.999,threshold=1./K[0,0])
    if E is None or E.shape!=(3,3):return row
    count,R,_,valid=cv2.recoverPose(E,pn,qn,np.eye(3),mask=inliers.copy());angle=float(np.arctan2(R[0,2],R[2,2]));tilt=float(np.arccos(np.clip(R[1,1],-1,1)))
    ecount=int(np.count_nonzero(inliers));cells={(int(x/w*4),int(y/h*3)) for x,y in p[inliers[:,0]>0]}
    okay=count>=40 and ecount/len(p)>.6 and len(cells)>=5 and tilt<np.radians(3) and abs(angle-prior)<np.radians(8)
    row.update(accepted=bool(okay),reason='relative_rotation_candidate' if okay else 'weak_or_degenerate_static_pose',inliers=count,
               epipolar_inliers=ecount,epipolar_cells=len(cells),visual_delta_deg=float(np.degrees(angle)),tilt_deg=float(np.degrees(tilt)))
    return row


def run(package,tracks,telemetry,lens_config,destination):
    from .full_timeline import own_motion
    package,tracks,telemetry=map(Path,[package,tracks,telemetry]);pm,tm,sm=[verify(p) for p in (package,tracks,telemetry)]
    if pm['artifact_id'] not in {i['artifact_id'] for i in tm['inputs']}:raise ValueError('Rotation/source/track mismatch')
    names=['background_rotation.py','ego_turn_rotation.py','full_timeline.py','affine_probe.py','wheel_scale.py'];revision=lambda:digest(b''.join(Path(__file__).with_name(n).read_bytes() for n in names));started=revision()
    frames=json.loads((package/'frames.json').read_text());front=sorted((f for f in frames if f['camera_id']=='front'),key=lambda f:f['camera_time_s']);times=np.array([f['camera_time_s'] for f in front]);sei=[json.loads(s) for s in (telemetry/'telemetry.jsonl').read_text().splitlines()]
    if {f['source_sha256'] for f in front}!={r['source_sha256'] for r in sei}:raise ValueError('Rotation/telemetry source mismatch')
    _,yaw=own_motion(times,sei);_,windows=corrected_yaw(times,yaw,{})
    windows=[r for r in windows if r['reason']!='not_a_large_own_turn'];observations={};profiles=json.loads(Path(lens_config).read_text())['cameras']
    for r in map(json.loads,(tracks/'observations.jsonl').read_text().splitlines()):observations.setdefault(r['frame_id'],[]).append(r)
    pairs={};cv2.setNumThreads(4)
    for camera in ['front','left_repeater','right_repeater']:
        K,D=camera_matrix(profiles[camera]);sequence=sorted((f for f in frames if f['camera_id']==camera and any(r['start_s']-.4<=f['camera_time_s']<=r['end_s']+.4 for r in windows)),key=lambda f:f['camera_time_s']);rows=[];previous=None;cv2.setRNGSeed(20261115)
        for f in sequence:
            image=np.asarray(Image.open(package/f['image']).convert('L'));mask=dynamic_exclusion(image,observations.get(f['frame_id'],[]),camera)
            if list(image.shape[::-1])!=profiles[camera]['resolution']:raise ValueError('Own rotation optical/source dimensions differ')
            if previous:
                pf,a,ma=previous;lo,hi=pf['camera_time_s'],f['camera_time_s']
                if 0<hi-lo<=.35:
                    prior=np.interp(hi,times,yaw)-np.interp(lo,times,yaw);review=pair_rotation(a,image,ma,mask,K,D,prior)
                    rows.append(dict(start_s=lo,end_s=hi,from_frame_id=pf['frame_id'],to_frame_id=f['frame_id'],sei_delta_deg=float(np.degrees(prior)),**review))
            previous=f,image,mask
        pairs[camera]=rows;print(json.dumps(dict(stage='background_rotation',camera=camera,pairs=len(rows),accepted=sum(r['accepted'] for r in rows))),flush=True)
    _,reviews=corrected_yaw(times,yaw,pairs)
    if revision()!=started:raise ValueError('Rotation engine changed during extraction')
    return publish(Path(destination),stage='own_background_rotation',payloads={'camera-pairs.json':canonical(pairs),'report.json':canonical(dict(windows=reviews,optics_sha256=digest(canonical(profiles)),uses_other_actor_world_positions=False,translation_source='own_SEI_signed_speed',cuda=False))},
                   inputs=[dict(role=role,artifact_id=m['artifact_id']) for role,m in [('source',pm),('tracks',tm),('telemetry',sm)]],config=dict(optics_sha256=digest(canonical(profiles)),dynamic_object_masks=True,minimum_independent_cameras=2),code_revision=started,runtime=dict(cuda=False,opencv=cv2.__version__))
