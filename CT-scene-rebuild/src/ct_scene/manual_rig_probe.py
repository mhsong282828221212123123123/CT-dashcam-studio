"""Conditional multi-view landmark diagnostic; never publishes an accepted rig."""
from pathlib import Path
import io
import json
import cv2
import numpy as np
from PIL import Image,ImageDraw
from .artifacts import verify,publish,canonical,digest
from .affine_probe import camera_matrix
from .geometry import validate_transform


def flow_step(a,b,points,active,mask):
    """Track only live raw points; forward/backward and dynamic-mask rejection."""
    output=np.asarray(points,np.float32).copy();accepted=np.zeros(len(points),bool)
    ids=np.flatnonzero(active)
    if not len(ids):return output,accepted
    p=output[ids].reshape(-1,1,2)
    q,s,_=cv2.calcOpticalFlowPyrLK(a,b,p,None,winSize=(31,31),maxLevel=4,
        criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,40,.001))
    if q is None:return output,accepted
    back,bs,_=cv2.calcOpticalFlowPyrLK(b,a,q,None,winSize=(31,31),maxLevel=4,
        criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,40,.001))
    if back is None:return output,accepted
    q=q.reshape(-1,2);back=back.reshape(-1,2);h,w=b.shape
    valid=(s.ravel()==1)&(bs.ravel()==1)&np.isfinite(q).all(1)&(np.linalg.norm(back-p.reshape(-1,2),axis=1)<=1.5)
    valid&=(q[:,0]>=8)&(q[:,0]<w-8)&(q[:,1]>=8)&(q[:,1]<h-8)
    for j in np.flatnonzero(valid):
        x,y=np.rint(q[j]).astype(int)
        if mask[y,x]:output[ids[j]]=q[j];accepted[ids[j]]=True
    return output,accepted


def triangulate(pixels,poses,profile):
    pixels=np.asarray(pixels,float);poses=np.asarray(poses,float)
    if len(pixels)<3 or poses.shape!=(len(pixels),3,4):raise ValueError('Need three camera views')
    if not np.isfinite(pixels).all() or not np.isfinite(poses).all():raise ValueError('Nonfinite triangulation input')
    for P in poses:validate_transform(np.vstack([P,[0,0,0,1]]))
    K,D=camera_matrix(profile);rays=cv2.undistortPoints(pixels.reshape(-1,1,2),K,D).reshape(-1,2)
    A=np.array([row for (x,y),P in zip(rays,poses) for row in (x*P[2]-P[0],y*P[2]-P[1])])
    _,singular,V=np.linalg.svd(A)
    if abs(V[-1,3])<1e-10:raise ValueError('Landmark at infinity')
    point=V[-1,:3]/V[-1,3]
    pc=np.einsum('nij,j->ni',poses[:,:,:3],point)+poses[:,:,3]
    if np.any(pc[:,2]<=0):raise ValueError('Landmark behind camera')
    centres=np.array([-P[:,:3].T@P[:,3] for P in poses]);directions=point-centres
    if np.max(np.linalg.norm(centres-centres[0],axis=1))<1e-8:raise ValueError('No translation baseline')
    directions/=np.linalg.norm(directions,axis=1)[:,None]
    angles=np.degrees(np.arccos(np.clip(directions@directions.T,-1,1)))
    predictions=[]
    for X in pc:predictions.append(cv2.projectPoints(X[None],np.zeros(3),np.zeros(3),K,D)[0].reshape(2))
    errors=np.linalg.norm(np.asarray(predictions)-pixels,axis=1)
    return point,{'max_ray_angle_deg':float(angles.max()),'fit_p95_px':float(np.percentile(errors,95)),
                  'fit_errors_px':errors.tolist(),'dlt_smallest_singular':float(singular[-1])}


def project(points,pose,profile):
    validate_transform(np.vstack([pose,[0,0,0,1]]))
    K,D=camera_matrix(profile);pc=np.asarray(points)@pose[:,:3].T+pose[:,3]
    if not np.isfinite(pc).all() or np.any(pc[:,2]<=0):raise ValueError('Projection behind camera or nonfinite')
    return cv2.projectPoints(pc,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2)


def run(package,masks,sfm,lens_config,reference,destination,fit_offsets=(-2,-1.5,-1,-.5,0,.5)):
    import pycolmap as pc
    package,masks,sfm=Path(package),Path(masks),Path(sfm)
    if Path(destination).exists():raise FileExistsError(destination)
    manifests=[verify(p) for p in (package,masks,sfm)]
    for manifest in manifests[1:]:
        if not any(i['artifact_id']==manifests[0]['artifact_id'] for i in manifest['inputs']):raise ValueError('Source artifact mismatch')
    ref=json.loads(Path(reference).read_text());profiles=json.loads(Path(lens_config).read_text())['cameras']
    sr=json.loads((sfm/'report.json').read_text())
    if ref['source_camera']!=sr['camera'] or ref['space_id']!='raw':raise ValueError('Reference camera/pixel space mismatch')
    source=ref['source_camera'];target=ref['target_camera'];profile=profiles[source]
    lens=sr['optical_candidate']
    if lens['model']!=profile['model'] or not np.allclose(lens['params'],profile['params'],rtol=0,atol=1e-10):raise ValueError('Front map/lens mismatch')
    frames=json.loads((package/'frames.json').read_text())
    observations={r['frame_id']:r for r in map(json.loads,(masks/'observations.jsonl').read_text().splitlines())}
    anchor=min((f for f in frames if f['camera_id']==source),key=lambda f:abs(f['relative_request_s']-ref['source_request_s']))
    target_frame=min((f for f in frames if f['camera_id']==target),key=lambda f:abs(f['relative_request_s']-ref['target_request_s']))
    if abs(anchor['relative_request_s']-ref['source_request_s'])>.11 or abs(target_frame['relative_request_s']-ref['target_request_s'])>.11:
        raise ValueError('Reference frame missing')
    for f,c in [(anchor,source),(target_frame,target)]:
        if [f['width'],f['height']]!=profiles[c]['resolution']:raise ValueError('Reference/lens resolution mismatch')
    index={f['sfm_name']:f for f in json.loads((sfm/'input-index.json').read_text())}
    model=pc.Reconstruction(sfm/'reconstruction');model_frames={}
    for im in model.images.values():model_frames[index[im.name]['frame_id']]=(index[im.name],im.cam_from_world().matrix())
    # Fixed experiment schedule; check view never participates in triangulation.
    if len(fit_offsets)<3 or len(set(fit_offsets))!=len(fit_offsets) or any(not np.isfinite(t) or t>=1 for t in fit_offsets):
        raise ValueError('Need distinct fit offsets preceding the excluded +1s view')
    fit_times=[ref['source_request_s']+t for t in sorted(fit_offsets)]
    check_time=ref['source_request_s']+1
    chosen=[];missing_fit_times=[]
    for t in fit_times+[check_time]:
        candidates=list(model_frames.values());f,P=min(candidates,key=lambda v:abs(v[0]['selection_time_s']-t))
        if abs(f['selection_time_s']-t)>.01:
            if t==check_time:raise ValueError('Required front check pose missing')
            missing_fit_times.append(t);continue
        chosen.append((f,P))
    if len(chosen)<4:raise ValueError('Fewer than three available front fit poses')
    bounds=[f['relative_request_s'] for f,_ in chosen]
    sequence=sorted((f for f in frames if f['camera_id']==source and min(bounds)-.01<=f['relative_request_s']<=max(bounds)+.01),key=lambda f:f['camera_time_s'])
    image_cache={f['frame_id']:np.asarray(Image.open(package/f['image']).convert('L')) for f in sequence}
    mask_cache={f['frame_id']:np.asarray(Image.open(masks/observations[f['frame_id']]['static_mask'])) for f in sequence}
    points=np.array([p['source'] for p in ref['points']],np.float32);live=np.ones(len(points),bool)
    track={anchor['frame_id']:(points.copy(),live.copy())};anchor_idx=next(i for i,f in enumerate(sequence) if f['frame_id']==anchor['frame_id'])
    cv2.setNumThreads(4);cv2.setRNGSeed(20261114)
    for direction in (-1,1):
        prev=anchor;positions=points.copy();active=live.copy()
        for i in range(anchor_idx+direction,-1 if direction<0 else len(sequence),direction):
            f=sequence[i];fid=f['frame_id']
            positions,active=flow_step(image_cache[prev['frame_id']],image_cache[fid],positions,active,mask_cache[fid])
            track[fid]=(positions.copy(),active.copy());prev=f
    payloads={};records=[];accepted=[]
    payloads['front-track-proposals.json']=canonical([{'frame':f,'points_raw':track[f['frame_id']][0].tolist(),
        'active':track[f['frame_id']][1].tolist(),'status':'FLOW_PROPOSALS_NOT_MANUAL_GT'} for f in sequence])
    for n,p in enumerate(ref['points']):
        record={'id':p['id'],'target_split':p['split'],'status':'REJECTED'}
        fit_frames=[(f,P) for f,P in chosen[:-1] if track[f['frame_id']][1][n]];check_frame,check_pose=chosen[-1]
        record['front_fit_count']=len(fit_frames)
        if len(fit_frames)<3 or not track[check_frame['frame_id']][1][n]:record['reason']='lost_or_masked_front_track'
        else:
            try:
                X,diag=triangulate([track[f['frame_id']][0][n] for f,_ in fit_frames],[P for _,P in fit_frames],profile)
                predicted=project(X[None],check_pose,profile)[0];measured=track[check_frame['frame_id']][0][n]
                error=float(np.linalg.norm(predicted-measured));record.update(diag,point_world_gauge=X.tolist(),front_check_px=error)
                if diag['max_ray_angle_deg']<1:record['reason']='insufficient_parallax'
                elif diag['fit_p95_px']>3 or error>5:record['reason']='front_reprojection_failed'
                else:record['status']='CONDITIONAL_FRONT_POINT';accepted.append((n,X))
            except ValueError as exc:record['reason']=str(exc)
        records.append(record)
    # Inspect propagation, including rejected points; source coordinates are raw.
    proof=Image.new('RGB',(2560,((len(chosen)+1)//2)*1000),'#111');draw=ImageDraw.Draw(proof)
    for j,(f,_) in enumerate(chosen):
        x=(j%2)*1280;y=(j//2)*1000;proof.paste(Image.open(package/f['image']).convert('RGB'),(x,y+40))
        draw.text((x+10,y+10),f'{f["camera_time_s"]:.4f}s '+('TRIANGULATION' if j<len(chosen)-1 else 'POINT CHECK: excluded from triangulation, camera pose used in source map'),fill='white')
        xy,active=track[f['frame_id']]
        for p,(u,v),valid in zip(ref['points'],xy,active):
            if valid:draw.ellipse((x+u-4,y+v+36,x+u+4,y+v+44),outline='yellow',width=2);draw.text((x+u+5,y+v+40),p['id'],fill='yellow')
    buf=io.BytesIO();proof.save(buf,format='JPEG',quality=92);payloads['proof/front-tracks.jpg']=buf.getvalue()
    result={'gate':'FAIL','accepted_front_points':len(accepted),'landmarks':records,'source_map_gate':sr['gate'],
            'missing_fit_times_s':missing_fit_times,
            'scope':'Conditional on unaccepted front SfM poses and approximate manual controls. No metric or rig acceptance.',
            'clock':'UNVERIFIED','rig':'NOT_ACCEPTED','front_fit_frames':[f for f,_ in chosen[:-1]],'front_check_frame':chosen[-1][0],
            'target_frame':target_frame}
    if len(accepted)>=4:
        xyz=np.array([X for _,X in accepted]);centred=xyz-xyz.mean(0);_,sv,V=np.linalg.svd(centred)
        extent=float(np.linalg.norm(np.ptp(xyz,axis=0)))
        result['source_structure_diagnostic']={'plane_smallest_largest_singular_ratio':float(sv[-1]/max(sv[0],1e-12)),
            'plane_p95_distance_over_extent':float(np.percentile(np.abs(centred@V[-1]),95)/max(extent,1e-12)),
            'note':'Useful only under assumed facade planarity; no flattening, scale inference or target pose correction is performed.'}
    fit_accepted=[(n,X) for n,X in accepted if ref['points'][n]['split']=='fit']
    check_accepted=[(n,X) for n,X in accepted if ref['points'][n]['split']=='check']
    if len(fit_accepted)>=4 and len(check_accepted)>=3:
        X=np.array([X for _,X in fit_accepted],float);uv=np.array([ref['points'][n]['target'] for n,_ in fit_accepted],float)
        K,D=camera_matrix(profiles[target]);count,rv,tv,_=cv2.solvePnPGeneric(X,uv,K,D,flags=cv2.SOLVEPNP_SQPNP)
        candidates=[]
        for r,t in zip(rv,tv):
            pose=np.column_stack([cv2.Rodrigues(r)[0],t.reshape(3)])
            try:fit_error=np.linalg.norm(project(X,pose,profiles[target])-uv,axis=1)
            except ValueError:continue
            candidates.append((float(np.median(fit_error)),pose))
        result['pnp_candidate_count']=len(candidates)
        if candidates:
            _,pose=min(candidates,key=lambda c:c[0]) # selection uses FIT controls only
            ids=[n for n,_ in accepted];allX=np.array([X for _,X in accepted]);observed=np.array([ref['points'][n]['target'] for n in ids])
            try:
                predicted=project(allX,pose,profiles[target]);errors=np.linalg.norm(predicted-observed,axis=1)
                is_fit=np.array([ref['points'][n]['split']=='fit' for n in ids]);checked=errors[~is_fit]
                result.update(target_fit_count=int(is_fit.sum()),target_check_count=int((~is_fit).sum()),
                    target_fit_median_px=float(np.median(errors[is_fit])),target_check_median_px=float(np.median(checked)),
                    target_check_p95_px=float(np.percentile(checked,95)),target_pose_C_W=pose.tolist())
                result['gate']='PASS_CONDITIONAL_POINT_PROBE_ONLY' if is_fit.sum()>=6 and len(checked)>=5 and np.median(checked)<=3 and np.percentile(checked,95)<=8 else 'FAIL_TARGET_REPROJECTION'
                image=Image.open(package/target_frame['image']).convert('RGB');d=ImageDraw.Draw(image)
                for n,(u,v),(pu,pv) in zip(ids,observed,predicted):
                    d.ellipse((u-4,v-4,u+4,v+4),outline='lime',width=2);d.line((u,v,pu,pv),fill='red',width=2);d.text((u+4,v+4),ref['points'][n]['id'],fill='yellow')
                buf=io.BytesIO();image.save(buf,format='JPEG',quality=95);payloads['proof/target-check.jpg']=buf.getvalue()
            except ValueError as exc:result['reason']=str(exc)
    else:result['reason']='Insufficient independently checked front points for split target pose test'
    payloads['report.json']=canonical(result);payloads['reference.json']=canonical(ref);payloads['lens-candidates.json']=canonical(profiles)
    return publish(destination,stage='manual_multiview_rig_probe',payloads=payloads,
        inputs=[{'role':role,'artifact_id':m['artifact_id']} for role,m in zip(['source','masks','front_map'],manifests)],
        config={'reference':ref,'front_fit_times':fit_times,'front_check_time':check_time,'min_parallax_deg':1,'front_fit_p95_px':3,'front_check_px':5,'target_median_px':3,'target_p95_px':8},
        code_revision=digest(Path(__file__).read_bytes()),runtime={'opencv':cv2.__version__,'pycolmap':pc.__version__,'device':'cpu'})
