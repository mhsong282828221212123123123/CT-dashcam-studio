"""Fixed-rig/static-object diagnostic. No scene or camera mutation."""
import json, argparse
from pathlib import Path
import numpy as np
import cv2
from ct_scene.prototype_front import model_vertices,rotation
from ct_scene.affine_probe import camera_matrix

ap=argparse.ArgumentParser();ap.add_argument('--asset',required=True);ap.add_argument('--pose-file',type=Path);a=ap.parse_args()
case=Path('runs/new-date-suite-006/2026-08-25_08-20-56')
obs=[json.loads(s) for s in (case/'tracks/observations.jsonl').read_text().splitlines()]
frames={r['frame_id']:r for r in json.loads((case/'source/frames.json').read_text())}
poses=json.loads((a.pose_file or case/'base/trajectory.json').read_text());t=np.array([r['camera_time_s'] for r in poses]);p=np.array([r['ego_position_W'] for r in poses]);yaw=np.unwrap([r['ego_yaw_rad'] for r in poses])
cfg=json.loads(Path('configs/surrounding-prototype.json').read_text());profiles=json.loads(Path('configs/multicam-optical-candidates.json').read_text())['cameras']
vertices=np.unique(model_vertices(a.asset)[:,:3],axis=0)
def project(P,angle,frame):
    c=frame['camera_id'];m=cfg['cameras'][c];tm=frame['camera_time_s'];e=np.array([np.interp(tm,t,p[:,i]) for i in range(3)]);ey=np.interp(tm,t,yaw)
    cp=e+rotation(ey)@np.array(m['offset_W_ego_m']);cp[2]+=m['height_m'];phi=ey+np.radians(m['yaw_deg'])
    basis=np.array([[np.sin(phi),-np.cos(phi),0],[0,0,-1],[np.cos(phi),np.sin(phi),0]])
    v=(vertices@rotation(angle).T+P-cp)@basis.T
    if v[:,2].min()<=.1:return None
    K,D=camera_matrix(profiles[c]);uv=cv2.projectPoints(v,np.zeros(3),np.zeros(3),K,D)[0][:,0,:]
    return np.r_[uv.min(0),uv.max(0)]

rows=[]
for local,tm in [('front-0044',38),('right_repeater-0050',45),('right_repeater-0050',50),('right_repeater-0050',54)]:
    pool=[r for r in obs if r['local_track_id'].endswith(local) and r['support']=='detector_observation']
    if not pool:
        print('ids',sorted({r['local_track_id'] for r in obs if local.split(':')[-1] in r['local_track_id']}));continue
    r=min(pool,key=lambda r:abs(frames[r['frame_id']]['camera_time_s']-tm));f=frames[r['frame_id']];q=project(np.array([142,94,0]),np.radians(83),f)
    rows.append(dict(local=local,time=f['camera_time_s'],observed=r['bbox_xyxy'],projected=None if q is None else q.round(1).tolist()))
print(json.dumps(dict(vertices=len(vertices),rows=rows)))
samples=[]
for local in ['front-0044','right_repeater-0050']:
    for r in obs:
        if r['local_track_id']!=local or r['support']!='detector_observation':continue
        b=np.array(r['bbox_xyxy']);f=frames[r['frame_id']]
        if min(b[0],b[1],1280-b[2],960-b[3])<5 or b[2]-b[0]<80 or b[3]-b[1]<60:continue
        if samples and f['camera_time_s']-samples[-1][0]['camera_time_s']<.5:continue
        samples.append((f,b))
train=[s for s in samples if s[0]['camera_time_s']<49];test=[s for s in samples if s[0]['camera_time_s']>=49]
def residual(x,rows):
    values=[]
    for f,b in rows:
        q=project(np.r_[x[:2],0],x[2],f)
        values.extend((q-b)/np.tile(np.maximum(b[2:]-b[:2],1),2) if q is not None else [100]*4)
    return np.array(values)
x=np.array([142,94,np.radians(83.)]);lo=np.array([137,89,np.radians(63)]);hi=np.array([147,99,np.radians(103)])
def cost(x):
    r=residual(x,train);return np.mean(np.sqrt(1+(r/.1)**2)-1)
for size in [1,.5,.2,.1,.05,.02]:
    step=np.array([size,size,size*.1])
    for _ in range(20):
        candidates=[x]+[np.clip(x+sign*step*np.eye(3)[i],lo,hi) for i in range(3) for sign in [-1,1]]
        best=min(candidates,key=cost)
        if np.array_equal(best,x):break
        x=best
print(json.dumps(dict(fitted=x.tolist(),train_n=len(train),held_out_n=len(test),train_abs_median=float(np.median(abs(residual(x,train)))),held_out_abs_median=float(np.median(abs(residual(x,test)))),per_sample=[dict(camera=f['camera_id'],time=f['camera_time_s'],error=float(np.mean(abs(residual(x,[(f,b)]))))) for f,b in samples])))
