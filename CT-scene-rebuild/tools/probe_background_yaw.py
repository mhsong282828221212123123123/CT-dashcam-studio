"""Image/own-SEI rotation comparison; output evidence only, never edits ego."""
import argparse,json
from pathlib import Path
import cv2,numpy as np
from PIL import Image
from ct_scene.affine_probe import camera_matrix
from ct_scene.artifacts import canonical
ap=argparse.ArgumentParser();ap.add_argument('--case',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--camera',default='front');ap.add_argument('--start',type=float,default=30);ap.add_argument('--end',type=float,default=56);a=ap.parse_args();a.output.mkdir(exist_ok=False)
fs=sorted((f for f in json.loads((a.case/'source/frames.json').read_text()) if f['camera_id']==a.camera and a.start<=f['camera_time_s']<=a.end),key=lambda f:f['camera_time_s'])
obs={}
for r in map(json.loads,(a.case/'tracks/observations.jsonl').read_text().splitlines()):obs.setdefault(r['frame_id'],[]).append(r)
poses=json.loads((a.case/'base/trajectory.json').read_text());times=[r['camera_time_s'] for r in poses];yaw=np.unwrap([r['ego_yaw_rad'] for r in poses]);K,D=camera_matrix(json.loads(Path('configs/multicam-optical-candidates.json').read_text())['cameras'][a.camera]);cv2.setNumThreads(4);cv2.setRNGSeed(20261115)
rows=[];previous=None;accum=0.
for f in fs:
    image=np.asarray(Image.open(a.case/'source'/f['image']).convert('L'));h,w=image.shape;mask=np.zeros_like(image);mask[80:850,30:w-30]=255
    if a.camera=='right_repeater':mask[:300,700:]=0
    if a.camera=='left_repeater':mask[:300,:550]=0
    for r in obs.get(f['frame_id'],[]):
        b=r['bbox_xyxy'];x0,y0,x1,y1=np.round(b).astype(int);mask[max(0,y0-15):min(h,y1+15),max(0,x0-15):min(w,x1+15)]=0
    if previous:
        pf,im,ma=previous;p=cv2.goodFeaturesToTrack(im,1500,.01,8,mask=ma,blockSize=7)
        if p is not None:
            q,s,_=cv2.calcOpticalFlowPyrLK(im,image,p,None,winSize=(25,25),maxLevel=4);back,bs,_=cv2.calcOpticalFlowPyrLK(image,im,q,None,winSize=(25,25),maxLevel=4)
            p=p[:,0];q=q[:,0];back=back[:,0];good=(s[:,0]>0)&(bs[:,0]>0)&(np.linalg.norm(back-p,axis=1)<1)&(q[:,0]>=0)&(q[:,0]<w)&(q[:,1]>=0)&(q[:,1]<h)
            ids=np.flatnonzero(good);xy=np.clip(np.rint(q[ids]).astype(int),[0,0],[w-1,h-1]);ids=ids[mask[xy[:,1],xy[:,0]]>0];p=p[ids];q=q[ids]
            dt=f['camera_time_s']-pf['camera_time_s'];prior=np.interp(f['camera_time_s'],times,yaw)-np.interp(pf['camera_time_s'],times,yaw)
            row=dict(start_s=pf['camera_time_s'],end_s=f['camera_time_s'],matches=len(p),sei_delta_deg=float(np.degrees(prior)),accepted=False)
            if len(p)>=40:
                pn=cv2.undistortPoints(p[:,None].astype(float),K,D);qn=cv2.undistortPoints(q[:,None].astype(float),K,D);E,inliers=cv2.findEssentialMat(pn,qn,np.eye(3),method=cv2.RANSAC,prob=.999,threshold=1./K[0,0])
                if E is not None and E.shape==(3,3):
                    count,R,T,valid=cv2.recoverPose(E,pn,qn,np.eye(3),mask=inliers.copy());angle=float(np.arctan2(R[0,2],R[2,2]));tilt=float(np.arccos(np.clip(R[1,1],-1,1)));cells={(int(x/w*4),int(y/h*3)) for x,y in p[valid[:,0]>0]}
                    ecount=int(np.count_nonzero(inliers));ecells={(int(x/w*4),int(y/h*3)) for x,y in p[inliers[:,0]>0]}
                    okay=count>=40 and ecount/len(p)>.6 and len(ecells)>=5 and tilt<np.radians(3) and abs(angle-prior)<np.radians(8)
                    row.update(accepted=bool(okay),inliers=count,cells=len(cells),epipolar_inliers=ecount,epipolar_cells=len(ecells),visual_delta_deg=float(np.degrees(angle)),tilt_deg=float(np.degrees(tilt)))
                    accum+=angle if okay else prior
                else:accum+=prior
            else:accum+=prior
            row['accumulated_visual_yaw_deg']=float(np.degrees(accum));row['accumulated_sei_yaw_deg']=float(np.degrees(np.interp(f['camera_time_s'],times,yaw)-np.interp(fs[0]['camera_time_s'],times,yaw)));rows.append(row)
    previous=(f,image,mask)
(a.output/'review.json').write_bytes(canonical(dict(scope='diagnostic only, no trajectory mutation',rows=rows)))
print(json.dumps(dict(pairs=len(rows),accepted=sum(r['accepted'] for r in rows),samples=[r for i,r in enumerate(rows) if i%10==0]),indent=2))
