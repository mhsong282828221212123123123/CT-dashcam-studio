"""Static lane-vanishing-point experiment. Never changes rig/pose predictions."""
import argparse,json,math
from pathlib import Path
import cv2,numpy as np
from ct_scene.affine_probe import camera_matrix

ap=argparse.ArgumentParser();ap.add_argument('--case',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(exist_ok=False)
cfg=json.loads(Path('configs/multicam-optical-candidates.json').read_text())['cameras']['front'];K,D=camera_matrix(cfg)
frames=[f for f in json.loads((a.case/'source/frames.json').read_text()) if f['camera_id']=='front'];masks={r['frame_id']:r for r in json.loads((a.case/'road-masks/frames.json').read_text())}
obs={}
for r in map(json.loads,(a.case/'tracks/observations.jsonl').read_text().splitlines()):obs.setdefault(r['frame_id'],[]).append(r)
poses=json.loads((a.case/'base/trajectory.json').read_text());t=np.array([r['camera_time_s'] for r in poses]);p=np.array([r['ego_position_W'] for r in poses]);yaw=np.unwrap([r['ego_yaw_rad'] for r in poses]);rate=np.gradient(yaw,t);speed=np.linalg.norm(np.gradient(p,t,axis=0),axis=1)
rows=[]
for f in frames[::3]:
    now=f['camera_time_s']
    if abs(np.interp(now,t,rate))>np.radians(1.5) or np.interp(now,t,speed)<2:continue
    im=cv2.imread(str(a.case/'source'/f['image']));h,w=im.shape[:2];hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
    lane=cv2.imread(str(a.case/'road-masks'/masks[f['frame_id']]['lane']),0)
    binary=((hsv[:,:,1]<80)&(hsv[:,:,2]>110)&(cv2.dilate(lane,np.ones((7,7),'uint8'))>0)).astype('uint8')*255
    binary[:int(h*.5)]=0;binary[int(h*.92):]=0
    for r in obs.get(f['frame_id'],[]):
        x,y,X,Y=np.rint(r['bbox_xyxy']).astype(int);binary[max(0,y-5):min(h,Y+5),max(0,x-5):min(w,X+5)]=0
    lines=cv2.HoughLinesP(binary,1,np.pi/180,40,minLineLength=65,maxLineGap=20)
    if lines is None:continue
    left=[];right=[]
    for x,y,X,Y in lines.reshape(-1,4):
        if abs(Y-y)<40:continue
        slope=(X-x)/(Y-y)
        if not .08<abs(slope)<3:continue
        q=cv2.undistortPoints(np.array([[[x,y]],[[X,Y]]],float),K,D).reshape(2,2);line=np.cross(np.r_[q[0],1],np.r_[q[1],1])
        (left if slope<0 else right).append((line,[int(x),int(y),int(X),int(Y)]))
    points=[]
    for l in left[:25]:
        for r in right[:25]:
            q=np.cross(l[0],r[0])
            if abs(q[2])<1e-9:continue
            q=q[:2]/q[2];u,v=K[0,0]*q[0]+K[0,2],K[1,1]*q[1]+K[1,2]
            if w*.2<u<w*.8 and h*.25<v<h*.65:points.append((float(u),float(v),l[1],r[1]))
    if len(points)<4:continue
    bins={}
    for q in points:bins.setdefault(round(q[1]/10)*10,[]).append(q)
    mode=max(bins,key=lambda k:len(bins[k]));good=[q for q in points if abs(q[1]-mode)<15]
    if len(good)<4:continue
    u,v=np.median(np.array([q[:2] for q in good]),axis=0);pitch=math.atan((K[1,2]-v)/K[1,1])
    rows.append(dict(frame_id=f['frame_id'],time_s=now,vanishing_point_undistorted=[float(u),float(v)],pitch_deg=float(np.degrees(pitch)),pair_votes=len(good),pixels=good[0][2:]))
    if len(rows)<=3:
        und=cv2.undistort(im,K,D)
        cv2.circle(und,(round(u),round(v)),9,(0,0,255),3);cv2.line(und,(0,round(v)),(w-1,round(v)),(0,0,255),2)
        cv2.putText(und,f'estimated horizon / pitch {np.degrees(pitch):.2f} deg',(20,30),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,0,255),2)
        cv2.imwrite(str(a.output/f'raw-static-vp-{len(rows)}.jpg'),und)
values=[r['pitch_deg'] for r in rows];spread=float(np.quantile(values,.8)-np.quantile(values,.2)) if values else None
report=dict(status='CANDIDATE_NOT_ADOPTED' if len(rows)>=8 and spread<1 else 'INSUFFICIENT_OR_UNSTABLE_STATIC_VP',frames=len(rows),median_pitch_deg=float(np.median(values)) if values else None,spread_80_20_deg=spread,ego_modified=False,actor_modified=False,rig_modified=False,scope='static ground-plane pitch experiment only; no focal/rig fitting and no actor GT used')
(a.output/'report.json').write_text(json.dumps(report,indent=2));(a.output/'frame-evidence.json').write_text(json.dumps(rows,indent=2));print(report)
