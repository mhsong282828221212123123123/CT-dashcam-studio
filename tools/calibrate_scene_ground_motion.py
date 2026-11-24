"""Extract independently checked static road correspondences and calibrate front."""
import json,sys,cv2
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_ground_calibration import fit_ground_camera
from core.scene_world_tracks import rotation

def collect(result,end=5.5,lag=3,dense_rate=0):
    root=Path(result);m=json.loads((root/'manifest.json').read_text(encoding='utf8'))
    if not m.get('source_front_path'):
        catalog=json.loads((Path(__file__).resolve().parents[2]/'scene-results/quality-rebuild/catalog/catalog.json').read_text(encoding='utf8'))
        m['source_front_path']=next(c['source'] for c in catalog['inventory'] if c['id']+'-front.mp4'==m['cameras']['front']['file'])
    rows=[json.loads(x) for x in (root/'observations.jsonl').read_text(encoding='utf8').splitlines()]
    rows=[r for r in rows if r['time_s']<=end]
    from core.decoder import RealTeslaSEIDecoder
    decoder=RealTeslaSEIDecoder(m['source_front_path'])
    for row in rows:
        frame=row['cameras']['front']['frame_index'];raw=decoder.get_frame_telemetry(frame)
        if decoder.has_sei_data and 'speed_mps' in raw:
            row['ego']['data']=dict(raw,speed_kmh=raw['speed_mps']*3.6)
    if dense_rate:
        base=rows;times=np.array([r['time_s'] for r in base]);rows=[]
        for time in np.arange(times[0],times[-1],1/dense_rate):
            nearest=min(base,key=lambda r:abs(r['time_s']-time));data=dict(nearest['ego']['data'])
            for k in ['speed_kmh','heading']:data[k]=float(np.interp(time,times,[r['ego']['data'][k] for r in base]))
            rows.append(dict(time_s=float(time),ego=dict(data=data),cameras=nearest['cameras']))
    origins=[];angles=[];pose=np.zeros(2);angle=0.
    for i,row in enumerate(rows):
        d=row['ego']['data']
        if i:
            prev=rows[i-1];pd=prev['ego']['data'];dt=row['time_s']-prev['time_s'];delta=np.radians((d['heading']-pd['heading']+180)%360-180)
            pose=pose+rotation(angle+delta/2)@np.array([0.,(d['speed_kmh']+pd['speed_kmh'])/7.2*dt]);angle+=delta
        origins.append(pose.copy());angles.append(rotation(angle))
    cap=cv2.VideoCapture(m['source_front_path']);frames=[];masks=[]
    for row in rows:
        cap.set(cv2.CAP_PROP_POS_MSEC,row['time_s']*1000);ok,f=cap.read()
        if not ok:raise ValueError('Missing original frame')
        gray=cv2.cvtColor(f,cv2.COLOR_BGR2GRAY);h,w=gray.shape;mask=np.zeros_like(gray)
        mask[int(h*.55):int(h*.82),int(w*.08):int(w*.92)]=255
        road=np.zeros_like(gray)
        cv2.fillConvexPoly(road,np.array([[.45*w,.55*h],[.57*w,.55*h],[.88*w,.82*h],[.12*w,.82*h]],np.int32),255)
        mask=cv2.bitwise_and(mask,road)
        for obj in row['cameras']['front'].get('objects',[]):
            x,y,bw,bh=obj['bbox'];cv2.rectangle(mask,(max(0,int(x)-15),max(0,int(y)-15)),(int(x+bw)+15,int(y+bh)+15),0,-1)
        frames.append(gray);masks.append(mask)
    cap.release();pairs=[]
    for i in range(0,len(rows)-lag,2):
        j=i+lag
        if np.linalg.norm(origins[j]-origins[i])<.2:continue
        points=cv2.goodFeaturesToTrack(frames[i],500,.003,7,mask=masks[i])
        if points is None:continue
        q,ok,_=cv2.calcOpticalFlowPyrLK(frames[i],frames[j],points,None,winSize=(41,41),maxLevel=5)
        back,ok2,_=cv2.calcOpticalFlowPyrLK(frames[j],frames[i],q,None,winSize=(41,41),maxLevel=5)
        a=points.reshape(-1,2);b=q.reshape(-1,2)
        valid=(ok.ravel()!=0)&(ok2.ravel()!=0)&(np.linalg.norm(back.reshape(-1,2)-a,axis=1)<1.)
        pixels=np.rint(b).astype(int);pixels[:,0]=np.clip(pixels[:,0],0,1279);pixels[:,1]=np.clip(pixels[:,1],0,959)
        valid &= masks[j][pixels[:,1],pixels[:,0]]>0
        a,b=a[valid],b[valid]
        if len(a)<30:continue
        _,inliers=cv2.findHomography(a,b,cv2.RANSAC,1.2)
        if inliers is None:continue
        a,b=a[inliers.ravel()!=0],b[inliers.ravel()!=0]
        if len(a)<30:continue
        pairs.append(dict(time_a=rows[i]['time_s'],time_b=rows[j]['time_s'],a=a,b=b,
            origin_a=origins[i],origin_b=origins[j],R_a=angles[i],R_b=angles[j]))
    return pairs,m

if __name__=='__main__':
    root=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(exist_ok=True,parents=True)
    pairs,m=collect(root,end=float(sys.argv[3]) if len(sys.argv)>3 else 5.5,lag=int(sys.argv[4]) if len(sys.argv)>4 else 3,dense_rate=float(sys.argv[5]) if len(sys.argv)>5 else 0)
    if not pairs:raise ValueError('No supported static road correspondences; calibration not produced')
    knots=np.arange(pairs[0]['time_a'],pairs[-1]['time_b']+.5,.5)
    report={}
    for key,f in [('legacy_focal_reoptimized_pitch',850.),('free_focal',None)]:
        report[key]=fit_ground_camera(pairs,knots,fixed_focal=f);print(key,report[key],flush=True)
    report['source']=m['source_front_path'];report['pair_count']=len(pairs)
    report['correspondences']=sum(len(p['a']) for p in pairs)
    (out/'front-ground-motion.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    np.savez_compressed(out/'ground-correspondences.npz',**{f'{i}_{k}':v for i,p in enumerate(pairs) for k,v in p.items()})
