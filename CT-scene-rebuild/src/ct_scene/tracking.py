"""Camera-local raw tracking plus offline tracklet stitching.

Detector observations stay separate from reconstructed gaps. No metric/world
position is inferred here, including for the rear reference camera.
"""
from pathlib import Path
import json
import cv2
import numpy as np
from PIL import Image
from .artifacts import verify,publish,canonical,digest


def iou(a,b):
    a,b=np.asarray(a,float),np.asarray(b,float)
    lo=np.maximum(a[:2],b[:2]);hi=np.minimum(a[2:],b[2:]);inter=float(np.prod(np.maximum(0,hi-lo)))
    area=float(np.prod(a[2:]-a[:2])+np.prod(b[2:]-b[:2])-inter)
    return inter/max(area,1e-9)


def assignment(cost,limit):
    """Global minimum assignment with explicit unmatched dummy columns."""
    cost=np.asarray(cost,float)
    if cost.ndim!=2:raise ValueError('Assignment matrix required')
    n,m=cost.shape
    if not n or not m:return []
    a=np.column_stack([np.where(np.isfinite(cost),cost,1e6),np.full((n,n),limit)])
    width=a.shape[1];u=np.zeros(n+1);v=np.zeros(width+1);p=np.zeros(width+1,int);way=np.zeros(width+1,int)
    for row in range(1,n+1):
        p[0]=row;j0=0;minimum=np.full(width+1,np.inf);used=np.zeros(width+1,bool)
        while True:
            used[j0]=True;i0=p[j0];delta=np.inf;j1=0
            for j in range(1,width+1):
                if used[j]:continue
                current=a[i0-1,j-1]-u[i0]-v[j]
                if current<minimum[j]:minimum[j]=current;way[j]=j0
                if minimum[j]<delta:delta=minimum[j];j1=j
            for j in range(width+1):
                if used[j]:u[p[j]]+=delta;v[j]-=delta
                else:minimum[j]-=delta
            j0=j1
            if not p[j0]:break
        while True:
            j1=way[j0];p[j0]=p[j1];j0=j1
            if not j0:break
    return [(p[j]-1,j-1) for j in range(1,m+1) if p[j] and cost[p[j]-1,j-1]<limit]


def appearance(rgb,box):
    x1,y1,x2,y2=np.rint(box).astype(int);h,w=rgb.shape[:2]
    dx,dy=(x2-x1)*.15,(y2-y1)*.12
    crop=rgb[max(0,int(y1+dy)):min(h,int(y2-dy)),max(0,int(x1+dx)):min(w,int(x2-dx))]
    if crop.size<48:return np.zeros(128,np.float32)
    hsv=cv2.cvtColor(cv2.resize(crop,(48,48)),cv2.COLOR_RGB2HSV)
    hist=cv2.calcHist([hsv],[0,1,2],None,[8,4,4],[0,180,0,256,0,256]).reshape(-1)
    return hist/max(float(hist.sum()),1e-9)


def appearance_distance(a,b):
    # Hellinger distance; colour is a weak cue, never sufficient for global ID.
    return float(np.sqrt(max(0,1-np.sqrt(np.asarray(a)*np.asarray(b)).sum())))


def family(cls):
    return 'four_wheeler' if cls in (2,5,7) else 'two_wheeler' if cls in (1,3) else str(cls)


def suppress_family_duplicates(detections,threshold=.75):
    kept=[];removed=[]
    for d in sorted(detections,key=lambda d:d['score'],reverse=True):
        if any(family(d['class_id'])==family(k['class_id']) and iou(d['bbox_xyxy'],k['bbox_xyxy'])>threshold for k in kept):removed.append(d)
        else:kept.append(d)
    return kept,removed


def predict(track,t):
    dt=t-track['last_time'];box=np.array(track['last_box'])+np.tile(track['velocity']*dt,2)
    return box


def match_cost(track,detection,t):
    if family(track['class_id'])!=family(detection['class_id']):return 1e6
    box=predict(track,t);observed=np.array(detection['bbox_xyxy']);overlap=iou(box,observed)
    centre=(observed[:2]+observed[2:]-box[:2]-box[2:])/2
    size=np.maximum(box[2:]-box[:2],20);motion=float(np.linalg.norm(centre/size))
    shape=float(np.max(np.abs(np.log(np.maximum(observed[2:]-observed[:2],1)/size))))
    app=appearance_distance(track['appearance'],detection['appearance'])
    if app>.65:return 1e6
    border_growth=False
    if 'truncated' in track.get('last_quality_flags',[]) and 'truncated' in detection.get('quality_flags',[]) and t-track['last_time']<=.4:
        same_vertical_edge=(box[0]<=2 and observed[0]<=2) or (abs(box[2]-observed[2])<=2 and abs(box[0]-observed[0])>2)
        old_size=np.maximum(box[2:]-box[:2],1);new_size=np.maximum(observed[2:]-observed[:2],1)
        ratios=np.maximum(new_size/old_size,old_size/new_size)
        border_growth=bool(same_vertical_edge and ratios[0]<=4 and ratios[1]<=2 and overlap>=.1)
    if motion>2 or (shape>np.log(2.2+min(t-track['last_time'],1.2)) and not border_growth) or (overlap<.01 and app>.45):return 1e6
    return .45*(1-overlap)+.25*min(motion,2)/2+.15*app+.15*min(shape/np.log(2),1)


def update(track,detection,t,frame,support='detector_observation'):
    box=np.array(detection['bbox_xyxy'],float);old=np.array(track['last_box']);dt=t-track['last_time']
    if dt>0:
        velocity=((box[:2]+box[2:])-(old[:2]+old[2:]))/(2*dt)
        track['velocity']=.5*track['velocity']+.5*velocity
    track['last_box']=box;track['last_time']=t;track['class_id']=detection['class_id']
    track['last_quality_flags']=list(detection['quality_flags'])
    if support=='detector_observation':
        track['appearance']=.8*track['appearance']+.2*detection['appearance'];track['last_detection_time']=t
    row={'frame_id':frame['frame_id'],'camera_id':frame['camera_id'],'camera_time_s':t,
         'bbox_xyxy':box.tolist(),'observed_bbox_xyxy':box.tolist() if support=='detector_observation' else None,'score':detection['score'],'class_id':detection['class_id'],
         'support':support,'image_presence':'visible','position_support':'unresolved','world_position':None,
         'quality_flags':detection['quality_flags']}
    if support=='flow_observation':row.update(flow_inliers=detection['flow_inliers'],previous_frame_id=detection['previous_frame_id'])
    track['observations'].append(row)


def flow_box(previous,current,box):
    h,w=previous.shape;box=np.asarray(box,float);lo=box[:2]+.2*(box[2:]-box[:2]);hi=box[2:]-.15*(box[2:]-box[:2])
    mask=np.zeros_like(previous);x1,y1=np.maximum(np.rint(lo).astype(int),0);x2,y2=np.minimum(np.rint(hi).astype(int),[w,h])
    mask[y1:y2,x1:x2]=255
    p=cv2.goodFeaturesToTrack(previous,100,.01,5,mask=mask)
    if p is None or len(p)<16:return None
    q,s,_=cv2.calcOpticalFlowPyrLK(previous,current,p,None,winSize=(25,25),maxLevel=4)
    if q is None:return None
    back,bs,_=cv2.calcOpticalFlowPyrLK(current,previous,q,None,winSize=(25,25),maxLevel=4)
    if back is None:return None
    good=(s.ravel()==1)&(bs.ravel()==1)&(np.linalg.norm(back.reshape(-1,2)-p.reshape(-1,2),axis=1)<1.5)
    if good.sum()<12:return None
    transform,inliers=cv2.estimateAffinePartial2D(p[good],q[good],method=cv2.RANSAC,ransacReprojThreshold=2)
    if transform is None or inliers is None or inliers.sum()<12 or inliers.mean()<.6:return None
    scale=np.linalg.norm(transform[0,:2])
    if not .7<scale<1.4:return None
    centre=(box[:2]+box[2:])/2;new_centre=transform[:,:2]@centre+transform[:,2];size=(box[2:]-box[:2])*scale
    result=np.r_[new_centre-size/2,new_centre+size/2];result[[0,2]]=np.clip(result[[0,2]],0,w);result[[1,3]]=np.clip(result[[1,3]],0,h)
    if np.min(result[2:]-result[:2])<8:return None
    return result,int(inliers.sum())


def stitch(tracklets,max_gap_s=3):
    """Join only unambiguous same-camera non-overlapping intervals."""
    costs={};candidates=[]
    for a in tracklets:
        for b in tracklets:
            gap=b['observations'][0]['camera_time_s']-a['last_time']
            if not 0<gap<=max_gap_s or a['camera_id']!=b['camera_id'] or family(a['class_id'])!=family(b['class_id']):continue
            predicted=predict(a,b['observations'][0]['camera_time_s']);start=np.array(b['observations'][0]['bbox_xyxy'])
            motion=np.linalg.norm(((start[:2]+start[2:])-(predicted[:2]+predicted[2:]))/2/np.maximum(predicted[2:]-predicted[:2],20))
            app=appearance_distance(a['appearance'],b['initial_appearance'])
            if motion>1.5 or app>.4:continue
            cost=.5*app+.4*motion/1.5+.1*gap/max_gap_s
            costs[a['id'],b['id']]=float(cost)
    outgoing={};incoming={}
    for (a,b),cost in costs.items():outgoing.setdefault(a,[]).append(cost);incoming.setdefault(b,[]).append(cost)
    used_end,used_start=set(),set();selected=[]
    for (a,b),cost in sorted(costs.items(),key=lambda item:item[1]):
        ambiguous=any(len(v)>1 and sorted(v)[1]-sorted(v)[0]<.12 for v in (outgoing[a],incoming[b]))
        reason='ambiguous_alternatives' if ambiguous else 'already_assigned' if a in used_end or b in used_start else 'accepted_same_camera'
        chosen=reason=='accepted_same_camera'
        candidates.append({'from':a,'to':b,'cost':cost,'selected':chosen,'reason':reason})
        if chosen:selected.append((a,b));used_end.add(a);used_start.add(b)
    root={t['id']:t['id'] for t in tracklets}
    def find(x):
        while root[x]!=x:x=root[x]
        return x
    for a,b in selected:root[find(b)]=find(a)
    return {key:find(key) for key in root},candidates


def fill_camera_gaps(observations,frames,max_gap_s=3):
    """Two observed endpoints required; no tail extrapolation or camera mixing."""
    observed=sorted(observations,key=lambda r:r['camera_time_s']);filled=[]
    if observed and any(f['camera_id']!=observed[0]['camera_id'] for f in frames):raise ValueError('Mixed camera frame grid')
    for a,b in zip(observed,observed[1:]):
        dt=b['camera_time_s']-a['camera_time_s']
        if a['camera_id']!=b['camera_id']:raise ValueError('Cannot interpolate raw pixels across cameras')
        if not 0<dt<=max_gap_s:continue
        for f in frames:
            t=f['camera_time_s']
            if a['camera_time_s']<t<b['camera_time_s']:
                alpha=(t-a['camera_time_s'])/dt;box=(1-alpha)*np.array(a['bbox_xyxy'])+alpha*np.array(b['bbox_xyxy'])
                filled.append({'frame_id':f['frame_id'],'camera_id':a['camera_id'],'camera_time_s':t,'bbox_xyxy':box.tolist(),
                    'observed_bbox_xyxy':None,'support':'offline_two_endpoint_gap','image_presence':'unknown',
                    'position_support':'unresolved','world_position':None,'score':None,'class_id':a['class_id'],
                    'endpoint_frame_ids':[a['frame_id'],b['frame_id']],'uses_future_observation':True,'quality_flags':['interpolated_raw_box_not_3d']})
    return sorted(observed+filled,key=lambda r:r['camera_time_s'])


def run(package,detections,destination):
    package,detections=Path(package),Path(detections)
    if Path(destination).exists():raise FileExistsError(destination)
    pm,dm=verify(package),verify(detections)
    if not any(i['artifact_id']==pm['artifact_id'] for i in dm['inputs']):raise ValueError('Detection/source mismatch')
    frames=json.loads((package/'frames.json').read_text());raw={r['frame_id']:r for r in map(json.loads,(detections/'observations.jsonl').read_text().splitlines())}
    tracklets=[];rejected=[];cv2.setNumThreads(2);cv2.setRNGSeed(20261114)
    for camera in sorted({f['camera_id'] for f in frames}):
        sequence=sorted((f for f in frames if f['camera_id']==camera),key=lambda f:f['camera_time_s']);live=[];counter=0;previous=None;previous_frame=None
        for f in sequence:
            t=f['camera_time_s'];live=[tr for tr in live if t-tr['last_detection_time']<=1.2]
            rgb=np.asarray(Image.open(package/f['image']).convert('RGB'));gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);dets=[]
            for d in raw[f['frame_id']]['detections']:
                d=dict(d);box=np.array(d['bbox_xyxy']);size=box[2:]-box[:2]
                if np.any(size<=0) or d['score']<.15:continue
                d['appearance']=appearance(rgb,box);d['quality_flags']=[]
                if min(box[0],box[1],f['width']-box[2],f['height']-box[3])<=2:d['quality_flags'].append('truncated')
                if np.prod(size)/(f['width']*f['height'])>.85:
                    rejected.append({'frame_id':f['frame_id'],'detection':{k:v for k,v in d.items() if k!='appearance'},'reason':'image_filling_detection_unreliable'});continue
                dets.append(d)
            dets,duplicates=suppress_family_duplicates(dets)
            for d in duplicates:rejected.append({'frame_id':f['frame_id'],'detection':{k:v for k,v in d.items() if k!='appearance'},'reason':'duplicate_same_vehicle_family_box'})
            unmatched=set(range(len(dets)));remaining=list(live)
            for high in (True,False):
                ids=[i for i in sorted(unmatched) if (dets[i]['score']>=.4)==high]
                matrix=np.array([[match_cost(tr,dets[i],t) for i in ids] for tr in remaining]).reshape(len(remaining),len(ids))
                matches=assignment(matrix,.8 if high else .68);matched_tracks=set()
                for a,b in matches:
                    update(remaining[a],dets[ids[b]],t,f);unmatched.remove(ids[b]);matched_tracks.add(a)
                remaining=[tr for i,tr in enumerate(remaining) if i not in matched_tracks]
            if previous is not None:
                for tr in remaining:
                    if abs(tr['last_time']-previous_frame['camera_time_s'])>.001:continue
                    result=flow_box(previous,gray,tr['last_box'])
                    if result is None:continue
                    box,count=result;hist=appearance(rgb,box)
                    if appearance_distance(hist,tr['appearance'])>.4:continue
                    update(tr,{'bbox_xyxy':box,'class_id':tr['class_id'],'score':None,'appearance':hist,
                        'quality_flags':['flow_estimated_box'],'flow_inliers':count,'previous_frame_id':previous_frame['frame_id']},t,f,'flow_observation')
            for i in sorted(unmatched):
                d=dets[i]
                if d['score']<.55:continue
                counter+=1;tr={'id':f'{camera}-{counter:04d}','camera_id':camera,'class_id':d['class_id'],
                    'last_time':t,'last_box':np.array(d['bbox_xyxy']),'velocity':np.zeros(2),
                    'appearance':d['appearance'].copy(),'initial_appearance':d['appearance'].copy(),'observations':[]}
                update(tr,d,t,f);live.append(tr);tracklets.append(tr)
            previous=gray;previous_frame=f
        print('Tracked camera',camera,len(sequence),'frames',counter,'tracklets',flush=True)
    identities,edges=stitch(tracklets);groups={}
    for tr in tracklets:groups.setdefault(identities[tr['id']],[]).append(tr)
    rows=[];tracks=[]
    for key,members in groups.items():
        camera=members[0]['camera_id'];sequence=[f for f in frames if f['camera_id']==camera]
        observations=[dict(r,tracklet_id=tr['id']) for tr in members for r in tr['observations']]
        completed=fill_camera_gaps(observations,sequence)
        for row in completed:rows.append(dict(row,local_track_id=key,rear_role='association_only' if camera=='back' else None))
        tracks.append({'local_track_id':key,'camera_id':camera,'tracklet_ids':[t['id'] for t in members],
            'observed_frames':len(observations),'offline_filled_frames':len(completed)-len(observations),
            'start_s':min(r['camera_time_s'] for r in observations),'end_s':max(r['camera_time_s'] for r in observations)})
    report={'source_frames':len(frames),'tracklets':len(tracklets),'local_tracks_after_stitch':len(tracks),
        'detector_observation_frames':sum(r['support']=='detector_observation' for r in rows),
        'flow_observation_frames':sum(r['support']=='flow_observation' for r in rows),
        'observations':sum(t['observed_frames'] for t in tracks),'two_endpoint_gap_frames':sum(t['offline_filled_frames'] for t in tracks),
        'joined_tracklet_edges':sum(e['selected'] for e in edges),'rejected_detection_count':len(rejected),
        'geometry':'NOT_ASSIGNED','cross_camera_identity':'NOT_ASSIGNED','rear_geometry':'DISABLED',
        'input_use':'Existing raw detector cache reused for camera-local 2D baseline, not fresh 3D observations',
        'quality_gate':'NOT_EVALUATED_AGAINST_EXHAUSTIVE_GT'}
    return publish(destination,stage='offline_camera_tracks',payloads={'report.json':canonical(report),'tracks.json':canonical(tracks),
        'observations.jsonl':b''.join(canonical(r)+b'\n' for r in rows),'stitch-review.json':canonical(edges),'rejected-detections.json':canonical(rejected)},
        inputs=[{'role':'source','artifact_id':pm['artifact_id']},{'role':'raw_detection_cache','artifact_id':dm['artifact_id']}],
        config={'high_threshold':.4,'new_threshold':.55,'live_gap_s':1.2,'offline_gap_s':3,'appearance':'HSV128_weak_cue','vehicle_family_nms_iou':.75,'max_local_appearance_distance':.65,
            'truncated_edge_max_gap_s':.4,'truncated_edge_max_width_ratio':4,'truncated_edge_max_height_ratio':2},
        code_revision=digest(Path(__file__).read_bytes()),runtime={'opencv':cv2.__version__,'device':'cpu'})
