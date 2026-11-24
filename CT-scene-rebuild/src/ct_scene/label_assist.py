"""Camera-local annotation proposals, never automatic ground truth or 3D tracks."""
from collections import defaultdict
from pathlib import Path
import io
import json
import cv2
import numpy as np
from PIL import Image, ImageDraw
from .artifacts import canonical, digest, publish, verify


def flow_box(previous, current, box):
    """FB checked KLT + robust similarity. No motion if support is insufficient."""
    h,w=previous.shape
    x1,y1,x2,y2=box
    mask=np.zeros_like(previous)
    mx,my=(x2-x1)*.12,(y2-y1)*.12
    mask[max(0,int(y1+my)):min(h,int(y2-my)),max(0,int(x1+mx)):min(w,int(x2-mx))]=255
    a=cv2.goodFeaturesToTrack(previous,150,.02,5,mask=mask)
    if a is None or len(a)<12:return None,{'reason':'insufficient_features'}
    b,s,_=cv2.calcOpticalFlowPyrLK(previous,current,a,None,winSize=(25,25),maxLevel=3)
    if b is None:return None,{'reason':'forward_flow_failed'}
    back,r,_=cv2.calcOpticalFlowPyrLK(current,previous,b,None,winSize=(25,25),maxLevel=3)
    if back is None:return None,{'reason':'backward_flow_failed'}
    good=(s.ravel()==1)&(r.ravel()==1)&(np.linalg.norm(back-a,axis=2).ravel()<1.5)
    if good.sum()<12:return None,{'reason':'insufficient_fb_support'}
    M,inliers=cv2.estimateAffinePartial2D(a[good],b[good],method=cv2.RANSAC,ransacReprojThreshold=2,maxIters=1000)
    if M is None or inliers.mean()<.65:return None,{'reason':'inconsistent_object_flow'}
    scale=float(np.hypot(M[0,0],M[1,0]))
    angle=float(np.degrees(np.arctan2(M[1,0],M[0,0])))
    if not .85<scale<1.18 or abs(angle)>4:return None,{'reason':'similarity_outside_assist_range'}
    # Rotating an axis-aligned enclosure on every step inflates it indefinitely.
    # Use the robust centre and isotropic scale; this remains an annotation hint.
    center=M@np.array([(x1+x2)/2,(y1+y2)/2,1])
    half=np.array([x2-x1,y2-y1])*scale/2
    lo,hi=center-half,center+half
    result=np.r_[np.maximum(lo,[0,0]),np.minimum(hi,[w,h])]
    if not (result[2]>result[0] and result[3]>result[1]):return None,{'reason':'empty_box'}
    return result,{'features':int(good.sum()),'inlier_ratio':float(inliers.mean()),'scale':scale,'image_rotation_deg':angle}


def propose(package, reference_artifact, destination):
    package,reference_artifact=Path(package),Path(reference_artifact)
    pm,rm=verify(package),verify(reference_artifact)
    frames=json.loads((package/'frames.json').read_text(encoding='utf-8'))
    refs=json.loads((reference_artifact/'sparse-references.json').read_text(encoding='utf-8'))['reference_labels']
    groups=defaultdict(list)
    for f in frames:groups[f['camera_id']].append(f)
    by_id={f['frame_id']:f for f in frames}
    objects=defaultdict(list)
    for ref in refs:
        if ref['space_id']!='raw':raise ValueError('Only raw reference pixels supported')
        objects[ref['object_id']].append(ref)
    records=[]
    cv2.setRNGSeed(20261114);cv2.setNumThreads(2)
    for identity,anchors in objects.items():
        cameras={by_id[a['frame_id']]['camera_id'] for a in anchors}
        if len(cameras)!=1:raise ValueError('Annotation assist cannot infer cross-camera identity')
        sequence=sorted(groups[next(iter(cameras))],key=lambda f:f['camera_time_s'])
        positions={f['frame_id']:i for i,f in enumerate(sequence)}
        images=[np.asarray(Image.open(package/f['image']).convert('L')) for f in sequence]
        estimates=defaultdict(list)
        for anchor in anchors:
            i=positions[anchor['frame_id']]
            for direction in (-1,1):
                box=np.array(anchor['bbox_xyxy'],float)
                estimates[i].append((box,{'method':'manual_anchor','anchor_id':anchor['frame_id']}))
                j=i+direction
                while 0<=j<len(sequence):
                    next_box,diag=flow_box(images[j-direction],images[j],box)
                    if next_box is None:break
                    estimates[j].append((next_box,dict(diag,method='klt_propagation',anchor_id=anchor['frame_id'])))
                    box=next_box;j+=direction
        anchor_map={r['frame_id']:r for r in anchors}
        for i,f in enumerate(sequence):
            options=estimates.get(i,[])
            support_ids=sorted({d['anchor_id'] for b,d in options})
            if f['frame_id'] in anchor_map:
                box=anchor_map[f['frame_id']]['bbox_xyxy'];spread=0.
            elif options:
                boxes=np.array([b for b,d in options]);box=np.median(boxes,axis=0).tolist()
                spread=float(np.max(np.abs(boxes-np.median(boxes,axis=0)))) if len(support_ids)>1 else None
            else:box=None;spread=None
            records.append(dict(frame_id=f['frame_id'],object_id=identity,camera_id=f['camera_id'],
                                camera_time_s=f['camera_time_s'],bbox_xyxy=box,
                                anchor_disagreement_px=spread,proposal_sources=[d for b,d in options],
                                support_anchor_ids=support_ids,
                                reviewed=False,score_eligible=False,space_id='raw',
                                status='proposal' if box is not None else 'unresolved'))
    payloads={'proposals.jsonl':b''.join(canonical(r)+b'\n' for r in records)}
    for identity in objects:
        rows=[r for r in records if r['object_id']==identity]
        for start in range(0,len(rows),35):
            sheet=Image.new('RGB',(5*320,7*220),(25,25,25));draw=ImageDraw.Draw(sheet)
            for k,row in enumerate(rows[start:start+35]):
                f=by_id[row['frame_id']];image=Image.open(package/f['image']).convert('RGB');b=row['bbox_xyxy']
                if b is None:crop=(0,0,image.width,image.height)
                else:
                    x1,y1,x2,y2=b;pad=max(x2-x1,y2-y1)*.2
                    crop=(max(0,int(x1-pad)),max(0,int(y1-pad)),min(image.width,int(x2+pad+1)),min(image.height,int(y2+pad+1)))
                tile=image.crop(crop);scale=min(316/tile.width,188/tile.height)
                tile=tile.resize((max(1,round(tile.width*scale)),max(1,round(tile.height*scale))))
                x,y=(k%5)*320,(k//5)*220
                sheet.paste(tile,(x,y+30))
                if b is not None:draw.rectangle([x+(b[0]-crop[0])*scale,y+30+(b[1]-crop[1])*scale,x+(b[2]-crop[0])*scale,y+30+(b[3]-crop[1])*scale],outline='yellow',width=2)
                spread=row['anchor_disagreement_px']
                draw.text((x+3,y+2),f"{start+k} t={f['relative_request_s']:.1f}s spread={spread if spread is None else round(spread,1)}",fill='white')
            buf=io.BytesIO();sheet.save(buf,format='JPEG',quality=90)
            payloads[f'review/{identity}-{start//35:02d}.jpg']=buf.getvalue()
    report=dict(annotation_only=True,proposals=len(records),with_box=sum(r['bbox_xyxy'] is not None for r in records),
                reviewed=0,score_eligible=0,warning='No propagated label is ground truth until explicit source review. Not a production object tracker.')
    payloads['report.json']=canonical(report)
    return publish(destination,stage='annotation_proposals',payloads=payloads,
                   inputs=[{'role':'raw_package','artifact_id':pm['artifact_id']},{'role':'manual_references','artifact_id':rm['artifact_id']}],
                   config={'fb_error_px':1.5,'min_features':12,'manual_anchors_only':True},
                   code_revision=digest(Path(__file__).read_bytes()),runtime={'opencv':cv2.__version__})
