"""Raw-image median-side evidence, independent of noisy metric car contacts.

Whole-body evidence, or repeated approaching narrow-body centre evidence,
can exclude a front vehicle beyond the median. Observed crossings,
partial bodies, missing paint and ego turns abstain; repeater geometry is not
used. Future own-side observations veto an exclusion.
"""
from collections import defaultdict
from pathlib import Path
import cv2
import numpy as np


def review(actors,segments,times,yaws):
    lines=defaultdict(list)
    for row in segments:
        if row['kind']=='yellow':lines[row['frame_id']].append(row['pixels'])
    rate=np.gradient(np.unwrap(yaws),times);result={}
    for actor in actors:
        if actor['camera_id']!='front':continue
        evidence=[]
        for row in actor['samples']:
            if row.get('support')!='detector_observation' or 'bbox_xyxy' not in row:continue
            if abs(np.interp(row['time_s'],times,rate))>np.radians(3):continue
            w=row.get('image_width',1280);h=row.get('image_height',960);box=row['bbox_xyxy'];contact_y=box[3];candidates=[]
            if box[2]-box[0]<60 or box[3]-box[1]<50:continue
            for x,y,X,Y in lines[row['frame_id']]:
                if abs(Y-y)<h*.04:continue
                slope=(X-x)/(Y-y)
                if not -4<slope<-.05:continue
                bottom=x+(h*.9-y)*slope
                if not -w*.1<bottom<w*.45:continue
                gap=max(0,min(y,Y)-contact_y,contact_y-max(y,Y))
                if gap>h*.15:continue
                at_contact=x+(contact_y-y)*slope
                candidates.append((gap,at_contact,[x,y,X,Y]))
            if not candidates:continue
            gap,median_x,line=min(candidates,key=lambda p:p[0]);margin=w*.02
            beyond=box[2]+margin<median_x;own=(box[0]+box[2])/2>median_x+margin
            evidence.append(dict(frame_id=row['frame_id'],time_s=row['time_s'],wholly_beyond=bool(beyond),own_side=bool(own),
                                 median_x_at_contact=float(median_x),median_pixels=line,bbox_xyxy=box))
        beyond=[r for r in evidence if r['wholly_beyond']];own=[r for r in evidence if r['own_side']]
        approaching_crossing=False
        if len(evidence)>=5:
            t=np.array([r['time_s'] for r in evidence]);gap=np.array([r['bbox_xyxy'][2]-r['median_x_at_contact'] for r in evidence])
            trend=np.linalg.lstsq(np.c_[t-t[0],np.ones(len(t))],gap,rcond=None)[0][0]
            approaching_crossing=trend>actor['samples'][0].get('image_width',1280)*.02
        if not evidence:continue
        boxes=np.asarray([r['bbox_xyxy'] for r in evidence],float)
        aspect=float(np.median((boxes[:,2]-boxes[:,0])/np.maximum(1,boxes[:,3]-boxes[:,1])))
        ts=np.asarray([r['time_s'] for r in evidence]);centres=(boxes[:,0]+boxes[:,2])/2
        xtrend=float(np.linalg.lstsq(np.c_[ts-ts[0],np.ones(len(ts))],centres,rcond=None)[0][0])
        bottom_trend=float(np.linalg.lstsq(np.c_[ts-ts[0],np.ones(len(ts))],boxes[:,3],rcond=None)[0][0])
        scale_trend=float(np.linalg.lstsq(np.c_[ts-ts[0],np.ones(len(ts))],np.log(np.maximum(1,boxes[:,2]-boxes[:,0])),rcond=None)[0][0])
        # A car leaving the frame becomes a smaller *clipped box*, not a
        # retreating car. Establish looming from repeated whole clear bodies
        # when available; retain every median-side/crossing observation below.
        width=actor['samples'][0].get('image_width',1280)
        clear=(boxes[:,0]>6)&(boxes[:,2]<width-6)
        if np.count_nonzero(clear)>=5 and np.ptp(ts[clear])>=.8:
            ct=ts[clear];cb=boxes[clear];design=np.c_[ct-ct[0],np.ones(len(ct))]
            clear_bottom=float(np.linalg.lstsq(design,cb[:,3],rcond=None)[0][0])
            clear_scale=float(np.linalg.lstsq(design,np.log(np.maximum(1,cb[:,2]-cb[:,0])),rcond=None)[0][0])
            if clear_bottom>actor['samples'][0].get('image_height',960)*.025 and clear_scale>.04:
                bottom_trend=clear_bottom;scale_trend=clear_scale
        # A side view alone is ambiguous: an approaching oncoming car becomes
        # oblique too. Require increasing apparent size AND a descending contact
        # before overriding that veto. Lateral approach to our side still vetoes.
        looming=bottom_trend>actor['samples'][0].get('image_height',960)*.025 and scale_trend>.04
        approaching_crossing=approaching_crossing or (aspect>1.8 and not looming) or xtrend>actor['samples'][0].get('image_width',1280)*.02
        centre_beyond=[r for r in evidence if (r['bbox_xyxy'][0]+r['bbox_xyxy'][2])/2+max(actor['samples'][0].get('image_width',1280)*.008,(r['bbox_xyxy'][2]-r['bbox_xyxy'][0])*.1)<r['median_x_at_contact']]
        narrow_oncoming=looming and aspect<1.8 and len(centre_beyond)>=8 and len(centre_beyond)>=len(evidence)*.8 and np.ptp([r['time_s'] for r in centre_beyond])>=1.2
        whole_beyond=len(beyond)>=5 and len(beyond)>=len(evidence)*.6 and np.ptp([r['time_s'] for r in beyond])>=.8
        excluded=(whole_beyond or narrow_oncoming) and len(own)<3 and not approaching_crossing
        exclusion_reason='repeated_whole_body_beyond_raw_yellow_median; no_future_crossing' if whole_beyond else 'repeated_approaching_narrow_body_centre_beyond_raw_yellow; no_own_side_or_crossing'
        if evidence:result[actor['id']]=dict(excluded=bool(excluded),reason=exclusion_reason if excluded else 'side_or_crossing_evidence_abstained' if approaching_crossing else 'insufficient_or_own_side_observations',
                                            beyond_samples=len(beyond),own_side_samples=len(own),evidence=evidence,
                                            metric_projection_used=False,ego_modified=False,crossing_guard=bool(approaching_crossing),median_body_aspect=aspect,
                                            approaching_size_and_contact=bool(looming),centre_beyond_samples=len(centre_beyond),narrow_approaching_body_beyond_median=bool(narrow_oncoming))
    return result


def extract_lines(package,frames,observations):
    """Pixel-only median candidates; no ground-plane projection or road fitting."""
    result=[]
    for f in frames:
        if f['camera_id']!='front':continue
        im=cv2.imread(str(Path(package)/f['image']));h,w=im.shape[:2];hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
        contrast=cv2.subtract(hsv[:,:,2],cv2.GaussianBlur(hsv[:,:,2],(0,0),5))
        mask=((hsv[:,:,0]>=12)&(hsv[:,:,0]<=40)&(hsv[:,:,1]>30)&(hsv[:,:,2]>65)&(contrast>4)).astype('uint8')*255
        mask[:int(h*.55)]=0;mask[int(h*.97):]=0
        for row in observations.get(f['frame_id'],[]):
            x,y,X,Y=np.rint(row['bbox_xyxy']).astype(int);mask[max(0,y-4):min(h,Y+4),max(0,x-4):min(w,X+4)]=0
        lines=cv2.HoughLinesP(mask,1,np.pi/180,35,minLineLength=60,maxLineGap=12)
        if lines is None:continue
        for x,y,X,Y in lines.reshape(-1,4):
            if abs(Y-y)<h*.04:continue
            slope=(X-x)/(Y-y);bottom=x+(h*.9-y)*slope
            if -4<slope<-.05 and -w*.1<bottom<w*.45:
                result.append(dict(kind='yellow',frame_id=f['frame_id'],time_s=f['camera_time_s'],pixels=[int(x),int(y),int(X),int(Y)],support='raw_pixel_side_only; never_metric_ground_paint'))
    return result
