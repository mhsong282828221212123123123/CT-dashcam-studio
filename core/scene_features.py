"""Image-derived road markings and conservative traffic-light appearance checks."""
import cv2
import numpy as np
from core.scene_analysis import iou


def detect_lane_segments(frame,objects=()):
    h,w=frame.shape[:2]
    small=cv2.resize(frame,(640,round(h*640/w)))
    sh,sw=small.shape[:2]; scale=w/sw
    hsv=cv2.cvtColor(small,cv2.COLOR_BGR2HSV)
    white=cv2.inRange(hsv,np.array([0,0,145]),np.array([179,85,255]))
    yellow=cv2.inRange(hsv,np.array([15,85,100]),np.array([40,255,255]))
    mask=cv2.bitwise_or(white,yellow)
    gray=cv2.cvtColor(small,cv2.COLOR_BGR2GRAY)
    ridge=cv2.morphologyEx(gray,cv2.MORPH_TOPHAT,np.ones((21,21),np.uint8))
    mask[ridge<18]=0
    mask[:int(sh*.53)]=0; mask[int(sh*.94):]=0
    for obj in objects:
        x,y,bw,bh=obj['bbox']
        cv2.rectangle(mask,(max(0,int(x/scale)-3),max(0,int(y/scale)-3)),
                      (min(sw,int((x+bw)/scale)+3),min(sh,int((y+bh)/scale)+3)),0,-1)
    edges=cv2.Canny(mask,60,160)
    lines=cv2.HoughLinesP(edges,1,np.pi/180,25,minLineLength=sh*.09,maxLineGap=12)
    candidates={'left':[],'right':[]}
    if lines is None: return []
    for x1,y1,x2,y2 in lines.reshape(-1,4):
        if abs(y2-y1)<sh*.065: continue
        slope=(x2-x1)/(y2-y1)
        if not .3<abs(slope)<2.7: continue
        intercept=x1-slope*y1
        foot=slope*(sh*.92)+intercept
        side='left' if slope<0 else 'right'
        if not ((-.15*sw<foot<.48*sw) if side=='left' else (.52*sw<foot<1.15*sw)): continue
        length=float(np.hypot(x2-x1,y2-y1))
        candidates[side].append((length,slope,intercept,min(y1,y2),max(y1,y2)))
    chosen=[]
    for side,items in candidates.items():
        if not items: continue
        best=max(items)
        # Aggregate only nearby collinear paint fragments, not arbitrary parallel curbs.
        group=[v for v in items if abs(v[1]-best[1])<.18 and abs((v[1]-best[1])*sh*.8+v[2]-best[2])<12]
        weights=np.array([v[0] for v in group]); slope=float(np.average([v[1] for v in group],weights=weights))
        intercept=float(np.average([v[2] for v in group],weights=weights))
        y0,y1=min(v[3] for v in group),max(v[4] for v in group)
        chosen.append({'side':side,'slope':slope,'intercept':intercept,'y0':y0,'y1':y1,
                       'support_px':float(weights.sum()*scale)})
    if len(chosen)==2:
        a,b=chosen; vy=(b['intercept']-a['intercept'])/(a['slope']-b['slope']); vx=a['slope']*vy+a['intercept']
        if not (.25*sw<vx<.75*sw and .25*sh<vy<.60*sh): return []
    result=[]
    for line in chosen:
        result.append({'id':'lane-'+line['side'],'points':[[float((line['slope']*y+line['intercept'])*scale),float(y*scale)] for y in (line['y0'],line['y1'])],
                       'source':'image_color_edges_hough','support_px':line['support_px'],'extrapolated':False})
    return result


def signal_appearance(frame,bbox):
    """Only classify color inside a trusted head ROI; glare always remains unknown."""
    x,y,w,h=map(int,bbox); roi=frame[max(0,y):min(frame.shape[0],y+h),max(0,x):min(frame.shape[1],x+w)]
    if roi.size==0: return {'state':'unknown','reason':'empty_roi'}
    hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
    bright=hsv[:,:,2]>235
    washed=bright&(hsv[:,:,1]<70)
    if washed.mean()>.06 or ((roi.max(axis=2)>250).mean()>.22):
        return {'state':'unknown','reason':'saturated_or_glare'}
    lit=(hsv[:,:,1]>120)&(hsv[:,:,2]>160)
    hue=hsv[:,:,0]
    colors={'red':lit&((hue<10)|(hue>170)), 'amber':lit&(hue>=10)&(hue<35), 'green':lit&(hue>=35)&(hue<95)}
    counts={name:int(mask.sum()) for name,mask in colors.items()}
    ordered=sorted(counts,key=counts.get,reverse=True)
    # Red lamps can acquire a yellow core when clipped by the sensor. Never turn
    # a red rim + yellow core into a confident amber state.
    rim=(hsv[:,:,1]>100)&(hsv[:,:,2]>60)&((hue<10)|(hue>170))
    red_rim=int(rim.sum())
    if ordered[0]=='amber' and red_rim>max(4,counts['amber']*.3):
        # A yellow core enclosed by one red lamp differs from a separately lit
        # amber lamp. Require a compact enclosing rim and reject mixed green.
        closed=cv2.morphologyEx(rim.astype(np.uint8),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
        contours,_=cv2.findContours(closed,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        # Sensor clipping can split the rim into disconnected arcs. A compact
        # hull may join those arcs, but cannot span separate adjacent lamps.
        rim_points=cv2.findNonZero(rim.astype(np.uint8))
        if rim_points is not None and len(rim_points)>=8:
            contours=list(contours)+[cv2.convexHull(rim_points)]
        for contour in contours:
            rx,ry,rw,rh=cv2.boundingRect(contour)
            if min(rw,rh)<5 or not .5<rw/rh<2 or max(rw,rh)>roi.shape[0]*1.2:continue
            filled=np.zeros_like(closed);cv2.drawContours(filled,[contour],-1,1,-1)
            area=int(filled.sum())
            enclosed=int((colors['amber']&(filled!=0)).sum())
            rim_support=int((rim&(filled!=0)).sum())
            if counts['amber']>=5 and enclosed>=counts['amber']*.9 and rim_support>=max(8,area*.25) and counts['green']<5:
                return {'state':'red','reason':'yellow_core_enclosed_by_red_lamp_rim'}
        return {'state':'unknown','reason':'red_rim_with_ambiguous_core'}
    if counts[ordered[0]]<max(5,roi.shape[0]*roi.shape[1]*.015) or counts[ordered[0]]<3*max(1,counts[ordered[1]]):
        return {'state':'unknown','reason':'insufficient_color_evidence'}
    return {'state':ordered[0],'reason':'color_candidate_not_direction_or_lane_assignment'}


class SignalStateTracker:
    def __init__(self): self.previous=[]; self.counter=0; self.histories={}

    def update(self,frame,detections,t):
        output=[]; used=set(); histories={}
        for detection in detections:
            x,y,w,h=detection.bbox
            if w*h>frame.shape[0]*frame.shape[1]*.03 or y+h/2>frame.shape[0]*.65: continue
            matches=[(iou(detection.bbox,p['bbox']),i,p) for i,p in enumerate(self.previous)
                     if i not in used and 0<t-p['time_s']<.5]
            score,index,old=max(matches,key=lambda v:v[0],default=(0,None,None))
            if score>.3: used.add(index)
            else: self.counter+=1; old=None
            appearance=signal_appearance(frame,detection.bbox)
            # The supplied Korean-road samples contain pedestrian heads too.
            # Until symbol recognition exists, only horizontal heads may acquire
            # a vehicle-light color; vertical/side-facing heads stay unknown.
            horizontal=w/max(h,1)>1.4
            color=appearance['state'] if detection.confidence>=.45 and horizontal else 'unknown'
            identity=old['id'] if old else f'signal:{self.counter}'
            history=[(time,c) for time,c in self.histories.get(identity,[]) if 0<t-time<=.5]
            # A short uncertain observation does not erase recent evidence, but
            # never displays a remembered color on an uncertain current frame.
            # Any contradictory color starts a fresh confirmation sequence.
            if color!='unknown' and any(c not in ('unknown',color) for _,c in history):history=[]
            history.append((t,color));history=history[-5:]
            hits=sum(c==color for _,c in history) if color!='unknown' else 0
            state=color if hits>=3 else 'unknown'
            histories[identity]=history
            output.append({'id':identity,'bbox':detection.bbox,
                'confidence':detection.confidence,'state':state,'candidate_state':color,'support':hits,
                'time_s':t,'source':'detector_color_temporal','reason':appearance['reason'],
                'head_type':'horizontal_candidate' if horizontal else 'unclassified',
                'lane_assignment':'unknown','arrow':'unknown'})
        self.previous=output
        self.histories=histories
        return output
