"""Cheap image-space central-boundary gating, before metric fitting/lane votes.

Yellow longitudinal paint is evidence of a separator, not a semantic claim
about a concrete barrier. Missing/occluded paint never creates an invisible
separator through a junction. Source detections remain available for review.
"""
from collections import deque
import cv2
import numpy as np


def detect_central_boundary(frame,objects,geometry):
    if frame is None or geometry is None:return None
    h,w=frame.shape[:2];scale=w/640.;im=cv2.resize(frame,(640,round(h/scale)))
    sh=im.shape[0];hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array([15,45,85]),np.array([40,255,255]))
    mask[:int(sh*.51)]=0;mask[int(sh*.94):]=0
    for obj in objects:
        x,y,bw,bh=np.asarray(obj['bbox'])/scale
        cv2.rectangle(mask,(int(x)-3,int(y)-3),(int(x+bw)+3,int(y+bh)+3),0,-1)
    lines=cv2.HoughLinesP(mask,1,np.pi/180,22,minLineLength=40,maxLineGap=12)
    if lines is None:return None
    choices=[]
    for values in lines.reshape(-1,4):
        p,q=values.reshape(2,2).astype(float)*scale
        if abs(q[1]-p[1])<sh*.07*scale:continue
        slope=(q[0]-p[0])/(q[1]-p[1])
        if not .2<abs(slope)<4.:continue
        ground=[geometry.ground(*point) for point in (p,q)]
        if any(point is None for point in ground):continue
        ground=np.asarray(ground)
        if np.ptp(ground[:,1])<.6 or not .4<float(np.median(ground[:,0]))<15:continue
        road_slope,offset=np.polyfit(ground[:,1],ground[:,0],1)
        if abs(road_slope)>.65:continue
        ego=geometry.pixels(np.array([[0.,0.,float(ground[:,1].mean())]]))
        if ego is None:continue
        direction=q-p;length=float(np.linalg.norm(direction))
        cross=float(direction[0]*(ego[0,1]-p[1])-direction[1]*(ego[0,0]-p[0]))
        if abs(cross)/length<8:continue
        choices.append((float(np.median(ground[:,0])), -length,p,q,np.sign(cross)))
    if not choices:return None
    # The innermost visible yellow line is the conservative separator when
    # double lines are present. Never use object positions to select it.
    _,_,p,q,side=min(choices,key=lambda c:(c[0],c[1]))
    return dict(points=[p.tolist(),q.tolist()],ego_side=int(side),
                kind='yellow_longitudinal_boundary',source='camera_colour_line',
                semantics='visible central-boundary candidate; barrier type unverified')


def foot_side(box,boundary):
    p,q=np.asarray(boundary['points'],float);direction=q-p
    length=float(np.linalg.norm(direction))
    if length<1:return 'unknown'
    x,y,w,h=box
    feet=np.array([[x,y+h],[x+w,y+h]])
    signed=(direction[0]*(feet[:,1]-p[1])-direction[1]*(feet[:,0]-p[0]))/length*boundary['ego_side']
    margin=max(5.,w*.04)
    if np.max(signed)<-margin:return 'beyond'
    if np.min(signed)>margin:return 'inside'
    return 'straddling'


class CentralBoundaryTracker:
    def __init__(self):
        self.history=deque(maxlen=5);self.last=None;self.objects={}

    def update(self,frame,objects,geometry,t):
        if self.last is not None and not 0<t-self.last<=.3:
            self.history.clear();self.objects.clear()
        self.last=t
        boundary=detect_central_boundary(frame,objects,geometry)
        self.history.append((t,boundary is not None))
        active=boundary is not None and sum(v for _,v in self.history)>=3 and t-self.history[0][0]>=.19
        evidence=dict(boundary or {},active=bool(active))
        for obj in objects:
            obj['road_relation']='unknown'
            if obj.get('kind','car') not in ('car','truck','bus','bicycle','motorcycle'):continue
            state=self.objects.setdefault(obj['track_id'],dict(inside=0,beyond=0,last=t))
            if t-state['last']>.5:state.update(inside=0,beyond=0)
            state['last']=t
            if not active:continue
            side=foot_side(obj['bbox'],boundary)
            if side=='inside':state['inside']+=1;state['beyond']=0
            elif side=='beyond':state['beyond']+=1
            else:state['beyond']=0
            # Established own-road tracks survive a later noisy extrapolation
            # or genuine crossing. Whole contact width must be on the far side.
            if side=='beyond' and state['beyond']>=3 and state['inside']<3:
                obj['road_relation']='beyond_central_boundary'
            elif side=='inside':obj['road_relation']='host_side_of_central_boundary'
        self.objects={k:v for k,v in self.objects.items() if t-v['last']<=1.}
        return evidence
