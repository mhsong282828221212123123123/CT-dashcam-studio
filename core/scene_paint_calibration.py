"""Road vanishing point from complete, parallel zebra paint edges.

No vehicle geometry or camera focal prior is used. Paint dimensions are not
treated as surveyed distances. Unsupported views return None.
"""
import itertools
import cv2,numpy as np
from scipy.optimize import least_squares

def paint_vanishing_point(frame,objects=()):
    height,width=frame.shape[:2];hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array([0,0,125]),np.array([179,65,255]))
    mask[:int(height*.52)]=0;mask[int(height*.87):]=0
    for obj in objects:
        x,y,w,h=obj['bbox'];cv2.rectangle(mask,(int(x)-5,int(y)-5),(int(x+w)+5,int(y+h)+5),0,-1)
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);segments=[];rows=[]
    for contour in contours:
        if cv2.contourArea(contour)<100:continue
        polygon=cv2.approxPolyDP(contour,.02*cv2.arcLength(contour,True),True).reshape(-1,2)
        if len(polygon)!=4 or not cv2.isContourConvex(polygon):continue
        if polygon[:,0].min()<3 or polygon[:,0].max()>width-3:continue
        ids=np.argsort(polygon[:,1]);top=polygon[ids[:2]];bottom=polygon[ids[2:]]
        top=top[np.argsort(top[:,0])];bottom=bottom[np.argsort(bottom[:,0])]
        if max(abs(top[0,1]-top[1,1]),abs(bottom[0,1]-bottom[1,1]))>6:continue
        if bottom[:,1].mean()-top[:,1].mean()<8:continue
        if min(np.ptp(top[:,0]),np.ptp(bottom[:,0]))<15:continue
        key=np.array([top[:,1].mean(),bottom[:,1].mean()]);pair=[(top[k].astype(float),bottom[k].astype(float)) for k in [0,1]]
        for group in rows:
            if np.max(abs(group[0]-key))<5:group[1].append(pair);break
        else:rows.append([key,[pair]])
    # Actual zebra rows contain several aligned bars; unrelated arrows and
    # distant marks must not become calibration edges from another direction.
    for _,group in sorted(rows,key=lambda g:-len(g[1]))[:2]:
        if len(group)>=3:segments.extend(edge for pair in group for edge in pair)
    if len(segments)<8:return None
    lines=np.array([np.cross(np.r_[a,1.],np.r_[b,1.]) for a,b in segments]);lines/=np.linalg.norm(lines[:,:2],axis=1)[:,None]
    train=np.arange(len(lines))%3!=1;L=lines[train];candidates=[]
    for a,b in itertools.combinations(L,2):
        p=np.cross(a,b)
        if abs(p[2])<1e-6:continue
        p=p[:2]/p[2]
        if not width*.25<p[0]<width*.8 or not height*.35<p[1]<height*.61:continue
        residual=abs(L@np.r_[p,1.]);candidates.append((np.sum(residual<4),-np.median(residual),p))
    if not candidates:return None
    _,_,seed=max(candidates,key=lambda x:x[:2]);support=abs(L@np.r_[seed,1.])<5
    if support.sum()<5:return None
    fit=least_squares(lambda p:L[support]@np.r_[p,1.],seed,loss='soft_l1',f_scale=1.)
    residual=abs(lines[~train]@np.r_[fit.x,1.])
    if np.median(residual)>4 or np.ptp([a[0] for a,b in segments])<width*.25:return None
    return dict(vanishing_point_px=fit.x.tolist(),edges=len(segments),training_inliers=int(support.sum()),
        withheld_edge_median_error_px=float(np.median(residual)),
        withheld_edge_p95_error_px=float(np.percentile(residual,95)),
        segments=[[a.tolist(),b.tolist()] for a,b in segments],source='parallel_paint_edges_not_vehicle_boxes')
