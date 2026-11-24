"""Image-supported box propagation; outputs are proposals, never detections."""
import cv2
import numpy as np


def propagate_box(previous, current, box):
    x,y,w,h=map(float,box)
    if min(w,h)<8:return None
    a=cv2.cvtColor(previous,cv2.COLOR_BGR2GRAY)
    b=cv2.cvtColor(current,cv2.COLOR_BGR2GRAY)
    mask=np.zeros_like(a)
    cv2.rectangle(mask,(max(0,int(x+2)),max(0,int(y+2))),
                  (min(a.shape[1]-1,int(x+w-2)),min(a.shape[0]-1,int(y+h-2))),255,-1)
    points=cv2.goodFeaturesToTrack(a,80,.015,2,mask=mask)
    if points is None or len(points)<6:return None
    moved,ok,_=cv2.calcOpticalFlowPyrLK(a,b,points,None,winSize=(15,15),maxLevel=3)
    if moved is None:return None
    back,ok2,_=cv2.calcOpticalFlowPyrLK(b,a,moved,None,winSize=(15,15),maxLevel=3)
    if back is None:return None
    valid=(ok.ravel()!=0)&(ok2.ravel()!=0)&(np.linalg.norm(points-back,axis=2).ravel()<.8)
    if valid.sum()<6:return None
    source=points.reshape(-1,2)[valid];target=moved.reshape(-1,2)[valid]
    transform,inliers=cv2.estimateAffinePartial2D(source,target,method=cv2.RANSAC,ransacReprojThreshold=1.5)
    if transform is None or inliers.mean()<.7:return None
    supported=source[inliers.ravel()!=0]
    if len(supported)<6 or np.ptp(supported[:,0])<w*.2 or np.ptp(supported[:,1])<h*.2:return None
    scale=float(np.linalg.norm(transform[:,0]))
    angle=float(np.arctan2(transform[1,0],transform[0,0]))
    if not .8<scale<1.2 or abs(angle)>.15:return None
    # Repeated rotation of an axis-aligned bounding rectangle grows it on every
    # step, even when the actual object size is unchanged. Track center/scale;
    # substantial rotation already fails the gate above.
    center=transform@np.array([x+w/2,y+h/2,1.])
    extent=np.array([w,h])*scale
    low=center-extent/2;high=center+extent/2
    if np.linalg.norm((low+high)/2-np.array([x+w/2,y+h/2]))>50:return None
    if low.min()<0 or high[0]>=b.shape[1] or high[1]>=b.shape[0]:return None
    return {'bbox':np.r_[low,high-low].tolist(),'inlier_count':len(supported),
            'scale':scale,'forward_backward_checked':True,'status':'image_track_proposal'}
