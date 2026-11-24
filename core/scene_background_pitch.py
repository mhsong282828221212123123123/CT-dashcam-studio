"""Relative camera pitch from static upper-background registration.

An explicit reference horizon remains required. Foreground traffic and road
markings are excluded; translation/zoom is separated from image displacement.
"""
import cv2
import numpy as np


def reference_points(gray):
    h,w=gray.shape;mask=np.zeros_like(gray)
    mask[int(.03*h):int(.438*h),int(.055*w):int(.945*w)]=255
    return cv2.goodFeaturesToTrack(gray,1500,.008,max(4,7*w/1280),mask=mask)


def estimate_pitch(reference,frame,points,horizon):
    if points is None or len(points)<100:return None
    moved,ok,_=cv2.calcOpticalFlowPyrLK(reference,frame,points,None,winSize=(25,25),maxLevel=4)
    if moved is None:return None
    back,ok2,_=cv2.calcOpticalFlowPyrLK(frame,reference,moved,None,winSize=(25,25),maxLevel=4)
    if back is None:return None
    valid=(ok.ravel()!=0)&(ok2.ravel()!=0)&(np.linalg.norm(back-points,axis=2).ravel()<.8)
    a=moved.reshape(-1,2)[valid];b=points.reshape(-1,2)[valid]
    if len(a)<100:return None
    train=np.arange(len(a))%3!=1
    T,inliers=cv2.estimateAffinePartial2D(a[train],b[train],method=cv2.RANSAC,
        ransacReprojThreshold=1.5,maxIters=3000,confidence=.999)
    if T is None or inliers.sum()<60:return None
    error=np.linalg.norm(np.c_[a[~train],np.ones((~train).sum())]@T.T-b[~train],axis=1)
    scale=float(np.linalg.norm(T[:,0]));roll=float(np.degrees(np.arctan2(T[1,0],T[0,0])))
    if not .8<scale<1.2 or abs(roll)>3 or np.median(error)>1.5 or np.mean(error<3)<.5:return None
    inv=cv2.invertAffineTransform(T)
    value=float((inv@np.array([reference.shape[1]/2,horizon,1]))[1])
    if abs(value-horizon)>35:return None
    return dict(horizon_px=value,training_inliers=int(inliers.sum()),
        withheld_features=len(error),withheld_median_error_px=float(np.median(error)),
        withheld_fraction_under_3px=float(np.mean(error<3)),scale=scale,roll_deg=roll)
