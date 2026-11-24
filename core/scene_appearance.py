"""Local colour evidence for association; not a vehicle re-identification model."""
import cv2
import numpy as np


def signature(frame,box):
    if frame is None:return None
    x,y,w,h=map(float,box)
    if w<30 or h<24 or x<2 or x+w>frame.shape[1]-2:return None
    crop=frame[max(0,int(y+h*.12)):min(frame.shape[0],int(y+h*.88)),
               max(0,int(x+w*.12)):min(frame.shape[1],int(x+w*.88))]
    if crop.size<100:return None
    light=cv2.cvtColor(crop,cv2.COLOR_BGR2LAB)[:,:,0]
    hist=cv2.calcHist([light],[0],None,[16],[0,256]).ravel();hist/=max(1.,hist.sum())
    return dict(hist=hist.tolist(),quantiles=np.percentile(light,[30,60,85]).tolist())


def agreement(a,b):
    if a is None or b is None:return True,1.
    distance=float(cv2.compareHist(np.asarray(a['hist'],np.float32),np.asarray(b['hist'],np.float32),cv2.HISTCMP_BHATTACHARYYA))
    difference=abs(np.asarray(a['quantiles'])-np.asarray(b['quantiles']))
    incompatible=distance>.32 and difference[1]>35 and difference[2]>25
    return not incompatible,1.-distance
