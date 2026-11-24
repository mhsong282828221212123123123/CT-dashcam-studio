"""Raw left-repeater ordering: a vehicle between own body and yellow median.

Pixel membership only. This never projects paint into metric road coordinates
or fits a camera mount. Missing/occluded yellow paint supplies no evidence.
"""
from collections import defaultdict
from pathlib import Path
import cv2,numpy as np


def inside_line(box,line,width,height):
    x,y,X,Y=line
    if abs(X-x)<65 or abs(Y-y)>abs(X-x)*.5:return False
    return bool(min(x,X)>box[2]+width*.01 and box[3]-height*.12<min(y,Y) and max(y,Y)<box[3]+height*.04)


def inside_pair(box,lines,width,height):
    lines=[np.array(line,float) for line in lines if inside_line(box,line,width,height)]
    for i,a in enumerate(lines):
        da=a[2:]-a[:2];normal=np.array([-da[1],da[0]])/np.linalg.norm(da)
        for b in lines[i+1:]:
            db=b[2:]-b[:2];angle=abs(float(np.arctan2(da[0]*db[1]-da[1]*db[0],np.dot(da,db))));angle=min(angle,abs(np.pi-angle));distance=abs(float(np.dot((b[:2]+b[2:]-a[:2]-a[2:])/2,normal)));overlap=min(max(a[0],a[2]),max(b[0],b[2]))-max(min(a[0],a[2]),min(b[0],b[2]))
            if angle<np.radians(8) and height*.004<distance<height*.04 and overlap>width*.05:return [a.astype(int).tolist(),b.astype(int).tolist()]
    return None


def review(package,frames,observations):
    evidence=defaultdict(list)
    for f in frames:
        if f['camera_id']!='left_repeater':continue
        candidates=[r for r in observations.get(f['frame_id'],[]) if r['class_id'] in (2,5,7) and r['support']=='detector_observation'
                    and r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>180 and r['bbox_xyxy'][3]-r['bbox_xyxy'][1]>130
                    and min(r['bbox_xyxy'][0],r['bbox_xyxy'][1],f['width']-r['bbox_xyxy'][2],f['height']-r['bbox_xyxy'][3])>6]
        if not candidates:continue
        image=cv2.imread(str(Path(package)/f['image']));h,w=image.shape[:2];hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        contrast=cv2.subtract(hsv[:,:,2],cv2.GaussianBlur(hsv[:,:,2],(0,0),5))
        mask=((hsv[:,:,0]>=12)&(hsv[:,:,0]<=40)&(hsv[:,:,1]>30)&(hsv[:,:,2]>65)&(contrast>4)).astype('uint8')*255;mask[:round(h*.52)]=0;mask[round(h*.97):]=0
        for r in observations.get(f['frame_id'],[]):
            x,y,X,Y=np.rint(r['bbox_xyxy']).astype(int);mask[max(0,y-5):min(h,Y+5),max(0,x-5):min(w,X+5)]=0
        lines=cv2.HoughLinesP(mask,1,np.pi/180,35,minLineLength=65,maxLineGap=12)
        if lines is None:continue
        for r in candidates:
            pair=inside_pair(r['bbox_xyxy'],lines.reshape(-1,4),w,h)
            if pair is not None:evidence[r['local_track_id']].append(dict(frame_id=f['frame_id'],time_s=f['camera_time_s'],bbox_xyxy=r['bbox_xyxy'],median_pixels=pair))
    return {key:dict(inside_median=True,samples=rows,source='repeated_left_raw_body_before_yellow_median; pixel_order_only',metric_projection_used=False)
            for key,rows in evidence.items() if len({r['frame_id'] for r in rows})>=8 and rows[-1]['time_s']-rows[0]['time_s']>=1.2}
