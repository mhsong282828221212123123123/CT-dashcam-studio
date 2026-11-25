"""Small supervised image-space heads with date-isolated evaluation.

No metric XYZ labels are inferred from the existing scene or lane assignments.
"""
import numpy as np
CAMERAS=('front','left_repeater','right_repeater')
DIRECTIONS=('away','toward','left','right')


def guard_splits(records):
    seen={};images={}
    for r in records:
        if r['split'] not in ('train','validation','test'): raise ValueError('Unknown split')
        for values,key in [(seen,r['session']),(images,r['source_image_sha256'])]:
            if key in values and values[key]!=r['split']: raise ValueError('Source/session leakage across splits')
            values[key]=r['split']
        if r['annotation_status'] not in ('visual_review_by_codex','visual_reviewed'): raise ValueError('Unreviewed annotation')
        pair=r.get('wheel_contacts_tile_xy')
        if pair is not None:
            points=np.asarray(pair,float)
            if points.shape!=(2,2) or not np.isfinite(points).all() or np.any(points<0) or np.any(points>[360,230]):raise ValueError('Invalid visible tire landmarks')
            if r.get('contact_tile_xy') is None or not np.allclose(points.mean(0),r['contact_tile_xy']):raise ValueError('Contact must match marked tire midpoint')
        if r.get('contact_visibility','').startswith('occluded') and r.get('contact_tile_xy') is not None:raise ValueError('Occluded contact cannot be a target')
    if set(seen.values())!={'train','validation','test'}: raise ValueError('Three isolated splits required')


def contact_target(r):
    if r['contact_tile_xy'] is None: return None
    transform=r['tile_transform'];point=np.array(r['contact_tile_xy'],float)
    point=(point-[transform['dx'],transform['dy']])/transform['scale']+np.array(r['crop_rect_xyxy'][:2])
    box=np.array(r['bbox_xyxy']);return (point-box[:2])/(box[2:]-box[:2])


def metadata(r):
    b=np.array(r['bbox_xyxy']);w,h=b[2:]-b[:2]
    return np.r_[[float(r['camera']==c) for c in CAMERAS],(b[:2]+b[2:])/[2560,1920],w/1280,h/960,w/h]


def rich_metadata(r):
    b=np.array(r['bbox_xyxy'],float);w,h=b[2:]-b[:2]
    cx,cy=(b[0]+b[2])/2560.0-0.5,(b[1]+b[3])/1920.0-0.5
    ar=w/h;cams=[float(r['camera']==c) for c in CAMERAS]
    inter=[v for c in cams for v in (c*cx,c*cy,c*ar)]
    return np.r_[cams,cx,cy,w/1280.0,h/960.0,ar,inter]


def flip_record(r):
    cam_map={'front':'front','left_repeater':'right_repeater','right_repeater':'left_repeater'}
    b=r['bbox_xyxy']
    return dict(camera=cam_map[r['camera']],bbox_xyxy=[1280.0-b[2],b[1],1280.0-b[0],b[3]])


def flip_contact_uv(uv):
    arr=np.asarray(uv,float)
    if arr.ndim==1:return np.array([1.0-arr[0],arr[1]],float)
    return np.c_[1.0-arr[:,0],arr[:,1]]


def flip_direction_index(d):
    swap=np.array([0,1,3,2])
    arr=np.asarray(d,int);out=arr.copy();valid=arr>=0;out[valid]=swap[arr[valid]]
    return out


def ridge(x,y,alpha):
    mx=x.mean(0);my=y.mean(0);a=x-mx;b=y-my
    weights=a.T@np.linalg.solve(a@a.T+np.eye(len(a))*alpha,b)
    return weights,my-mx@weights


def metrics(predicted,targets,records):
    boxes=np.array([r['bbox_xyxy'] for r in records]);ratio=(boxes[:,2]-boxes[:,0])/(boxes[:,3]-boxes[:,1])
    error=np.linalg.norm((predicted-targets)*np.c_[ratio,np.ones(len(ratio))],axis=1)
    return dict(samples=len(error),mean_error_vehicle_height=float(np.mean(error)),median_error_vehicle_height=float(np.median(error)),p90_error_vehicle_height=float(np.quantile(error,.9)))
