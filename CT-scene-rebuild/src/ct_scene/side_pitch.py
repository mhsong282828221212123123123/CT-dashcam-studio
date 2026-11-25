"""Track-balanced side-camera pitch evidence from upright body/ground ratios.

Class heights are priors, not surveyed labels. A shared correction is accepted
only when several clear tracks agree and leave-one-track-out evidence improves.
No road, lane, ego pose or current fitted world position is used in calibration.
"""
import math
from collections import defaultdict
import cv2
import numpy as np
from .affine_probe import camera_matrix


def pitch_candidate(box,profile,camera_height,vehicle_height,width=1280,height=960):
    b=np.asarray(box,float);w,h=b[2:]-b[:2]
    if not np.isfinite(b).all() or min(w,h)<40 or not .55<w/h<3.3 or min(b[0],b[1],width-b[2],height-b[3])<15:
        return None
    K,D=camera_matrix(profile);cx=(b[0]+b[2])/2
    rays=cv2.undistortPoints(np.array([[[cx,b[1]]],[[cx,b[3]]]],float),K,D).reshape(2,2)
    yt,yb=rays[:,1];apparent=yb-yt
    if apparent<=.02 or not 4<vehicle_height/apparent<55:return None
    # Exact vertical pinhole ratio after a common pitch rotation.
    def residual(delta):
        top=math.tan(math.atan(yt)+delta);bottom=math.tan(math.atan(yb)+delta)
        return (vehicle_height-camera_height)*bottom+camera_height*top
    lo,hi=-.09,.09
    if residual(lo)*residual(hi)>0:return None
    for _ in range(32):
        mid=(lo+hi)/2
        if residual(mid)>0:hi=mid
        else:lo=mid
    delta=(lo+hi)/2
    return dict(delta_pitch_rad=float(delta),height_depth_m=float(vehicle_height/apparent),bbox_xyxy=b.tolist())


def fit_camera_pitch(records):
    by_track=defaultdict(list)
    for r in records:by_track[r['track_id']].append(r)
    tracks=[]
    for key,rows in sorted(by_track.items()):
        if len(rows)<6 or np.ptp([r['time_s'] for r in rows])<1.:continue
        d=np.array([r['delta_pitch_rad'] for r in rows]);med=float(np.median(d));mad=float(1.4826*np.median(abs(d-med)))
        if mad>.015:continue
        tracks.append(dict(track_id=key,model=rows[0].get('model'),median_rad=med,mad_rad=mad,samples=len(rows),span_s=[min(r['time_s'] for r in rows),max(r['time_s'] for r in rows)]))
    result=dict(accepted=False,status='insufficient_consistent_tracks',delta_pitch_rad=0.,sigma_pitch_rad=.025,tracks=tracks,records=len(records),
                calibration_role='class_height_assisted_relative_pitch; not_measured_rig_or_metric_truth')
    if len(tracks)<3:return result
    d=np.array([t['median_rad'] for t in tracks]);delta=float(np.median(d));mad=float(1.4826*np.median(abs(d-delta)))
    old=[];new=[]
    for i,t in enumerate(tracks):
        other=float(np.median(np.delete(d,i)));old.append(abs(d[i]));new.append(abs(d[i]-other))
    consistent=float(np.mean(abs(d-delta)<.018));improved=float(np.median(new))<max(.003,float(np.median(old))*.65)
    accepted=abs(delta)<.07 and mad<.012 and consistent>=.75 and (improved or abs(delta)<.005)
    result.update(accepted=bool(accepted),status='track_disjoint_consistency_accepted' if accepted else 'cross_track_consistency_failed',
                  delta_pitch_rad=delta if accepted else 0.,sigma_pitch_rad=max(.006,mad) if accepted else .025,
                  fitted_delta_pitch_rad=delta,track_mad_rad=mad,consistent_track_fraction=consistent,
                  leave_one_track_out=dict(before_median_rad=float(np.median(old)),after_median_rad=float(np.median(new)),
                       scope='track_disjoint height-prior consistency; not independent 3D ground truth'))
    return result
