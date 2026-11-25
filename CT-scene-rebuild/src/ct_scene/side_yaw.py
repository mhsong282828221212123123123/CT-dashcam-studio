"""Estimate the camera's fixed yaw from clear straight relative-image tracks.
No actor lane centre, fitted world pose or GPS map match enters this estimator.
"""
import math
import cv2,numpy as np
from .affine_probe import camera_matrix


def track_axis_candidates(rows,profile,ego_times,ego_yaws,seed_deg,vehicle_height=1.5):
    K,D=camera_matrix(profile);rs=[]
    yaw_rate=np.gradient(np.unwrap(ego_yaws),ego_times)
    for r in rows:
        b=r.get('bbox_xyxy');t=float(r.get('time_s',r.get('camera_time_s')))
        if r.get('support')!='detector_observation' or r.get('score',1.)<.55 or 'truncated' in r.get('quality_flags',[]) or b is None or min(b[2]-b[0],b[3]-b[1])<35 or min(b[0],b[1],1280-b[2],960-b[3])<15:continue
        if abs(np.interp(t,ego_times,yaw_rate))>math.radians(1.5):continue
        x=(b[0]+b[2])/2;rays=cv2.undistortPoints(np.array([[[x,b[1]]],[[x,b[3]]]],float),K,D).reshape(2,2);dh=rays[1,1]-rays[0,1]
        if dh<=.02:continue
        z=vehicle_height/dh
        if not 4<z<65:continue
        rs.append((t,z,-z*float(np.mean(rays[:,0]))))
    if len(rs)<10:return []
    data=np.array(rs);results=[]
    for start in np.arange(data[0,0],data[-1,0]-2.4,2.):
        win=data[(data[:,0]>=start)&(data[:,0]<start+4)]
        if len(win)<10 or np.ptp(win[:,0])<2 or np.max(np.diff(win[:,0]))>.8:continue
        ys=np.interp(win[:,0],ego_times,np.unwrap(ego_yaws))
        if np.ptp(ys)>math.radians(3):continue
        q=win[:,1:];_,sv,vh=np.linalg.svd(q-np.mean(q,axis=0),full_matrices=False);axis=vh[0];along=(q-q.mean(0))@axis;cross=(q-q.mean(0))@vh[1]
        span=float(np.ptp(along));noise=float(np.percentile(abs(cross),90))
        if span<4 or noise>max(.45,.08*span):continue
        phi=math.atan2(axis[1],axis[0]);yaw=-phi
        seed=math.radians(seed_deg);yaw=seed+(yaw-seed+math.pi/2)%math.pi-math.pi/2
        if abs(yaw-seed)>math.radians(30):continue
        results.append(dict(start_s=float(start),yaw_deg=math.degrees(yaw),span_m=span,noise_p90_m=noise))
    return results


def fit_camera_yaw(records,seed_deg):
    result=dict(accepted=False,status='insufficient_consistent_tracks',used_yaw_deg=float(seed_deg),sigma_yaw_deg=8.,tracks=records,
                calibration_role='parallel_relative_motion_axis; estimated_not_measured_rig')
    if len(records)<3:return result
    values=np.array([r['median_yaw_deg'] for r in records],float);median=float(np.median(values));mad=float(1.4826*np.median(abs(values-median)))
    leave_out=[abs(values[i]-np.median(np.delete(values,i))) for i in range(len(values))]
    accepted=mad<5. and np.mean(abs(values-median)<7.)>=.7 and abs(median-seed_deg)<25. and np.median(leave_out)<3.5
    result.update(accepted=bool(accepted),status='track_disjoint_axis_consistency_accepted' if accepted else 'axis_consistency_failed',fitted_yaw_deg=median,
                  used_yaw_deg=median if accepted else float(seed_deg),sigma_yaw_deg=max(3.,mad) if accepted else 8.,
                  leave_one_track_out_median_deg=float(np.median(leave_out)))
    return result
