"""Offline body yaw from repeated image evidence; never XYZ/road/identity.

Only free-heading car proxies are eligible. Established parallel-lane priors,
the frozen lead, buses and trucks retain their existing orientation behavior.
"""
import numpy as np
from .actor_heading import resolve_axis_flips


def delta(a,b):return np.arctan2(np.sin(a-b),np.cos(a-b))


def stationary_continuation(row,rows,anchor):
    if np.linalg.norm(np.asarray(row['position_W'])[:2]-np.asarray(anchor['position_W'])[:2])>3:return False
    if row.get('stationary_handoff_support'):return True
    if row.get('support') not in ('detector_observation','flow_observation'):return False
    clear=[]
    for r in rows:
        b=r.get('bbox_xyxy')
        if abs(r['time_s']-row['time_s'])>1.1 or not b or r.get('support')!='detector_observation':continue
        if min(b[0],b[1],r.get('image_width',1280)-b[2],r.get('image_height',960)-b[3])<=5 or b[2]-b[0]<40 or b[3]-b[1]<24:continue
        clear.append(r)
    if len(clear)<5 or clear[-1]['time_s']-clear[0]['time_s']<.8:return False
    t=np.array([r['time_s']-row['time_s'] for r in clear]);p=np.array([r['position_W'] for r in clear]);X=np.c_[t,np.ones(len(t))];fit=np.linalg.lstsq(X,p,rcond=None)[0]
    return bool(np.linalg.norm(fit[0,:2])<.7 and np.quantile(np.linalg.norm(X@fit-p,axis=1),.9)<.5)


def refine_actor(actor,predictions,ego_times,ego_yaws,max_gap_s=4.):
    if actor.get('frozen_lead'):return dict(status='frozen_lead_preserved',changed_samples=0)
    if actor.get('lane_index')!=-2 or actor.get('model') not in ('sedan','suv','van'):
        return dict(status='established_lane_or_other_asset_preserved',changed_samples=0)
    ids=set(actor['local_ids']);unique={}
    for p in predictions:
        if p['local_track_id'] not in ids or not p.get('accepted') or 'camera_heading_rad' not in p:continue
        if p['frame_id'] not in unique or p['mirror_error_deg']<unique[p['frame_id']]['mirror_error_deg']:unique[p['frame_id']]=p
    candidates=[];eyaw=np.unwrap(ego_yaws)
    for p in sorted(unique.values(),key=lambda r:r['camera_time_s']):
        t=p['camera_time_s'];angle=float(p['camera_heading_rad']+np.interp(t,ego_times,eyaw))
        if not np.isfinite(angle):continue
        candidates.append(dict(time_s=t,yaw_rad=angle,frame_id=p['frame_id']))
    anchors=[]
    for p in candidates:
        near=[q for q in candidates if abs(q['time_s']-p['time_s'])<=.5]
        if len(near)<3:continue
        angles=np.array([q['yaw_rad'] for q in near]);dist=abs(delta(angles[:,None],angles[None,:]));centre=angles[np.argmin(np.sum(dist,axis=1))]
        inliers=[q for q in near if abs(delta(q['yaw_rad'],centre))<=np.radians(25)]
        if len(inliers)<3 or np.ptp([q['time_s'] for q in inliers])<.3 or abs(delta(p['yaw_rad'],centre))>np.radians(25):continue
        mean=float(np.angle(np.mean(np.exp(1j*np.array([q['yaw_rad'] for q in inliers])))))
        anchors.append(dict(p,yaw_rad=mean,support_frame_ids=[q['frame_id'] for q in inliers]))
    if len(anchors)<3:return dict(status='insufficient_consistent_image_heading',changed_samples=0,raw_candidates=len(candidates))
    at=np.array([p['time_s'] for p in anchors]);angles=np.unwrap([p['yaw_rad'] for p in anchors])
    if np.max(np.diff(at))>max_gap_s:return dict(status='separated_image_windows_abstained',changed_samples=0,anchors=len(anchors))
    changed=0;held=0;rows=actor['samples'];row_times=np.array([r['time_s'] for r in rows])
    for row in actor['samples']:
        t=row['time_s'];distance=max(at[0]-t,t-at[-1],0.)
        nearest_anchor=rows[int(np.argmin(abs(row_times-(at[0] if t<at[0] else at[-1]))))]
        stationary=distance>0 and stationary_continuation(row,rows,nearest_anchor)
        if distance>max_gap_s and not stationary:continue
        target=float(np.interp(t,at,angles));hold=min(2.,max_gap_s)
        weight=1. if distance<=hold or stationary else (max_gap_s-distance)/max(1e-6,max_gap_s-hold)
        adjustment=float(delta(target,row['yaw_rad'])*weight)
        if abs(adjustment)<1e-8:continue
        row['yaw_before_image_heading']=row['yaw_rad'];row['yaw_rad']+=adjustment
        if 'yaw_before_ego_maneuver' in row:row['yaw_before_ego_maneuver']+=adjustment
        row['heading_support']='repeated_image_body_heading' if distance==0 else 'image_heading_held_by_source_observed_stationarity' if stationary else 'offline_inferred_image_heading; bounded_future_or_past_anchor'
        held+=int(stationary)
        changed+=1
    # Falling back from image evidence to a low-motion velocity estimate must
    # not reverse the body front by pi. Gradual real turns keep their rotation.
    continuous=resolve_axis_flips(row_times,[r['yaw_rad'] for r in rows]);phase_changes=0
    for row,angle in zip(rows,continuous):
        adjustment=float(delta(angle,row['yaw_rad']))
        if abs(adjustment)<1e-8:continue
        row['yaw_before_image_axis_continuity']=row['yaw_rad'];row['yaw_rad']+=adjustment
        if 'yaw_before_ego_maneuver' in row:row['yaw_before_ego_maneuver']+=adjustment
        phase_changes+=1
    return dict(status='image_heading_refined',changed_samples=changed,stationary_hold_samples=held,phase_continuity_adjustments=phase_changes,anchors=anchors,positions_modified=False,
                observer='own_SEI_heading_plus_front_image_heading; not_other_vehicle_motion_or_GPS',extrapolation_budget_s=max_gap_s)
