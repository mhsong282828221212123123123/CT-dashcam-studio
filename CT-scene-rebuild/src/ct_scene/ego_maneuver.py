"""Bounded compensation only around an isolated ego lane change.

No ego trajectory fitting and no changes to observations outside the window.
"""
import math
import numpy as np
from .road_geometry import lane_at


def windows(events,times,positions,yaws,road,pre_s=2.5,settle_s=1.5):
    slope=road.get('heading_evidence',{}).get('slope',0.)
    t=np.asarray(times);p=np.asarray(positions);yaw=np.unwrap(yaws);result=[]
    for event in events:
        finish=event.get('completion_s')
        if finish is None or not 0<finish-event['start_s']<=10 or abs(event['to_lane']-event['from_lane'])!=1:continue
        start=max(float(t[0]),event['start_s']-pre_s);end=min(float(t[-1]),finish+settle_s)
        before_yaw=float(np.interp(start,t,yaw));after_yaw=float(np.interp(end,t,yaw))
        if abs(after_yaw-before_yaw)>math.radians(6):continue
        x=float(np.interp(start,t,p[:,0]));y=float(np.interp(start,t,p[:,1]))
        result.append(dict(start_s=start,end_s=end,maneuver_start_s=event['start_s'],completion_s=finish,
                           reference_lateral_m=y-slope*x,reference_yaw_rad=before_yaw,
                           from_lane=event['from_lane'],to_lane=event['to_lane']))
    return result


def compensate_actors(actors,intervals,road,max_one_sided_s=5):
    slope=road.get('heading_evidence',{}).get('slope',0.);reviews=[]
    for a in actors:
        rows=a['samples'];times=np.array([r['time_s'] for r in rows]);original=np.array([r['position_W'] for r in rows]);lateral=original[:,1]-slope*original[:,0]
        for w in intervals:
            indices=np.flatnonzero((times>=w['start_s'])&(times<=w['end_s']))
            if not len(indices):continue
            review=dict(global_id=a['global_id'],start_s=w['start_s'],end_s=w['end_s'],modified_samples=0)
            reviews.append(review)
            evidence=[e for source in a.get('lane_motion_evidence_by_camera',[]) for e in source.get('events',[])]
            if any(e['start_s']<=w['end_s'] and e['end_s']>=w['start_s'] for e in evidence):
                review['reason']='independent_actor_lane_change_evidence';continue
            pre=np.flatnonzero(times<w['start_s']);post=np.flatnonzero(times>w['end_s'])
            before=int(pre[-1]) if len(pre) and w['start_s']-times[pre[-1]]<=1 else None
            after=int(post[0]) if len(post) and times[post[0]]-w['end_s']<=1 else None
            valid=[k for k in [before,after] if k is not None]
            if not valid:review['reason']='no_outside_window_anchor';continue
            lanes=[lane_at(road,float(original[k,0]),float(lateral[k])) for k in valid]
            if any(lane is None for lane in lanes) or len(set(lanes))!=1:
                review['reason']='anchor_lane_disagreement_or_unknown';continue
            if len(valid)==1:
                if max(abs(times[indices]-times[valid[0]]))>max_one_sided_s:
                    review['reason']='one_sided_support_too_long';continue
                target=np.full(len(indices),lateral[valid[0]]);support='bounded_one_sided_lane_hold'
            else:
                target=np.interp(times[indices],times[valid],lateral[valid]);support='two_sided_lane_keeping_reconstruction'
            for i,value in zip(indices,target):
                r=rows[int(i)];r['position_W']=list(r['position_W']);r['position_before_ego_maneuver']=original[i].tolist()
                r['position_W'][1]=float(slope*r['position_W'][0]+value)
                r['yaw_before_ego_maneuver']=r['yaw_rad'];r['yaw_rad']=math.atan(slope)
                r.update(ego_maneuver_correction=support,uses_future_observations=True)
            review.update(reason=support,modified_samples=len(indices),anchor_times_s=[float(times[k]) for k in valid],lane_id=lanes[0],
                          lateral_range_before_m=float(np.ptp(lateral[indices])),lateral_range_after_m=float(np.ptp(target)))
    return reviews


def camera_view(t,ego_position,ego_yaw,intervals,road):
    slope=road.get('heading_evidence',{}).get('slope',0.)
    for w in intervals:
        if not w['start_s']<t<w['end_s']:continue
        if t<w['maneuver_start_s']:u=(t-w['start_s'])/max(.001,w['maneuver_start_s']-w['start_s'])
        elif t<=w['completion_s']:u=1.
        else:u=(w['end_s']-t)/max(.001,w['end_s']-w['completion_s'])
        u=float(np.clip(u,0,1));weight=u*u*(3-2*u)
        origin=np.asarray(ego_position,float).copy()
        origin[1]=(1-weight)*origin[1]+weight*(slope*origin[0]+w['reference_lateral_m'])
        yaw=(1-weight)*ego_yaw+weight*w['reference_yaw_rad']
        return origin,float(yaw),weight
    return np.asarray(ego_position),float(ego_yaw),0.
