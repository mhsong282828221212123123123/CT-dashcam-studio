"""Resolve body front/back using repeated source rear appearance.

Source anchors select between theta and theta+pi. Offline interpolation between
those anchors rejects spurious velocity-axis rotations; XYZ never changes.
"""
import numpy as np


def orient_unpublished(rows,cues,ego_times,ego_positions):
    """Choose one initial body phase before freezing; never change XYZ/shape."""
    draft=dict(frozen_lead=False,samples=[dict(r) for r in rows])
    evidence=align_actor(draft,cues,ego_times,ego_positions)
    votes=evidence['votes'];negative=evidence.get('negative_votes',0)
    accepted=votes>=5 and negative/votes>=.9
    result=[dict(r,yaw_rad=float(r['yaw_rad']+np.pi)) if accepted else dict(r) for r in rows]
    return result,dict(accepted=bool(accepted),votes=votes,negative_votes=negative,
                       phase_delta_rad=float(np.pi if accepted else 0),evidence=evidence.get('evidence',[]),
                       support='repeated_source_rear_appearance_before_anchor_publication',positions_modified=False)


def align_actor(actor,cues,ego_times,ego_positions):
    if actor.get('frozen_lead'):return dict(status='frozen_anchor_preserved',votes=0)
    rows=actor['samples'];ts=np.asarray([r['time_s'] for r in rows]);ep=np.asarray(ego_positions);votes=[]
    for cue in cues:
        t=cue['time_s'];i=int(np.argmin(abs(ts-t)))
        if abs(ts[i]-t)>.15:continue
        r=rows[i];eye=np.array([np.interp(t,ego_times,ep[:,k]) for k in range(2)]);ray=np.asarray(r['position_W'])[:2]-eye
        length=np.linalg.norm(ray)
        if length<2:continue
        dot=float(np.dot(ray/length,[np.cos(r['yaw_rad']),np.sin(r['yaw_rad'])]))
        if abs(dot)>.85:votes.append(dict(frame_id=cue['frame_id'],time_s=t,rear_axis_dot=dot,yaw_rad=float(r['yaw_rad']),resolved_yaw_rad=float(r['yaw_rad']+(np.pi if dot<0 else 0))))
    distinct={v['frame_id'] for v in votes};negative=sum(v['rear_axis_dot']<0 for v in votes)
    flip=len({v['frame_id'] for v in votes if v['rear_axis_dot']<0})>=3
    if flip:
        votes.sort(key=lambda v:v['time_s'])
        at=np.asarray([v['time_s'] for v in votes]);angles=np.unwrap([v['resolved_yaw_rad'] for v in votes])
        original=np.asarray([r['yaw_rad'] for r in rows]);corrected=original.copy()
        for i,t in enumerate(ts):
            if t<at[0]:corrected[i]+=angles[0]-votes[0]['yaw_rad']
            elif t>at[-1]:corrected[i]+=angles[-1]-votes[-1]['yaw_rad']
            else:
                j=max(0,min(len(at)-2,int(np.searchsorted(at,t)-1)))
                # Do not use a long unseen interval to smooth an actual turn.
                if at[j+1]-at[j]<=4.:corrected[i]=np.interp(t,at,angles)
        for r,before,after in zip(rows,original,corrected):
            delta=float(np.arctan2(np.sin(after-before),np.cos(after-before)))
            if abs(delta)<1e-8:continue
            r['yaw_before_rear_appearance']=r['yaw_rad'];r['yaw_rad']+=delta
            if 'yaw_before_ego_maneuver' in r:r['yaw_before_ego_maneuver']+=delta
            r['heading_axis_support']='source_rear_appearance_anchors_and_offline_interpolation; xyz_unchanged'
    return dict(status='source_rear_antipodal_flip' if flip else 'kept_or_insufficient_rear_appearance',votes=len(distinct),negative_votes=negative,
                evidence=votes,positions_modified=False,axis_angle_modified_between_supported_anchors=bool(flip),source='front_paired_red_rear_appearance_not_motion_sign')
