"""Offline lane support for surrounding actors, independent of ego lane ID.

Returns a soft lateral target, never an ego pose or a new road boundary.
Membership is estimated geometry; events still require source review.
"""
import numpy as np


def observation_usable(row):
    camera=row.get('camera_id')
    if camera is None:return True  # geometry-only callers / synthetic fixtures
    if camera not in ('front','left_repeater','right_repeater'):return False
    if row['support']!='detector_observation':return False
    b=row['bbox_xyxy'];w=row['image_width'];h=row.get('image_height',960)
    minimum=(100,120) if camera=='front' else (120,160)
    return b[2]-b[0]>=minimum[0] and b[3]-b[1]>=minimum[1] and min(b[0],b[1],w-b[2],h-b[3])>6


def estimate(rows,boundaries,slope,fallback,dwell_s=.6,margin_m=.2):
    times=np.array([r['time_s'] for r in rows]);bounds=np.asarray(boundaries,float)
    if not len(rows) or np.any(np.diff(times)<=0) or np.any(np.diff(bounds)>=0):raise ValueError('Ordered actor timeline and descending road boundaries required')
    centres=(bounds[:-1]+bounds[1:])/2
    positions=np.array([r.get('unrefined_position_W',r['position_W']) for r in rows])
    lateral=positions[:,1]-slope*positions[:,0]
    filtered=np.array([np.median(lateral[abs(times-t)<=.4]) for t in times])
    usable=[observation_usable(r) for r in rows];raw=[]
    for i,y in enumerate(filtered):
        lane=next((k for k in range(len(centres)) if bounds[k+1]+margin_m<=y<=bounds[k]-margin_m),None)
        raw.append(lane if usable[i] else None)
    stable=[];i=0
    while i<len(rows):
        j=i
        while j+1<len(rows) and raw[j+1]==raw[i] and rows[j+1].get('camera_id')==rows[i].get('camera_id') and times[j+1]-times[j]<=.5:j+=1
        if raw[i] is not None and times[j]-times[i]+1e-6>=dwell_s:stable.append((i,j,raw[i]))
        i=j+1
    lanes=np.full(len(rows),stable[0][2] if stable else fallback,dtype=int)
    target=centres[lanes].copy();events=[];rejected=[];previous=None
    for start,end,lane in stable:
        current=int(lanes[start]);camera=rows[start].get('camera_id')
        if lane==current:
            previous=(start,end,lane);continue
        old_start,old_end,old_lane=previous
        reason=None
        if camera!=rows[old_end].get('camera_id'):reason='camera_transition_without_within_camera_change_evidence'
        elif lane is None or abs(lane-current)!=1:reason='non_adjacent_lane_candidate'
        elif times[start]-times[old_end]>4:reason='unobserved_interval_too_long'
        if reason:
            rejected.append(dict(time_s=float(times[start]),from_lane=current,to_lane=int(lane),reason=reason));continue
        # Let actual lateral observations shape the manoeuvre instead of a
        # step from one lane centre to the next at the confirmation timestamp.
        near_old=[k for k in range(old_start,old_end+1) if abs(filtered[k]-centres[current])<=.35]
        departure=near_old[-1] if near_old else old_start
        near_new=[k for k in range(start,end+1) if abs(filtered[k]-centres[lane])<=.35]
        arrival=near_new[0] if near_new else end
        delta=centres[lane]-centres[current]
        progress=np.maximum.accumulate(np.clip((filtered[departure:arrival+1]-centres[current])/delta,0,1))
        target[departure:arrival+1]=centres[current]+progress*delta
        lanes[start:]=lane;target[arrival+1:]=centres[lane]
        events.append(dict(from_lane=current,to_lane=int(lane),start_s=float(times[departure]),end_s=float(times[arrival]),
                           camera_id=camera,support='sustained_clear_within_camera_geometry',uses_future_observations=True))
        previous=(start,end,lane)
    review=dict(events=events,rejected_candidates=rejected,clear_observations=sum(usable),
                support='clear_observation' if stable else 'fallback_lane_unresolved',
                first_clear_lane_time_s=float(times[stable[0][0]]) if stable else None,
                observer='fixed_world_road_frame; ego_lane_not_used',metric_accuracy='NOT_EVALUATED')
    return lanes,target,review
