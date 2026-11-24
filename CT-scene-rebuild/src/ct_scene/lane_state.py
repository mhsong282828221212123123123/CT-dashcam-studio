"""Offline road-relative membership; never changes a pose or road boundary."""
import math
import numpy as np
from .road_geometry import lane_at,lane_bounds,boundaries_at,section_at,support_kind
from .road_frame import coordinates


def timeline(times,positions,yaws,road,length,width,centre_x_offset=0.,dwell_s=.6,margin_m=.12,start_offset_m=.4,min_progress_m=1.):
    t=np.asarray(times,float);p=np.asarray(positions,float);yaw=np.asarray(yaws,float)
    bounds=np.asarray(road['boundaries_y'],float);slope=road.get('heading_evidence',{}).get('slope',0.)
    if p.shape!=(len(t),3) or yaw.shape!=t.shape or len(t)<2 or np.any(np.diff(t)<=0):raise ValueError('Ordered pose timeline required')
    if not np.isfinite(p).all() or not np.isfinite(t).all() or not np.isfinite(yaw).all() or min(length,width)<=0:raise ValueError('Finite pose and positive footprint required')
    if len(bounds)<2 or np.any(np.diff(bounds)>=0):raise ValueError('Descending road boundaries required')
    if not np.isfinite([dwell_s,margin_m,start_offset_m,min_progress_m]).all() or dwell_s<=0 or min(margin_m,start_offset_m,min_progress_m)<0:raise ValueError('Invalid lane-state policy')
    # The renderer centres ego behind the front-camera origin. Use that same
    # footprint, rather than treating the camera as the vehicle centre.
    centre=p.copy();centre[:,0]+=centre_x_offset*np.cos(yaw);centre[:,1]+=centre_x_offset*np.sin(yaw)
    local=coordinates(road,centre);station=local[:,0]
    section_ids=[(section_at(road,float(s)) or {}).get('id') for s in station]
    norm=1. if road.get('reference_line') else math.sqrt(1+slope*slope)
    lateral=local[:,1]/norm;normal_bounds=bounds/norm
    heading=yaw-local[:,2]
    half_extent=width/2*abs(np.cos(heading))+length/2*abs(np.sin(heading))
    raw=[];clearance=[]
    for i,y in enumerate(lateral):
        lane=lane_at(road,float(station[i]),float(y*norm))
        if lane is not None and lane>=0 and road.get('marking_station_x') is not None and station[i]>road['marking_station_x']+12:
            active=road.get('post_marking_boundaries_y',bounds.tolist())
            if bounds[lane] not in active or bounds[lane+1] not in active:lane=None
        raw.append(lane)
        local_bounds=lane_bounds(road,float(station[i]),lane) if lane is not None else None
        clearance.append(None if local_bounds is None else float(min(local_bounds[0]/norm-y,y-local_bounds[1]/norm)-half_extent[i]))
    membership=list(raw);events=[];current=raw[0];i=1
    while i<len(t):
        if current is not None and current>=len(boundaries_at(road,float(station[i])))-1:
            # A taper can remove the outer lane; disappearance of its index
            # alone is not evidence that this vehicle changed lanes.
            current=raw[i];i+=1;continue
        if section_ids[i]!=section_ids[i-1]:current=raw[i];i+=1;continue
        if raw[i] is None:current=None;i+=1;continue
        if current is None:current=raw[i];i+=1;continue
        if raw[i]==current:i+=1;continue
        target=raw[i];j=i
        while j+1<len(t) and raw[j+1]==target and section_ids[j+1]==section_ids[i]:j+=1
        active_bounds=np.asarray(boundaries_at(road,float(station[i])),float)/norm
        shared=active_bounds[max(current,target)] if abs(target-current)==1 else None
        penetration=max(abs(lateral[k]-shared) for k in range(i,j+1)) if shared is not None else 0
        if t[j]-t[i]<dwell_s or shared is None or penetration<.15:
            for k in range(i,j+1):membership[k]=current
            i=j+1;continue
        previous=i-1
        while previous>0 and raw[previous] is None:previous-=1
        denom=lateral[i]-lateral[previous]
        crossing=float(t[previous]+(shared-lateral[previous])/denom*(t[i]-t[previous])) if abs(denom)>1e-9 else float(t[i])
        local_bounds=lane_bounds(road,float(station[previous]),current);old_centre=(local_bounds[0]+local_bounds[1])/2/norm
        start=previous
        while start>0 and section_ids[start-1]==section_ids[i] and crossing-t[start]<8 and raw[start]==current and abs(lateral[start]-old_centre)>start_offset_m:start-=1
        if abs(lateral[start]-old_centre)<=start_offset_m and start<previous:start+=1
        completion=None
        for k in range(i,j+1):
            if clearance[k] is None or clearance[k]<margin_m:continue
            end=int(np.searchsorted(t,t[k]+dwell_s))
            if end<=j and all(raw[q]==target and clearance[q] is not None and clearance[q]>=margin_m for q in range(k,end+1)):
                completion=float(t[k]);break
        finish=int(np.searchsorted(t,completion)) if completion is not None else j
        if abs(station[finish]-station[start])<min_progress_m:
            for k in range(i,j+1):membership[k]=current
            i=j+1;continue
        events.append({'from_lane':int(current),'to_lane':int(target),'direction':'left' if target<current else 'right',
            'road_section_id':section_ids[i],
            'road_geometry_support':support_kind(road,float(station[i])),
            'start_s':float(t[start]),'crossing_s':crossing,'completion_s':completion,'status':'completed' if completion is not None else 'completion_unresolved',
            'last_supported_s':float(t[j]),
            'completion_rule':'whole_vehicle_footprint_inside_target_lane_with_dwell','uses_future_observations':True,'road_support':'estimated_camera_road_geometry'})
        current=target;i=j+1
    records=[]
    for i,now in enumerate(t):
        phase='stable' if raw[i] is not None else 'outside_or_unknown_road'
        for event in events:
            if event['start_s']<=now and (now<=event['last_supported_s'] if event['completion_s'] is None else now<event['completion_s']):
                phase='changing_before_crossing' if now<event['crossing_s'] else 'crossed_not_fully_inside'
        records.append({'time_s':float(now),'lane_id':membership[i],'observed_geometric_lane':raw[i],'phase':phase,
            'road_section_id':section_ids[i],
            'road_geometry_support':support_kind(road,float(station[i])),
            'road_lateral_m':float(lateral[i]),'footprint_clearance_m':clearance[i],'position_modified':False,'uses_future_observations':True})
    return {'records':records,'events':events,'lane_ids':'section-local boundary-pair indices; section transition never implies lane change' if road.get('lane_sections') else 'fixed boundary-pair indices, never renumbered by ego motion',
        'ego_centre_x_offset_m':centre_x_offset,'dwell_s':dwell_s,'footprint_width_m':width,'footprint_length_m':length,'pose_modified':False,'road_modified':False,
        'policy':{'dwell_s':dwell_s,'margin_m':margin_m,'start_offset_m':start_offset_m,'min_progress_m':min_progress_m}}
