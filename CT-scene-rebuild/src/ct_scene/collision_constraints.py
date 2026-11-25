"""Ground-plane body overlap and bounded offline position constraints.

These are geometry constraints, not a crash/force simulation or identity merge.
Unresolvable conflicts remain visible and fail the overlap review.
"""
from collections import defaultdict
import copy,math
import numpy as np
from .road_frame import coordinates,world,frame_for
from .road_geometry import lane_at,lane_bounds
from .junctions import inside as inside_junction


def axes(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,s],[-s,c]])


def overlap(a,b,dimensions):
    """SAT minimum body penetration; exact rotated rectangles, not AABBs."""
    aa,bb=axes(a['yaw_rad']),axes(b['yaw_rad'])
    ha=np.asarray(dimensions[a['model']][:2],float)/2
    hb=np.asarray(dimensions[b['model']][:2],float)/2
    delta=np.asarray(b['position_W'][:2])-a['position_W'][:2]
    if delta@delta>(np.linalg.norm(ha)+np.linalg.norm(hb))**2:return None
    smallest=float('inf');normal=None
    for v in np.vstack([aa,bb]):
        extent=np.abs(aa@v)@ha+np.abs(bb@v)@hb
        depth=float(extent-abs(delta@v))
        if depth<=0:return None
        if depth<smallest:
            smallest=depth;normal=v*(1 if delta@v>=0 else -1)
    return smallest,normal


def bodies(frame):
    result=list(frame['actors'])
    if frame.get('ego_position_W') is not None:
        yaw=frame['ego_yaw_rad'];p=np.asarray(frame['ego_position_W'],float).copy()
        p[:2]+=axes(yaw)[0]*-1.2
        result.append(dict(id='__ego__',model='ego',position_W=p,yaw_rad=yaw,collision_cap_m=0.))
    return result


def omit_conflicting_predictions(frames,dimensions,near_m=30.,penetration_m=.5):
    """Prefer visible bodies over an intersecting, unobserved nearby prediction.

    This is a display omission, not an identity merge or geometry correction.
    Keep every observation and record each omitted prediction for review.
    """
    result=copy.deepcopy(frames);omitted=[]
    for frame in result:
        ego=frame.get('ego_position_W')
        if ego is None:continue
        observed=[a for a in bodies(frame) if a['id']=='__ego__' or
                  (a.get('source_visible') and not a.get('ghost'))]
        kept=[]
        for actor in frame['actors']:
            blockers=[]
            if actor.get('ghost') and not actor.get('source_visible') and np.linalg.norm(
                    np.asarray(actor['position_W'][:2])-np.asarray(ego[:2]))<=near_m:
                for other in observed:
                    hit=overlap(actor,other,dimensions)
                    threshold=0. if actor.get('position_support')=='bounded_pillar_blindspot_prediction' else penetration_m
                    if hit and hit[0]>threshold:blockers.append(other['id'])
            if blockers:
                omitted.append(dict(time_s=frame['time_s'],id=actor['id'],observed_ids=blockers,
                                    reason='unobserved_prediction_intersects_visible_body'))
            else:kept.append(actor)
        frame['actors']=kept
    return result,dict(omitted_samples=omitted,omitted_count=len(omitted),near_m=near_m,
                       penetration_m=penetration_m,observations_or_identity_modified=False,
                       role='display_only; source actors retained')


def audit(frames,dimensions,tolerance_m=.1):
    events=defaultdict(list)
    for frame in frames:
        bs=bodies(frame)
        for i,a in enumerate(bs):
            for b in bs[i+1:]:
                hit=overlap(a,b,dimensions)
                if hit and hit[0]>tolerance_m:
                    events[tuple(sorted([a['id'],b['id']]))].append((frame['time_s'],hit[0]))
    intervals=[]
    for pair,rows in sorted(events.items()):
        groups=[]
        for row in rows:
            if not groups or row[0]-groups[-1][-1][0]>.076:groups.append([])
            groups[-1].append(row)
        for group in groups:
            peak=max(group,key=lambda r:r[1])
            intervals.append(dict(ids=list(pair),start_s=group[0][0],end_s=group[-1][0],
                                  frames=len(group),peak_time_s=peak[0],penetration_peak_m=peak[1]))
    return dict(gate='FAIL' if intervals else 'PASS',body_overlap_intervals=intervals,
                overlapping_pair_frames=sum(len(r) for r in events.values()),
                tolerance_m=tolerance_m,scope='Estimated GLB XY footprints including ego; not physical crash detection or source accuracy')


def constrain(frames,road,dimensions,max_iterations=16):
    """Joint temporal correction, bounded by raw position and original lane.

    Only explicitly movable side estimates may move. Ego, front observations,
    IDs, headings, visibility, model scale and the road remain fixed.
    """
    result=copy.deepcopy(frames);original={};tracks=defaultdict(list)
    for i,f in enumerate(result):
        for a in f['actors']:
            a['position_W']=np.asarray(a['position_W'],float).copy()
            original[i,a['id']]=a['position_W'].copy();tracks[a['id']].append((i,a))
    before=audit(result,dimensions);frame=frame_for(road);lanes={}
    ext_line=(road.get('reference_line') or {}).get('supported_extent_m')
    unsup_iv=(road.get('reference_line') or {}).get('unsupported_intervals_m',[])
    def _effective_station(s_val,allow_ext=False):
        if not allow_ext or not ext_line:return float(s_val)
        if any(j['road_station_range_m'][0]<s_val<j['road_station_range_m'][1] for j in road.get('junctions',[])):return float(s_val)
        if any(a<s_val<b for a,b in unsup_iv):return float(s_val)
        if ext_line[0]-90.<=s_val<ext_line[0] or ext_line[1]<s_val<=ext_line[1]+90.:
            return float(np.clip(s_val,ext_line[0]+1e-3,ext_line[1]-1e-3))
        return float(s_val)
    for key,origin in original.items():
        q=frame.project(origin) if frame is not None else coordinates(road,origin)
        lanes[key]=lane_at(road,float(q[0]),float(q[1]))
    # Vehicles ahead of ego at clip start/end keep corridor lane continuity instead of snapping to None at ego_end+40m
    if ext_line:
        for aid,rows_a in tracks.items():
            if not any(lanes[i_r,aid] is not None for i_r,_ in rows_a):continue
            for i_r,_ in rows_a:
                if lanes[i_r,aid] is None:
                    origin=original[i_r,aid]
                    q=frame.project(origin) if frame is not None else coordinates(road,origin)
                    s_eff=_effective_station(float(q[0]),True)
                    if s_eff!=float(q[0]):
                        lanes[i_r,aid]=lane_at(road,s_eff,float(q[1]))
    corridor_ext_ids={aid for aid,rows_a in tracks.items() if ext_line and any(lanes[i_r,aid] is not None for i_r,_ in rows_a)}

    ego_conflict_keys=set()
    for i,f in enumerate(result):
        bs=bodies(f);ego_body=next((b for b in bs if b['id']=='__ego__'),None)
        if ego_body is None:continue
        for a in bs:
            if a['id']!='__ego__' and overlap(ego_body,a,dimensions):ego_conflict_keys.add(a['id'])

    def project(i,a,candidate):
        origin=original[i,a['id']];cap=float(a.get('collision_cap_m',0))
        lane=lanes[i,a['id']]
        allow_unresolved_ego=lane is None and a['id'] in ego_conflict_keys
        allow_ray_depth=lane is None and a.get('collision_axis_W') is not None
        allow_junction_side=lane is None and road.get('junctions') and inside_junction(road,origin,margin=5.)
        if cap<=0 or (lane is None and not allow_unresolved_ego and not allow_ray_depth and not allow_junction_side):return origin.copy()
        delta=np.asarray(candidate)-origin;delta[2]=0
        if a.get('collision_axis_W') is not None:
            axis=np.asarray(a['collision_axis_W'],float);axis/=np.linalg.norm(axis)
            delta[:2]=axis*np.dot(delta[:2],axis)
        norm=np.linalg.norm(delta[:2])
        if norm<1e-10:return origin.copy()
        if norm>cap:delta*=cap/norm
        point=origin+delta
        if lane is not None:
            local=frame.project(point) if frame is not None else coordinates(road,point)
            s_proj=_effective_station(float(local[0]),a['id'] in corridor_ext_ids)
            upper,lower=lane_bounds(road,s_proj,lane)
            if a.get('collision_axis_W') is not None and not (lower+.02<=local[1]<=upper-.02):
                # Keep ray-constrained depth adjustment strictly on the viewing ray and inside the original lane.
                lo,hi=0.,1.
                for _ in range(12):
                    mid=(lo+hi)/2
                    cand=origin+delta*mid
                    ql=frame.project(cand) if frame is not None else coordinates(road,cand)
                    u_m,l_m=lane_bounds(road, _effective_station(float(ql[0]),a['id'] in corridor_ext_ids),lane)
                    if l_m+.02<=ql[1]<=u_m-.02:lo=mid
                    else:hi=mid
                point=origin+delta*lo
            else:
                # Preserve the original centre's lane. No new lane or road is introduced.
                d=np.clip(local[1],lower+.02,upper-.02)
                if abs(d-local[1])>1e-8:
                    point[:2]=frame.world(float(local[0]),float(d))[:2] if frame is not None else world(road,float(local[0]),float(d))[:2]
        offset=point[:2]-origin[:2]
        if a.get('collision_axis_W') is not None and abs(axis[0]*offset[1]-axis[1]*offset[0])>1e-6:return origin.copy()
        if np.linalg.norm(point[:2]-origin[:2])>cap+1e-6:return origin.copy()
        return point

    iterations=0
    for iteration in range(max_iterations):
        iterations=iteration+1
        # Smooth the whole correction sequence in both time directions first,
        # then reproject contact constraints. No per-frame bounce integration.
        for key,rows in tracks.items():
            offsets=np.array([a['position_W']-original[i,key] for i,a in rows])
            smoothed=offsets.copy()
            for k,(i,a) in enumerate(rows):
                options=[j for j in range(max(0,k-2),min(len(rows),k+3)) if abs(result[rows[j][0]]['time_s']-result[i]['time_s'])<=.101]
                weights=np.array([3-abs(j-k) for j in options],float)
                smoothed[k]=np.average(offsets[options],axis=0,weights=weights)
            for (i,a),delta in zip(rows,smoothed):a['position_W']=project(i,a,original[i,key]+delta)
        for i,f in enumerate(result):
            bs=bodies(f)
            for ai,a in enumerate(bs):
                for b in bs[ai+1:]:
                    hit=overlap(a,b,dimensions)
                    if not hit or hit[0]<=.015:continue
                    wa=float(a.get('collision_cap_m',0));wb=float(b.get('collision_cap_m',0))
                    if wa+wb<=0:continue
                    # Evaluate all separating axes. A front depth estimate may
                    # move only along its observed viewing ray, never sideways
                    # to invent a different lane just to remove a collision.
                    options=[];delta_ab=np.asarray(b['position_W'][:2])-a['position_W'][:2]
                    for normal in np.vstack([axes(a['yaw_rad']),axes(b['yaw_rad'])]):
                        signed=float(delta_ab@normal)
                        if abs(signed)<.25:continue
                        normal=normal*(1 if signed>0 else -1)
                        ha=np.asarray(dimensions[a['model']][:2])/2;hb=np.asarray(dimensions[b['model']][:2])/2
                        depth=float(np.abs(axes(a['yaw_rad'])@normal)@ha+np.abs(axes(b['yaw_rad'])@normal)@hb-abs(signed))
                        proposals=[]
                        for body,weight,sign in [(a,wa,-1),(b,wb,1)]:
                            if weight<=0:proposals.append(np.asarray(body['position_W']));continue
                            direction=normal
                            if body.get('collision_axis_W') is not None:
                                axis=np.asarray(body['collision_axis_W']);alignment=float(axis@normal)
                                if abs(alignment)<.15:proposals.append(np.asarray(body['position_W']));continue
                                direction=axis/alignment
                            movement=np.r_[direction*sign*(depth+.03)*weight/(wa+wb),0.]
                            proposals.append(project(i,body,np.asarray(body['position_W'])+movement))
                        pa,pb=proposals;remaining=overlap(dict(a,position_W=pa),dict(b,position_W=pb),dimensions)
                        score=(remaining[0] if remaining else 0.,np.linalg.norm(pa-a['position_W'])+np.linalg.norm(pb-b['position_W']))
                        options.append((score,pa,pb))
                    if options:
                        score,pa,pb=min(options,key=lambda r:r[0])
                        if score[0]<hit[0]-1e-7:
                            # Damped step during early iterations lets temporal smoothing build a smooth ramp instead of a 1-frame velocity spike.
                            relax=.6 if iteration<max_iterations-3 else 1.
                            if wa>0:a['position_W']=project(i,a,np.asarray(a['position_W'])+relax*(pa-np.asarray(a['position_W'])))
                            if wb>0:b['position_W']=project(i,b,np.asarray(b['position_W'])+relax*(pb-np.asarray(b['position_W'])))
        if iteration>=3 and audit(result,dimensions)['gate']=='PASS':break
    # A smaller overlap count cannot justify a newly impossible motion. Reject
    # the affected temporal correction window, retaining its unresolved overlap.
    rejected=[]
    for key,rows in tracks.items():
        bad=[]
        for (i,a),(j,b) in zip(rows,rows[1:]):
            dt=result[j]['time_s']-result[i]['time_s']
            if not 0<dt<=.076:continue
            raw_speed=np.linalg.norm(original[j,key]-original[i,key])/dt
            speed=np.linalg.norm(b['position_W']-a['position_W'])/dt
            if speed>65 and raw_speed<=65:bad.append(result[j]['time_s'])
        if not bad:continue
        lo,hi=min(bad)-.25,max(bad)+.25
        for i,a in rows:
            t=result[i]['time_s'];distance=max(lo-t,t-hi,0.)
            weight=min(distance/.25,1.);u=weight*weight*(3-2*weight)
            a['position_W']=project(i,a,original[i,key]+u*(a['position_W']-original[i,key]))
        rejected.append(dict(id=key,start_s=lo,end_s=hi,reason='collision_separation_would_introduce_motion_discontinuity',source_review_required=True))
    changed=[]
    for i,f in enumerate(result):
        for a in f['actors']:
            before_pos=original[i,a['id']];delta=a['position_W']-before_pos
            if np.linalg.norm(delta)>.00001:
                a['position_before_collision_W']=before_pos.tolist()
                a['collision_correction_W']=delta.tolist()
                a['position_support_before_collision']=a.get('position_support')
                a['position_support']='bounded_collision_postcorrection'
                changed.append(dict(time_s=f['time_s'],id=a['id'],before_W=before_pos.tolist(),after_W=a['position_W'].tolist(),correction_m=float(np.linalg.norm(delta))))
    return result,dict(before=before,after=audit(result,dimensions),iterations=iterations,
                       rejected_correction_intervals=rejected,
                       changed_samples=changed,max_correction_m=max([r['correction_m'] for r in changed],default=0.),
                       identity_merge=False,ego_road_heading_scale_or_visibility_modified=False,
                       correction_role='bounded_offline_estimate; observation_samples_preserved_in_actors_json')
