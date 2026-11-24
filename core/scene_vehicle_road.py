"""Vehicle-supported travel lanes, anchored independently of the moving host.

This is an inferred corridor display, not surveyed lane marking geometry.
Road direction and lateral phase are observed separately from lane identity. Host
changes move the host through that road. Sustained residence establishes a lane
in road coordinates, never from the apparent opposite motion of other cars.
"""
from collections import defaultdict
import math
import numpy as np

VEHICLE_KINDS={'car','suv','van','compact_car','compact_van','truck','truck_box',
               'truck_medium','truck_heavy','small_truck','bus','motorcycle','bicycle',
               'motorcycle_body','bicycle_body'}


def lane_evidence_actor(actor):
    """Current image-supported positions can establish a lane, ghosts cannot."""
    if actor.kind not in VEHICLE_KINDS or actor.position_hypothesis:return False
    if actor.tracking_state=='observed':return True
    return bool(actor.tracking_state=='reconstructed' and actor.image_observed
                and any(not s.startswith(('back:','review:back:')) for s in actor.sources))


def _road_basis(scenes,poses,start,end):
    origin,initial=poses[start]
    slopes=[]
    # Paint can orient the initial road, but neither its absence nor the host's
    # subsequent steering changes the established lane reference frame.
    for i in range(start,end):
        if scenes[i].time-scenes[start].time>1.:break
        o,R=poses[i]
        for lane in scenes[i].lanes:
            points=np.asarray(lane.get('points',[]),float)
            if points.ndim!=2 or points.shape[0]<2 or points.shape[1]!=2:continue
            points=(points@R.T+o-origin)@initial
            if not np.isfinite(points).all() or np.ptp(points[:,1])<5:continue
            slope=float(np.polyfit(points[:,1],points[:,0],1)[0])
            if abs(slope)<.25:slopes.append(slope)
    slope=float(np.median(slopes)) if slopes else 0.
    forward=np.array([slope,1.]);forward/=np.linalg.norm(forward)
    lateral=np.array([forward[1],-forward[0]])
    return origin.copy(),initial@np.column_stack([lateral,forward])


def _paint_frame(scene,width):
    lines=[]
    for lane in scene.lanes:
        points=np.asarray(lane.get('points',[]),float)
        if points.ndim!=2 or points.shape[0]<2 or points.shape[1]!=2:continue
        points=points[(points[:,1]>=3)&(points[:,1]<=25)]
        if len(points)<2 or np.ptp(points[:,1])<4:continue
        slope,offset=np.polyfit(points[:,1],points[:,0],1)
        if abs(slope)<.3 and abs(offset)<10:lines.append((slope,offset))
    # One far/curved fragment cannot distinguish road curvature from the host's
    # lateral lane phase. Require a consistent pair before updating that phase.
    if len(lines)<2:return None
    if not any(abs(a[0]-b[0])<.15 and width*.7<abs(a[1]-b[1])<width*1.3
               for j,a in enumerate(lines) for b in lines[j+1:]):return None
    slope=float(np.median([v[0] for v in lines]));angle=math.atan(slope)
    c,s=math.cos(angle),math.sin(angle);Q=np.array([[c,-s],[s,c]])
    phases=np.array([-offset*c-width/2 for _,offset in lines])
    # A boundary crossing wraps this phase by exactly one lane. Unwrapping over
    # time keeps the host's actual lane change instead of adding an opposite lane.
    circular=np.exp(phases*2j*math.pi/width).mean()
    if abs(circular)<.65:return None
    return Q,float(np.angle(circular)*width/(2*math.pi))


def _road_frames(scenes,poses,start,end,width):
    anchor,basis=_road_basis(scenes,poses,start,end)
    evidence=[None if getattr(scenes[i],'road_junction_state',None)=='inside' else _paint_frame(scenes[i],width)
              for i in range(start,end)]
    if not any(v is not None for v in evidence):
        # Without a road reference a bend and a lane change are ambiguous.
        # Do not integrate a fixed world tangent into an invented lateral pose.
        return [(np.eye(2),0.,'unobserved_road_reference') for i in range(start,end)]
    times=np.array([scenes[i].time for i in range(start,end)])
    anchors=[i for i,v in enumerate(evidence) if v is not None]
    # Road orientation comes only from paint and host odometry. Interpolate its
    # world heading through missing paint; moving objects never enter this path.
    headings=np.unwrap([math.atan2(poses[start+i][1][1,0],poses[start+i][1][0,0])
                        -math.atan2(evidence[i][0][1,0],evidence[i][0][0,0]) for i in anchors])
    road_heading=np.interp(times,times[anchors],headings)
    phase_origin=evidence[anchors[0]][1]
    frames=[];offset=0.;last_anchor=None
    for k,i in enumerate(range(start,end)):
        paint=evidence[k]
        # np.interp holds its endpoint heading outside the observed interval.
        # Continuing odometry there turns every road bend into lateral drift.
        # Hold the last visual lane phase; keep this uncertainty explicit.
        if paint is None and (k<anchors[0] or k>anchors[-1]):
            Q=frames[-1][0].copy() if frames else evidence[anchors[0]][0].copy()
            frames.append((Q,offset,'held_without_two_sided_road_support'))
            continue
        if k:
            angle=(road_heading[k-1]+road_heading[k])/2
            lateral=np.array([math.cos(angle),math.sin(angle)])
            offset+=float(lateral@(poses[i][0]-poses[i-1][0]))
        angle=road_heading[k];c,s=math.cos(angle),math.sin(angle)
        Q=np.array([[c,s],[-s,c]])@poses[i][1]
        source='host_odometry_between_road_observations'
        if paint is not None:
            phase=paint[1]-phase_origin
            reference=frames[-1][1] if last_anchor==k-1 and frames else offset
            target=phase+round((reference-phase)/width)*width
            correction=target-offset
            # Use the later road observation to correct drift over the preceding
            # gap, rather than waiting until that frame to jump to the new pose.
            if last_anchor is not None:
                duration=times[k]-times[last_anchor]
                for j in range(last_anchor+1,k):
                    old_Q,old_x,_=frames[j]
                    q=(times[j]-times[last_anchor])/duration
                    frames[j]=(old_Q,old_x+correction*q,'two_sided_road_odometry')
            offset=target;last_anchor=k;Q=paint[0]
            source='camera_lane_phase'
        frames.append((Q.copy(),offset,source))
    return frames


def stabilize_road_frames(frames,times):
    """Offline symmetric local fits: suppress jitter without causal display lag.

    Called separately for each continuous odometry segment. Lane occupancy is
    decided from the original evidence, independently of display stabilization.
    """
    if len(frames)<5:return frames
    times=np.asarray(times,float)
    offsets=np.array([f[1] for f in frames])
    angles=np.unwrap([math.atan2(f[0][1,0],f[0][0,0]) for f in frames])
    values=np.column_stack([offsets,angles]);result=[]
    for i,t in enumerate(times):
        indices=np.flatnonzero(abs(times-t)<=1.2)
        if len(indices)<5:result.append(frames[i]);continue
        dt=times[indices]-t
        A=np.column_stack([np.ones(len(dt)),dt,dt*dt])
        if i<5 or i>=len(times)-5:A=A[:,:2]
        fitted=[]
        for column in range(2):
            y=values[indices,column];base=np.exp(-.5*(dt/.45)**2);weights=base.copy()
            for _ in range(4):
                p=np.linalg.lstsq(A*np.sqrt(weights[:,None]),y*np.sqrt(weights),rcond=None)[0]
                error=abs(y-A@p)
                floor=.025 if column==0 else math.radians(.15)
                scale=max(floor,1.4826*float(np.median(error)))
                weights=base*np.minimum(1.,1.5*scale/np.maximum(error,1e-9))
            fitted.append(float(p[0]))
        offset,angle=fitted;c,s=math.cos(angle),math.sin(angle)
        result.append((np.array([[c,-s],[s,c]]),offset,frames[i][2]))
    return result


def add_vehicle_corridors(scenes,poses,audits,width=3.5):
    if not scenes:return
    if len(scenes)!=len(poses) or len(scenes)!=len(audits):raise ValueError('Road timeline mismatch')
    if not math.isfinite(width) or width<=0:raise ValueError('Invalid display lane width')
    starts=[0]+[i for i in range(1,len(scenes)) if scenes[i].time-scenes[i-1].time>.5
                or audits[i].get('road_junction',{}).get('state')=='exit'
                or any('tracks reset' in e.get('reason','') for e in audits[i].get('world_track_events',[]))]
    for start,end in zip(starts,starts[1:]+[len(scenes)]):
        frames=_road_frames(scenes,poses,start,end,width)
        track_samples=defaultdict(list)
        has_road_reference=any(frame[2]=='camera_lane_phase' for frame in frames)
        aliases={}
        times=np.array([s.time for s in scenes[start:end]])
        headings=np.unwrap([math.atan2(R[1,0],R[0,0]) for _,R in poses[start:end]])
        turning=np.zeros(len(times),bool)
        for k,t in enumerate(times):
            nearby=np.flatnonzero(abs(times-t)<=1.5)
            turning[k]=len(nearby)>1 and abs(headings[nearby[-1]]-headings[nearby[0]])>=math.radians(12)
        if turning.any():
            turning=np.min(abs(times[:,None]-times[turning][None,:]),axis=1)<=2.
        for i in range(start,end):
            Q,offset,reference_source=frames[i-start]
            if has_road_reference and reference_source!='camera_lane_phase':
                # A distant lead car on a bend can cross a held straight display
                # corridor. That is not evidence of another physical lane.
                continue
            track_samples['ego'].append((scenes[i].time,offset))
            # Cross traffic and rotating camera-relative positions during a
            # large turn do not establish additional longitudinal lanes.
            if turning[i-start] or audits[i].get('road_junction',{}).get('state')=='inside':continue
            for actor in scenes[i].actors:
                if not lane_evidence_actor(actor):continue
                direction=Q@np.array([math.sin(actor.yaw),math.cos(actor.yaw)])
                if direction[1]<math.cos(math.radians(30)):continue
                if not (-20<actor.z<60 if has_road_reference else -12<actor.z<22):continue
                point=Q@np.array([actor.x,actor.z])+[offset,0.]
                sources=[s for s in actor.sources if not s.startswith(('back:','review:back:'))]
                key=next((aliases[s] for s in sources if s in aliases),actor.key)
                for source in sources:aliases[source]=key
                if np.isfinite(point).all():track_samples[key].append((scenes[i].time,float(point[0])))
        occupied={0};support={}
        if not has_road_reference:
            # With no paint phase, one source's drifting pose cannot vote for
            # several lanes. Use its first stable near-field window, grouping
            # overlapping lane-position evidence without moving any actor.
            anchors=[]
            for key,samples in track_samples.items():
                if key=='ego':continue
                for j in range(len(samples)):
                    window=[v for v in samples[j:] if v[0]-samples[j][0]<=1.2]
                    if len(window)<5 or window[-1][0]-window[0][0]<.8:continue
                    if max(np.diff([v[0] for v in window]))>.3:continue
                    xs=np.array([v[1] for v in window])
                    if np.ptp(xs)>width*.65:continue
                    anchors.append((float(np.median(xs)),key));break
            prototypes=[(0.,0)]
            for x,key in sorted(anchors,key=lambda v:abs(v[0])):
                old,index=min(prototypes,key=lambda v:abs(v[0]-x));gap=x-old
                if abs(gap)>width*.55:
                    steps=max(1,int(math.floor(abs(gap)/width+.4)))
                    index+=int(math.copysign(steps,gap))
                    if abs(index)>4:continue
                    prototypes.append((x,index))
                occupied.add(index);support.setdefault(str(index),[]).append(key)
        for key,samples in track_samples.items():
            if not has_road_reference and key!='ego':continue
            # Count sustained residence near a lane centre, not the intermediate
            # offsets of a lane change or a single inaccurate placement.
            runs=[];run=[];last_lane=None
            for t,x in samples:
                lane=int(np.rint(x/width))
                eligible=abs(lane)<=4 and abs(x-lane*width)<=width*.28
                if not eligible or lane!=last_lane or (run and t-run[-1][0]>.3):
                    if run:runs.append((last_lane,run))
                    run=[]
                if eligible:run.append((t,x));last_lane=lane
                else:last_lane=None
            if run:runs.append((last_lane,run))
            for lane,run in runs:
                if len(run)<5 or run[-1][0]-run[0][0]<.8:continue
                occupied.add(lane)
                support.setdefault(str(lane),[]).append(key)
        first,last=min(occupied),max(occupied)
        boundaries=np.arange(first-.5,last+1.,1.)*width
        raw_frames=frames
        stable_frames=stabilize_road_frames(frames,[s.time for s in scenes[start:end]])
        for i in range(start,end):
            scene=scenes[i];Q,offset,phase_source=frames[i-start]
            zs=np.linspace(-15,65,161)
            paths=[]
            for x in boundaries:
                road=np.column_stack([np.full(len(zs),x-offset),zs])
                paths.append((road@Q).tolist())
            scene.observed_lane_segments=list(scene.lanes)
            scene.lanes=[dict(points=path,dashed=0<j<len(paths)-1,observed=False,
                inferred=True,source='vehicle_supported_world_corridor',
                edge_role='outer_display_boundary' if j in (0,len(paths)-1) else 'internal',
                extrapolated=True) for j,path in enumerate(paths)]
            scene.road_surface=paths[-1]+paths[0][::-1]
            scene.road_surfaces=[]
            # One rigid transform for host, actors, road, and signals. The camera
            # follows longitudinal travel but not lateral lane-changing motion.
            scene.road_view=dict(rotation=Q.tolist(),offset=[offset,0.])
            stable_Q,stable_offset,_=stable_frames[i-start]
            scene.ego_display_override=(stable_offset,0.,math.atan2(stable_Q[0,1],stable_Q[1,1]))
            scene.ego_position_state='held_uncertain' if phase_source in ('held_without_two_sided_road_support','unobserved_road_reference') else 'estimated'
            audits[i]['travel_corridors']=dict(source='vehicle_supported_world_corridor',
                display_lane_count=last-first+1,lane_width_assumed_m=width,
                occupied_lane_indices=sorted(occupied),supporting_tracks=support,
                host_lane_coordinate=stable_offset/width,lane_votes_coordinate_frame='persistent_lane_identity',
                raw_host_lane_coordinate=raw_frames[i-start][1]/width,
                display_stabilization='symmetric_robust_local_quadratic',
                phase_source=phase_source,outer_edges_confirmed=False,
                geometry_accuracy='inferred display; not surveyed',segment_start_s=scenes[start].time)
