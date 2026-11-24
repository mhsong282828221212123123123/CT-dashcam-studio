"""Vehicle-heading display and bounded travel corridors through turns.

Lateral lane phase remains independent. The odometry trace is an estimated
travel corridor, not evidence of intersection topology or lane count.
"""
import math
import numpy as np


def _rotation(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s],[s,c]])


def _heading_available(scene):
    data=scene.telemetry or {}
    heading=data.get('heading');speed=data.get('speed_kmh')
    return (isinstance(heading,(int,float)) and math.isfinite(heading)
            and isinstance(speed,(int,float)) and math.isfinite(speed) and 0<=speed<250)


def _sample_route(centres,distances,targets):
    distinct=np.r_[True,np.diff(distances)>.05]
    centres=centres[distinct];distances=distances[distinct]
    if len(centres)<3:return None
    points=np.column_stack([np.interp(targets,distances,centres[:,axis]) for axis in (0,1)])
    directions=np.gradient(centres,distances,axis=0)
    directions/=np.maximum(np.linalg.norm(directions,axis=1)[:,None],1e-8)
    before=targets<distances[0];after=targets>distances[-1]
    points[before]=centres[0]+(targets[before]-distances[0])[:,None]*directions[0]
    points[after]=centres[-1]+(targets[after]-distances[-1])[:,None]*directions[-1]
    tangent=np.column_stack([np.interp(targets,distances,directions[:,axis]) for axis in (0,1)])
    tangent/=np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1e-8)
    return points,np.column_stack([tangent[:,1],-tangent[:,0]])


def apply_ego_turns(scenes,poses,audits):
    if not (len(scenes)==len(poses)==len(audits)):raise ValueError('Ego turn timeline mismatch')
    starts=[0]+[i for i in range(1,len(scenes)) if scenes[i].time-scenes[i-1].time>.5
        or any('tracks reset' in e.get('reason','') for e in audits[i].get('world_track_events',[]))]
    for start,end in zip(starts,starts[1:]+[len(scenes)]):
        if start==end:continue
        times=np.array([s.time for s in scenes[start:end]])
        angles=np.unwrap([math.atan2(R[1,0],R[0,0]) for _,R in poses[start:end]])
        valid=np.array([_heading_available(s) for s in scenes[start:end]])
        camera=angles.copy()
        for k in range(1,len(camera)):
            # Follow yaw gradually, so a rear-upper view shows turn-in while
            # retaining a useful view ahead after a large turn. No steering-
            # wheel angle is ever substituted for actual vehicle yaw.
            alpha=1.-math.exp(-(times[k]-times[k-1])/.8)
            camera[k]=camera[k-1]+alpha*(angles[k]-camera[k-1]) if valid[k] and valid[k-1] else angles[k]
        turns=np.zeros(len(times),bool)
        for k,t in enumerate(times):
            window=np.flatnonzero(abs(times-t)<=1.5)
            if len(window)<5 or not valid[window].all():continue
            steps=np.diff(angles[window]);net=abs(float(angles[window[-1]]-angles[window[0]]))
            # A lane change's returning yaw excursion must not become a road
            # bend. Require sustained, substantially one-directional rotation.
            if net>=math.radians(12) and net>=.65*float(abs(steps).sum()) and np.max(abs(steps))<math.radians(12):
                turns[k]=True
        weights=np.zeros(len(times))
        if turns.any():
            distance=np.min(abs(times[:,None]-times[turns][None,:]),axis=1)
            weights=np.clip((2.-distance)/1.,0.,1.)
        centres=[]
        for i in range(start,end):
            scene=scenes[i];view=getattr(scene,'road_view',None)
            if not view:centres.append(poses[i][0]);continue
            phase=getattr(scene,'ego_display_override',(view['offset'][0],0,0))[0]
            lateral=poses[i][1]@np.asarray(view['rotation']).T@np.array([1.,0.])
            centres.append(poses[i][0]-lateral*phase)
        centres=np.asarray(centres)
        distances=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(centres,axis=0),axis=1))]
        for k,i in enumerate(range(start,end)):
            scene=scenes[i];view=getattr(scene,'road_view',None)
            if not view or not valid[k]:
                audits[i]['ego_heading']=dict(source='unavailable',turn_path_estimated=False)
                continue
            original=dict(rotation=np.asarray(view['rotation']).tolist(),offset=list(view['offset']))
            # Membership/lane votes use the road reference, not the camera's
            # display-follow angle. Rotating the view cannot change road scope.
            scene.road_scope_view=original
            lag=float(angles[k]-camera[k]);follow=_rotation(lag)
            # Body yaw must not inherit a noisy paint-derived road angle. The
            # camera follows vehicle heading; paint continues to orient the
            # road geometry and lateral phase independently.
            Q=follow
            scene.road_view=dict(rotation=Q.tolist(),offset=list(view['offset']))
            x,z,yaw=scene.ego_display_override
            scene.ego_display_override=(x,z,-lag)
            applied=False
            if weights[k]>0:
                road=audits[i].get('travel_corridors',{})
                lanes=road.get('occupied_lane_indices',[]);width=road.get('lane_width_assumed_m',3.5)
                zs=np.linspace(-15.,65.,161)
                sampled=_sample_route(centres,distances,distances[k]+zs)
                if sampled is not None and lanes and len(scene.lanes)==max(lanes)-min(lanes)+2:
                    points,normals=sampled;origin,R=poses[i];paths=[]
                    for j,boundary in enumerate(np.arange(min(lanes)-.5,max(lanes)+1.,1.)*width):
                        curve=(points+normals*boundary-origin)@R
                        old=np.asarray(scene.lanes[j]['points'])
                        if old.shape!=curve.shape:break
                        paths.append((old*(1.-weights[k])+curve*weights[k]).tolist())
                    if len(paths)==len(scene.lanes):
                        scene.lanes=[dict(lane,points=path,source='heading_odometry_turn_corridor',
                            extrapolated=True) for lane,path in zip(scene.lanes,paths)]
                        scene.road_surface=paths[-1]+paths[0][::-1]
                        applied=True
            scene.ego_turn_path_estimated=applied
            audits[i]['ego_heading']=dict(source='vehicle_heading_odometry',
                world_heading_change_deg=math.degrees(float(angles[k]-angles[0])),
                camera_follow_lag_deg=math.degrees(lag),
                display_yaw_deg=math.degrees(scene.ego_display_override[2]),
                lateral_phase_unchanged=True,turn_path_estimated=applied,
                road_geometry_accuracy='estimated travel corridor; not intersection topology')
