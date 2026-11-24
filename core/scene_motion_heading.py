"""Offline motion direction evidence, not a stationary vehicle pose estimator."""
from collections import defaultdict
import math
import numpy as np


def motion_headings(scenes, poses):
    tracks = defaultdict(list)
    for i, scene in enumerate(scenes):
        for actor in scene.actors:
            if actor.tracking_state == 'observed':
                tracks[actor.key].append((i, actor))
    evidence = {}
    for key, history in tracks.items():
        for i, actor in history:
            window = [(k, a) for k, a in history if abs(scenes[k].time-scenes[i].time) <= .8]
            if len(window) < 8:
                continue
            times = np.array([scenes[k].time-scenes[i].time for k, _ in window])
            if np.ptp(times) < .8 or np.max(np.diff(times)) > .3:
                continue
            positions = np.array([poses[k][0]+poses[k][1]@np.array([a.x,a.z]) for k,a in window])
            # Curved travel needs the instantaneous tangent, not a 1.6-second
            # straight chord. Weight nearby samples while retaining noise checks.
            design = np.column_stack([np.ones(len(times)),times,times*times])
            weight=np.exp(-.5*(times/.35)**2)
            fit = np.linalg.lstsq(design*np.sqrt(weight[:,None]),positions*np.sqrt(weight[:,None]),rcond=None)[0]
            speed = np.linalg.norm(fit[1])
            if not np.isfinite(fit).all() or not 3 <= speed <= 55:
                continue
            residual = np.linalg.norm(design@fit-positions,axis=1)
            if np.max(residual) > .8:
                continue
            # Independent halves must agree; a jump between cameras must not
            # become a confident direction of travel.
            velocities=[]
            for indices in np.array_split(np.arange(len(times)),2):
                if len(indices)<4 or np.ptp(times[indices]) < .2:
                    break
                velocities.append(np.linalg.lstsq(design[indices],positions[indices],rcond=None)[0][1])
            if len(velocities)!=2 or any(np.linalg.norm(v)<2 for v in velocities):
                continue
            if np.dot(*velocities)/(np.linalg.norm(velocities[0])*np.linalg.norm(velocities[1])) < .95:
                continue
            if np.linalg.norm(velocities[0]-velocities[1]) > 3:
                continue
            local = poses[i][1].T@fit[1]
            evidence[i,key] = {'yaw':math.atan2(local[0],local[1]),'speed_mps':float(speed),
                'residual_max_m':float(residual.max()),'source':'two_sided_world_motion',
                'body_orientation_verified':False}
    return evidence


def apply_motion_headings(scenes,poses,audits,source_directions=None):
    """Point model fronts along verified travel; retain world direction at rest.

    Reversing vehicles cannot be distinguished from forward travel from motion
    alone. Position estimates are never changed by this display orientation.
    """
    evidence=motion_headings(scenes,poses);memory={};applied=0
    source_directions=source_directions or {};heading_times={}
    for i,(scene,audit) in enumerate(zip(scenes,audits)):
        if i and (scene.time-scenes[i-1].time>.3 or any('tracks reset' in e.get('reason','') for e in audit.get('world_track_events',[]))):
            memory.clear();heading_times.clear()
        report=[];present={a.key for a in scene.actors}
        memory={k:v for k,v in memory.items() if k in present}
        for actor in scene.actors:
            if actor.kind not in ('car','truck','bus','motorcycle','bicycle'):continue
            item=evidence.get((i,actor.key))
            image_items=[source_directions[i,s] for s in actor.sources if (i,s) in source_directions]
            if image_items:
                item=image_items[0]
            if item:
                axis=poses[i][1]@np.array([math.sin(item['yaw']),math.cos(item['yaw'])])
                if actor.key in memory and actor.key in heading_times:
                    old=math.atan2(memory[actor.key][0],memory[actor.key][1])
                    target=math.atan2(axis[0],axis[1]);change=(target-old+math.pi)%(2*math.pi)-math.pi
                    limit=math.radians(90)*max(0.,scene.time-heading_times[actor.key])
                    angle=old+float(np.clip(change,-limit,limit));axis=np.array([math.sin(angle),math.cos(angle)])
                memory[actor.key]=axis
                source='image_motion_forward_assumption' if image_items else 'world_motion_forward_assumption'
            elif actor.key in memory:source='retained_world_heading'
            else:
                report.append(dict(id=actor.key,source='unknown',body_orientation_verified=False))
                continue
            axis=poses[i][1].T@memory[actor.key]
            actor.yaw=math.atan2(axis[0],axis[1]);applied+=1
            heading_times[actor.key]=scene.time
            report.append(dict(id=actor.key,yaw_rad=actor.yaw,source=source,
                               body_orientation_verified=False))
        audit['vehicle_headings']=report
    return applied
