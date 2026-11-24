"""Offline front-camera motion priors while the host is verifiably stopped.

Several independently tracked silhouettes can constrain a shared pitch proxy.
This is conditional on representative vehicle heights, not factory calibration
or a surveyed distance. It never supplies a host position or a new detection.
"""
from collections import defaultdict
from dataclasses import replace
import math
import numpy as np


def stopped_motion_priors(rows,camera):
    horizons={};directions={};reports={}
    if camera is None:return horizons,directions,reports
    groups=[]
    for i,row in enumerate(rows):
        data=(row.get('ego') or {}).get('data') or {}
        speed=data.get('speed_kmh')
        if speed is None or not math.isfinite(speed) or not 0<=speed<=.5:continue
        if not groups or i!=groups[-1][-1]+1 or row['time_s']-rows[groups[-1][-1]]['time_s']>.3:groups.append([])
        groups[-1].append(i)
    for group in groups:
        if rows[group[-1]]['time_s']-rows[group[0]]['time_s']<2:continue
        tracks=defaultdict(list)
        for i in group:
            front=rows[i].get('cameras',{}).get('front',{})
            if not front.get('available') or abs(front.get('source_time_s',rows[i]['time_s'])-rows[i]['time_s'])>.08:continue
            for obj in front.get('objects',[]):
                if obj.get('track_status')!='confirmed' or obj.get('association_only') or obj.get('kind')!='car':continue
                if obj.get('road_relation')=='beyond_central_boundary':continue
                x,y,w,h=map(float,obj['bbox'])
                if not 3<x<x+w<camera.width-3 or not 20<=h<=100:continue
                tracks[obj['track_id']].append((i,obj))
        votes=[]
        for history in tracks.values():
            samples=[]
            for i,obj in history:
                x,y,w,h=obj['bbox']
                if not 24<=h<=90 or not 1.3<w/h<3.6:continue
                samples.append((x+w/2,y+h-camera.camera_height/1.65*h))
            a=np.asarray(samples)
            if len(a)<8 or np.ptp(a[:,0])<100:continue
            value=float(np.median(a[:,1]))
            if np.median(abs(a[:,1]-value))<8: votes.append(value)
        if len(votes)<6:continue
        target=float(np.median(votes));mad=float(np.median(abs(np.asarray(votes)-target)))
        if mad>7 or abs(target-camera.horizon)>45:continue
        geometry=replace(camera,horizon=target)
        report=dict(source='stopped_multi_vehicle_height_consensus',horizon_px=target,
                    track_votes=len(votes),median_absolute_deviation_px=mad,
                    surveyed_calibration=False,vehicle_height_assumed_m=1.65)
        for i in group:horizons[i]=target;reports[i]=report
        for key,history in tracks.items():
            points=[]
            for i,obj in history:
                x,y,w,h=obj['bbox'];p=geometry.ground(x+w/2,y+h)
                if p is not None:points.append((i,p))
            for i,_ in points:
                window=[(j,p) for j,p in points if abs(rows[j]['time_s']-rows[i]['time_s'])<=.6]
                if len(window)<7:continue
                times=np.array([rows[j]['time_s']-rows[i]['time_s'] for j,_ in window])
                if np.ptp(times)<.6 or np.max(np.diff(times))>.25:continue
                design=np.column_stack([np.ones(len(times)),times,times*times])
                position=np.array([p for _,p in window]);fit=np.linalg.lstsq(design,position,rcond=None)[0]
                speed=np.linalg.norm(fit[1]);residual=np.linalg.norm(design@fit-position,axis=1)
                if not 3<=speed<=45 or max(residual)>1.:continue
                # A box jump or identity switch must not become a travel angle.
                velocities=[]
                for half in np.array_split(np.arange(len(times)),2):
                    if len(half)<3 or np.ptp(times[half])<.2:break
                    velocities.append(np.linalg.lstsq(np.column_stack([np.ones(len(half)),times[half]]),position[half],rcond=None)[0][1])
                if len(velocities)!=2 or min(np.linalg.norm(v) for v in velocities)<2:continue
                if np.dot(*velocities)/(np.linalg.norm(velocities[0])*np.linalg.norm(velocities[1]))<.9:continue
                directions[i,key]=dict(yaw=math.atan2(fit[1,0],fit[1,1]),speed_mps=float(speed),
                    source='stopped_image_ground_motion',body_orientation_verified=False)
            supported=[i for i,_ in points if (i,key) in directions]
            for i,_ in points:
                if (i,key) in directions or not supported:continue
                nearest=min(supported,key=lambda j:abs(rows[j]['time_s']-rows[i]['time_s']))
                if abs(rows[nearest]['time_s']-rows[i]['time_s'])>.6:continue
                interval=[j for j,_ in points if min(i,nearest)<=j<=max(i,nearest)]
                if any(rows[b]['time_s']-rows[a]['time_s']>.25 for a,b in zip(interval,interval[1:])):continue
                directions[i,key]=dict(directions[nearest,key],source='bounded_neighbour_image_motion',
                                       support_time_s=rows[nearest]['time_s'])
        # The bottom-centre of an image rectangle is not the vehicle centre.
        # During a turn its motion biases the travel angle. Constrain the body
        # angle with silhouette aspect, and solve the sequence jointly so an
        # ambiguous single rectangle cannot flip the model's orientation.
        from core.scene_fusion import corners,dimensions
        angles=np.radians(np.arange(-180,180,5))
        delta=lambda a,b:(a-b+np.pi)%(2*np.pi)-np.pi
        for key,history in tracks.items():
            batches=[]
            for i,obj in history:
                if (i,key) not in directions:continue
                x,y,w,h=obj['bbox'];ground=geometry.ground(x+w/2,y+h)
                if ground is None:continue
                scores=np.full(len(angles),np.inf)
                prior=directions[i,key]['yaw'];ray=ground-geometry.origin[[0,2]];ray/=np.linalg.norm(ray)
                size=dimensions('car')
                for k,yaw in enumerate(angles):
                    change=delta(yaw,prior)
                    if abs(change)>np.pi/4:continue
                    side=np.array([np.cos(yaw),-np.sin(yaw)]);forward=np.array([np.sin(yaw),np.cos(yaw)])
                    support=(abs(ray@side)*size[0]+abs(ray@forward)*size[1])/2
                    p=geometry.visible_bounds(corners(*(ground+ray*support),size,yaw=yaw,kind='car'))
                    if p is None or p[3]<=p[1]:continue
                    aspect=(p[2]-p[0])/(p[3]-p[1])
                    scores[k]=np.log(aspect/(w/h))**2+.02*(change/(np.pi/4))**2
                if not np.isfinite(scores).any():continue
                if not batches or rows[i]['time_s']-rows[batches[-1][-1][0]]['time_s']>.3:batches.append([])
                batches[-1].append((i,scores))
            for batch in batches:
                costs=batch[0][1].copy();parents=[]
                for (previous,_),(i,scores) in zip(batch,batch[1:]):
                    dt=rows[i]['time_s']-rows[previous]['time_s']
                    changes=delta(angles[:,None],angles[None,:])
                    transition=.025*(changes/(dt*np.radians(60)))**2
                    transition[abs(changes)>np.radians(6+60*dt)]=np.inf
                    total=costs[None,:]+transition;parent=np.argmin(total,axis=1)
                    costs=scores+total[np.arange(len(angles)),parent];parents.append(parent)
                if not np.isfinite(costs).any():continue
                k=int(np.argmin(costs))
                for index in range(len(batch)-1,-1,-1):
                    i=batch[index][0];item=directions[i,key]
                    item['motion_yaw_rad']=item['yaw'];item['yaw']=float(angles[k])
                    item['source']='image_motion_and_temporal_silhouette'
                    if index:k=int(parents[index-1][k])
    return horizons,directions,reports
