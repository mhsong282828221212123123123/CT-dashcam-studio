"""Offline recovery from two-sided world anchors and retained image evidence.

No new object identities, rear-only positions, or unlimited extrapolation.
The resulting position is inferred even when its bounding box was observed.
"""
from collections import defaultdict
from dataclasses import replace
import numpy as np
from core.scene_analysis import iou,vehicle_family
from core.scene_world_tracks import bridge_motion_consistent
from core.scene_geometry import footprints_overlap


def recover_positions(scenes,rows,poses,camera_frames,audits,max_gap=3.):
    # Imported here to avoid the fusion module's constructor import cycle.
    from core.scene_fusion import corners,dimensions
    anchors=defaultdict(list);observations={}
    for i,(scene,row) in enumerate(zip(scenes,rows)):
        for camera,data in row.get('cameras',{}).items():
            if camera=='back' or not data.get('available'):continue
            if abs(data.get('source_time_s',scene.time)-scene.time)>.08:continue
            for obj in data.get('objects',[]):
                if obj.get('track_status')=='confirmed' and not obj.get('association_only'):
                    observations[i,obj['track_id']]=(camera,obj)
        for actor in scene.actors:
            if actor.tracking_state!='observed':continue
            for source in actor.sources:
                if (i,source) in observations:anchors[source].append((i,actor))

    def world(i,a):return poses[i][0]+poses[i][1]@np.array([a.x,a.z])
    def insert(k,a,b,point,source,event,age,shape,minimum_iou,method):
        scene=scenes[k]
        if any(x.key in (a.key,b.key) and x.tracking_state=='reconstructed' for x in scene.actors):return 0
        evidence=observations.get((k,source))
        if evidence is None:return 0 # blind intervals still use bounded tracker ghosts
        camera,obj=evidence;geometry=camera_frames[k].get(camera)
        if geometry is None or vehicle_family(obj['kind'])!=vehicle_family(a.kind):return 0
        if any(source in actor.sources and actor.tracking_state=='observed' for actor in scene.actors):return 0
        if not np.isfinite(point).all() or np.linalg.norm(point)>65:return 0
        bounds=geometry.visible_bounds(corners(*point,dimensions(shape),yaw=a.yaw,kind=shape))
        if bounds is None:return 0
        box=np.r_[bounds[:2],bounds[2:]-bounds[:2]]
        score=iou(box,obj['bbox'])
        if score<minimum_iou:return 0
        x,y,w,h=obj['bbox'];contact=(x+w/2,y+h)
        blocked=any(other.get('track_id')!=source and
            other['bbox'][0]<contact[0]<other['bbox'][0]+other['bbox'][2] and
            other['bbox'][1]<contact[1]<other['bbox'][1]+other['bbox'][3]-10
            for other in rows[k]['cameras'][camera].get('objects',[]) if other.get('track_status')=='confirmed')
        # IoU alone can hide a large ground-contact error on a close side view.
        # Where the contact is visible, require it to agree as well.
        if not blocked and y+h<geometry.height-3 and abs(bounds[3]-(y+h))>max(4.,h*.18):return 0
        previous=next((x for x in scene.actors if x.key in (a.key,b.key)),None)
        previous_score=None
        if previous is not None:
            old_bounds=geometry.visible_bounds(corners(previous.x,previous.z,dimensions(previous.geometry_kind or previous.kind),yaw=previous.yaw,kind=previous.geometry_kind or previous.kind))
            if old_bounds is not None:
                previous_score=iou(np.r_[old_bounds[:2],old_bounds[2:]-old_bounds[:2]],obj['bbox'])
                # Merely revalidating the same interpolation is not an accuracy
                # improvement. Replace a ghost only with better image agreement.
                if score<previous_score+.02:return 0
        # The same image evidence must not explain another localized ID.
        ambiguous=False
        for other in scene.actors:
            if other.key in (a.key,b.key):continue
            if footprints_overlap(a,other,position_a=point):
                ambiguous=True;break
            other_bounds=geometry.visible_bounds(corners(other.x,other.z,dimensions(other.geometry_kind or other.kind),yaw=other.yaw,kind=other.geometry_kind or other.kind))
            if other_bounds is not None and iou(np.r_[other_bounds[:2],other_bounds[2:]-other_bounds[:2]],obj['bbox'])>=score-.15:
                ambiguous=True;break
        if ambiguous:return 0
        # Replace only ghosts sharing an endpoint ID, not observations.
        if any(x.key in (a.key,b.key) and x.tracking_state=='observed' for x in scene.actors):return 0
        prior_state=next((x.tracking_state for x in scene.actors if x.key in (a.key,b.key)),None)
        scene.actors=[x for x in scene.actors if x.key not in (a.key,b.key)]
        actor=replace(a,x=float(point[0]),z=float(point[1]),sources=(source,),
            tracking_state='reconstructed',prediction_age=age,prediction_horizon=max_gap,
            position_uncertainty=max(a.position_uncertainty,b.position_uncertainty)+age*.5)
        scene.actors.append(actor)
        audits[k].setdefault('postprocess',[]).append(dict(event,status='recovered',id=actor.key,
            prior_state=prior_state,previous_reprojection_iou=previous_score,reprojection_iou=score,position=[actor.x,actor.z],method=method))
        return 1

    total=0
    for source,track in sorted(anchors.items()):
        for n in range(1,len(track)):
            left,a=track[n-1];right,b=track[n]
            gap=scenes[right].time-scenes[left].time
            if right-left<2 or not 0<gap<=max_gap:continue
            event={'source':source,'left_time_s':scenes[left].time,'right_time_s':scenes[right].time}
            def reject(reason):audits[right].setdefault('postprocess',[]).append(dict(event,status='rejected',reason=reason))
            if n<2 or n+1>=len(track):reject('insufficient two-sided motion support');continue
            pre,old=track[n-2];post,new=track[n+1]
            dt0=scenes[left].time-scenes[pre].time;dt1=scenes[post].time-scenes[right].time
            if not 0<dt0<=.5 or not 0<dt1<=.5:reject('endpoint support too distant');continue
            if any(s.telemetry is None or s.telemetry.get('speed_kmh') is None or
                   (s.telemetry.get('speed_kmh',0)>0 and s.telemetry.get('heading') is None)
                   for s in scenes[left:right+1]):reject('missing ego motion');continue
            if any(scenes[k].time-scenes[k-1].time>.3 or
                   any('reset' in e.get('reason','') for e in audits[k].get('world_track_events',[]))
                   for k in range(left+1,right+1)):reject('discontinuous ego motion');continue
            p0,p1=world(left,a),world(right,b);velocity=(p1-p0)/gap
            v0=(p0-world(pre,old))/dt0;v1=(world(post,new)-p1)/dt1
            if np.linalg.norm(velocity)>55 or not bridge_motion_consistent(v0,v1,gap) or max(np.linalg.norm(velocity-v0),np.linalg.norm(velocity-v1))>4+3*gap:
                reject('inconsistent endpoint motion');continue
            shape=a.geometry_kind or a.kind
            if vehicle_family(a.kind)!=vehicle_family(b.kind) or shape!=(b.geometry_kind or b.kind):
                reject('inconsistent endpoint shape');continue
            for k in range(left+1,right):
                q=(scenes[k].time-scenes[left].time)/gap
                point=poses[k][1].T@((1-q)*p0+q*p1-poses[k][0])
                age=min(scenes[k].time-scenes[left].time,scenes[right].time-scenes[k].time)
                total+=insert(k,a,b,point,source,event,age,shape,.5,'two_sided_world_motion_and_image_validation')
        # Offline backward pass: several later accepted positions can explain
        # earlier rejected observations, without pretending they were measured.
        if len(track)<4:continue
        support=track[:5];first=track[0][0]
        times=np.array([scenes[i].time for i,_ in support]);times-=times[0]
        if times[-1]>.7 or np.any(np.diff(times)>.3):continue
        points=np.array([world(i,a) for i,a in support])
        fit=np.linalg.lstsq(np.column_stack([np.ones(len(times)),times]),points,rcond=None)[0]
        rms=float(np.sqrt(np.mean((np.column_stack([np.ones(len(times)),times])@fit-points)**2)))
        if rms>.25 or np.linalg.norm(fit[1])>55:continue
        a=support[0][1];shape=a.geometry_kind or a.kind
        if any((x.geometry_kind or x.kind)!=shape for _,x in support):continue
        for k in range(first-1,-1,-1):
            age=scenes[first].time-scenes[k].time
            if age>1.5 or scenes[k+1].time-scenes[k].time>.3:break
            if any('reset' in e.get('reason','') for e in audits[k+1].get('world_track_events',[])):break
            data=scenes[k].telemetry
            if data is None or data.get('speed_kmh') is None or (data.get('speed_kmh',0)>0 and data.get('heading') is None):break
            point=poses[k][1].T@(fit[0]-age*fit[1]-poses[k][0])
            event={'source':source,'left_time_s':None,'right_time_s':scenes[first].time,
                   'future_support_count':len(support),'world_fit_rms_m':rms}
            total+=insert(k,a,a,point,source,event,age,shape,.6,'future_supported_backfill')
    return total
