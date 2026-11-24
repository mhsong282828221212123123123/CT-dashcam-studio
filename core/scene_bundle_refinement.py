"""Opt-in whole-track reconstruction for reviewed clip results."""
from collections import defaultdict,Counter
from dataclasses import replace
import math
import numpy as np
from scipy.interpolate import BSpline


def occluded(obj,objects):
    if obj.get('silhouette_partial'):return True
    x,y,w,h=obj['bbox'];contact=(x+w/2,y+h)
    if obj.get('kind')=='car' and not obj.get('geometry_kind') and w/max(h,1)<.85:return True
    for other in objects:
        if other is obj:continue
        a,b,c,d=other['bbox']
        if a<contact[0]<a+c and b<contact[1]<b+d-8:return True
        overlap=max(0,min(x+w,a+c)-max(x,a))*max(0,min(y+h,b+d)-max(y,b))
        if b+d>=y+h-3 and d>=h*.7 and overlap/max(1,w*h)>.1:return True
    return False


def refine_bundles(scenes,rows,poses,cameras,audits,progress=None,source_filter=None,bearing_only=False):
    from core.scene_motion_bundle import fit_motion_bundle
    from core.scene_fusion import dimensions
    from core.driving_scene import Actor
    from core.scene_telemetry import odometry_speed_kmh
    observations=defaultdict(list);aliases=defaultdict(set);templates={}
    for i,(row,scene) in enumerate(zip(rows,scenes)):
        actors={source:a for a in scene.actors for source in a.sources}
        for actor in scene.actors:
            for source in actor.sources:aliases[source].add(actor.key)
        for name,data in row['cameras'].items():
            if name=='back' or name not in cameras[i] or not data.get('available'):continue
            if abs(data.get('source_time_s',scene.time)-scene.time)>.08:continue
            for obj in data.get('objects',[]):
                if obj['kind'] not in ('car','truck','bus','motorcycle','bicycle') or obj.get('association_only'):continue
                if obj.get('road_relation')=='beyond_central_boundary':continue
                x,y,w,h=obj['bbox']
                if w<12 or h<16:continue
                source=obj['track_id'];actor=actors.get(source)
                if actor is not None:templates[source]=actor
                blocked=occluded(obj,data['objects'])
                input_yaw=actor.yaw if actor else next((a.get('fit_yaw_rad') for a in audits[i].get('associations',[])
                    if source in a.get('position_sources',())),None)
                observations[source].append(dict(index=i,time=scene.time,camera=cameras[i][name],
                    camera_name=name,bbox=obj['bbox'],ego_origin=poses[i][0],ego_rotation=poses[i][1],
                    blocked=blocked,obj=obj,initial_yaw=input_yaw,
                    measurement_reliable=bool(w>=30 and h>=(24 if name=='front' else 50)),
                    reviewed_world_yaw=obj.get('reviewed_world_yaw'),
                    host_speed_mps=(odometry_speed_kmh(scene.telemetry) or 0.)/3.6,
                    host_stopped=bool(scene.telemetry and 0<=scene.telemetry.get('speed_kmh',999)<=.5)))
    accepted={};report=[]
    for source,history in observations.items():
        if source_filter is not None and source not in source_filter:continue
        if sum(o['obj'].get('track_status')=='confirmed' for o in history)<8:continue
        for item in history:
            near=[o for o in history if abs(o['time']-item['time'])<=.5 and o['host_stopped']]
            if len(near)<8 or not item['host_stopped']:continue
            boxes=np.array([o['bbox'] for o in near]);centres=boxes[:,:2]+boxes[:,2:]/2
            item['stationary']=bool(np.linalg.norm(np.ptp(centres,axis=0))<3 and
                max(np.ptp(boxes[:,2:],axis=0)/np.maximum(1,np.median(boxes[:,2:],axis=0)))<.04)
        shape=Counter(o['obj'].get('geometry_kind') or o['obj'].get('render_kind') or o['obj']['kind'] for o in history).most_common(1)[0][0]
        if shape in ('motorcycle','bicycle'):shape+='_body'
        speed_limit=max(22.,min(55.,max(o.get('host_speed_mps',0.) for o in history)+10.))
        model=fit_motion_bundle(history,kind=shape,ground_contact=not bearing_only,max_speed=speed_limit)
        attempted=[shape]
        if not bearing_only and (not model or (not model['accepted'] and not model.get('occlusion_hypothesis'))):
            alternatives=['compact_car','suv'] if shape=='car' else ['motorcycle'] if shape=='motorcycle_body' else []
            for candidate in alternatives:
                trial=fit_motion_bundle(history,kind=candidate,max_speed=speed_limit);attempted.append(candidate)
                if trial and trial['accepted']:
                    model=trial;shape=candidate;break
        if not bearing_only and (not model or not (model['accepted'] or model.get('occlusion_hypothesis'))):
            trial=fit_motion_bundle(history,kind=shape,ground_contact=False,max_speed=speed_limit)
            if trial and trial.get('projection_hypothesis'):model=trial
        summary={k:v for k,v in (model or {}).items() if k not in ('positions','knots','coefficients')}
        summary['source']=source;summary['observations']=len(history)
        summary['attempted_shapes']=attempted
        report.append(summary)
        if model and (model['accepted'] or model.get('occlusion_hypothesis') or model.get('projection_hypothesis')):accepted[source]=model
        if progress:progress(summary)
    parent={source:source for source in accepted}
    def root(source):
        while parent[source]!=source:
            parent[source]=parent[parent[source]];source=parent[source]
        return source
    for scene in scenes:
        for actor in scene.actors:
            related=[s for s in actor.sources if s in parent]
            for source in related[1:]:parent[root(source)]=root(related[0])
    canonical={source:'bundle:'+root(source) for source in accepted}
    owners=defaultdict(set)
    for source in accepted:
        for key in aliases[source]:owners[key].add(source)
    new_actors=defaultdict(list)
    for source,model in accepted.items():
        history=observations[source];template=templates.get(source);shape=model['kind']
        spline=BSpline(np.asarray(model['knots']),np.asarray(model['coefficients']),3)
        first,last=history[0]['time'],history[-1]['time'];by_index={o['index']:o for o in history}
        geometry=model['dimensions']
        for i,scene in enumerate(scenes):
            if not first<=scene.time<=last+.5:continue
            previous=max((o for o in history if o['time']<=scene.time),key=lambda o:o['time'],default=None)
            following=min((o for o in history if o['time']>=scene.time),key=lambda o:o['time'],default=None)
            if previous is None:continue
            if following is not None and following['time']-previous['time']>1.:continue
            elapsed=min(scene.time,last)-model['reference_time_s'];state=spline(elapsed)
            if scene.time>last:
                state=state+spline.derivative()(elapsed)*(scene.time-last)
            point=poses[i][1].T@(state[:2]-poses[i][0]);axis=poses[i][1].T@np.array([math.sin(state[2]),math.cos(state[2])])
            if np.linalg.norm(point)>100:continue
            item=by_index.get(i);obj=(item or previous)['obj']
            if template is None:template=Actor(canonical[source],obj.get('render_kind') or obj['kind'],*point)
            actor=replace(template,key=canonical[source],kind=obj.get('render_kind') or template.kind,
                x=float(point[0]),z=float(point[1]),yaw=math.atan2(axis[0],axis[1]),
                sources=(source,) if item else (),tracking_state='reconstructed' if item else 'predicted',
                prediction_age=0. if item else scene.time-previous['time'],image_observed=bool(item))
            actor.confidence=float(obj.get('confidence',template.confidence))
            if shape=='suv':actor.kind='suv'
            actor.position_hypothesis=bool(model.get('occlusion_hypothesis') or model.get('projection_hypothesis'))
            actor.width,actor.length,actor.height=geometry;actor.geometry_kind=shape
            if shape in ('motorcycle_body','bicycle_body'):actor.height=dimensions(shape.removesuffix('_body'))[2]
            new_actors[i].append(actor)
            audits[i].setdefault('motion_bundle',[]).append(dict(id=actor.key,source=source,
                position=[actor.x,actor.z],yaw_rad=actor.yaw,method='bearing_size_hypothesis_bspline' if model.get('projection_hypothesis') else 'whole_track_image_validated_bspline',
                state=actor.tracking_state,metric_accuracy='not surveyed'))
    for i,scene in enumerate(scenes):
        kept=[]
        for actor in scene.actors:
            related=owners.get(actor.key,set())
            if related:continue # Never splice an unvalidated per-frame pose into a fitted path.
            kept.append(actor)
        kept.extend(new_actors[i]);unique={}
        for actor in kept:
            existing=unique.get(actor.key)
            if existing is None:unique[actor.key]=actor
            elif actor.sources and not existing.sources:unique[actor.key]=actor
            elif actor.sources:existing.sources=tuple(sorted(set(existing.sources)|set(actor.sources)))
        scene.actors=list(unique.values())
    return report


def reject_unestablished_outliers(scenes,rows,poses,audits):
    """Abstain on brief, unfitted 3D tracks with impossible position jumps.

    Keep the original pixel detections. Established tracks must be repaired,
    never suppressed to pass this check.
    """
    confirmed=Counter();history=defaultdict(list)
    for row in rows:
        for data in row.get('cameras',{}).values():
            for obj in data.get('objects',[]):
                if obj.get('track_status')=='confirmed':confirmed[obj['track_id']]+=1
    for i,scene in enumerate(scenes):
        for actor in scene.actors:
            if not actor.key.startswith('bundle:'):
                history[actor.key].append((i,actor))
    rejected={}
    for key,items in history.items():
        sources={s for _,actor in items for s in actor.sources}
        if not sources or max(confirmed[s] for s in sources)>=8:continue
        for (i,a),(j,b) in zip(items,items[1:]):
            dt=scenes[j].time-scenes[i].time
            if not 0<dt<=.2:continue
            p=poses[i][0]+poses[i][1]@np.array([a.x,a.z])
            q=poses[j][0]+poses[j][1]@np.array([b.x,b.z])
            speed=float(np.linalg.norm(q-p)/dt)
            if speed>25:rejected[key]=dict(id=key,sources=sorted(sources),max_step_speed_mps=speed,
                reason='brief_unfitted_track_impossible_3d_step; pixel observations retained')
    for scene,audit in zip(scenes,audits):
        removed=[rejected[a.key] for a in scene.actors if a.key in rejected]
        if removed:audit['abstained_3d_tracks']=removed
        scene.actors=[a for a in scene.actors if a.key not in rejected]
    return list(rejected.values())
