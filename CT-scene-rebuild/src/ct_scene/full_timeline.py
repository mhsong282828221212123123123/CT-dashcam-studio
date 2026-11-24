"""Full source clock/ego trajectory; an actor cannot shorten the export."""
import json
import math
from pathlib import Path
import av
import cv2
import numpy as np
from .artifacts import verify,publish,canonical,digest
from .ingest import file_hash
from .wheel_scale import SpeedTrace
from .affine_probe import camera_matrix
from .prototype_front import model_vertices,rotation
from .offline_paths import smooth_positions
from .actor_heading import resolve_axis_flips
from .steering_cue import SteeringCue


def own_motion(times,sei,camera_pairs=None):
    t=np.asarray(times,float);trace=SpeedTrace(sei);speed=np.interp(t,trace.times,trace.speeds)
    rows=[r for r in trace.rows if r['fields']['heading_deg']['valid']]
    if not rows:raise ValueError('No ego heading telemetry')
    heading=np.interp(t,[r['camera_time_s'] for r in rows],np.unwrap(np.radians([r['fields']['heading_deg']['value'] for r in rows])))
    moving=np.flatnonzero(abs(speed)>=.5);last=heading[moving[0]] if len(moving) else heading[0]
    for i in range(len(t)):
        if abs(speed[i])>=.5:last=heading[i]
        else:heading[i]=last
    yaw=-(heading-heading[0]);ego=np.zeros((len(t),3))
    if camera_pairs is not None:
        from .ego_turn_rotation import corrected_yaw
        yaw,_=corrected_yaw(t,yaw,camera_pairs,SteeringCue(sei))
    for i in range(1,len(t)):
        distance=trace.distance(t[i-1],t[i]);angle=(yaw[i]+yaw[i-1])/2;ego[i]=ego[i-1]+distance*np.array([np.cos(angle),np.sin(angle),0])
    return ego,yaw


def inferred_anchor_asset(rows,assets_root):
    classes=[r['class_id'] for r in rows if r['support']=='detector_observation']
    dominant=max(set(classes),key=lambda c:classes.count(c)) if classes else 2
    boxes=np.array([r['bbox_xyxy'] for r in rows]) if rows else np.array([[0,0,2,1]])
    aspect=float(np.median((boxes[:,3]-boxes[:,1])/np.maximum(1,boxes[:,2]-boxes[:,0])))
    kind='bus' if dominant==5 else 'box_truck' if dominant==7 else 'suv' if aspect>.8 else 'sedan'
    return Path(assets_root)/('tesla_fsd_'+kind+'.glb')


def run(package,tracks,telemetry,lens_config,source,actor_asset,destination,assets_root=None,ego_rotation=None):
    package,tracks,telemetry,source=map(Path,[package,tracks,telemetry,source]);pm,tm,sm=[verify(p) for p in (package,tracks,telemetry)]
    if not any(i['artifact_id']==pm['artifact_id'] for i in tm['inputs']):raise ValueError('Source/track mismatch')
    frames=sorted([f for f in json.loads((package/'frames.json').read_text()) if f['camera_id']=='front'],key=lambda f:f['camera_time_s'])
    sei=[json.loads(s) for s in (telemetry/'telemetry.jsonl').read_text().splitlines()]
    if {f['source_sha256'] for f in frames}!={file_hash(source)} or {r['source_sha256'] for r in sei}!={file_hash(source)}:raise ValueError('Source/telemetry mismatch')
    with av.open(str(source)) as c:
        first=last=None;count=0
        for f in c.decode(video=0):
            now=float(f.pts*f.time_base);first=now if first is None else first;last=now;count+=1
        duration=c.duration/av.time_base
    times=np.array([f['camera_time_s'] for f in frames]);extra=last>times[-1]+1e-6
    if extra:times=np.append(times,last)
    rotation_manifest=None;rotation_pairs=None;rotation_review=[]
    if ego_rotation:
        rotation_manifest=verify(Path(ego_rotation));expected={pm['artifact_id'],tm['artifact_id'],sm['artifact_id']}
        if rotation_manifest['stage']!='own_background_rotation' or {r['artifact_id'] for r in rotation_manifest['inputs']}!=expected:raise ValueError('Own rotation/source/track/telemetry mismatch')
        profiles=json.loads(Path(lens_config).read_text())['cameras']
        if json.loads((Path(ego_rotation)/'report.json').read_text())['optics_sha256']!=digest(canonical(profiles)):raise ValueError('Own rotation optics mismatch')
        rotation_pairs=json.loads((Path(ego_rotation)/'camera-pairs.json').read_text())
        from .ego_turn_rotation import corrected_yaw
        _,original_yaw=own_motion(times,sei);_,rotation_review=corrected_yaw(times,original_yaw,rotation_pairs,SteeringCue(sei))
    ego,yaw=own_motion(times,sei,rotation_pairs);groups={}
    for r in map(json.loads,(tracks/'observations.jsonl').read_text().splitlines()):
        if r['camera_id']=='front' and r['class_id'] in (2,5,7):groups.setdefault(r['local_track_id'],[]).append(r)
    candidates=[]
    for key,rows in groups.items():
        b=np.array([r['bbox_xyxy'] for r in rows])
        if len(rows)<len(frames)*.3 or abs(np.median((b[:,0]+b[:,2])/2)-frames[0]['width']/2)>frames[0]['width']*.15:continue
        candidates.append((len(rows)*np.median((b[:,2]-b[:,0])*(b[:,3]-b[:,1])),key,rows))
    selected=max(candidates,key=lambda x:x[0]) if candidates else None
    if str(actor_asset)=='auto':
        if assets_root is None:raise ValueError('Auto actor model requires assets root')
        actor_asset=inferred_anchor_asset(selected[2] if selected else [],assets_root)
    key=selected[1] if selected else None;by_frame={r['frame_id']:r for r in selected[2]} if selected else {}
    K,D=camera_matrix(json.loads(Path(lens_config).read_text())['cameras']['front']);length=float(np.ptp(model_vertices(actor_asset)[:,0]));observed=[]
    for i,f in enumerate(frames):
        if f['frame_id'] not in by_frame:continue
        r=by_frame[f['frame_id']];b=r['bbox_xyxy'];ray=cv2.undistortPoints(np.array([[[(b[0]+b[2])/2,b[3]]]],float),K,D).reshape(2)
        if ray[1]<=.02:continue
        depth=1.3/ray[1]
        if not 3<depth<80:continue
        observed.append((i,ego[i]+rotation(yaw[i])@np.array([depth+length/2,-ray[0]*depth,0]),r))
    fit=smooth_positions([times[i] for i,_,_ in observed],[p for _,p,_ in observed],strength=.2) if len(observed)>=3 else np.array([p for _,p,_ in observed])
    velocity=np.gradient(fit,[times[i] for i,_,_ in observed],axis=0) if len(observed)>1 else np.zeros_like(fit);support={};angle=0
    for j,((i,p,r),v) in enumerate(zip(observed,velocity)):
        if np.linalg.norm(v[:2])>1.5:angle=math.atan2(v[1],v[0])
        support[i]=dict(fitted_actor_position_W=fit[j].tolist(),raw_actor_position_W=p.tolist(),actor_yaw_rad=angle,source_support=r['support'])
    # Establish body-axis continuity before publishing the immutable lead anchor.
    # Composition must never reinterpret an already frozen source heading.
    indices=sorted(support)
    original_headings=np.array([support[i]['actor_yaw_rad'] for i in indices])
    visible_times=sorted({r['camera_time_s'] for r in by_frame.values() if r['support'] in ('detector_observation','flow_observation')})
    continuous=resolve_axis_flips(times[indices],original_headings,visible_times=visible_times)
    for i,angle in zip(indices,continuous):support[i]['actor_yaw_rad']=float(angle)
    phase_review={'accepted':False,'reason':'no_supported_car_anchor'}
    if key and Path(actor_asset).stem.removeprefix('tesla_fsd_') in ('sedan','suv','van'):
        from .surrounding_lamps import extract
        from .actor_rear_axis import orient_unpublished
        cues=extract(package,frames,list(by_frame.values()),{key:key}).get(key,[])
        draft=[dict(time_s=float(times[i]),position_W=support[i]['fitted_actor_position_W'],yaw_rad=support[i]['actor_yaw_rad']) for i in indices]
        resolved,phase_review=orient_unpublished(draft,cues,times,ego)
        for i,row in zip(indices,resolved):support[i]['actor_yaw_rad']=row['yaw_rad']
    axis_changes=int(np.count_nonzero(abs(np.arctan2(np.sin(continuous-original_headings),np.cos(continuous-original_headings)))>1e-7))
    trajectory=[dict(frame_id=frames[i]['frame_id'] if i<len(frames) else None,camera_time_s=float(t),ego_position_W=ego[i].tolist(),ego_yaw_rad=float(yaw[i]),
        **support.get(i,dict(fitted_actor_position_W=None,raw_actor_position_W=None,actor_yaw_rad=None,source_support='unobserved'))) for i,t in enumerate(times)]
    report={'selected_local_track':key,'actor_model_kind':Path(actor_asset).stem.removeprefix('tesla_fsd_'),'source_duration_s':float(duration),'source_first_pts_s':first,'source_last_pts_s':last,'source_decoded_frames':count,
        'duplicate_motion_timestamps_collapsed':sum(r.get('duplicate_motion_messages',1)-1 for r in SpeedTrace(sei).rows),
        'body_axis_continuity_changed_samples':axis_changes,
        'initial_body_phase_review':phase_review,
        'ego_source':'own SEI signed speed/heading with bounded two-camera static-background turn timing' if any(r['accepted'] for r in rotation_review) else 'own SEI signed speed/heading only','timeline_scope':'complete source; actor visibility cannot truncate export','observed_anchor_samples':len(observed),'metric_accuracy':'NOT_EVALUATED', 'own_rotation_review':rotation_review,'steering_direction_reference':SteeringCue(sei).report}
    revision=digest(b''.join(Path(__file__).with_name(n).read_bytes() for n in ['full_timeline.py','steering_cue.py','ego_turn_rotation.py','actor_heading.py','actor_rear_axis.py','surrounding_lamps.py','rear_view_cue.py']))
    return publish(Path(destination),stage='full_source_ego_anchor',payloads={'trajectory.json':canonical(trajectory),'report.json':canonical(report)},
        inputs=[{'role':role,'artifact_id':m['artifact_id']} for role,m in zip(['source','tracks','telemetry'],[pm,tm,sm])]+([dict(role='own_background_rotation',artifact_id=rotation_manifest['artifact_id'])] if rotation_manifest else []),
        config={'height_prior_m':1.3,'pitch_prior_rad':0,'source_duration_s':float(duration),'actor_asset':str(actor_asset)},model_hashes=[file_hash(Path(actor_asset))],code_revision=revision,runtime={'av':av.__version__,'cuda':False})
