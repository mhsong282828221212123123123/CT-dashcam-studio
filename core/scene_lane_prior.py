"""Soft lane containment for established same-direction vehicle tracks.

Lane membership is a prior, not ground truth. It never moves the host, changes
lane count or snaps every car to a centreline. Turns, sustained lateral motion
and ambiguous boundaries release the prior. Raw positions remain in the audit.
"""
from collections import defaultdict
import math
import numpy as np

from core.scene_vehicle_road import VEHICLE_KINDS, lane_evidence_actor


def apply_lane_containment_prior(scenes,audits,rows=None,cameras=None):
    if len(scenes)!=len(audits):raise ValueError('Lane prior timeline mismatch')
    if (rows is None)!=(cameras is None):raise ValueError('Image rows and cameras must be supplied together')
    if rows is not None and (len(rows)!=len(scenes) or len(cameras)!=len(scenes)):
        raise ValueError('Lane prior image timeline mismatch')
    tracks=defaultdict(list)
    for i,(scene,audit) in enumerate(zip(scenes,audits)):
        audit['lane_containment_prior']=[]
        road=audit.get('travel_corridors',{})
        view=getattr(scene,'road_scope_view',getattr(scene,'road_view',None))
        if not view or not road.get('occupied_lane_indices'):continue
        if getattr(scene,'ego_turn_path_estimated',False):continue
        if road.get('phase_source') not in ('camera_lane_phase','two_sided_road_odometry','host_odometry_between_road_observations'):
            continue # Vehicle-inferred road alone cannot independently correct the same vehicle's position.
        if audit.get('road_junction',{}).get('state')=='inside' or getattr(scene,'road_junction_state',None)=='inside':continue
        if audit.get('camera_junction',{}).get('active'):continue
        Q=np.asarray(view['rotation'],float);offset=np.asarray(view['offset'],float)
        width=road.get('lane_width_assumed_m');lanes=road['occupied_lane_indices']
        if (Q.shape!=(2,2) or offset.shape!=(2,) or not np.isfinite(Q).all() or not np.isfinite(offset).all()
                or not np.allclose(Q.T@Q,np.eye(2),atol=1e-5) or np.linalg.det(Q)<.99):continue
        if width is None or not math.isfinite(width) or not 2.5<=width<=4.5:continue
        for actor in scene.actors:
            if actor.kind not in VEHICLE_KINDS or actor.position_hypothesis:continue
            direction=Q@np.array([math.sin(actor.yaw),math.cos(actor.yaw)])
            angle=math.atan2(direction[0],direction[1])
            point=Q@np.array([actor.x,actor.z])+offset
            if not np.isfinite(point).all() or not -30<point[1]<45:continue
            lane=min(lanes,key=lambda k:abs(point[0]-k*width))
            observed=lane_evidence_actor(actor)
            tracks[(road.get('segment_start_s',0.),actor.key)].append(dict(
                index=i,actor=actor,time=scene.time,point=point,Q=Q,offset=offset,width=width,
                lane=lane,angle=angle,observed=observed,
                eligible=bool(abs(angle)<math.radians(20) and abs(point[0]-lane*width)<width*.45)))
    for history in tracks.values():
        # A smooth sustained lateral displacement is a maneuver, not an error
        # to pull back into its old lane. A one-frame camera jump is not enough.
        maneuver=set()
        for item in history:
            near=[p for p in history if p['observed'] and abs(p['time']-item['time'])<=.7]
            if len(near)<6 or near[-1]['time']-near[0]['time']<.7:continue
            times=np.array([p['time']-item['time'] for p in near]);xs=np.array([p['point'][0] for p in near])
            A=np.column_stack([np.ones(len(times)),times]);fit=np.linalg.lstsq(A,xs,rcond=None)[0]
            if abs(fit[1])>.45 and np.ptp(xs)>.65 and np.max(abs(A@fit-xs))<.18:
                maneuver.add(item['index'])
        runs=[];run=[]
        for item in history:
            valid=item['eligible'] and item['index'] not in maneuver
            if not valid or (run and (item['lane']!=run[-1]['lane'] or item['time']-run[-1]['time']>.3)):
                if run:runs.append(run)
                run=[]
            if valid:run.append(item)
        if run:runs.append(run)
        for run in runs:
            supported=[p for p in run if p['observed']]
            if len(supported)<8 or supported[-1]['time']-supported[0]['time']<.8:continue
            for item in run:
                i=item['index'];actor=item['actor'];width=item['width'];point=item['point']
                if not item['observed'] and actor.prediction_age>1.:continue
                # Fit the oriented footprint inside the lane, allowing ordinary
                # off-centre driving. Large conflicts are calibration failures.
                half=(actor.width*abs(math.cos(item['angle']))+actor.length*abs(math.sin(item['angle'])))/2
                clearance=width/2-half-.08
                if clearance<=.15:continue
                centre=item['lane']*width
                target=float(np.clip(point[0],centre-clearance,centre+clearance))
                violation=target-point[0]
                if abs(violation)<.02 or abs(violation)>.9:continue
                shift=float(np.clip(.7*violation,-.35,.35))
                raw=np.array([actor.x,actor.z]);chosen=None;limited=False
                for _ in range(6):
                    proposal=item['Q'].T@(point+np.array([shift,0.])-item['offset'])
                    if rows is None or _image_consistent(actor,raw,proposal,rows[i],cameras[i]):
                        chosen=proposal;break
                    shift*=.5;limited=True
                if chosen is None or abs(shift)<.01:continue
                actor.x,actor.z=map(float,chosen)
                audits[i]['lane_containment_prior'].append(dict(id=actor.key,lane_index=item['lane'],
                    source='established_track_soft_lane_containment',raw_position=raw.tolist(),
                    position=chosen.tolist(),road_lateral_correction_m=shift,
                    image_limited=limited,centre_snapped=False,metric_accuracy_verified=False))
                for record in audits[i].get('world_tracks',[]):
                    if record['id']==actor.key:record['position']=chosen.tolist()
    return _smooth_corrections(scenes,audits,rows,cameras)


def _smooth_corrections(scenes,audits,rows,cameras,rate=.35):
    """Offline magnitude envelope: release in advance, never spill into a turn.

    Only reduce previously admitted corrections. Zero-support frames stay zero,
    including lane changes/junctions. A disappearing road reference therefore
    cannot introduce a one-frame lateral correction jump.
    """
    tracks=defaultdict(list)
    for i,(scene,audit) in enumerate(zip(scenes,audits)):
        records={r['id']:r for r in audit['lane_containment_prior']}
        segment=audit.get('travel_corridors',{}).get('segment_start_s',0.)
        for actor in scene.actors:
            record=records.get(actor.key)
            raw=np.array(record['raw_position'] if record else [actor.x,actor.z])
            shift=record['road_lateral_correction_m'] if record else 0.
            tracks[(segment,actor.key)].append(dict(index=i,time=scene.time,actor=actor,record=record,
                raw=raw,delta=np.array([actor.x,actor.z])-raw,shift=shift))
    count=0
    for history in tracks.values():
        values=np.array([abs(e['shift']) for e in history])
        for attempt in range(3):
            for step in (1,-1):
                indices=range(1,len(history)) if step==1 else range(len(history)-2,-1,-1)
                for j in indices:
                    k=j-step
                    dt=abs(history[j]['time']-history[k]['time'])
                    if dt>.3:continue
                    limit=values[k]+rate*dt
                    if history[j]['shift']*history[k]['shift']<0:limit=rate*dt/2
                    values[j]=min(values[j],limit)
            failed=False
            if rows is not None:
                for j,e in enumerate(history):
                    if values[j]<=1e-8 or not e['record']:continue
                    proposal=e['raw']+e['delta']*values[j]/abs(e['shift'])
                    if not _image_consistent(e['actor'],e['raw'],proposal,rows[e['index']],cameras[e['index']]):
                        values[j]=0.;failed=True
            if not failed:break
        else:values[:]=0. # An unstable projection gate cannot dictate a jump.
        for value,e in zip(values,history):
            if not e['record']:continue
            record=e['record'];actor=e['actor'];i=e['index']
            if value<=1e-8:
                proposal=e['raw'];audits[i]['lane_containment_prior'].remove(record)
            else:
                proposal=e['raw']+e['delta']*value/abs(e['shift']);count+=1
                record.update(position=proposal.tolist(),road_lateral_correction_m=math.copysign(value,e['shift']),
                              transition_rate_limit_mps=rate)
            actor.x,actor.z=map(float,proposal)
            for r in audits[i].get('world_tracks',[]):
                if r['id']==actor.key:r['position']=proposal.tolist()
    return count


def _image_consistent(actor,before,after,row,cameras):
    from core.scene_fusion import corners,dimensions
    from core.scene_analysis import iou
    shape=actor.geometry_kind or actor.kind
    size=(actor.width,actor.length,actor.height or dimensions(shape)[2])
    checked=False
    for name,data in row.get('cameras',{}).items():
        camera=cameras.get(name)
        if camera is None or name=='back' or not data.get('available'):continue
        for obj in data.get('objects',[]):
            if obj['track_id'] not in actor.sources:continue
            if obj.get('association_only') or obj.get('silhouette_partial'):continue
            box=obj['bbox'];a=camera.visible_bounds(corners(*before,size,yaw=actor.yaw,kind=shape))
            b=camera.visible_bounds(corners(*after,size,yaw=actor.yaw,kind=shape))
            if a is None or b is None:return False
            old=iou(np.r_[a[:2],a[2:]-a[:2]],box);new=iou(np.r_[b[:2],b[2:]-b[:2]],box)
            if new<max(.45,old-.1):return False
            checked=True
    return checked
