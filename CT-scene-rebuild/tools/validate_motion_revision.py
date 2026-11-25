"""Compare motion revisions without mistaking overlap checks for accuracy."""
import argparse,json,math
from pathlib import Path
from collections import defaultdict
import numpy as np
from ct_scene.artifacts import verify,canonical
from ct_scene.image_motion_audit import audit as image_audit
from ct_scene.quality_review import review


def load(path):
    verify(path)
    return ([json.loads(x) for x in (path/'render-states.jsonl').read_text().splitlines()],json.loads((path/'actors.json').read_text()),json.loads((path/'report.json').read_text()))


def raw_index(actors):
    return {(a.get('global_id',a['id']),r.get('frame_id'),r['time_s']):{k:r.get(k) for k in ('bbox_xyxy','camera_id','unrefined_position_W','support')}
            for a in actors for r in a['samples']}


def motion(states):
    tracks=defaultdict(list)
    for s in states:
        for a in s['actors']:tracks[a['id']].append((s['time_s'],np.array(a['position_W']),a['yaw_rad']))
    out={}
    for key,rows in tracks.items():
        speed=[];yaw=[]
        for a,b in zip(rows,rows[1:]):
            dt=b[0]-a[0]
            if dt>.11:continue
            speed.append(float(np.linalg.norm(b[1]-a[1])/dt));yaw.append(abs(math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2])))/dt)
        if speed:out[key]=dict(max_speed_mps=max(speed),p95_speed_mps=float(np.percentile(speed,95)),max_yaw_rate_rps=max(yaw))
    return out


p=argparse.ArgumentParser();p.add_argument('--before',type=Path,required=True);p.add_argument('--after',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--allow-projection-change',action='store_true')
a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
before,ba,br=load(a.before);after,aa,ar=load(a.after)
assert len(before)==len(after)
for x,y in zip(before,after):
    assert x['time_s']==y['time_s'] and x['ego_position_W']==y['ego_position_W'] and x['ego_yaw_rad']==y['ego_yaw_rad']
projection_exact=raw_index(ba)==raw_index(aa)
if a.allow_projection_change:
    def roles(path):return {r['role']:r['artifact_id'] for r in verify(path)['inputs']}
    old_roles,new_roles=roles(a.before),roles(a.after)
    for role in ('source','tracks','frozen_ego_lead'):
        assert role in old_roles and old_roles[role]==new_roles.get(role),'Changed source dependency '+role
else:
    assert projection_exact,'Raw source geometry/bbox evidence changed'
changes=defaultdict(lambda:dict(frames=0,max_displacement_m=0.,max_yaw_change_rad=0.));membership=[]
for x,y in zip(before,after):
    xa={r['id']:r for r in x['actors']};ya={r['id']:r for r in y['actors']}
    if set(xa)!=set(ya):membership.append(dict(time_s=x['time_s'],removed=sorted(xa.keys()-ya.keys()),added=sorted(ya.keys()-xa.keys())))
    for key in xa.keys()&ya.keys():
        d=float(np.linalg.norm(np.array(xa[key]['position_W'])-ya[key]['position_W']));dyaw=abs(math.atan2(math.sin(xa[key]['yaw_rad']-ya[key]['yaw_rad']),math.cos(xa[key]['yaw_rad']-ya[key]['yaw_rad'])))
        if d>1e-6 or dyaw>1e-6:
            row=changes[key];row['frames']+=1;row['max_displacement_m']=max(d,row['max_displacement_m']);row['max_yaw_change_rad']=max(dyaw,row['max_yaw_change_rad'])
profile=json.loads(Path('configs/multicam-optical-candidates.json').read_text())['cameras']['front'];mount=json.loads(Path('configs/surrounding-prototype.json').read_text())['cameras']['front']
bi=image_audit(before,ba,profile,br['vehicle_dimensions_m'],mount);ai=image_audit(after,aa,profile,ar['vehicle_dimensions_m'],mount)
quality_dir=a.output.with_suffix('').with_name(a.output.stem+'-decode')
qr=review(a.after,quality_dir)
result=dict(before=str(a.before.resolve()),after=str(a.after.resolve()),ego_and_clock_exact=True,raw_camera_evidence_exact=True,
    raw_projection_estimates_exact=projection_exact,projection_change_authorized=a.allow_projection_change,road_equal=br['road']==ar['road'],changed_actor_states=dict(changes),membership_changes=membership,
    motion_before=motion(before),motion_after=motion(after),image_motion_before=bi,image_motion_after=ai,quality_review=qr,
    metric_accuracy='NOT_EVALUATED',scope='Source-image consistency proxy and estimated footprint/continuity checks; actual source video review required')
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_bytes(canonical(result))
print('raw/ego exact; quality',qr['numerical_gate'],'source review events',bi['range_size_event_count'],'->',ai['range_size_event_count'],'samples',bi['direct_samples'],'->',ai['direct_samples'])
print('Changed actors:',list(changes),'membership-change frames',len(membership))
print('Bearing p95:',[(r['id'],round(r['bearing_error_p95_px'],1)) for r in ai['tracks']])
