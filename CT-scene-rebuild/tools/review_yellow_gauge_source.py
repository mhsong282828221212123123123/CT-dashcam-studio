"""Development source witnesses: native front frame 59s shows right-lane Soul/truck."""
import argparse,json
from pathlib import Path
import numpy as np
from ct_scene.artifacts import verify,canonical
from ct_scene.road_frame import coordinates
from ct_scene.road_geometry import boundaries_at,lane_at

p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,required=True);p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=False)
verify(a.scene);bm=verify(a.base);road=json.loads((a.scene/'report.json').read_text())['road'];base=json.loads((a.base/'trajectory.json').read_text());states=[json.loads(s) for s in (a.scene/'render-states.jsonl').read_text().splitlines()];actors=json.loads((a.scene/'actors.json').read_text());mapping={k:r['global_id'] for r in actors for k in r['local_ids']}
t=np.array([r['camera_time_s'] for r in base]);positions=np.array([r['ego_position_W'] for r in base]);yaws=np.unwrap([r['ego_yaw_rad'] for r in base]);checks={};details={}
for when in (50.,59.):
    ep=np.array([np.interp(when,t,positions[:,k]) for k in range(3)]);q=coordinates(road,ep);checks[f'raw_{when:g}s_two_lanes']=len(boundaries_at(road,float(q[0])))==3 and lane_at(road,float(q[0]),float(q[1])) is not None
state=min(states,key=lambda s:abs(s['time_s']-59.));checks['source_time_matches']=abs(state['time_s']-59.)<.06
ep=np.array([np.interp(state['time_s'],t,positions[:,k]) for k in range(3)]);yaw=float(np.interp(state['time_s'],t,yaws));forward=np.array([np.cos(yaw),np.sin(yaw),0]);eq=coordinates(road,ep);elane=lane_at(road,float(eq[0]),float(eq[1]))
for key in ('front-0075','front-0080'):
    r=next((r for r in state['actors'] if r['id']==mapping.get(key)),None);ahead=float(np.dot(np.array(r['position_W'])-ep,forward)) if r else None
    checks[key+'_raw_visible_solid_ahead']=bool(r and not r['ghost'] and ahead>3)
    if r:
        q=coordinates(road,r['position_W']);alane=lane_at(road,float(q[0]),float(q[1]));checks[key+'_raw_one_lane_right']=elane is not None and alane is not None and alane-elane==1
        details[key]=dict(ahead_m=ahead,actor_lane=alane,ego_lane=elane)
result=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,details=details,base_artifact=bm['artifact_id'],scope='Post-failure development witnesses from native raw59 image. Not independent holdout, dense visibility recall, or metric ground truth.');(a.output/'review.json').write_bytes(canonical(result));print(json.dumps(result));raise SystemExit(0 if all(checks.values()) else 2)
