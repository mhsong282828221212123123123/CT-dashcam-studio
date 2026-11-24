"""Source-first review of a selected long pillar gap; no production ID rules."""
import argparse,json,math
from pathlib import Path
import numpy as np
from ct_scene.artifacts import verify,digest
from ct_scene.ingest import file_hash
from ct_scene.road_frame import coordinates
from ct_scene.road_geometry import lane_at
from review_guards import require_window

p=argparse.ArgumentParser()
for key in ['scene','baseline','control','output']:p.add_argument('--'+key,type=Path,required=True)
a=p.parse_args();manifest=verify(a.scene);control=json.loads(a.control.read_text(encoding='utf-8'))
if not any(x['role']=='source' and x['artifact_id']==control['source_artifact_id'] for x in manifest['inputs']):raise ValueError('Source mismatch')
for src in control['source_files']:
 if file_hash(Path(src['path']))!=src['sha256']:raise ValueError('Raw source changed')
actors=json.loads((a.scene/'actors.json').read_text());members={k:x['global_id'] for x in actors for k in x['local_ids']}
past=members[control['past_local_id']];future=members[control['future_local_id']]
road=json.loads((a.scene/'report.json').read_text())['road'];states=[json.loads(x) for x in (a.scene/'render-states.jsonl').read_text().splitlines()];before=[json.loads(x) for x in (a.baseline/'render-states.jsonl').read_text().splitlines()]
require_window(states,*control['ghost_interval_s']);require_window(states,*control['continuity_interval_s'])
rows=json.loads((a.scene/'blindspot-inference-review.json').read_text())['predictions'];proof=[r for r in rows if r['global_id']==past and r.get('future_global_id')==future]
ghost=[];lanes=[]
for state in states:
 if not control['ghost_interval_s'][0]<=state['time_s']<=control['ghost_interval_s'][1]:continue
 actor=next((x for x in state['actors'] if x['id']==past),None);ghost.append(bool(actor and actor['ghost'] and not actor['source_visible']))
 lanes.append(lane_at(road,*map(float,coordinates(road,actor['position_W'])[:2])) if actor else None)
trace=[]
for state in states:
 if not control['continuity_interval_s'][0]<=state['time_s']<=control['continuity_interval_s'][1]:continue
 options=[x for x in state['actors'] if x['id'] in (past,future)]
 chosen=next((x for x in options if x['source_visible']),options[0] if options else None)
 trace.append((state['time_s'],chosen))
continuous=bool(trace and all(r for t,r in trace));max_speed=None;max_yaw=None
if continuous:
 stamps=np.array([t for t,r in trace]);positions=np.array([r['position_W'] for t,r in trace]);angles=np.unwrap([r['yaw_rad'] for t,r in trace]);max_speed=float(max(np.linalg.norm(np.diff(positions,axis=0),axis=1)/np.diff(stamps)));max_yaw=float(np.degrees(max(abs(np.diff(angles)))))
changed=0
for old,new in zip(before,states):
 indexed={r['id']:r for r in new['actors']}
 for actor in old['actors']:
  if actor.get('source_visible') and actor.get('position_support')=='observed_geometry' and indexed.get(actor['id'])!=actor:changed+=1
checks=dict(raw_actor_file_exact=(a.baseline/'actors.json').read_bytes()==(a.scene/'actors.json').read_bytes(),directly_supported_render_states_exact=changed==0,identity_ids_not_merged=past!=future,future_supported_proof=bool(proof),entire_blind_interval_has_ghost=bool(ghost and all(ghost)),in_original_lane=bool(lanes and all(x==control['expected_lane'] for x in lanes)),full_transition_continuous=continuous,no_teleport_or_flip=bool(continuous and max_speed<control.get('max_transition_speed_mps',45.) and max_yaw<25))
result=dict(gate='PASS' if all(checks.values()) else 'FAIL',checks=checks,ghost_frames=len(ghost),max_transition_speed_mps=max_speed,max_yaw_step_deg=max_yaw,source_control_sha256=digest(a.control.read_bytes()),scene_artifact_id=manifest['artifact_id'],scope='Selected source-reviewed development gap; not all blind spots or independent holdout')
a.output.mkdir(parents=True,exist_ok=False);(a.output/'review.json').write_text(json.dumps(result,indent=2));print(json.dumps(result));raise SystemExit(0 if result['gate']=='PASS' else 2)
