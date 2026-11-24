"""Selected source-reviewed road width at the ego, not surveyed metric accuracy."""
import argparse,json
from pathlib import Path
from ct_scene.artifacts import verify,digest
from ct_scene.ingest import file_hash
from ct_scene.road_frame import coordinates
from ct_scene.road_geometry import boundaries_at
from review_guards import require_window

p=argparse.ArgumentParser()
for flag in ['scene','baseline','control','output']:p.add_argument('--'+flag,type=Path,required=True)
a=p.parse_args();m=verify(a.scene);bm=verify(a.baseline);c=json.loads(a.control.read_text(encoding='utf-8'))
base=lambda x:next(r['artifact_id'] for r in x['inputs'] if r['role']=='frozen_ego_lead')
if base(m)!=base(bm):raise ValueError('Frozen base changed')
if not any(x['role']=='source' and x['artifact_id']==c['source_artifact_id'] for x in m['inputs']):raise ValueError('Wrong source artifact')
if file_hash(Path(c['source_path']))!=c['source_sha256']:raise ValueError('Raw source changed')
load=lambda p:[json.loads(l) for l in p.read_text().splitlines()]
states=load(a.scene/'render-states.jsonl');old=load(a.baseline/'render-states.jsonl');road=json.loads((a.scene/'report.json').read_text())['road'];anchors=[]
for label in c['anchors']:
 t=label['time_s'];require_window(states,t-.1,t+.1);s=min(states,key=lambda r:abs(r['time_s']-t));q=coordinates(road,s['ego_position_W']);count=len(boundaries_at(road,float(q[0])))-1
 anchors.append(dict(time_s=t,expected_lanes=label['lanes'],display_lanes=count,passed=count==label['lanes'],basis=label['basis']))
clock_ego_same=len(states)==len(old) and all(x['time_s']==y['time_s'] and x['ego_position_W']==y['ego_position_W'] and x['ego_yaw_rad']==y['ego_yaw_rad'] for x,y in zip(states,old))
result=dict(gate='PASS' if clock_ego_same and all(x['passed'] for x in anchors) else 'FAIL',ego_clock_and_pose_unchanged=clock_ego_same,anchors=anchors,source_artifact_id=c['source_artifact_id'],scene_artifact_id=m['artifact_id'],control_sha256=digest(a.control.read_bytes()),scope='Selected visually reviewed lane-count/topology anchors only; exact edges, track identity and all-frame accuracy not evaluated')
a.output.mkdir(exist_ok=False,parents=True);(a.output/'review.json').write_text(json.dumps(result,indent=2));print(json.dumps(result));raise SystemExit(0 if result['gate']=='PASS' else 2)
