"""Selected source topology plus generated-road consistency; not metric GT."""
import argparse,json
from pathlib import Path
import cv2,numpy as np
from ct_scene.artifacts import verify,digest
from ct_scene.ingest import file_hash

p=argparse.ArgumentParser()
for key in ['scene','baseline','control','output']:p.add_argument('--'+key,type=Path,required=True)
a=p.parse_args();m=verify(a.scene);c=json.loads(a.control.read_text(encoding='utf-8'))
if not any(x['role']=='source' and x['artifact_id']==c['source_artifact_id'] for x in m['inputs']):raise ValueError('Wrong source')
if file_hash(Path(c['source_path']))!=c['source_sha256']:raise ValueError('Raw source changed')
actors=json.loads((a.scene/'actors.json').read_text());actor=next(x for x in actors if c['source_local_id'] in x['local_ids']);road=json.loads((a.scene/'report.json').read_text())['road'];j=next(x for x in road['junctions'] if x['id']==c['junction_id'])
proof_path=a.scene/'camera-crossing-road-review.json';proof=json.loads(proof_path.read_text()) if proof_path.exists() else dict(changes=[])
arms=[x for x in j.get('branch_display_hints',[]) if x.get('source','').startswith('camera_observed_crossing')]
rows=[r for r in actor['samples'] if c['source_interval_s'][0]<=r['time_s']<=c['source_interval_s'][1] and r.get('support')=='detector_observation']
inside=[any(cv2.pointPolygonTest(np.asarray(arm['polygon_W_xy'],'float32'),tuple(map(float,r['position_W'][:2])),False)>=0 for arm in arms) for r in rows]
checks=dict(same_actor_samples=(a.scene/'actors.json').read_bytes()==(a.baseline/'actors.json').read_bytes(),same_world_timeline=(a.scene/'render-states.jsonl').read_bytes()==(a.baseline/'render-states.jsonl').read_bytes(),camera_supported_additional_road=bool(arms),camera_provenance=any(x['source_actor_id']==actor['global_id'] and not x['map_used'] for x in proof['changes']),crossing_points_on_estimated_pavement=bool(inside and all(inside)))
result=dict(gate='PASS' if all(checks.values()) else 'FAIL',checks=checks,selected_source_topology=c['visual_label'],covered_observed_samples=sum(inside),source_samples=len(rows),metric_width_accuracy='NOT_EVALUATED',camera_projection_accuracy='NOT_EVALUATED',scope='Selected visual topology and data-fitting consistency; containment is not independent metric accuracy',scene_artifact_id=m['artifact_id'],control_sha256=digest(a.control.read_bytes()))
a.output.mkdir(parents=True,exist_ok=False);(a.output/'review.json').write_text(json.dumps(result,indent=2));print(json.dumps(result));raise SystemExit(0 if result['gate']=='PASS' else 2)
