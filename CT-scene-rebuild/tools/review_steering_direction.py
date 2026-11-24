"""Audit raw steering cue against cached own-turn camera evidence, no re-render."""
import argparse,json
from pathlib import Path
import numpy as np
from ct_scene.steering_cue import SteeringCue
from ct_scene.full_timeline import own_motion
from ct_scene.ego_turn_rotation import corrected_yaw
from ct_scene.artifacts import verify,digest
p=argparse.ArgumentParser();p.add_argument('--cases',nargs='+',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
results=[]
for case in a.cases:
 telemetry=case/'telemetry';base=case/'base';rotation=case/'ego-rotation'
 tm=verify(telemetry);bm=verify(base)
 rows=[json.loads(x) for x in (telemetry/'telemetry.jsonl').read_text().splitlines()]
 trajectory=json.loads((base/'trajectory.json').read_text());times=np.array([r['camera_time_s'] for r in trajectory])
 cue=SteeringCue(rows);pairs=json.loads((rotation/'camera-pairs.json').read_text()) if rotation.exists() else {}
 _,original=own_motion(times,rows);old,old_review=corrected_yaw(times,original,pairs)
 new,review=corrected_yaw(times,original,pairs,cue)
 results.append(dict(case=str(case),telemetry_artifact_id=tm['artifact_id'],base_artifact_id=bm['artifact_id'],calibration=cue.report,rotation_review=review,max_heading_difference_deg=float(np.degrees(max(abs(new-old)))),changed_intervals=sum(r.get('steering_direction_rejected_intervals',0) for r in review),all_heading_samples_unchanged=bool(np.array_equal(old,new))))
a.output.mkdir(parents=True,exist_ok=False)
result=dict(cases=results,code_sha256=digest(Path('src/ct_scene/steering_cue.py').read_bytes()),scope='Development clips; same-clip alternating-second validation against SEI heading is not independent surveyed heading accuracy. Steering is a direction veto, not heading integration.')
(a.output/'review.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
