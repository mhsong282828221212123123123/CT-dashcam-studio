"""Extract independent paint evidence before enabling a curved road in exports."""
import argparse,json
from pathlib import Path
from collections import defaultdict
import numpy as np
from ct_scene.curved_road import paint_segments,fit_reference
from ct_scene.artifacts import canonical

p=argparse.ArgumentParser();p.add_argument('--name',required=True);a=p.parse_args()
root=Path('runs');name=a.name;source=root/(name+'-full-source-001');base=root/(name+'-full-base-001')
load=lambda p:json.loads(p.read_text(encoding='utf-8'))
rows=load(base/'trajectory.json');times=np.array([r['camera_time_s'] for r in rows]);ego=np.array([r['ego_position_W'] for r in rows]);yaw=np.array([r['ego_yaw_rad'] for r in rows])
cfg=load(Path('configs/surrounding-prototype.json'));profiles=load(Path('configs/multicam-optical-candidates.json'))['cameras']
obs=defaultdict(list)
tracks=root/(name+'-full-tracks-001')
for r in map(json.loads,(tracks/'observations.jsonl').read_text().splitlines()):obs[r['frame_id']].append(r)
out=root/(name+'-curve-probe-001');out.mkdir(exist_ok=True)
masks=root/(name+'-road-masks-001')
segments=paint_segments(source,load(source/'frames.json'),obs,times,ego,yaw,profiles['front'],cfg['cameras']['front'],masks if masks.exists() else None)
previous=root/(name+'-ego-maneuver-001')
if not previous.exists():previous=root/(name+'-actor-lanes-001')
road=load(previous/'report.json')['road']
line,review=fit_reference(segments,ego,road['boundaries_y'],cfg['lane_width_m'],yaws=yaw)
(out/'paint-segments.json').write_bytes(canonical(segments));(out/'reference.json').write_bytes(canonical(line));(out/'review.json').write_bytes(canonical(review))
print(json.dumps(dict(name=name,segments=len(segments),**{k:v for k,v in review.items() if k!='stations'},stations=len(review['stations']))))
