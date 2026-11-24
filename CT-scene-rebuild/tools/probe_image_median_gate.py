import argparse,json
from collections import defaultdict
from pathlib import Path
from ct_scene.image_median_gate import extract_lines,review

ap=argparse.ArgumentParser();ap.add_argument('--case',type=Path,required=True);ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(exist_ok=False)
load=lambda p:json.loads(p.read_text(encoding='utf-8'));frames=load(a.case/'source/frames.json');obs=defaultdict(list)
for row in map(json.loads,(a.case/'tracks/observations.jsonl').read_text().splitlines()):obs[row['frame_id']].append(row)
lines=extract_lines(a.case/'source',frames,obs);actors=load(a.scene/'pre-road-geometry.json');poses=load(a.case/'base/trajectory.json');r=review(actors,lines,[p['camera_time_s'] for p in poses],[p['ego_yaw_rad'] for p in poses])
(a.output/'lines.json').write_text(json.dumps(lines),encoding='utf-8');(a.output/'review.json').write_text(json.dumps(r,indent=2),encoding='utf-8');print('excluded',[(k,v['beyond_samples'],v['own_side_samples']) for k,v in r.items() if v['excluded']]);print('selected',[(k,r.get(k,{}).get('excluded')) for k in ['front-0001','front-0002','front-0020','front-0033','front-0083']])
