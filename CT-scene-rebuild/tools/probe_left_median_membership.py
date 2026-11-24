import argparse,json
from collections import defaultdict
from pathlib import Path
from ct_scene.left_median_membership import review
ap=argparse.ArgumentParser();ap.add_argument('--case',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(exist_ok=False);obs=defaultdict(list)
for r in map(json.loads,(a.case/'tracks/observations.jsonl').read_text().splitlines()):obs[r['frame_id']].append(r)
r=review(a.case/'source',json.loads((a.case/'source/frames.json').read_text()),obs);(a.output/'review.json').write_text(json.dumps(r,indent=2));print(json.dumps({k:dict(frames=len(v['samples']),span=[v['samples'][0]['time_s'],v['samples'][-1]['time_s']]) for k,v in r.items()}))
