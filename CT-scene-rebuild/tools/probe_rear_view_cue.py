import argparse,json
from pathlib import Path
from functools import lru_cache
import cv2
from ct_scene.rear_view_cue import paired_red_lamps

ap=argparse.ArgumentParser();ap.add_argument('--case',type=Path,required=True);ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(exist_ok=False)
load=lambda p:json.loads(p.read_text(encoding='utf-8'));frames={f['frame_id']:f for f in load(a.case/'source/frames.json')};actors=load(a.scene/'pre-road-geometry.json')
@lru_cache(maxsize=8)
def im(fid):return cv2.imread(str(a.case/'source'/frames[fid]['image']))
results={}
for actor in actors:
    if actor['id'] not in ['front-0020','front-0033','front-0043','front-0082']:continue
    rows=[]
    for r in actor['samples']:
        cue=paired_red_lamps(im(r['frame_id']),r['bbox_xyxy']);rows.append(dict(frame_id=r['frame_id'],time_s=r['time_s'],**cue))
    results[actor['id']]=rows
(a.output/'cues.json').write_text(json.dumps(results,indent=2),encoding='utf-8');print({k:dict(samples=len(v),rear_cues=sum(r['rear_cue'] for r in v)) for k,v in results.items()})
