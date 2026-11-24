"""Add camera-only evidence to a NEW cache without rerunning object inference."""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import CameraReader
from core.scene_junction import junction_evidence,JunctionTracker

p=argparse.ArgumentParser();p.add_argument('--result',type=Path,required=True)
p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
manifest=json.loads((a.result/'manifest.json').read_text(encoding='utf-8'))
rows=[json.loads(s) for s in (a.result/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
manifest.pop('road_map',None);manifest['osm']='disabled for camera-only junction validation'
manifest['junction_analysis']='camera-only repeated markings + lateral appearance; experimental'
reader=CameraReader(a.source/manifest['cameras']['front']['file']);tracker=JunctionTracker()
try:
    for row in rows:
        _,frame=reader.read_at(row['time_s'])
        if frame is None:raise RuntimeError('Missing original frame')
        front=row['cameras']['front']
        front['junction_evidence']=tracker.update(junction_evidence(frame,front.get('objects',[])),row['time_s'])
finally:reader.close()
a.output.mkdir(parents=True)
(a.output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
(a.output/'observations.jsonl').write_text('\n'.join(json.dumps(r) for r in rows)+'\n',encoding='utf-8')
print('Frames',len(rows),'active',sum(r['cameras']['front']['junction_evidence']['active'] for r in rows))
