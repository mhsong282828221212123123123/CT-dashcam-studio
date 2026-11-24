"""Re-evaluate retained detected signal heads against source pixels, without new detections."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import CameraReader,Detection
from core.scene_features import SignalStateTracker


def enrich(result,source,output):
    result,output=Path(result),Path(output)
    if output.exists():raise FileExistsError(output)
    raw=(result/'observations.jsonl').read_bytes()
    rows=[json.loads(line) for line in raw.decode('utf-8').splitlines()]
    manifest=json.loads((result/'manifest.json').read_text(encoding='utf-8'))
    reader=CameraReader(Path(source));tracker=SignalStateTracker();changes=[]
    try:
        for row in rows:
            _,frame=reader.read_at(row['time_s'])
            if frame is None:raise ValueError(f"Unreadable source at {row['time_s']}")
            camera=row['cameras']['front'];old=camera.get('signals',[])
            camera['signals']=tracker.update(frame,[Detection('traffic_light',s['confidence'],s['bbox']) for s in old],row['time_s'])
            for before,after in zip(old,camera['signals']):
                if before['state']!=after['state']:
                    changes.append({'time_s':row['time_s'],'bbox':after['bbox'],
                                    'before':before['state'],'after':after['state']})
    finally:reader.close()
    manifest['signal_postprocess']={'input_sha256':hashlib.sha256(raw).hexdigest(),
        'source':'retained head detection and original pixels','new_head_detections':False,
        'method':'red rim/core spatial evidence and bounded temporal color support',
        'lane_assignment_verified':False}
    output.mkdir(parents=True)
    (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    (output/'observations.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows),encoding='utf-8')
    (output/'signal-changes.json').write_text(json.dumps(changes,indent=2),encoding='utf-8')
    print('Updated signal states',len(changes),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    for name in ('result','source','output'):parser.add_argument('--'+name,required=True)
    args=parser.parse_args();enrich(args.result,args.source,args.output)
