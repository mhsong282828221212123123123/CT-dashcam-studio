"""Build a raw-source review queue for large tracked bodies excluded by scope.

This is a triage queue, not a recall score or a judgement that exclusions are
wrong. It uses missing observation IDs, rather than only admitted world actors.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
from PIL import Image,ImageDraw
from ct_scene.artifacts import canonical,verify

p=argparse.ArgumentParser();p.add_argument('--manifest',type=Path,required=True)
p.add_argument('--output',type=Path,required=True);p.add_argument('--limit',type=int,default=12)
a=p.parse_args();a.output.mkdir(exist_ok=False)
index={}
for pattern in ('*/artifact.json','*/*/artifact.json','*/*/*/artifact.json'):
    for f in Path('runs').glob(pattern):
        m=json.loads(f.read_text());index.setdefault(m['artifact_id'],f.parent)
queue=[]
for case in json.loads(a.manifest.read_text())['cases']:
    scene=Path(case['scene']);m=verify(scene);inputs={r['role']:r['artifact_id'] for r in m['inputs']}
    source=index[inputs['source']];tracks=index[inputs['tracks']]
    verify(source);verify(tracks)
    frames={f['frame_id']:f for f in json.loads((source/'frames.json').read_text())}
    audit=json.loads((scene/'observation-world-audit.json').read_text())
    missing={(r['local_id'],r['frame_id']):r for r in audit['missing_or_excluded']
             if r['reason']=='outside_same_direction_corridor'}
    groups=defaultdict(list)
    for line in (tracks/'observations.jsonl').read_text().splitlines():
        r=json.loads(line)
        if (r['local_track_id'],r['frame_id']) in missing and r['camera_id']!='back':groups[r['local_track_id']].append(r)
    for key,rows in groups.items():
        clear=[r for r in rows if r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>80 and r['bbox_xyxy'][3]-r['bbox_xyxy'][1]>70]
        if len(clear)<5 or max(r['camera_time_s'] for r in clear)-min(r['camera_time_s'] for r in clear)<1:continue
        largest=max(clear,key=lambda r:(r['bbox_xyxy'][2]-r['bbox_xyxy'][0])*(r['bbox_xyxy'][3]-r['bbox_xyxy'][1]))
        f=frames[largest['frame_id']];box=largest['bbox_xyxy']
        queue.append(dict(case_id=case['case_id'],local_id=key,camera_id=largest['camera_id'],
                          time_s=largest['camera_time_s'],frame_id=largest['frame_id'],bbox_xyxy=box,
                          clear_missing_observations=len(clear),all_missing_observations=len(rows),
                          review_interval_s=[min(r['camera_time_s'] for r in clear),max(r['camera_time_s'] for r in clear)],
                          image=str((source/f['image']).resolve()),scene=str(scene.resolve()),
                          image_area_fraction=(box[2]-box[0])*(box[3]-box[1])/(f['width']*f['height']),
                          source_verdict='NOT_EVALUATED'))
queue.sort(key=lambda r:r['image_area_fraction'],reverse=True)
(a.output/'review-queue.json').write_bytes(canonical(dict(scope='Large tracked exclusions requiring raw review; opposing/parked/out-of-scope vehicles may be correctly excluded. Not recall.',candidates=queue)))
for start in range(0,min(a.limit,len(queue)),6):
    sheet=Image.new('RGB',(1440,780),(25,25,25));draw=ImageDraw.Draw(sheet)
    for i,row in enumerate(queue[start:start+min(6,a.limit-start)]):
        im=Image.open(row['image']).convert('RGB');ImageDraw.Draw(im).rectangle(row['bbox_xyxy'],outline='red',width=5)
        im.thumbnail((480,360));x=(i%3)*480;y=(i//3)*390;sheet.paste(im,(x,y+30))
        draw.text((x+5,y+2),row['case_id']+' '+row['local_id']+' '+str(round(row['time_s'],2))+'s',fill='white')
    sheet.save(a.output/f'raw-queue-{start//6+1:02d}.jpg')
print(json.dumps(dict(candidates=len(queue),displayed=min(a.limit,len(queue)),output=str(a.output))))
