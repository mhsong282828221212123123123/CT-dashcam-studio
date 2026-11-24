"""Read-only source inventory and reproducible, date-separated review sampling.

Selection is a review queue, never a claim that a scenario is covered or passed.
Duplicate timestamps stay together. Date grouping also prevents adjacent events
from leaking between tuning and held-out evaluation.
"""
import argparse
import collections
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def catalog(roots, output, per_root=12):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    groups = collections.defaultdict(list)
    for root in roots:
        for path in sorted(Path(root).rglob('*-front.mp4')):
            groups[path.name].append(path)
    inventory = []
    for name, paths in sorted(groups.items()):
        path = paths[0]
        prefix = name.removesuffix('-front.mp4')
        channels = {c: str(path.with_name(prefix+'-'+c+'.mp4'))
                    for c in ('front', 'back', 'left_repeater', 'right_repeater', 'left_pillar', 'right_pillar')
                    if path.with_name(prefix+'-'+c+'.mp4').exists()}
        inventory.append(dict(id=prefix, date=prefix[:10], source=str(path),
                              copies=[str(p) for p in paths], cameras=channels,
                              source_root=next(str(r) for r in roots if path.is_relative_to(Path(r)))))
    dates = sorted({r['date'] for r in inventory},
                   key=lambda s: hashlib.sha256(s.encode()).hexdigest())
    holdout = set(dates[::3])
    for row in inventory:
        row['split'] = 'held_out' if row['date'] in holdout else 'development'
    selected = []
    for root in roots:
        rows = [r for r in inventory if r['source_root'] == str(root)]
        # Spread by date/hour and driving/parking storage group before filling.
        buckets = collections.defaultdict(list)
        for row in rows:
            storage = 'sentry' if 'SentryClips' in row['source'] else 'driving'
            buckets[(row['date'], row['id'][11:13], storage)].append(row)
        ordered = sorted(buckets, key=lambda k: hashlib.sha256(str(k).encode()).hexdigest())
        queue = [buckets[k][len(buckets[k])//2] for k in ordered]
        for row in queue[:per_root]:
            cap = cv2.VideoCapture(row['source'])
            fps = cap.get(cv2.CAP_PROP_FPS)
            count = cap.get(cv2.CAP_PROP_FRAME_COUNT)
            duration = count/fps if fps > 0 else 0
            t = min(15., duration*.5)
            cap.set(cv2.CAP_PROP_POS_MSEC, t*1000)
            ok, frame = cap.read()
            cap.release()
            item = dict(row, duration_s=duration, review_time_s=t,
                        review_status='unreviewed', scenarios=[], decode_ok=bool(ok))
            if ok:
                image = cv2.resize(frame, (384, 288))
                item['preview'] = row['id']+'.jpg'
                cv2.imencode('.jpg', image)[1].tofile(str(output/item['preview']))
            selected.append(item)
    for start in range(0, len(selected), 12):
        sheet = np.full((4*325, 3*384, 3), 25, np.uint8)
        for j, row in enumerate(selected[start:start+12]):
            x,y=(j%3)*384,(j//3)*325
            if row.get('preview'):
                frame = cv2.imdecode(np.fromfile(str(output/row['preview']),np.uint8),1)
                sheet[y:y+288,x:x+384]=frame
            label=f"{start+j:02d} {Path(row['source_root']).name} {row['id']}"
            cv2.putText(sheet,label,(x+4,y+303),0,.40,(255,255,255),1)
            cv2.putText(sheet,row['split'],(x+4,y+320),0,.4,(255,255,255),1)
        cv2.imencode('.jpg',sheet)[1].tofile(str(output/f'contact-{start//12:02d}.jpg'))
    result=dict(inventory=inventory,review_queue=selected,
                split_unit='recording date; timestamp duplicates grouped, not byte-deduplicated',
                required_scenarios=['day','night','stopped','low_speed','high_speed',
                                    'curve','intersection','lane_change','side_approach','occlusion'],
                acceptance='NOT EVALUATED')
    (output/'catalog.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(unique_timestamps=len(inventory),source_files=sum(map(len,groups.values())),
                         dates=dates,review_clips=len(selected),
                         by_root=dict(collections.Counter(r['source_root'] for r in inventory))),ensure_ascii=False))


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--roots',nargs='+',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--per-root',type=int,default=12)
    args=parser.parse_args()
    catalog(args.roots,args.output,args.per_root)
