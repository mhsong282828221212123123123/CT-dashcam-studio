"""Prioritize source identity sheets by visible size and observation duration.

This changes review order only. No detections/tracks are removed from analysis.
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.review_scene_sources import review

if __name__=='__main__':
    root=Path(sys.argv[1]);out=root/'source-review'
    if not (out/'identity-index.json').exists():review(root/'analysis',out)
    entries=json.loads((out/'identity-index.json').read_text())
    values=defaultdict(list)
    for line in (root/'analysis/observations.jsonl').read_text().splitlines():
        for camera in json.loads(line)['cameras'].values():
            for obj in camera.get('objects',[]):values[obj['track_id']].append(obj['bbox'][2]*obj['bbox'][3])
    order=sorted(range(len(entries)),key=lambda i:np.percentile(values[entries[i]['source']],95)*np.sqrt(entries[i]['frames']),reverse=True)[:24]
    pages={};rows=[];index=[]
    for i in order:
        page=i//10+1
        if page not in pages:pages[page]=cv2.imdecode(np.fromfile(out/f'identities-{page}.jpg',np.uint8),cv2.IMREAD_COLOR)
        rows.append(pages[page][(i%10)*130:(i%10+1)*130])
        index.append(entries[i])
    for i in range(0,len(rows),8):
        cv2.imencode('.jpg',np.vstack(rows[i:i+8]))[1].tofile(out/f'prominent-{i//8+1}.jpg')
    (out/'prominent-index.json').write_text(json.dumps(index,indent=2),encoding='utf8')
    print('REVIEW ORDER',len(index),'of',len(entries),'tracks; no track filtering',flush=True)
