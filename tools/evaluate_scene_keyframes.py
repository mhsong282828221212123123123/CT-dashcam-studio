"""Compare hand-marked source objects to detections and final scene placements."""
import json
from pathlib import Path
import sys
import numpy as np
from scipy.optimize import linear_sum_assignment
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import iou
from core.scene_fusion import FusedSceneTimeline

root=Path(sys.argv[1])
annotations=json.loads((root/'source-keyframe-annotations.json').read_text(encoding='utf-8'))
report=[]
for frame in annotations['frames']:
    path=root/'analysis'/frame['id']
    if not (path/'manifest.json').exists():
        report.append({'id':frame['id'],'status':'analysis_pending'});continue
    manifest=json.loads((path/'manifest.json').read_text(encoding='utf-8'))
    rows=[json.loads(x) for x in (path/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    timeline=FusedSceneTimeline(rows,manifest)
    index=min(range(len(rows)),key=lambda i:abs(rows[i]['time_s']-frame['time_s']))
    if abs(rows[index]['time_s']-frame['time_s'])>.05:raise ValueError('Annotated time not sampled')
    objects=rows[index]['cameras'][frame['camera']]['objects'];targets=frame['targets']
    scores=np.zeros((len(targets),len(objects)))
    for i,target in enumerate(targets):
        for j,obj in enumerate(objects):
            allowed=('car','suv','truck','bus') if target['kind']=='car' else (target['kind'],)
            if obj['kind'] in allowed:scores[i,j]=iou(target['bbox'],obj['bbox'])
    assigned=dict(zip(*linear_sum_assignment(-scores))) if scores.size else {}
    evaluated=[]
    for i,target in enumerate(targets):
        j=assigned.get(i);obj=objects[j] if j is not None and scores[i,j]>=.5 else None
        actors=[a for a in timeline.scenes[index].actors if obj and obj['track_id'] in a.sources]
        evaluated.append(dict(target=target['id'],detected=obj is not None,
            source_id=obj['track_id'] if obj else None,box_iou=float(scores[i,j]) if obj else None,
            placed_ids=[a.key for a in actors],placement_count=len(actors),
            position_accuracy_verified=False,heading_verified=False))
    signals=rows[index]['cameras'][frame['camera']].get('signals',[])
    signal_review=[]
    for target in frame.get('signals',[]):
        best=max(signals,key=lambda s:iou(target['bbox'],s['bbox']),default=None)
        overlap=iou(target['bbox'],best['bbox']) if best else 0.
        matched=best if overlap>=.3 else None
        signal_review.append(dict(expected_state=target['state'],detected=matched is not None,
            observed_state=matched['state'] if matched else None,
            color_correct=bool(matched and matched['state']==target['state']),
            box_iou=float(overlap),lane_assignment_verified=False))
    report.append(dict(id=frame['id'],time_s=frame['time_s'],targets=evaluated,signals=signal_review,
                       acceptance=False,scope='annotated keyframe only'))
(root/'keyframe-evaluation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
