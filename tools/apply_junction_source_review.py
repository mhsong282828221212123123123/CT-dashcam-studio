"""Explicit source-crop review: body class, initial nose direction, ID continuity.

No positions, GPS snapping or per-frame trajectory keyframes are supplied.
"""
import json,pickle,math,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

REVIEWS={
 '2026-08-15_17-39-32':dict(
  shapes={'front:3':'truck_heavy','front:4':'suv','front:6':'suv','front:7':'suv','front:8':'suv','front:9':'suv','front:11':'small_truck','front:13':'suv','front:15':'suv','front:22':'suv','front:33':'suv','front:34':'small_truck'},
  forward=['front:7'],opposite=['front:1','front:3','front:4','front:5','front:6','front:8','front:9','front:11','front:13','front:15','front:16','front:17','front:18','front:19','front:20','front:21','front:22','front:25','front:33','front:34']),
 '2026-10-15_17-33-13':dict(
  shapes={'front:2':'van','front:3':'suv','front:5':'van','front:8':'suv','front:9':'suv','front:11':'compact_van','front:18':'suv','right_repeater:2':'small_truck','right_repeater:3':'small_truck','right_repeater:5':'small_truck','left_repeater:3':'small_truck','right_repeater:8':'small_truck','left_repeater:6':'suv','left_repeater:1':'truck_box'},
  stopped_facing_until={'front:3':29.,'right_repeater:6':29.,'left_repeater:1':29.},
  partial_silhouettes=['right_repeater:6'],
  forward=['front:1','front:2','front:3','right_repeater:1','left_repeater:1'],opposite=['front:4','front:5','front:6','front:8','front:9','front:11','front:13','front:18']),
 '2026-09-16_22-15-39':dict(
  shapes={'front:24':'bus','front:28':'bus','left_repeater:4':'bus','left_repeater:10':'bus','left_repeater:16':'bus'},
  aliases={'front:27':'front:8','front:28':'front:24'},
  forward=['front:8','front:24','left_repeater:1','left_repeater:4','left_repeater:10','left_repeater:16'],opposite=['front:1','front:2','front:3','front:4','front:5','front:6','front:7','front:12','front:14','front:16','front:18','front:20','front:21'])}

if __name__=='__main__':
 root=Path(sys.argv[1]);config=REVIEWS[root.name]
 t=pickle.load((root/'initial.pkl').open('rb'));derived=root/'reviewed-input'
 rows=[json.loads(x) for x in (root/'analysis/observations.jsonl').read_text(encoding='utf8').splitlines()]
 initial={};used={}
 for i,row in enumerate(rows):
  for data in row['cameras'].values():
   grouped={}
   for obj in data.get('objects',[]):
    original=obj['track_id'];source=config.get('aliases',{}).get(original,original)
    if source!=original:obj['original_track_id']=original;obj['track_id']=source
    shape=config['shapes'].get(original) or config['shapes'].get(source)
    if shape:obj.update(geometry_kind=shape,render_kind={'truck_box':'truck'}.get(shape,shape))
    if source in config.get('partial_silhouettes',[]):obj['silhouette_partial']=True
    if source not in initial and (source in config['forward'] or source in config['opposite']):
     local=0. if source in config['forward'] else math.pi
     axis=t.ego_poses[i][1]@np.array([math.sin(local),math.cos(local)])
     initial[source]=math.atan2(axis[0],axis[1]);used[source]=0
    if source in initial and used[source]<3:
     obj['reviewed_world_yaw']=initial[source];used[source]+=1
    if source in config.get('stopped_facing_until',{}) and row['time_s']<=config['stopped_facing_until'][source]:obj['reviewed_world_yaw']=0.
    if source not in grouped or obj['confidence']>grouped[source]['confidence']:grouped[source]=obj
   data['objects']=list(grouped.values())
 m=json.loads((derived/'manifest.json').read_text(encoding='utf8'));m['review_assisted']=True
 m['reference_review_notes']='Independent first/middle/last source crop review. Body classes and initial nose direction only; two night same-car/bus ID continuities reviewed. No hand-keyframed positions.'
 (root/'source-review/annotations.json').write_text(json.dumps(config,indent=2),encoding='utf8')
 (derived/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf8')
 (derived/'observations.jsonl').write_text('\n'.join(json.dumps(r,ensure_ascii=False) for r in rows)+'\n',encoding='utf8')
 print(root.name,len(initial),'initial direction reviews')
