"""Evaluate bounded trajectory proposals on development camera tracks."""
import collections,json,math,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_fusion import FusedSceneTimeline,corners,dimensions
from core.scene_trajectory_fit import fit_trajectory
from core.scene_analysis import iou

root=Path(sys.argv[1]);out=root/'trajectory-review';out.mkdir(exist_ok=True)
cases=json.loads((root/'baseline-cases.json').read_text(encoding='utf-8'))
reports=[]
for case in cases:
    if case['split']!='development':continue
    target=root/'baseline'/case['id']
    rows=[json.loads(s) for s in (target/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    manifest=json.loads((target/'manifest.json').read_text(encoding='utf-8'))
    timeline=FusedSceneTimeline(rows,manifest)
    tracks=collections.defaultdict(list)
    for index,(row,scene) in enumerate(zip(rows,timeline.scenes)):
        if not scene.telemetry or scene.telemetry.get('speed_kmh') is None:continue
        if scene.telemetry['speed_kmh']>0 and scene.telemetry.get('heading') is None:continue
        actors={s:a for a in scene.actors for s in a.sources}
        origin,rotation=timeline.ego_poses[index]
        for camera,data in row['cameras'].items():
            if camera=='back' or camera not in timeline.camera_frames[index]:continue
            for obj in data.get('objects',[]):
                if obj.get('track_status')!='confirmed' or obj['kind']!='car':continue
                x,y,w,h=obj['bbox']
                if y+h>=manifest['cameras'][camera]['height']-3:continue
                if any(other is not obj and other['bbox'][0]<x+w/2<other['bbox'][0]+other['bbox'][2]
                       and other['bbox'][1]<y+h<other['bbox'][1]+other['bbox'][3]-10 for other in data['objects']):continue
                actor=actors.get(obj['track_id'])
                seed=origin+rotation@np.array([actor.x,actor.z]) if actor else np.full(2,np.nan)
                observation=dict(time=row['time_s'],camera=timeline.camera_frames[index][camera],
                                 bbox=obj['bbox'],ego_origin=origin,ego_rotation=rotation)
                baseline=0.
                if actor:
                    b=observation['camera'].visible_bounds(corners(actor.x,actor.z,dimensions(actor.geometry_kind or actor.kind),kind=actor.geometry_kind or actor.kind))
                    if b is not None:baseline=iou(np.r_[b[:2],b[2:]-b[:2]],obj['bbox'])
                tracks[obj['track_id']].append((observation,seed,baseline))
    local=[]
    for source,items in tracks.items():
        items=[item for item in items if item[0]['time']-items[0][0]['time']<=3.]
        if len(items)<8:continue
        if any(b[0]['time']-a[0]['time']>.3 for a,b in zip(items,items[1:])):continue
        proposal=fit_trajectory([i[0] for i in items],[i[1] for i in items])
        if proposal is None:continue
        previous=float(np.median([v[2] for v in items[1::2]]))
        proposal.update(source=source,clip=case['id'],baseline_withheld_iou_median=previous,
                        improvement=proposal['validation_iou_median']-previous,applied=False)
        local.append(proposal)
    reports.extend(local)
    (out/(case['id']+'.json')).write_text(json.dumps(local,indent=2),encoding='utf-8')
    print(case['id'],'proposals',len(local),'passing image gates',sum(p['accepted'] for p in local),flush=True)
(out/'report.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')
