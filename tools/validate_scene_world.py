"""Replay held-out observation dropouts; distinguish consistency from accuracy."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_fusion import FusedSceneTimeline,corners,dimensions
from core.driving_scene import DrivingRenderer
from core.scene_analysis import iou


def validate(root,output,source='front:4'):
    root=Path(root);output=Path(output)
    if output.exists():raise FileExistsError(output)
    rows=[json.loads(line) for line in (root/'observations.jsonl').read_text().splitlines()]
    manifest=json.loads((root/'manifest.json').read_text())
    base=FusedSceneTimeline(rows,manifest)
    windows=[(t,t+.6) for t in (1.,3.,5.,7.,9.) if t+.9<=rows[-1]['time_s']]
    masked=copy.deepcopy(rows)
    for row in masked:
        if any(a<=row['time_s']<b for a,b in windows):
            for camera in row['cameras'].values():
                camera['objects']=[obj for obj in camera.get('objects',[]) if obj['track_id']!=source]
    trial=FusedSceneTimeline(masked,manifest)
    dropouts=[];errors=[];switches=[]
    for start,end in windows:
        before=max(i for i,r in enumerate(rows) if r['time_s']<start)
        previous=next((a for a in trial.at(before).actors if source in a.sources),None)
        if previous is None:
            dropouts.append({'start':start,'status':'unassessable','reason':'no confirmed seed'});continue
        samples=[]
        for i,r in enumerate(rows):
            if not start<=r['time_s']<end:continue
            expected=next((a for a in base.at(i).actors if source in a.sources),None)
            predicted=next((a for a in trial.at(i).actors if a.key==previous.key),None)
            error=None if expected is None or predicted is None else float(np.hypot(predicted.x-expected.x,predicted.z-expected.z))
            if error is not None:errors.append(error)
            samples.append({'t':r['time_s'],'state':None if predicted is None else predicted.tracking_state,'baseline_difference_m':error})
        after=next(i for i,r in enumerate(rows) if r['time_s']>=end)
        returned=next((a for a in trial.at(after).actors if source in a.sources),None)
        retained=returned is not None and returned.key==previous.key
        if not retained:switches.append(start)
        passed=all(s['state'] in ('predicted','interpolated') and s['baseline_difference_m'] is not None and s['baseline_difference_m']<.5 for s in samples) and retained
        dropouts.append({'start':start,'end':end,'status':'pass' if passed else 'fail','identity_retained':retained,'samples':samples})
    projections=[]
    for i,row in enumerate(rows):
        if row['time_s']<.5:continue
        lookup={o['track_id']:(name,o) for name,c in row['cameras'].items() for o in c.get('objects',[])}
        for actor in base.at(i).actors:
            for key in actor.sources:
                if key not in lookup:continue
                name,obj=lookup[key];camera=base.camera_frames[i][name]
                shape=actor.geometry_kind or obj.get('render_kind') or obj['kind']
                bounds=camera.visible_bounds(corners(actor.x,actor.z,dimensions(shape),kind=shape))
                if bounds is None:continue
                lo,hi=bounds[:2],bounds[2:]
                overlap=float(iou(np.r_[lo,hi-lo],obj['bbox']))
                projections.append({'t':row['time_s'],'source':key,'camera':name,'iou':overlap})
    scores=[p['iou'] for p in projections if p['camera']!='back']
    rear_scores=[p['iou'] for p in projections if p['camera']=='back']
    duplicate_ids=sum(len(s.actors)-len({a.key for a in s.actors}) for s in trial.scenes)
    collisions=[{'time_s':s.time,'a':a.key,'b':b.key} for s in trial.scenes for i,a in enumerate(s.actors)
        for b in s.actors[i+1:] if abs(a.x-b.x)<(a.width+b.width)/2 and abs(a.z-b.z)<(a.length+b.length)/2]
    gates={'dropout_continuity':bool(dropouts) and all(x['status']=='pass' for x in dropouts),
           'primary_reprojection_iou_p05_at_least_0_5':bool(scores) and float(np.percentile(scores,5))>=.5,
           'unique_world_ids':duplicate_ids==0,'no_overlapping_footprints':not collisions}
    report={'source_sha256':hashlib.sha256((root/'observations.jsonl').read_bytes()).hexdigest(),
        'scope':'Replay consistency under deliberately removed observations; not ground-truth depth accuracy.',
        'temporal_mode':'offline: future observations may bound a reconstructed gap; not causal prediction accuracy',
        'profile':base.profile,
        'implementation_sha256':{name:hashlib.sha256((Path(__file__).resolve().parents[1]/'core'/name).read_bytes()).hexdigest()
            for name in ('scene_fusion.py','scene_world_tracks.py','driving_scene.py')},
        'coverage':{'confirmed_camera_observations':sum(o.get('track_status')=='confirmed' for r in rows for c in r['cameras'].values() for o in c.get('objects',[])),
                    'reprojection_observations_evaluated':len(scores),'rear_support_observations':len(rear_scores),
                    'note':'Front and side cameras determine geometry. Rear IoU is reported separately, not a pose quality gate. Unplaced and inferred objects are excluded; this is not recall.'},
        'footprint_conflicts':collisions,
        'review_assisted_input':manifest.get('review_assisted',False),'evaluation_times':'t >= 0.5 seconds; development sample, not independent test data',
        'dropouts':dropouts,'identity_switches':switches,'max_dropout_difference_m':max(errors,default=None),
        'reprojection':{'scope':'front_and_side_only','count':len(scores),'median_iou':float(np.median(scores)) if scores else None,
                        'p05_iou':float(np.percentile(scores,5)) if scores else None,'worst':sorted([p for p in projections if p['camera']!='back'],key=lambda p:p['iou'])[:12]},
        'rear_support_reprojection':{'count':len(rear_scores),'median_iou':float(np.median(rear_scores)) if rear_scores else None},
        'gates':gates,'calibration_accuracy':'NOT VALIDATED','cross_camera_identity_accuracy':'NOT VALIDATED'}
    output.mkdir(parents=True)
    (output/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    (output/'masked-observations.jsonl').write_text('\n'.join(json.dumps(r,ensure_ascii=False) for r in masked),encoding='utf-8')
    (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    renderer=DrivingRenderer()
    index=min(range(len(rows)),key=lambda i:abs(rows[i]['time_s']-3.2))
    for name,timeline in [('observed',base),('occluded-prediction',trial)]:
        ok,buf=cv2.imencode('.png',renderer.render(timeline.at(index)))
        if not ok:raise RuntimeError('Cannot render')
        buf.tofile(str(output/(name+'.png')))
    (output/'world-audit.json').write_text(json.dumps(trial.audit,ensure_ascii=False,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--result',required=True);p.add_argument('--output',required=True);p.add_argument('--source',default='front:4')
    a=p.parse_args();r=validate(a.result,a.output,a.source)
    print(json.dumps({k:r[k] for k in ('gates','max_dropout_difference_m','reprojection')},indent=2))
    sys.exit(0 if all(r['gates'].values()) else 1)
