"""Rebuild the explicitly selected queue sample from fresh rectified detections."""
import hashlib
import json
import pickle
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_fusion import FusedSceneTimeline
from core.driving_scene import DrivingRenderer
from tools.audit_scene_motion import audit_motion
from tools.validate_single_clip_reference import motion_outliers


if __name__=='__main__':
    root=Path(sys.argv[1]);source=root/'analysis';derived=root/'sample-input'
    derived.mkdir(exist_ok=True)
    manifest=json.loads((source/'manifest.json').read_text(encoding='utf8'))
    expected='2026-10-15_17-33-13-front.mp4'
    if manifest['cameras']['front']['file']!=expected:
        raise ValueError('This sample review is bound to one source clip')
    rows=[json.loads(s) for s in (source/'observations.jsonl').read_text(encoding='utf8').splitlines()]
    # These are visually reviewed subtype labels in the new detection output.
    # No boxes, positions, lanes, identities or trajectories are manually moved.
    labels={'front:3':'van','front:1':'suv','right_repeater:9':'small_truck'}
    changed={key:0 for key in labels}
    for row in rows:
        for data in row['cameras'].values():
            for obj in data.get('objects',[]):
                kind=labels.get(obj['track_id'])
                if kind:
                    obj.update(render_kind=kind,geometry_kind=kind,
                               subtype_review='visual source inspection; subtype only, no coordinate edits')
                    changed[obj['track_id']]+=1
    cache=root.parent/'three-junction-review'/'2026-10-15_17-33-13'/'reviewed-input'/'manifest.json'
    previous=json.loads(cache.read_text(encoding='utf8'))
    if previous['cameras']['front']['file']!=expected:raise ValueError('Wrong map cache')
    if previous.get('road_map'):
        manifest['road_map']=previous['road_map'];manifest['osm']='cached OSM outline reference; not lane-level ground truth'
    manifest['motion_bundle_refinement']=True
    manifest['review_assisted']=True
    manifest['sample_provenance']=dict(status='experimental_current_code_sample_not_metric_accepted',
        fresh_detection=True,source_time_alignment='nearest container PTS',
        optical_profiles='explicit inferred front/side lens candidates, not installed defaults',
        extrinsics='existing estimated body-camera origins and angles; not independently verified',
        reviewed_subtypes=labels,reviewed_frames=changed,
        source_analysis_sha256=hashlib.sha256((source/'observations.jsonl').read_bytes()).hexdigest())
    (derived/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False),encoding='utf8')
    (derived/'observations.jsonl').write_text('\n'.join(json.dumps(r,ensure_ascii=False,allow_nan=False) for r in rows)+'\n',encoding='utf8')
    print('FUSION START',flush=True)
    def progress(r):print('Bundle',r['source'],r.get('accepted'),r.get('projection_hypothesis'),flush=True)
    timeline=FusedSceneTimeline(rows,manifest,motion_progress=progress)
    with (root/'current.pkl').open('wb') as f:pickle.dump(timeline,f)
    audit=audit_motion(timeline);outliers=motion_outliers(timeline)
    (root/'current-audit.json').write_text(json.dumps(audit,indent=2),encoding='utf8')
    (root/'current-motion-outliers.json').write_text(json.dumps(outliers,indent=2),encoding='utf8')
    (root/'current-models.json').write_text(json.dumps(timeline.motion_bundle_report,indent=2),encoding='utf8')
    renderer=DrivingRenderer(720,480);proof=[]
    for t in (2.,12.,24.,34.):
        image=renderer.render(timeline.sample(t));cv2.putText(image,f'SOURCE {t:.1f}s',(15,30),0,.7,(50,50,180),2)
        proof.append(image)
    cv2.imencode('.jpg',np.vstack([np.hstack(proof[:2]),np.hstack(proof[2:])]))[1].tofile(root/'scene-proof.jpg')
    print('COMPLETE',dict(observations=audit['observation_frames'],missing=audit['missing_frames'],outlier_steps=len(outliers)),flush=True)
