"""Verify reference-only integration and regenerate source matching proofs."""
import json
from pathlib import Path
import cv2
import numpy as np
from ct_scene.artifacts import verify


out=Path('runs/rear-reference-review-003');out.mkdir(exist_ok=True)
results={}
cases=[('oct21','oct21-full-source-001','oct21-rear-view-001','oct21-rear-reference-004'),
       ('sep17','sep17-full-source-001','sep17-local-lanes-002','sep17-rear-reference-003')]
for name,source,previous,current in cases:
    source,previous,current=[Path('runs')/p for p in (source,previous,current)]
    verify(current)
    load=lambda p:json.loads(p.read_text(encoding='utf-8'))
    report=load(current/'report.json');reference=load(current/'rear-identity-reference.json')
    checks={
        'actors_exact':(previous/'actors.json').read_bytes()==(current/'actors.json').read_bytes(),
        'ego_exact':(previous/'ego-lane-state.json').read_bytes()==(current/'ego-lane-state.json').read_bytes(),
        'existing_handoffs_exact':(previous/'association-review.json').read_bytes()==(current/'association-review.json').read_bytes(),
        'road_exact':load(previous/'report.json')['road']==report['road'],
        'rear_geometry_disabled':reference['rear_geometry']=='DISABLED',
        'no_rear_based_world_union':reference['world_id_merge']=='DISABLED',
        'no_observed_ghost_conflicts':not load(current/'observation-world-audit.json')['observed_ghost_conflicts'],
        'full_export_decoded':report['all_frames_decoded'],
    }
    by_frame={f['frame_id']:f for f in load(source/'frames.json')};panels=[]
    for interval in reference['intervals']:
        evidence=interval['evidence']
        assert len(evidence)>=3 and interval['end_s']-interval['start_s']>=.4-1e-6
        # Show both endpoints; no approval based only on the strongest frame.
        for e in [evidence[0],evidence[-1]]:
            crops=[]
            for fid,box,points in [(e['rear_frame_id'],e['rear_box'],e['matched_points_a']),
                                   (e['side_frame_id'],e['side_box'],e['matched_points_b'])]:
                image=cv2.imread(str(source/by_frame[fid]['image']))
                x,y,X,Y=np.rint(box).astype(int)
                crop=image[max(0,y):min(image.shape[0],Y),max(0,x):min(image.shape[1],X)]
                scale=min(1,640/max(crop.shape[:2]));crop=cv2.resize(crop,None,fx=scale,fy=scale)
                for point in points:cv2.circle(crop,tuple(np.rint(point).astype(int)),3,(0,0,255),-1)
                h,w=crop.shape[:2];scale=min(480/w,240/h);crop=cv2.resize(crop,None,fx=scale,fy=scale)
                canvas=np.full((280,480,3),230,np.uint8);canvas[40:40+crop.shape[0],:crop.shape[1]]=crop;crops.append(canvas)
            panel=np.hstack(crops)
            cv2.putText(panel,f"{e['rear_id']} -> {e['global_id']}  {e['time_s']:.2f}s",(12,27),0,.6,(0,0,0),2)
            panels.append(panel)
    if panels:cv2.imwrite(str(out/f'{name}-endpoints.jpg'),np.vstack(panels))
    if name=='oct21':
        # The rear ID seen switching white -> dark car must not be applied
        # wholesale. This fixture assertion is not a production ID rule.
        checks['stolen_rear_id_not_propagated']=not any(i['rear_id']=='back-0010' for i in reference['intervals'])
    results[name]=dict(checks=checks,intervals=len(reference['intervals']),
                       rear_coverage=report['observation_world_coverage']['back'])
    assert all(checks.values()),results[name]
(out/'review.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(results,ensure_ascii=False,indent=2))
