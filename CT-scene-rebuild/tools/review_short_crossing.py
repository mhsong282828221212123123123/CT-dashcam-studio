"""Source-reviewed development regression for a rapid occluded crossing truck."""
import argparse,json,math
from pathlib import Path
import numpy as np
from ct_scene.artifacts import verify,canonical
from ct_scene.road_frame import coordinates
from evaluate_source_controls import match_anchor,load

parser=argparse.ArgumentParser();parser.add_argument('--result',default='runs/oct26-short-crossing-full-001');parser.add_argument('--output',default='runs/oct26-short-crossing-source-review-001');args=parser.parse_args()
case=Path('runs/new-date-suite-002/2026-10-26_12-01-20');result=Path(args.result)
verify(result);actors=load(result/'actors.json');road=load(result/'report.json')['road']
mapping={local:a['global_id'] for a in actors for local in a['local_ids']}
obs=[json.loads(l) for l in (case/'tracks/observations.jsonl').read_text().splitlines()]
obs=[r for r in obs if r['camera_id']=='front']
annotation=dict(time_s=49.5916,bbox_xyxy=[260,320,985,592],source_frame='source/images/00248.jpg',
                direction='image left to right; across carriageway, behind stopped taxi',
                review_scope='development source review; not heldout or exhaustive recall')
gates=load('runs/dense-source-controls-001/gates.json')
anchor=match_anchor(annotation,obs,mapping,gates);gid=anchor['global_id']
if gid is None:
    out=Path(args.output);out.mkdir(exist_ok=True)
    report=dict(status='FAIL',reason='source_crossing_truck_missing_from_world',annotation=annotation,anchor=anchor,
                source_relation_quality='DEVELOPMENT_SELECTED_CROSSING_ONLY')
    with (out/'review.json').open('xb') as stream:stream.write(canonical(report))
    print(json.dumps(report,indent=2));raise SystemExit(2)
actor=next(a for a in actors if a['global_id']==gid)
states=[json.loads(l) for l in (result/'render-states.jsonl').read_text().splitlines()]
states=[s for s in states if 49.0<=s['time_s']<=50.2]
samples=[next((a for a in s['actors'] if a['id']==gid),None) for s in states]
present=[a for a in samples if a]
p=np.array([a['position_W'] for a in present]);yaw=np.array([a['yaw_rad'] for a in present]);local=coordinates(road,p)
delta=p[-1,:2]-p[0,:2];direction=math.atan2(delta[1],delta[0]);alignment=np.cos(yaw-direction)
old=load('runs/oct26-junction-repair-001/actors.json');old_lead=next(a for a in old if a['frozen_lead']);new_lead=next(a for a in actors if a['frozen_lead'])
checks=dict(source_anchor_matched=anchor['status']=='MATCH',same_observed_identity='front-0083' in actor['local_ids'],
            solid_entire_reviewed_interval=all(a and not a['ghost'] for a in samples),crossing_not_lane_follower=actor['lane_index']==-2,
            lateral_motion_matches_raw=bool(local[-1,1]-local[0,1]<-4),
            body_front_matches_motion=bool(np.min(alignment)>.7),
            original_lead_samples_unchanged=old_lead['samples']==new_lead['samples'])
report=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,annotation=annotation,anchor=anchor,
            frames_reviewed=len(samples),lateral_travel_m=float(local[-1,1]-local[0,1]),body_alignment_min=float(np.min(alignment)),
            approximation='Heavy-truck proxy asset, not an exact dump-truck body; depth and width are estimated',
            source_relation_quality='DEVELOPMENT_SELECTED_CROSSING_ONLY')
out=Path(args.output);out.mkdir(exist_ok=True)
with (out/'review.json').open('xb') as stream:stream.write(canonical(report))
print(json.dumps(report,indent=2))
