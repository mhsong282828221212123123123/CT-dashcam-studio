"""Independent raw-labelled object checks. Reads predictions; never refits them.

Identity is fixed by the FIRST manual box, not reassigned to the best actor at
each timestamp. Subsequent manual boxes test that decision independently.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from review_guards import source_exit_code,require_window
from ct_scene.artifacts import verify
from ct_scene.road_frame import coordinates
from ct_scene.road_geometry import lane_at,boundaries_at


def load(p):return json.loads(Path(p).read_text(encoding='utf-8'))


def iou(a,b):
    a,b=np.asarray(a,float),np.asarray(b,float)
    overlap=np.maximum(0,np.minimum(a[2:],b[2:])-np.maximum(a[:2],b[:2]))
    inter=float(np.prod(overlap));area=lambda q:float(np.prod(np.maximum(0,q[2:]-q[:2])))
    return inter/max(area(a)+area(b)-inter,1e-9)


def match_anchor(anchor,observations,local_to_global,gates):
    nearby=[o for o in observations if abs(o['camera_time_s']-anchor['time_s'])<=gates['anchor_time_tolerance_s']
            and o['support'] in ('detector_observation','flow_observation')]
    # One score per local identity, so a densely sampled track cannot create a
    # false runner-up ambiguity against itself.
    candidates={}
    for o in nearby:
        key=o['local_track_id'];score=iou(anchor['bbox_xyxy'],o['bbox_xyxy'])
        if score>candidates.get(key,(-1,None))[0]:candidates[key]=(score,o['camera_time_s'])
    rank=sorted(candidates.items(),key=lambda item:item[1][0],reverse=True)
    if not rank:return dict(status='NO_OBSERVATION',global_id=None)
    key,(score,t)=rank[0];margin=score-(rank[1][1][0] if len(rank)>1 else 0)
    accepted=score>=gates['anchor_iou_min'] and margin>=gates['anchor_match_margin_min']
    return dict(status='MATCH' if accepted else 'AMBIGUOUS_OR_LOW_IOU',local_id=key,
                global_id=local_to_global.get(key) if accepted else None,iou=score,margin=margin,observation_time_s=t)


def longest_absence(times,ok):
    longest=start=0.;active=False
    for t,valid in zip(times,ok):
        if not valid and not active:start=t;active=True
        if valid and active:longest=max(longest,t-start);active=False
    if active:longest=max(longest,times[-1]-start+(np.median(np.diff(times)) if len(times)>1 else 0))
    return float(longest)


def relative_lane_matches(row,offset):
    """Unknown membership cannot pass; source need not assert absolute index."""
    actor,ego=row.get('actor_lane'),row.get('ego_lane')
    return actor is not None and ego is not None and actor>=0 and ego>=0 and actor-ego==offset


def evaluate(target,case,gates,scene_dir=None):
    scene=Path(scene_dir) if scene_dir else case/'scene';manifest=verify(scene);verify(case/'tracks')
    if manifest['stage']=='manual_review' or load(scene/'report.json').get('manual_corrections'):
        raise ValueError('Human-assisted derivative is excluded from automatic quality scores')
    actors=load(scene/'actors.json');road=load(scene/'report.json')['road']
    mapping={local:a['global_id'] for a in actors for local in a['local_ids']}
    observations=[json.loads(line) for line in (case/'tracks/observations.jsonl').read_text().splitlines()]
    observations=[o for o in observations if o['camera_id']==target['camera']]
    assert all(o['frame_id'].startswith(target['source_sha256']+':') for o in observations),'Source hash mismatch'
    anchors=[match_anchor(a,observations,mapping,gates) for a in target['anchors']]
    fixed_id=anchors[0]['global_id'];identity_ok=fixed_id is not None and all(a['global_id']==fixed_id for a in anchors)
    all_states=[json.loads(line) for line in (scene/'render-states.jsonl').read_text().splitlines()]
    base=load(case/'base/trajectory.json')
    ego_times=np.array([r['camera_time_s'] for r in base]);ego_positions=np.array([r['ego_position_W'] for r in base]);ego_yaws=np.unwrap([r['ego_yaw_rad'] for r in base])
    lo,hi=target['interval_s'];states=[s for s in all_states if lo<=s['time_s']<=hi]
    require_window(all_states,lo,hi)
    checks=[]
    for s in states:
        a=next((a for a in s['actors'] if a['id']==fixed_id),None)
        # A stabilized render camera during an ego lane change is NOT the ego.
        ego=np.array([np.interp(s['time_s'],ego_times,ego_positions[:,k]) for k in range(3)])
        yaw=float(np.interp(s['time_s'],ego_times,ego_yaws))
        er=coordinates(road,ego);elane=lane_at(road,er[0],er[1])
        row=dict(time_s=s['time_s'],present=a is not None,solid=bool(a and not a['ghost']),ego_lane=elane,road_lane_count=len(boundaries_at(road,float(er[0])))-1)
        if a:
            p=np.asarray(a['position_W']);delta=p-ego
            ar=coordinates(road,p);alane=lane_at(road,ar[0],ar[1]);dyaw=math.atan2(math.sin(a['yaw_rad']-ar[2]),math.cos(a['yaw_rad']-ar[2]))
            row.update(actor_lane=alane,road_lateral_m=float(ar[1]),
                       ahead_m=float(math.cos(yaw)*delta[0]+math.sin(yaw)*delta[1]),
                       left_m=float(-math.sin(yaw)*delta[0]+math.cos(yaw)*delta[1]),
                       heading_error_deg=abs(math.degrees(dyaw)))
        checks.append(row)
    fraction=lambda predicate:sum(bool(predicate(r)) for r in checks)/max(len(checks),1)
    metrics=dict(solid_coverage=fraction(lambda r:r['solid']),
                 longest_absence_s=longest_absence([r['time_s'] for r in checks],[r['solid'] for r in checks]),
                 same_direction_fraction=fraction(lambda r:r.get('heading_error_deg',180)<=gates['max_relative_heading_deg']))
    passed=dict(identity_anchors=identity_ok,solid_coverage=metrics['solid_coverage']>=gates['solid_coverage_min'],
                longest_absence=metrics['longest_absence_s']<=gates['longest_absence_max_s'],
                same_direction=metrics['same_direction_fraction']>=gates['same_direction_fraction_min'])
    if 'expected_lane' in target:
        lane=target['expected_lane']
        intervals=target.get('expected_lane_intervals_s',[target['interval_s']])
        lane_checks=[r for r in checks if any(lo<=r['time_s']<=hi for lo,hi in intervals)]
        metrics['lane_relation_samples']=len(lane_checks)
        metrics['expected_lane_fraction']=sum(r.get('actor_lane')==lane and r['ego_lane']==lane for r in lane_checks)/max(1,len(lane_checks))
        metrics['lead_ahead_fraction']=fraction(lambda r:r.get('ahead_m',-1)>0)
        passed['expected_lane']=metrics['expected_lane_fraction']>=gates['expected_lane_fraction_min']
        passed['lead_ahead']=metrics['lead_ahead_fraction']>=gates['lead_ahead_fraction_min']
    if 'expected_lane_count' in target:
        intervals=target.get('expected_lane_intervals_s',[target['interval_s']])
        count_checks=[r for r in checks if any(lo<=r['time_s']<=hi for lo,hi in intervals)]
        metrics['expected_lane_count_fraction']=sum(r['road_lane_count']==target['expected_lane_count'] for r in count_checks)/max(1,len(count_checks))
        passed['expected_lane_count']=metrics['expected_lane_count_fraction']>=gates['expected_lane_count_fraction_min']
    if 'expected_relative_lane' in target:
        offset=target['expected_relative_lane']
        metrics['relative_lane_fraction']=fraction(lambda r:relative_lane_matches(r,offset))
        passed['relative_lane']=metrics['relative_lane_fraction']>=gates['expected_lane_fraction_min']
        if offset==0:
            metrics['lead_ahead_fraction']=fraction(lambda r:r.get('ahead_m',-1)>0)
            passed['lead_ahead']=metrics['lead_ahead_fraction']>=gates['lead_ahead_fraction_min']
    return dict(object_id=target['object_id'],source_id=target['source_id'],fixed_global_id=fixed_id,
                anchors=anchors,metrics=metrics,checks=passed,gate='PASS' if all(passed.values()) else 'FAIL',
                rows=checks,scope='selected continuously source-visible object only; no metric accuracy or exhaustive recall claim')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--suite',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--controls',type=Path,default=Path('runs/dense-source-controls-001'))
    args=ap.parse_args();root=args.controls;targets=load(root/'objects.json')['targets'];gates=load(root/'gates.json')
    if not targets:raise ValueError('No labelled source targets')
    assert all(t['dense_visibility_reviewed'] and not t['production_predictions_used'] for t in targets)
    missing=[t['source_id'] for t in targets if not (args.suite/t['source_id']/'scene/artifact.json').exists()]
    if missing:raise ValueError('Source-control evaluation requires every labelled source: '+', '.join(sorted(set(missing))))
    args.output.mkdir(parents=True,exist_ok=False);results=[]
    for target in targets:
        case=args.suite/target['source_id']
        result=evaluate(target,case,gates);results.append({k:v for k,v in result.items() if k!='rows'})
        (args.output/(target['object_id']+'.json')).write_text(json.dumps(result,indent=2),encoding='utf-8')
    report=dict(gates=gates,annotation_sha256=hashlib.sha256((root/'objects.json').read_bytes()).hexdigest(),
                complete=len(results)==len(targets),results=results)
    (args.output/'review.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report,indent=2))
    return source_exit_code(results)


if __name__=='__main__':raise SystemExit(main())
