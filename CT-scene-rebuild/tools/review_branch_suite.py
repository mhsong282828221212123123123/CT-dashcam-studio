"""Frozen development regressions: poses, observed lamps and source exclusions.

Selected source labels were reviewed during development; no holdout claim.
Full decode/numerical review and human source comparison are separate checks.
"""
import argparse,json
from pathlib import Path
from ct_scene.artifacts import verify,canonical

ap=argparse.ArgumentParser();ap.add_argument('--suite',type=Path,required=True);ap.add_argument('--baseline',type=Path,default=Path('runs/branch-orientation-suite-003'));ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(exist_ok=False)
load=lambda p:json.loads(p.read_text(encoding='utf-8'))
states=lambda p:[json.loads(l) for l in (p/'render-states.jsonl').read_text().splitlines()]
suite=load(a.suite/'suite.json');checks={'frozen_engine_unchanged':suite.get('engine_unchanged') is True,'all_four_finished':suite['status']=='finished' and len(suite['results'])==4 and all(x['returncode']==0 for x in suite['results'])};results={}
for name in ['oct26','aug28','aug26','oct22']:
    scene=a.suite/name;manifest=verify(scene);before=states(a.baseline/name);after=states(scene)
    actors=load(scene/'actors.json');old=load(a.baseline/name/'actors.json');by={x['global_id']:x for x in actors};oldby={x['global_id']:x for x in old};mapping={k:x['global_id'] for x in actors for k in x['local_ids']}
    common=set(by)&set(oldby);trace=load(scene/'surrounding-rear-appearance.json');lamp_errors=[];positive=0
    for state in after:
        for actor in state['actors']:
            lamp=actor['rear_appearance']
            if any(lamp[k] is not None for k in ['brake','left','right']):lamp_errors.append('fabricated_control')
            if lamp['rear_red_visible'] is True:
                positive+=1
                rows=trace.get(actor['id'],[]);source=next((r for r in rows if r['frame_id']==lamp['frame_id']),None)
                if actor['ghost'] or not source or abs(state['time_s']-lamp['source_time_s'])>.110001 or len(set(source['supporting_frame_ids']))<3:lamp_errors.append('unbacked_rear_appearance')
    local_checks=dict(same_frame_times=[x['time_s'] for x in before]==[x['time_s'] for x in after],ego_view_exactly_unchanged=[x['camera_view'] for x in before]==[x['camera_view'] for x in after],
                      common_actor_samples_exactly_unchanged=all(by[k]['samples']==oldby[k]['samples'] for k in common),no_added_actor_ids=not(set(by)-set(oldby)),source_backed_rear_appearance=not lamp_errors)
    if name=='oct22':local_checks['source_incoming_taxi_excluded']='front-0089' not in mapping;local_checks['crossing_bus_retained']='front-0095' in mapping;local_checks['crossing_car_retained']='front-0093' in mapping
    if name=='oct26':local_checks['source_crossing_truck_retained']='front-0083' in mapping
    results[name]=dict(artifact_id=manifest['artifact_id'],checks=local_checks,observed_rear_red_render_samples=positive,common_actors=len(common),removed_actor_ids=sorted(set(oldby)-set(by)),lamp_errors=lamp_errors[:10])
    checks[name]=all(local_checks.values())
report=dict(scope='selected development source regressions, not exhaustive recall or heldout accuracy',status='PASS' if all(checks.values()) else 'FAIL',checks=checks,results=results)
(a.output/'review.json').write_bytes(canonical(report));print(json.dumps(report,indent=2));raise SystemExit(0 if report['status']=='PASS' else 2)
