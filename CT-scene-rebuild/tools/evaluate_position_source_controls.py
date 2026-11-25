"""Source-identity presence and separation gates; absence never fixes overlap."""
import argparse,json
from pathlib import Path
from ct_scene.artifacts import verify,canonical
from ct_scene.collision_constraints import overlap


def resolve(actors,local_id):
    return {a.get('global_id',a['id']) for a in actors if local_id in set(a.get('local_ids',[])+[a.get('id'),a.get('global_id')])}


def evaluate(control,states,actors,dimensions):
    t=control['time_s'];s=min(states,key=lambda s:abs(s['time_s']-t))
    if abs(s['time_s']-t)>.026:raise ValueError('Control outside scene clock')
    if control['kind']=='presence':
        ids=resolve(actors,control['id']);times=control.get('interval_s');selected=[r for r in states if times[0]<=r['time_s']<=times[1]] if times else [s]
        if not selected:raise ValueError('Empty control interval')
        fraction=sum(bool(ids&{a['id'] for a in r['actors']}) for r in selected)/len(selected)
        passed=fraction>=control.get('min_fraction',1.) if control['expected']=='keep' else fraction==0
        return dict(passed=passed,present_fraction=fraction,resolved_ids=sorted(ids),frames=len(selected))
    if control['kind']=='distinct_nonoverlapping_bodies' and control.get('interval_s'):
        lo,hi=control['interval_s'];selected=[r for r in states if lo<=r['time_s']<=hi]
        if not selected:raise ValueError('Empty separation control interval')
        point={k:v for k,v in control.items() if k!='interval_s'}
        checks=[evaluate(dict(point,time_s=r['time_s']),[r],actors,dimensions) for r in selected]
        return dict(passed=all(c['passed'] for c in checks),frames=len(checks),failed_frames=sum(not c['passed'] for c in checks),missing_or_merged_frames=sum('reason' in c for c in checks),maximum_penetration_m=max(c.get('penetration_m',0.) for c in checks))
    if control['kind']=='distinct_nonoverlapping_bodies':
        sets=[resolve(actors,key) for key in control['ids']];active=[{a['id']:a for a in s['actors'] if a['id'] in ids} for ids in sets]
        if any(len(group)!=1 for group in active):return dict(passed=False,reason='required_body_missing_or_ambiguous')
        a,b=[next(iter(g.values())) for g in active]
        if a['id']==b['id']:return dict(passed=False,reason='different_source_vehicles_merged_into_one_identity')
        hit=overlap(a,b,dimensions);penetration=hit[0] if hit else 0.
        return dict(passed=penetration<=control.get('max_penetration_m',.1),penetration_m=penetration,body_ids=[a['id'],b['id']])
    raise ValueError('Unknown source control kind')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--scene',nargs=2,action='append',required=True,metavar=('NAME','PATH'));p.add_argument('--controls',type=Path,default=Path('configs/position-source-controls-v1.json'));p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    scenes={};checks=[]
    for name,path in a.scene:
        path=Path(path);manifest=verify(path);source=next(r['artifact_id'] for r in manifest['inputs'] if r['role']=='source');report=json.loads((path/'report.json').read_text());scenes[name]=(source,[json.loads(x) for x in (path/'render-states.jsonl').read_text().splitlines()],json.loads((path/'actors.json').read_text()),report['vehicle_dimensions_m'])
    for control in json.loads(a.controls.read_text())['controls']:
        if control['scene'] not in scenes:continue
        source,states,actors,dims=scenes[control['scene']]
        if source!=control['source_artifact_id']:raise ValueError('Wrong original source for control')
        checks.append(dict(control=control,**evaluate(control,states,actors,dims)))
    if not checks:raise ValueError('No source controls evaluated')
    result=dict(gate='PASS' if all(r['passed'] for r in checks) else 'FAIL',controls=len(checks),failed=sum(not r['passed'] for r in checks),checks=checks,scope='Selected original-source development controls; not metric accuracy or full detection recall')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_bytes(canonical(result));print(result['gate'],result['failed'],'/',result['controls'],'failed')
    for r in checks:
        if not r['passed']:print(r)
    raise SystemExit(0 if result['gate']=='PASS' else 2)
