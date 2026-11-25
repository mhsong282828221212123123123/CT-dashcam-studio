"""Compare filtered outputs to immutable baseline and source-reviewed controls."""
import argparse,json
from pathlib import Path
from collections import Counter
from ct_scene.artifacts import verify
from ct_scene.quality_review import review
p=argparse.ArgumentParser();p.add_argument('--case',nargs=3,action='append',required=True,metavar=('NAME','BEFORE','AFTER'));p.add_argument('--controls',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
results=[];states={};baselines={};sources={}
for name,before,after in a.case:
    before,after=Path(before),Path(after);bm=verify(before);am=verify(after)
    proof=json.loads((after/'driving-road-scope-review.json').read_text())
    if proof['parent_scene_artifact_id']!=bm['artifact_id']:raise ValueError('Wrong baseline')
    sources[name]=next(r['artifact_id'] for r in bm['inputs'] if r['role']=='source')
    b=[json.loads(x) for x in (before/'render-states.jsonl').read_text().splitlines()];s=[json.loads(x) for x in (after/'render-states.jsonl').read_text().splitlines()];baselines[name]=b;states[name]=s
    assert len(b)==len(s)
    for x,y in zip(b,s):
        assert {k:v for k,v in x.items() if k!='actors'}=={k:v for k,v in y.items() if k!='actors'}
        old={r['id']:r for r in x['actors']};assert all(r==old[r['id']] for r in y['actors'])
    assert (before/'actors.json').read_bytes()==(after/'actors.json').read_bytes()
    qr=after.with_name(after.name+'-quality')
    q=json.loads((qr/'review.json').read_text()) if qr.exists() else review(after,qr)
    assert q['artifact_id']==am['artifact_id']
    results.append(dict(scene=name,artifact_id=am['artifact_id'],decoded_frames=q['frames_decoded'],numerical_gate=q['numerical_gate'],failures=q['failure_count'],remaining_actor_states_exact=True,ego_and_road_exact=True,raw_actor_file_exact=True,removed_actor_frames=len(proof['removed']),removed_by_id=dict(Counter(r['id'] for r in proof['removed']))))
checks=[]
for c in json.loads(a.controls.read_text())['controls']:
    if c['source_artifact_id']!=sources[c['scene']]:raise ValueError('Wrong source control')
    old=min(baselines[c['scene']],key=lambda x:abs(x['time_s']-c['time_s']));new=min(states[c['scene']],key=lambda x:abs(x['time_s']-c['time_s']));was=any(x['id']==c['id'] for x in old['actors']);now=any(x['id']==c['id'] for x in new['actors']);checks.append(dict(**c,before_present=was,after_present=now,passed=bool(was and now==(c['expected']=='keep'))))
result=dict(scenes=results,source_controls=checks,gate='PASS' if all(x['passed'] for x in checks) and all(x['numerical_gate']=='PASS' for x in results) else 'FAIL',scope='Selected development source controls and exact retained-state preservation; not metric 3D accuracy')
a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result));raise SystemExit(0 if result['gate']=='PASS' else 2)
