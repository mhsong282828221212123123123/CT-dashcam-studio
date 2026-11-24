"""Compile explicit cached cases, reusing only verified identical revisions."""
import argparse
from concurrent.futures import ThreadPoolExecutor,as_completed
import json,os
from pathlib import Path
import subprocess,sys
from ct_scene.artifacts import canonical,digest,verify
from review_guards import require_cases

p=argparse.ArgumentParser()
for name in ('manifest','engine','output'):p.add_argument('--'+name,type=Path,required=True)
p.add_argument('--workers',type=int,default=2)
a=p.parse_args();source=json.loads(a.manifest.read_text());cases=source['cases']
require_cases([r['case_id'] for r in cases],source['expected_case_ids'])
engine=a.engine.resolve();revision=lambda:digest(b''.join(f.read_bytes() for f in sorted((engine/'src/ct_scene').glob('*.py'))))
started=revision();a.output.mkdir(exist_ok=False)
# An explicitly nominated current-scene revision determines reuse, never dates
# or a count of files. Remaining scenes are rebuilt from their cached parents.
target_revision=source['target_scene_code_revision'];rows=[]
for r in cases:
    old=verify(Path(r['scene']));base=verify(Path(r['base']))
    parents={x['role']:x['artifact_id'] for x in old['inputs']}
    if parents['frozen_ego_lead']!=base['artifact_id']:raise ValueError('Scene/base input mismatch')
    reuse=old['code_revision']==target_revision
    rows.append(dict(case_id=r['case_id'],base=r['base'],source_scene=r['scene'],
                     scene=r['scene'] if reuse else str((a.output/r['case_id'].replace('/','_')).resolve()),
                     action='reuse' if reuse else 'render',state='complete' if reuse else 'pending',
                     artifact_id=old['artifact_id'] if reuse else None))
record=dict(status='running',cases=rows,engine_root=str(engine),engine_sha256=started,
            expected_case_ids=source['expected_case_ids'],target_scene_code_revision=target_revision,
            source_quality='REVIEW_REQUIRED')
save=lambda:(a.output/'cases.json').write_bytes(canonical(record));save()
def compile_case(r):
    print('START',r['case_id'],flush=True)
    command=[sys.executable,'tools/render_rebased_case.py','--scene',r['source_scene'],'--base',r['base'],
             '--engine',str(engine),'--output',r['scene']]
    with (a.output/(r['case_id'].replace('/','_')+'.log')).open('w',encoding='utf-8') as log:
        result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
    return result.returncode
with ThreadPoolExecutor(max_workers=a.workers) as pool:
    pending={pool.submit(compile_case,r):r for r in rows if r['action']=='render'}
    for future in as_completed(pending):
        r=pending[future];r['returncode']=future.result();r['state']='complete' if r['returncode']==0 else 'failed'
        if r['returncode']==0:
            m=verify(Path(r['scene']))
            if m['code_revision']!=target_revision:raise ValueError('Mixed scene revision')
            r['artifact_id']=m['artifact_id']
        save();print('END',r['case_id'],r['state'],sum(x['state']=='complete' for x in rows),'/',len(rows),flush=True)
record.update(status='finished',engine_unchanged=revision()==started);save()
raise SystemExit(0 if record['engine_unchanged'] and all(r['state']=='complete' for r in rows) else 2)
