"""Recompute initial anchor phases using the same cached inputs as a baseline.

No detection, map queries or render regeneration. Any changed body phase requires
source review; all other trajectory fields must remain exactly equal.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

from ct_scene.artifacts import canonical, digest, verify
from review_guards import require_cases

p=argparse.ArgumentParser()
p.add_argument('--manifest',type=Path,required=True)
p.add_argument('--engine',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
manifest=json.loads(a.manifest.read_text())
require_cases([r['case_id'] for r in manifest['cases']],manifest['expected_case_ids'])
engine=a.engine.resolve()
revision=lambda:digest(b''.join(f.read_bytes() for f in sorted((engine/'src/ct_scene').glob('*.py'))))
started=revision()
a.output.mkdir(exist_ok=False)
index={}
for pattern in ('*/artifact.json','*/*/artifact.json','*/*/*/artifact.json'):
    for f in Path('runs').glob(pattern):
        m=json.loads(f.read_text())
        index.setdefault(m['artifact_id'],f.parent)
sources={}
for pattern in ('new-date-selection-*/selection.json','new-clip-selection-*/selection.json'):
    for f in sorted(Path('runs').glob(pattern)):
        for r in json.loads(f.read_text()).get('selected',[]):
            if 'source_files' in r:
                sources[r['source_sha256']['front']]=r['source_files']['front']
env=os.environ.copy()
env['PYTHONPATH']=str(engine/'src')
result=dict(engine_root=str(engine),engine_sha256=started,status='running',cases=[],
            scope='Initial anchor phase regression only; changed phases require raw-source review, not overall quality acceptance')
save=lambda:(a.output/'review.json').write_bytes(canonical(result))
save()
for case in manifest['cases']:
    old=Path(case['base']); oldm=verify(old)
    inputs={r['role']:index[r['artifact_id']] for r in oldm['inputs']}
    for role,path in inputs.items():
        if verify(path)['artifact_id']!=next(r['artifact_id'] for r in oldm['inputs'] if r['role']==role):
            raise ValueError('Input artifact mismatch')
    frame=json.loads((inputs['source']/'frames.json').read_text())
    sha=next(f['source_sha256'] for f in frame if f['camera_id']=='front')
    dest=a.output/case['case_id'].replace('/','_')
    command=[sys.executable,'-m','ct_scene','full-source-timeline',
             '--package',str(inputs['source']),'--tracks',str(inputs['tracks']),
             '--telemetry',str(inputs['telemetry']),'--source',sources[sha],
             '--lens-config',str(engine/'configs/multicam-optical-candidates.json'),
             '--actor-model','auto','--assets-root',str(engine/'assets'),'--output',str(dest)]
    if 'own_background_rotation' in inputs:
        command+=['--ego-rotation',str(inputs['own_background_rotation'])]
    print('START',case['case_id'],flush=True)
    with (a.output/(dest.name+'.log')).open('w',encoding='utf-8') as log:
        run=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT)
    row=dict(case_id=case['case_id'],base=str(dest.resolve()),returncode=run.returncode)
    if run.returncode==0:
        verify(dest)
        before=json.loads((old/'trajectory.json').read_text())
        after=json.loads((dest/'trajectory.json').read_text())
        invariant=len(before)==len(after) and all(
            {k:v for k,v in x.items() if k!='actor_yaw_rad'}==
            {k:v for k,v in y.items() if k!='actor_yaw_rad'} for x,y in zip(before,after))
        row.update(non_body_fields_identical=invariant,all_fields_identical=before==after,
                   phase_review=json.loads((dest/'report.json').read_text()).get('initial_body_phase_review'))
    result['cases'].append(row);save()
    print('END',case['case_id'],{k:v for k,v in row.items() if k not in ('phase_review','base')},flush=True)
result.update(status='finished',engine_unchanged=revision()==started)
result['invariant_gate']='PASS' if len(result['cases'])==len(manifest['cases']) and result['engine_unchanged'] and all(r.get('non_body_fields_identical') for r in result['cases']) else 'FAIL'
result['changed_cases']=[r['case_id'] for r in result['cases'] if not r.get('all_fields_identical',False)]
save()
raise SystemExit(0 if result['invariant_gate']=='PASS' else 2)
