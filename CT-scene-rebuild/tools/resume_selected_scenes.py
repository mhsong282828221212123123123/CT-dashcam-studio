"""Finish the two interrupted cases while preserving all stopped run outputs."""
import argparse,json,os,subprocess,sys,time
from pathlib import Path
from ct_scene.artifacts import canonical,verify,digest
from review_guards import require_cases

p=argparse.ArgumentParser();p.add_argument('--resume-from',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
prior=a.resume_from.resolve();out=a.output.resolve();stop=json.loads((prior/'STOPPED_BY_USER.json').read_text());freeze=prior/'frozen'
revision=lambda:digest(b''.join(p.read_bytes() for p in sorted((freeze/'src/ct_scene').glob('*.py'))))
if revision()!=stop['latest_engine_sha256']:raise ValueError('Stopped engine changed')
out.mkdir(exist_ok=False)
cases=[]
for row in stop['completed_exports']:
    scene=Path(row['video']).parent;m=verify(scene)
    if m['artifact_id']!=row['artifact_id']:raise ValueError('Reused scene changed')
    base=prior/row['group']/(row['source_id']+'-base') if row['group']=='branch' else scene.parent/'base'
    verify(base)
    cases.append(dict(case_id=row['group']+'/'+row['source_id'],scene=str(scene),base=str(base),artifact_id=m['artifact_id'],action='reuse',state='complete'))
jobs=[('aug26','runs/new-date-recovery-suite-001/2026-08-26_08-32-43'),('oct22','runs/new-date-recovery-suite-001/2026-10-22_08-11-29')]
for key,source in jobs:cases.append(dict(case_id='branch/'+key,scene=str(out/'branch'/key),base=str(prior/'branch'/(key+'-base')),source_case=source,action='render',state='pending'))
expected=['four/'+x for x in ['2026-08-16_14-59-58','2026-08-25_08-20-56','2026-09-02_12-07-46','2026-09-07_18-09-51']]+['six/'+x for x in ['2026-08-26_08-33-44','2026-09-14_22-14-31','2026-10-18_11-23-27','2026-10-02_14-58-03','2026-11-10_22-07-20','2026-08-20_07-34-02']]+['branch/'+x for x in ['oct26','aug28','aug26','oct22']]
require_cases([r['case_id'] for r in cases],expected)
record=dict(status='running',engine_sha256=revision(),engine_root=str(freeze),expected_case_ids=expected,cases=cases,source_quality='REVIEW_REQUIRED')
save=lambda:(out/'cases.json').write_bytes(canonical(record));save()
env=os.environ.copy();env['PYTHONPATH']=str(freeze/'src');env['PYTHONUNBUFFERED']='1'
for row in cases:
    if row['action']=='reuse':continue
    key=row['case_id'].split('/')[1];source=Path(row['source_case']);context=Path('runs')/(key+'-branch-context-001');headings=Path('runs')/(key+'-image-heading-evidence-001')
    inputs={name:verify(path)['artifact_id'] for name,path in [('source',source/'source'),('tracks',source/'tracks'),('base',Path(row['base'])),('road-masks',source/'road-masks'),('telemetry',source/'telemetry'),('osm-context',context),('image-headings',headings)]}
    row.update(state='running',inputs=inputs);save();start=time.perf_counter();print('START',key,flush=True)
    command=[sys.executable,'-m','ct_scene','surrounding-prototype','--package',str(source/'source'),'--tracks',str(source/'tracks'),'--base',row['base'],'--road-masks',str(source/'road-masks'),'--telemetry',str(source/'telemetry'),'--osm-context',str(context),'--image-headings',str(headings),'--lens-config',str(freeze/'configs/multicam-optical-candidates.json'),'--prototype-config',str(freeze/'configs/surrounding-prototype.json'),'--assets-root',str(freeze/'assets'),'--output',row['scene']]
    with (out/(key+'.log')).open('w',encoding='utf-8') as log:r=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT)
    row.update(returncode=r.returncode,state='complete' if r.returncode==0 else 'failed',elapsed_s=time.perf_counter()-start)
    if r.returncode==0:row['artifact_id']=verify(Path(row['scene']))['artifact_id']
    save();print('END',key,r.returncode,flush=True)
record.update(status='finished',engine_unchanged=revision()==record['engine_sha256']);save()
raise SystemExit(0 if record['engine_unchanged'] and all(r['state']=='complete' for r in cases) else 2)
