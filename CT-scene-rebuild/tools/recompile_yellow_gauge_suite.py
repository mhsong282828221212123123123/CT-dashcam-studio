"""Recompile 14 cached scenes; fixed ego/base and unchanged perception inputs."""
import argparse,json,os,shutil,subprocess,sys,time
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
from ct_scene.artifacts import canonical,digest,verify

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();root=Path.cwd();out=a.output.resolve();out.mkdir(exist_ok=False);freeze=out/'frozen'
for name in ('src/ct_scene','configs','tests'):shutil.copytree(root/name,freeze/name,ignore=shutil.ignore_patterns('__pycache__'))
shutil.copytree('E:/tesla/project/fsd-sedan-3d/models',freeze/'assets')
revision=lambda:digest(b''.join(p.read_bytes() for p in sorted((freeze/'src/ct_scene').glob('*.py'))));sha=revision();env=os.environ.copy();env['PYTHONPATH']=str(freeze/'src');env['PYTHONUNBUFFERED']='1'
jobs=[]
for kind,prior in [('four',Path('runs/new-date-suite-013')),('six',Path('runs/new-six-clip-suite-012'))]:
    for item in json.loads((prior/'suite.json').read_text())['results']:
        key=item['source_id'];case=prior/key;jobs.append(dict(kind=kind,key=key,case=case,base=case/'base',context=case/'osm-context',headings=case/'image-headings'))
for key,case in [('oct26','runs/new-date-suite-003/2026-10-26_12-01-20'),('aug28','runs/new-date-recovery-suite-001/2026-08-28_08-55-51'),('aug26','runs/new-date-recovery-suite-001/2026-08-26_08-32-43'),('oct22','runs/new-date-recovery-suite-001/2026-10-22_08-11-29')]:
    jobs.append(dict(kind='branch',key=key,case=Path(case),base=Path('runs/branch-orientation-suite-014')/(key+'-base'),context=Path('runs')/(key+'-branch-context-001'),headings=Path('runs')/(key+'-image-heading-evidence-001')))
records={kind:dict(status='running',engine_sha256=sha,results=[],scope='development regression after new-source failure; fixed upstream observations and ego/base') for kind in ('branch','four','six')}
for kind,r in records.items():(out/kind).mkdir();(out/kind/'suite.json').write_bytes(canonical(r))
def run(job):
    kind,key,case=job['kind'],job['key'],job['case'];dest=out/kind/key;dest.mkdir();start=time.perf_counter();inputs={}
    for name,path in [('source',case/'source'),('tracks',case/'tracks'),('base',job['base']),('telemetry',case/'telemetry'),('road-masks',case/'road-masks'),('osm-context',job['context']),('image-headings',job['headings'])]:inputs[name]=verify(path)['artifact_id']
    base_dest=out/kind/(key+'-base') if kind=='branch' else dest/'base';shutil.copytree(job['base'],base_dest)
    if kind!='branch':shutil.copytree(case/'tracks',dest/'tracks')
    target=dest if kind=='branch' else dest/'scene'
    # Branch outputs need an absent destination for immutable publication.
    if kind=='branch':dest.rmdir()
    command=[sys.executable,'-m','ct_scene','surrounding-prototype','--package',str(case/'source'),'--tracks',str(case/'tracks'),'--base',str(base_dest),'--telemetry',str(case/'telemetry'),'--road-masks',str(case/'road-masks'),'--osm-context',str(job['context']),'--image-headings',str(job['headings']),'--lens-config',str(freeze/'configs/multicam-optical-candidates.json'),'--prototype-config',str(freeze/'configs/surrounding-prototype.json'),'--assets-root',str(freeze/'assets'),'--output',str(target)]
    print('START',kind,key,flush=True)
    with (out/kind/(key+'.log')).open('w',encoding='utf-8') as log:r=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT)
    return kind,dict(**({'name':key} if kind=='branch' else {'source_id':key}),returncode=r.returncode,elapsed_s=time.perf_counter()-start,inputs=inputs)
with ThreadPoolExecutor(max_workers=2) as pool:
    for future in as_completed([pool.submit(run,j) for j in jobs]):
        kind,item=future.result();records[kind]['results'].append(item);(out/kind/'suite.json').write_bytes(canonical(records[kind]));print('END',kind,item.get('name',item.get('source_id')),item['returncode'],flush=True)
for kind,r in records.items():r.update(status='finished',engine_unchanged=revision()==sha);(out/kind/'suite.json').write_bytes(canonical(r))
print('ALL_RECOMPILED',flush=True)
