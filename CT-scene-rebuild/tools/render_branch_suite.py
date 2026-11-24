"""Three cached full-source development cases with an immutable engine copy."""
import argparse,json,os,shutil,subprocess,sys,time
from pathlib import Path
from ct_scene.artifacts import canonical,digest,verify

ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--include-oct22',action='store_true');ap.add_argument('--image-headings',action='store_true');ap.add_argument('--own-rotation',action='store_true');args=ap.parse_args();root=Path.cwd().resolve();out=args.output.resolve();out.mkdir(exist_ok=False)
freeze=out/'frozen';shutil.copytree('src/ct_scene',freeze/'src/ct_scene',ignore=shutil.ignore_patterns('__pycache__'));shutil.copytree('configs',freeze/'configs');shutil.copytree('E:/tesla/project/fsd-sedan-3d/models',freeze/'assets')
jobs=[('oct26',Path('runs/new-date-suite-003/2026-10-26_12-01-20')),('aug28',Path('runs/new-date-recovery-suite-001/2026-08-28_08-55-51')),('aug26',Path('runs/new-date-recovery-suite-001/2026-08-26_08-32-43'))]
if args.include_oct22:jobs.append(('oct22',Path('runs/new-date-recovery-suite-001/2026-10-22_08-11-29')))
revision=lambda:digest(b''.join(p.read_bytes() for p in sorted((freeze/'src/ct_scene').glob('*.py'))))
record=dict(scope='development regression; no new holdout claim',engine_sha256=revision(),status='running',results=[],network_requests=0);env=os.environ.copy();env['PYTHONPATH']=str(freeze/'src');env['PYTHONUNBUFFERED']='1'
sources={}
for p in Path('runs').glob('new-date-selection-*/selection.json'):
    for source in json.loads(p.read_text())['selected']:sources[source['source_id']]=source['source_files']['front']
for name,case in jobs:
    context=Path('runs')/(name+'-branch-context-001');inputs={s:verify(case/s)['artifact_id'] for s in ['source','tracks','base','road-masks','telemetry']};inputs['osm-context']=verify(context)['artifact_id']
    base=case/'base'
    if args.own_rotation:
        rotation=Path('runs')/(name+'-own-background-rotation-001');inputs['ego-rotation']=verify(rotation)['artifact_id'];inputs['prior-base-reference']=inputs['base'];base=out/(name+'-base')
        base_command=[sys.executable,'-m','ct_scene','full-source-timeline','--package',str(case/'source'),'--tracks',str(case/'tracks'),'--telemetry',str(case/'telemetry'),'--lens-config',str(freeze/'configs/multicam-optical-candidates.json'),'--source',sources[case.name],'--actor-model','auto','--assets-root',str(freeze/'assets'),'--ego-rotation',str(rotation),'--output',str(base)]
        with (out/(name+'-base.log')).open('w',encoding='utf-8') as log:base_result=subprocess.run(base_command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
        if base_result.returncode:
            record['results'].append(dict(name=name,returncode=base_result.returncode,failed_stage='base'));(out/'suite.json').write_bytes(canonical(record));continue
        inputs['base']=verify(base)['artifact_id']
    command=[sys.executable,'-m','ct_scene','surrounding-prototype','--package',str(case/'source'),'--tracks',str(case/'tracks'),'--base',str(base),'--road-masks',str(case/'road-masks'),'--telemetry',str(case/'telemetry'),'--osm-context',str(context),'--lens-config',str(freeze/'configs/multicam-optical-candidates.json'),'--prototype-config',str(freeze/'configs/surrounding-prototype.json'),'--assets-root',str(freeze/'assets'),'--output',str(out/name)]
    if args.image_headings:
        headings=Path('runs')/(name+'-image-heading-evidence-001');inputs['image-headings']=verify(headings)['artifact_id'];command+=['--image-headings',str(headings)]
    print('START '+name,flush=True);started=time.perf_counter()
    with (out/(name+'.log')).open('w',encoding='utf-8') as log:result=subprocess.run(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
    record['results'].append(dict(name=name,returncode=result.returncode,elapsed_s=time.perf_counter()-started,inputs=inputs));(out/'suite.json').write_bytes(canonical(record));print('END '+name+' '+str(result.returncode),flush=True)
record['status']='finished';record['engine_unchanged']=revision()==record['engine_sha256'];(out/'suite.json').write_bytes(canonical(record));print('FINISHED',flush=True)
