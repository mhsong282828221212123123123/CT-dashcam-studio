"""Frozen full-source regression of own rotation, then surrounding geometry."""
import argparse,json,os,shutil,subprocess,sys,time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
from ct_scene.artifacts import canonical,digest,verify

ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--all-image-headings',action='store_true');a=ap.parse_args();root=Path.cwd();out=a.output.resolve();out.mkdir(exist_ok=False);frozen=out/'frozen'
shutil.copytree(root/'src/ct_scene',frozen/'src/ct_scene',ignore=shutil.ignore_patterns('__pycache__'));shutil.copytree(root/'configs',frozen/'configs');shutil.copytree('E:/tesla/project/fsd-sedan-3d/models',frozen/'assets')
revision=lambda:digest(b''.join(p.read_bytes() for p in sorted((frozen/'src/ct_scene').glob('*.py'))));started=revision();sources={}
for f in ['runs/new-date-selection-006/selection.json','runs/new-date-selection-004/selection.json']:
    for r in json.loads(Path(f).read_text())['selected']:sources[r['source_id']]=r['source_files']['front']
jobs=[('aug25','runs/new-date-suite-006/2026-08-25_08-20-56'),('oct22','runs/new-date-recovery-suite-001/2026-10-22_08-11-29'),('aug16','runs/new-date-suite-006/2026-08-16_14-59-58')]
env=os.environ.copy();env['PYTHONPATH']=str(frozen/'src');env['PYTHONUNBUFFERED']='1'
def job(entry):
    name,path=entry;case=Path(path);dest=out/name;dest.mkdir();rotation=Path('runs')/(name+'-own-background-rotation-001');start=time.perf_counter()
    base=[sys.executable,'-m','ct_scene','full-source-timeline','--package',str(case/'source'),'--tracks',str(case/'tracks'),'--telemetry',str(case/'telemetry'),'--lens-config',str(frozen/'configs/multicam-optical-candidates.json'),'--source',sources[case.name],'--actor-model','auto','--assets-root',str(frozen/'assets'),'--ego-rotation',str(rotation),'--output',str(dest/'base')]
    scene=[sys.executable,'-m','ct_scene','surrounding-prototype','--package',str(case/'source'),'--tracks',str(case/'tracks'),'--telemetry',str(case/'telemetry'),'--road-masks',str(case/'road-masks'),'--base',str(dest/'base'),'--lens-config',str(frozen/'configs/multicam-optical-candidates.json'),'--prototype-config',str(frozen/'configs/surrounding-prototype.json'),'--assets-root',str(frozen/'assets'),'--output',str(dest/'scene')]
    if name=='aug25':scene+=['--image-headings','runs/aug25-image-heading-evidence-002']
    elif a.all_image_headings:scene+=['--image-headings','runs/'+name+'-image-heading-evidence-001']
    context=Path('runs')/(name+'-branch-context-001')
    fresh=Path('runs/new-four-map-context-001')/case.name
    if fresh.exists():context=fresh
    if context.exists():scene+=['--osm-context',str(context)]
    for label,cmd in [('base',base),('scene',scene)]:
        print('START '+name+' '+label,flush=True)
        with (dest/(label+'.log')).open('w',encoding='utf-8') as log:r=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
        if r.returncode:return dict(name=name,failed_stage=label,returncode=r.returncode)
    return dict(name=name,returncode=0,elapsed_s=time.perf_counter()-start,source_artifact=verify(case/'source')['artifact_id'],rotation_artifact=verify(rotation)['artifact_id'],scene_artifact=verify(dest/'scene')['artifact_id'])
with ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(job,jobs))
(out/'suite.json').write_bytes(canonical(dict(engine_sha256=started,engine_unchanged=revision()==started,results=results,network_requests=0,scope='development regression; own motion is independent of actor world trajectories')));print(json.dumps(results),flush=True)
