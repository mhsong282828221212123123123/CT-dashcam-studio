"""Run a frozen engine on preselected dates; preserve failures without retuning."""
import argparse,json,os,shutil,subprocess,sys,time
from pathlib import Path
from ct_scene.artifacts import canonical,digest
from ct_scene.ingest import file_hash

p=argparse.ArgumentParser();p.add_argument('--selections',nargs='+',required=True);p.add_argument('--controls',required=True);p.add_argument('--output',required=True);p.add_argument('--reuse-suite',nargs='+');p.add_argument('--osm-cache-dir');p.add_argument('--source-role',choices=['development_regression','provisional_new_date_validation','new_clip_validation_on_exposed_dates'],default='provisional_new_date_validation');a=p.parse_args()
root=Path.cwd().resolve();out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=False)
selections=[json.loads(Path(s).read_text(encoding='utf-8')) for s in a.selections];sources=[r for m in selections for r in m['selected']]
if len({s['date'] for s in sources})!=len(sources):raise ValueError('Distinct dates required')
controls=json.loads(Path(a.controls).read_text(encoding='utf-8'))
if {r['source_id'] for r in sources}!={r['source_id'] for r in controls['sources']}:raise ValueError('Source controls mismatch')
freeze=out/'frozen';shutil.copytree(root/'src/ct_scene',freeze/'src/ct_scene',ignore=shutil.ignore_patterns('__pycache__'))
shutil.copytree(root/'configs',freeze/'configs');shutil.copytree(Path('E:/tesla/project/fsd-sedan-3d/models'),freeze/'assets')
shutil.copytree(root/'tests',freeze/'tests',ignore=shutil.ignore_patterns('__pycache__'))
shutil.copy2(a.controls,out/'source-controls.json')
model=root.parent/'scene-models/yolo11x.onnx';road_model=root.parent/'scene-models/model-review/yolopv2-roads.onnx'
osm_files={}
if a.osm_cache_dir:
    for source in sources:
        cache=Path(a.osm_cache_dir)/(source['source_id']+'.osm')
        if cache.exists():
            target=freeze/'osm'/cache.name;target.parent.mkdir(exist_ok=True);shutil.copy2(cache,target);osm_files[source['source_id']]=dict(path=str(target),sha256=file_hash(target))
manifest=dict(sources=sources,controls_sha256=file_hash(out/'source-controls.json'),engine_sha256=digest(b''.join(f.read_bytes() for f in sorted((freeze/'src/ct_scene').glob('*.py')))),
              model_sha256=file_hash(model),road_model_sha256=file_hash(road_model),osm_policy='explicit supplied caches only; no automatic network query',
              outputs_inspected_at_freeze=False,source_role=a.source_role,osm_caches=osm_files,status='running',results=[])
(out/'suite.json').write_bytes(canonical(manifest));env=os.environ.copy();env['PYTHONPATH']=str(freeze/'src');env['PYTHONUNBUFFERED']='1'
for source in sources:
    if file_hash(model)!=manifest['model_sha256'] or file_hash(road_model)!=manifest['road_model_sha256']:raise ValueError('Frozen model changed')
    dest=out/source['source_id'];cache_reused=[]
    if a.reuse_suite:
        from ct_scene.artifacts import verify
        dest.mkdir(exist_ok=True)
        # Only unchanged upstream modules may reuse their own verified artifacts.
        # Changed scene/lamp/state code never reuses a prior compiled scene.
        for stage,module in [('source','annotation.py'),('telemetry','telemetry.py'),('detections','mask_prepass.py'),('tracks','tracking.py'),('base','full_timeline.py'),('road-masks','road_perception.py')]:
            for previous in a.reuse_suite:
                prior=Path(previous);old=prior/source['source_id']/stage
                if not (old/'artifact.json').exists():continue
                dependencies=[module]
                if stage=='base':dependencies+=['wheel_scale.py','offline_paths.py','affine_probe.py','prototype_front.py','ego_turn_rotation.py','actor_heading.py']
                if any((prior/'frozen/src/ct_scene'/m).read_bytes()!=(freeze/'src/ct_scene'/m).read_bytes() for m in dependencies):
                    print('INVALIDATE_CHANGED_STAGE',source['source_id'],stage,flush=True);continue
                verify(old);shutil.copytree(old,dest/stage);cache_reused.append(stage);break
    command=[sys.executable,'-m','ct_scene','analyze','--front',source['source_files']['front'],'--output',str(dest),
        '--detector',str(model),'--road-model',str(road_model),'--lens-config',str(freeze/'configs/multicam-optical-candidates.json'),
        '--prototype-config',str(freeze/'configs/surrounding-prototype.json'),'--assets-root',str(freeze/'assets')]
    if source['source_id'] in osm_files:
        cache=osm_files[source['source_id']]
        if file_hash(Path(cache['path']))!=cache['sha256']:raise ValueError('OSM cache changed')
        command.extend(['--osm',cache['path']])
    print('START',source['source_id'],flush=True);start=time.perf_counter()
    with (out/(source['source_id']+'.log')).open('w',encoding='utf-8') as log:
        result=subprocess.run(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
    record=dict(source_id=source['source_id'],returncode=result.returncode,elapsed_s=time.perf_counter()-start,cache_reused=cache_reused,source_relation_quality='PENDING_INDEPENDENT_CONTROLS')
    state=dest/'pipeline.json'
    if state.exists():record['pipeline_status']=json.loads(state.read_text(encoding='utf-8'))['status']
    manifest['results'].append(record);(out/'suite.json').write_bytes(canonical(manifest));print('END',json.dumps(record),flush=True)
manifest['status']='finished';manifest['engine_unchanged']=digest(b''.join(f.read_bytes() for f in sorted((freeze/'src/ct_scene').glob('*.py'))))==manifest['engine_sha256']
(out/'suite.json').write_bytes(canonical(manifest));print('SUITE_FINISHED',flush=True)
